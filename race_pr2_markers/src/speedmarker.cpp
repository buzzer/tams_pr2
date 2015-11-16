/*
 * 2013-04-25 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 */
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <race_pr2_markers/BaseDynamics.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>

#define PIV2 (M_PI+M_PI)

const bool absolute_values = false; // negative or only absolute values for speed
const float max_vel = 0.55; // maximum robot velocity

class SpeedMarker
{
  protected:

    ros::Publisher * marker_pub;
    ros::Publisher * marker_points;
    ros::Publisher * marker_circle;
    tf::TransformListener * tf_listener;
    ros::Publisher * base_dynamics_pub;
    ros::Publisher * distance_pub;
    float distance_traveled;
    float angular_traveled;

    struct circle
    {
      geometry_msgs::PointStamped center;
      double radius;
    };

  public:

    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
      ros::Time now = msg->header.stamp;
      static ros::Time lastTime = now;
      ros::Duration duration = now - lastTime;
      static std::vector<geometry_msgs::Point> poses;
      static unsigned int count = 0;
      circle c;
      geometry_msgs::Point p_map;
      static geometry_msgs::Point p_map_old;

      if (duration >= ros::Duration(0.05)) // 20 Hz
      //if (duration >= ros::Duration(0.10)) // 10 Hz
      {
        tf::StampedTransform transform;
        //ROS_INFO("Pose: %f\t%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        //ROS_INFO("Orientation: %f", msg->pose.pose.orientation.z);

        try
        {
          tf_listener->waitForTransform("/map", "/base_footprint", msg->header.stamp, ros::Duration(2.0) );
          tf_listener->lookupTransform( "/map", "/base_footprint", msg->header.stamp, transform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
        }

        // publishes marker and dynamics
        publishMarker(msg, &transform, &duration);
        // publish distance traveled
        geometry_msgs::Vector3Stamped vector;
        vector.header.stamp = msg->header.stamp;
        vector.vector.x = distance_traveled;
        distance_pub->publish(vector);

        //add vase offset
        tf::Transform map2object = tf::Transform(transform.getRotation(), transform.getOrigin()) *
          tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.3,0,0.35));
        //TODO frame ids are only dummies
        tf::StampedTransform map2object_stamp(map2object, ros::Time(0),transform.frame_id_, transform.child_frame_id_);
        odom2map(msg->pose.pose.position, p_map, map2object_stamp);

        //TODO p_map_old is not initialized on first call!
        float dist = point_dist(p_map, p_map_old);
        //ROS_WARN("distance: %4.2f", dist);
        if (dist > 0.01)
        {
          if (poses.size() < 3)
          {
            poses.push_back(p_map);
            //ROS_WARN("add another point");
          }

          if (poses.size() >= 3)
          {
            //ROS_WARN("override point %d", count);
            poses[count] = p_map;
            publishPoints(poses);
            getCircleCenter(poses, c);
            publishCircle(c);
          }
          count = (count + 1) % 3;
          p_map_old = p_map;
        }

        ROS_ASSERT(poses.size() <= 3);
        ROS_ASSERT(count < 3);

        lastTime = now;
      }
    }

    float point_dist(geometry_msgs::Point & p1, geometry_msgs::Point & p2)
    {
      // euklidean distance
      return sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2));
    }

    void odom2map(const geometry_msgs::Point & point,
                  geometry_msgs::Point & p_map,
                  const tf::StampedTransform & tf)
    {
      p_map.x = tf.getOrigin().getX();
      p_map.y = tf.getOrigin().getY();
      p_map.z = tf.getOrigin().getZ();
    }

    void publishPoints(std::vector<geometry_msgs::Point> & points)
    {
      unsigned int counter = 0;

      visualization_msgs::Marker marker;
      visualization_msgs::MarkerArray marker_array;
      //uint32_t shape = visualization_msgs::Marker::ARROW;
      uint32_t shape = visualization_msgs::Marker::CYLINDER;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      //marker.header.frame_id = "/base_footprint";

      marker.header.stamp = ros::Time::now();

      //printf("Points: ");
      for (auto &point: points)
      {
        // Set the namespace and id for this point.  This serves to create a unique ID
        // Any point sent with the same namespace and id will overwrite the old one
        marker.ns = "points";
        marker.id = counter++;

        // Set the point type.  Initially this is CUBE, and cycles between that
        // and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the point action.  Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the point.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z + 0.001;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        //
        marker.scale.x = marker.scale.y = 0.1;
        marker.scale.z = 0.001;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5f;

        marker.lifetime = ros::Duration(2); // Marker lifetime in s

        //ROS_WARN("publish point: %4.2f, %4.2f", point.x, point.y);
        marker_array.markers.push_back(marker);
        //marker_points->publish(marker);
        //printf("(%4.2f, %4.2f, %4.2f), ", point.x, point.y, point.z);
      }
      marker_points->publish(marker_array);
      //printf("\n");
    }

    void publishCircle(circle & c)
    {
      unsigned int counter = 0;

      visualization_msgs::Marker center;
      visualization_msgs::Marker radius;
      //uint32_t shape = visualization_msgs::Marker::ARROW;
      uint32_t shape = visualization_msgs::Marker::CYLINDER;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      center.header.frame_id = "/map";
      //center.header.frame_id = "/base_footprint";

      center.header.stamp = ros::Time::now();

      // Set the namespace and id for this center.  This serves to create a unique ID
      // Any center sent with the same namespace and id will overwrite the old one
      center.ns = "center";
      center.id = counter++;

      // Set the center type.  Initially this is CUBE, and cycles between that
      // and SPHERE, ARROW, and CYLINDER
      center.type = shape;

      // Set the center action.  Options are ADD and DELETE
      center.action = visualization_msgs::Marker::ADD;

      // Set the pose of the center.  This is a full 6DOF pose relative to the frame/time specified in the header
      center.pose.position.x = c.center.point.x;
      center.pose.position.y = c.center.point.y;
      center.pose.position.z = c.center.point.z + 0.001;
      //center.pose.orientation.x = 0;
      //center.pose.orientation.y = 0;
      //center.pose.orientation.z = 0;
      //center.pose.orientation.w = 1;
      //
      center.scale.x = center.scale.y = 0.1;
      center.scale.z = 0.001;
      center.color.r = 0.0f;
      center.color.g = 1.0f;
      center.color.b = 0.0f;
      center.color.a = 1.0;
      center.lifetime = ros::Duration(2.0);

      marker_circle->publish(center);

      shape = visualization_msgs::Marker::CYLINDER;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      radius.header.frame_id = center.header.frame_id;
      radius.header.stamp = center.header.stamp;

      // Set the namespace and id for this radius.  This serves to create a unique ID
      // Any radius sent with the same namespace and id will overwrite the old one
      radius.ns = "radius";
      radius.id = counter;

      // Set the radius type.  Initially this is CUBE, and cycles between that
      // and SPHERE, ARROW, and CYLINDER
      radius.type = shape;

      // Set the radius action.  Options are ADD and DELETE
      radius.action = visualization_msgs::Marker::ADD;

      // Set the pose of the radius.  This is a full 6DOF pose relative to the frame/time specified in the header
      radius.pose.position.x = c.center.point.x;
      radius.pose.position.y = c.center.point.y;
      radius.pose.position.z = c.center.point.z;
      //radius.pose.orientation.x = 0;
      //radius.pose.orientation.y = 0;
      //radius.pose.orientation.z = 0;
      //radius.pose.orientation.w = 1;
      //
      radius.scale.x = radius.scale.y = c.radius * 2;
      radius.scale.z = 0.001;
      radius.color.r = 1.0f;
      radius.color.g = 0.0f;
      radius.color.b = 0.0f;
      radius.color.a = 0.6;
      radius.lifetime = ros::Duration(2.0);

      marker_circle->publish(radius);
    }

    SpeedMarker(ros::Publisher * marker,
        ros::Publisher * dynamics,
        ros::Publisher * distance,
        ros::Publisher * points_pub,
        ros::Publisher * marker_cir,
        tf::TransformListener * listener)
    {
      marker_pub = marker;
      base_dynamics_pub = dynamics;
      distance_pub = distance;
      marker_points = points_pub;
      marker_circle = marker_cir;
      tf_listener = listener;
      distance_traveled = 0;
    }

    float getTransVel (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                        ros::Duration * duration)
    {
      float speed = 0.0;
      static geometry_msgs::PoseWithCovarianceStamped pose = *msg;

      float delta_x = fabs(msg->pose.pose.position.x - pose.pose.pose.position.x);
      float delta_y = fabs(msg->pose.pose.position.y - pose.pose.pose.position.y);
      float dist = sqrt(delta_x*delta_x + delta_y*delta_y);

      speed = dist / duration->toSec();

      // remember last pose info
      pose = *msg;
      // store distance traveled
      distance_traveled += dist;

      return speed;
    }

    // from:
    // http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    double difangrad(double x, double y)
    {
      double arg;

      arg = fmod(y-x, PIV2);
      if (arg < 0 )  arg  = arg + PIV2;
      if (arg > M_PI) arg  = arg - PIV2;

      return (-arg);
    }

    float getAngVel (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                     ros::Duration * duration)
    {
      float vel = 0.0;
      static geometry_msgs::PoseWithCovarianceStamped pose = *msg;

      tf::Quaternion cur_orientation(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w
      );
      tf::Quaternion last_orientation(
          pose.pose.pose.orientation.x,
          pose.pose.pose.orientation.y,
          pose.pose.pose.orientation.z,
          pose.pose.pose.orientation.w
      );
      double r, p, y;
      double r_l, p_l, y_l;
      tf::Matrix3x3(cur_orientation).getRPY(r, p, y);
      tf::Matrix3x3(last_orientation).getRPY(r_l, p_l, y_l);
      ROS_DEBUG("Yaw cur: %4.2f last: %4.2f delta: %4.2f", y, y_l, difangrad(y, y_l));
      ////Sign (direction) of angular velocity is lost here
      //float delta_y =  std::min(fmod((y - y_l+2*M_PI),2*M_PI), fmod((y_l - y +2*M_PI),2*M_PI));
      float delta_y = difangrad(y, y_l);

      vel = delta_y / duration->toSec();

      // remember last pose info
      pose = *msg;
      // store distance traveled
      angular_traveled += std::abs(delta_y);

      return vel;
    }

    /*
     * Considers only 2 dimensions (x,y)
     * Algorithm from:
     * http://mathforum.org/library/drmath/view/54323.html
     */
    bool getCircleCenter(const std::vector<geometry_msgs::Point> & points, circle & c)
    {
      ROS_ASSERT(points.size() >= 3);

      float bx = points[0].x; float by = points[0].y;
      float cx = points[1].x; float cy = points[1].y;
      float dx = points[2].x; float dy = points[2].y;
      float temp = cx*cx+cy*cy;
      float bc = (bx*bx + by*by - temp)/2.0;
      float cd = (temp - dx*dx - dy*dy)/2.0;
      float det = (bx-cx)*(cy-dy)-(cx-dx)*(by-cy);
      if (fabs(det) < 1.0e-6) {
        // points on a line?
        c.center.point.x = c.center.point.y = 1000.0;
        c.center.point.z = 0.0;
        c.radius = 1000.0;
        //c->v1 = *v1;
        //c->v2 = *v2;
        //c->v3 = *v3;
        return true;
      }
      det = 1/det;
      c.center.point.x = (bc*(cy-dy)-cd*(by-cy))*det;
      c.center.point.y = ((bx-cx)*cd-(cx-dx)*bc)*det;
      c.center.point.z = points[0].z;
      cx = c.center.point.x; cy = c.center.point.y;
      c.radius = sqrt((cx-bx)*(cx-bx)+(cy-by)*(cy-by));

      return true;
    }

    void publishMarker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                       tf::StampedTransform * transform,
                       ros::Duration * duration)
    {
      static uint32_t counter = 0;
      static float vel_t_l = 0.0;
      static float vel_a_l = 0.0;
      static float acc_t_l = 0.0;
      static float acc_a_l = 0.0;

      race_pr2_markers::BaseDynamics dynamics;
      visualization_msgs::Marker marker;
      visualization_msgs::Marker textmarker;
      visualization_msgs::Marker accmarker;
      //uint32_t shape = visualization_msgs::Marker::ARROW;
      uint32_t shape = visualization_msgs::Marker::SPHERE;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";

      marker.header.stamp = msg->header.stamp;

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "trajectory";
      marker.id = counter++;

      // Set the marker type.  Initially this is CUBE, and cycles between that
      // and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = transform->getOrigin().x();
      marker.pose.position.y = transform->getOrigin().y();
      marker.pose.position.z = transform->getOrigin().z();
      marker.pose.orientation.x = transform->getRotation().x();
      marker.pose.orientation.y = transform->getRotation().y();
      marker.pose.orientation.z = transform->getRotation().z();
      marker.pose.orientation.w = transform->getRotation().w();

      //move above floor
      //marker.pose.position.z += 0.3;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      float vel_t = getTransVel(msg, duration);
      float vel_a = getAngVel(msg, duration);
      //publish vel
      dynamics.header.frame_id = marker.header.frame_id;
      dynamics.header.stamp = marker.header.stamp;
      dynamics.linear.vel = vel_t;
      dynamics.angular.vel = vel_a;

      vel_t>max_vel ? vel_t=vel_t_l : vel_t=vel_t;
      vel_t<-max_vel ? vel_t=vel_t_l : vel_t=vel_t;
      //float scale = 1.82 * vel_t; [> 1 / max_vel (ms) <]
      float scale = (1/max_vel) * vel_t; /* 1 / max_vel (ms) */

      if (scale > 0.05) // little speed means small distance
      {
        marker.scale.x = marker.scale.y = marker.scale.z = scale;
      }
      else if (scale > max_vel)
      {
        marker.scale.x = marker.scale.y = marker.scale.z = max_vel;
      }
      else
      {
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
      }
      // normalization
      marker.scale.x = marker.scale.y = marker.scale.z /= 4;

      // acceleration: a = (v - v_0)/t
      float acc_t = (vel_t - vel_t_l)/duration->toSec();
      float acc_a = (vel_a - vel_a_l)/duration->toSec();
      // calculate acceleration
      if (absolute_values)
      {
        dynamics.linear.acc  = fabs(acc_t);
        dynamics.angular.acc = fabs(acc_a);
      }
      else
      {
        dynamics.linear.acc  = acc_t;
        dynamics.angular.acc = acc_a;
      }

      // calculate jerk and publish it
      // jerk: j = (a - a_0)/t
      float j_t = (acc_t - acc_t_l) / duration->toSec();
      float j_a = (acc_a - acc_a_l) / duration->toSec();
      if (absolute_values)
      {
        dynamics.linear.jerk  = fabs(j_t);
        dynamics.angular.jerk = fabs(j_a);
      }
      else
      {
        dynamics.linear.jerk  = j_t;
        dynamics.angular.jerk = j_a;
      }

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      //marker.color.a = 0.5;
      //
      float norm = 0.4;
      float offset = 0.4;
      if  (acc_t > 0.0) //red
      {
        marker.color.r = offset + fabs(norm * acc_t);
        marker.color.g = marker.color.b = 0.0;
      }
      else if (acc_t<0.0) //green
      {
        marker.color.g = offset + fabs(norm * acc_t);
        marker.color.b = marker.color.r = 0.0;
      }
      else // acc == 0.0
      {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
      }

      marker.lifetime = ros::Duration(10); // Marker lifetime in s

      // Set map pose
      //dynamics.pose.header.timestamp = msg.header.timestamp;
      dynamics.pose.header.stamp = transform->stamp_;
      dynamics.pose.header.frame_id = "/map";
      dynamics.pose.x = transform->getOrigin().x();
      dynamics.pose.y = transform->getOrigin().y();
      double roll, pitch, yaw = 0;
      tf::Matrix3x3(transform->getRotation()).getRPY(roll, pitch, yaw);
      dynamics.pose.alpha = yaw;

      // Publish the dynamics
      base_dynamics_pub->publish(dynamics);
      // Publish the marker
      marker_pub->publish(marker);

      // only print velocity on significant change
      if ( fabs(vel_t - vel_t_l ) > 0.15
          || vel_t >= 0.50 )
      //if ( acc_t > 0.5 ) // m/s^2
      {
        // Text marker
        textmarker.header.frame_id = marker.header.frame_id;
        textmarker.header.stamp = marker.header.stamp;
        textmarker.ns = "velocity";
        textmarker.id = counter++;
        textmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textmarker.action = marker.action;
        textmarker.lifetime = marker.lifetime;
        textmarker.scale.x = textmarker.scale.y = textmarker.scale.z = 0.1;
        // prepare velocity for displaying
        char buff[6]; sprintf(buff, "%4.2f", vel_t);
        textmarker.text = buff;
        textmarker.color.r = 0.0;
        textmarker.color.g = 0.0;
        textmarker.color.b = 1.0;
        textmarker.color.a = marker.color.a;
        textmarker.pose.position.x = marker.pose.position.x;
        textmarker.pose.position.y = marker.pose.position.y;
        textmarker.pose.position.z = marker.pose.position.z + 0.2;

        // Publish the marker
        marker_pub->publish(textmarker);
      }

      // only print acceleration on significant change
      if ( fabs(acc_t) > 1.0)
      {
        // Text marker
        accmarker.header.frame_id = marker.header.frame_id;
        accmarker.header.stamp = marker.header.stamp;
        accmarker.ns = "acceleration";
        accmarker.id = counter++;
        accmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        accmarker.action = marker.action;
        accmarker.lifetime = marker.lifetime;
        accmarker.scale.x = accmarker.scale.y = accmarker.scale.z = 0.1;
        // prepare acc_t for displaying
        char buff[6]; sprintf(buff, "%4.2f", acc_t);
        accmarker.text = buff;
        //accmarker.color.r = marker.color.r;
        //accmarker.color.g = marker.color.g;
        //accmarker.color.b = marker.color.b;
        accmarker.color.r = marker.color.r;
        accmarker.color.g = marker.color.g;
        accmarker.color.b = marker.color.b;
        accmarker.color.a = marker.color.a;
        accmarker.pose.position.x = marker.pose.position.x;
        accmarker.pose.position.y = marker.pose.position.y;
        accmarker.pose.position.z = marker.pose.position.z + 0.3;

        // Publish the marker
        marker_pub->publish(accmarker);
      }

      vel_t_l = vel_t;
      vel_a_l = vel_a;
      acc_t_l = acc_t;
      acc_a_l = acc_a;
    }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "speedmarker");
  ros::NodeHandle n;

  tf::TransformListener listener;

  ros::Publisher marker_pub    = n.advertise<visualization_msgs::Marker>(     ros::this_node::getName()+"/dynamics", 1);
  ros::Publisher dynamics_pub  = n.advertise<race_pr2_markers::BaseDynamics>( ros::this_node::getName()+"/pr2_base_dynamics", 1);
  ros::Publisher distance_pub  = n.advertise<geometry_msgs::Vector3Stamped>(  ros::this_node::getName()+"/pr2_distance_traveled", 1);
  ros::Publisher marker_points = n.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/points", 1);
  ros::Publisher marker_circle = n.advertise<visualization_msgs::Marker>(     ros::this_node::getName()+"/circle", 1);

  SpeedMarker speedmarker(&marker_pub,
      &dynamics_pub,
      &distance_pub,
      &marker_points,
      &marker_circle,
      &listener);
  ros::Subscriber sub = n.subscribe("/robot_pose_ekf/odom_combined", 10, &SpeedMarker::callback, &speedmarker);

  ros::spin();

  return 0;
}
