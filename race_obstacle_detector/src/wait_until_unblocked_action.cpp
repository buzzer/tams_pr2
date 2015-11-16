#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <race_obstacle_detector/WaitUntilUnblockedAction.h> 
#include <race_msgs/BoundingBox.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>

const float HUMAN_MIN = 1.2;
const float OBS_MIN   = 0.1;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class WaitUntilUnblockedAction
{
  protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<race_obstacle_detector::WaitUntilUnblockedAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    race_obstacle_detector::WaitUntilUnblockedFeedback feedback_;
    race_obstacle_detector::WaitUntilUnblockedResult result_;

    // Point head
    PointHeadClient* point_head_client_;

    // TF
    tf::TransformListener tf_listener;

    // human detection
    // TODO add mutex for activated
    bool activated;
    bool obstacle_detected;
    bool human_detected;

    struct BoundingBox { float x_min, x_max, y_min, y_max, z_min, z_max; } cloud_bb;
    boost::mutex bb_mutex;
    boost::mutex obstacle_mutex;

    ros::Publisher marker_pub;

  public:
    WaitUntilUnblockedAction (std::string name) :
      as_(nh_, name, boost::bind(&WaitUntilUnblockedAction::executeCB, this, _1), false),
      action_name_(name)
    {
      // Point head
      //Initialize the client for the Action interface to the head controller
      point_head_client_ = new
        PointHeadClient("/head_traj_controller/point_head_action", true);

      //wait for head controller action server to come up 
      while(!point_head_client_->waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the point_head_action server to come up");
      }

      // human detection
      activated = false;
      human_detected = false;
      obstacle_detected = false;

      marker_pub = nh_.advertise<visualization_msgs::Marker>("obstacle_boundingbox", 1);

      as_.start();
    }

    ~WaitUntilUnblockedAction(void)
    {
      delete point_head_client_;
    }

    void executeCB(const race_obstacle_detector::WaitUntilUnblockedGoalConstPtr &goal)
    {
      std::string obstacle_class = "unknown"; // means also there is NO obstacle

      // publish info to the console for the user
      ROS_INFO("%s: Executing, with pose %4.2f,%4.2f,%4.2f",
          action_name_.c_str(),
          goal->boundingbox.pose_stamped.pose.position.x,
          goal->boundingbox.pose_stamped.pose.position.y,
          goal->boundingbox.pose_stamped.pose.position.z
          );

      //Point the head
      pointHead(goal->boundingbox, 1.3);

      //add timeout
      ros::Duration duration;
      if ( goal->timeout <= ros::Duration(0.0) )
        duration = ros::Duration(1000000.0);   // wait "forever"
      else
        duration = goal->timeout;

      ros::Time exp_time = ros::Time::now() + duration;
      ROS_INFO("timeout duration: %4.2f, at time: %4.2f", duration.toSec(), exp_time.toSec());
      // Action takes until it is canceled, timeout occured or no obstacle has
      // been detected
      // TODO average obstacle check over time
      while (true)
      {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          return;
        }
        // check if the timeout has occured
        if (exp_time <= ros::Time::now())
        {
          ROS_INFO("Timeout hit at %4.2f", ros::Time::now().toSec());
          ROS_INFO("%s: Aborted", action_name_.c_str());
          as_.setAborted();
          return;
        }

        //==========Obstacle Decisions here===============
        obstacle_mutex.lock();
        feedback_.obstacle = obstacle_detected;
        if (!obstacle_detected)
          obstacle_class = "unknown";
        else if (human_detected)
          obstacle_class  = "human";
        else
          obstacle_class  = "table";
        obstacle_mutex.unlock();
        //================================================
        feedback_.obstacle_type = obstacle_class ;
        feedback_.confidence = 0.0; // dummy value
        feedback_.boundingbox.pose_stamped.header.frame_id="/base_link";
        feedback_.boundingbox.pose_stamped.header.stamp=ros::Time::now();
        //ROS_INFO("before lock");
        //mutex lock cloud_bb
        bb_mutex.lock();
        feedback_.boundingbox.pose_stamped.pose.position.x=(cloud_bb.x_max+cloud_bb.x_min)/2;
        feedback_.boundingbox.pose_stamped.pose.position.y=(cloud_bb.y_max+cloud_bb.y_min)/2;
        feedback_.boundingbox.pose_stamped.pose.position.z=(cloud_bb.z_max+cloud_bb.z_min)/2;
        feedback_.boundingbox.pose_stamped.pose.orientation.x=0;
        feedback_.boundingbox.pose_stamped.pose.orientation.y=0;
        feedback_.boundingbox.pose_stamped.pose.orientation.z=0;
        feedback_.boundingbox.pose_stamped.pose.orientation.w=1;
        feedback_.boundingbox.dimensions.x=cloud_bb.x_max-cloud_bb.x_min;
        feedback_.boundingbox.dimensions.y=cloud_bb.y_max-cloud_bb.y_min;
        feedback_.boundingbox.dimensions.z=cloud_bb.z_max-cloud_bb.z_min;
        bb_mutex.unlock();
        //obstacle = feedback_.obstacle;
        ROS_INFO("%s: Feedback publishing, obstacle: %s", action_name_.c_str(), obstacle_class .c_str());
        // publish feedback here
        as_.publishFeedback(feedback_);

        //finally set action to succeeded
        if (false == obstacle_detected)
        {
          as_.setSucceeded(result_);
          ROS_INFO("%s: Succeeded..", action_name_.c_str());
          return;
        }

        ros::Duration(1.0).sleep();
      }
    }
    bool pointHead (race_msgs::BoundingBox bb, float z=1.0)
    {
      bool success = true;

      //the goal message we will be sending
      pr2_controllers_msgs::PointHeadGoal goal;

      //the point to be looking at is expressed in the "base_link" frame
      geometry_msgs::PointStamped point;
      //point.header.frame_id = bb.pose_stamped.header.frame_id;
      //point.point.x = bb.pose_stamped.pose.position.x;
      //point.point.y = bb.pose_stamped.pose.position.y;
      point.header.frame_id = "/base_link";
      point.point.x = 1.0;
      point.point.y = 0.0;
      point.point.z = z;
      goal.target = point;

      //we want the X axis of the camera frame to be pointing at the target
      goal.pointing_frame = "high_def_frame";
      goal.pointing_axis.x = 1;
      goal.pointing_axis.y = 0;
      goal.pointing_axis.z = 0;

      //take at least 0.5 seconds to get there
      goal.min_duration = ros::Duration(0.5);

      //and go no faster than 1 rad/s
      goal.max_velocity = 1.0;

      //send the goal
      point_head_client_->sendGoal(goal);

      //wait for it to get there (abort after 2 secs to prevent getting stuck)
      point_head_client_->waitForResult(ros::Duration(2));

      return success;
    }

    void getBoundingBox(pcl::PointCloud<pcl::PointXYZ> & input, BoundingBox & cloud_bb)
    {
      if (input.points.size() > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::iterator pc_it = input.points.begin();
        // initialize bounding box with first point
        cloud_bb.x_max = cloud_bb.x_min = pc_it->x;
        cloud_bb.y_max = cloud_bb.y_min = pc_it->y;
        cloud_bb.z_max = pc_it->z;
        // Assumption: obstacle stands on the ground
        cloud_bb.z_min = 0.0;

        for ( ; pc_it < input.points.end(); ++pc_it)
        {
          if(pc_it->x < cloud_bb.x_min)
            cloud_bb.x_min = pc_it->x;
          else if (pc_it->x > cloud_bb.x_max)
            cloud_bb.x_max= pc_it->x;
          if(pc_it->y < cloud_bb.y_min)
            cloud_bb.y_min = pc_it->y;
          else if (pc_it->y > cloud_bb.y_max)
            cloud_bb.y_max= pc_it->y;
          if (pc_it->z > cloud_bb.z_max)
            cloud_bb.z_max= pc_it->z;
        }
      }
      else
      {
        cloud_bb.x_max=cloud_bb.y_max=cloud_bb.z_max=0.0;
        cloud_bb.x_min=cloud_bb.y_min=cloud_bb.z_min=0.0;
      }
    }

    void publishMarker(BoundingBox & cloud_bb)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/base_link";
      marker.header.stamp = ros::Time::now();
      marker.ns = "obstacle_detector";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);

      //Set the pose of the marker.  This is a full 6DOF pose relative to the
      //frame/time specified in the header
      marker.pose.position.x = (cloud_bb.x_max+cloud_bb.x_min)/2;
      marker.pose.position.y = (cloud_bb.y_max+cloud_bb.y_min)/2;
      marker.pose.position.z = (cloud_bb.z_max+cloud_bb.z_min)/2;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here
      //means 1m on a side
      marker.scale.x = cloud_bb.x_max-cloud_bb.x_min;
      marker.scale.y = cloud_bb.y_max-cloud_bb.y_min;
      marker.scale.z = cloud_bb.z_max-cloud_bb.z_min;

      // Set the color -- be sure to set
      //alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;

      // Publish the marker
      marker_pub.publish(marker);

    }

    //only called on obstacle detected
    //Relies on empty point clouds is still being published!
    void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      //if (activated == true)
      //{
      static uint8_t obstacle_state = 0;
      static ros::Time last_time = ros::Time::now();
      ros::Duration duration(3.0);
      ros::Time cur_time = ros::Time::now();

      //ROS_INFO("cur_time %.2f and last_time %.2f and duration %.2f", cur_time.toSec(), last_time.toSec(), duration.toSec());
      if ((cur_time - last_time) > duration)
      {
        ROS_INFO("Resetting obstacle state");
        obstacle_state = 0;
      }

      //ROS_INFO("Got point cloud detection result..");
      pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);

      pcl::fromROSMsg(*msg.get(), *input.get());

      // transform
      pcl_ros::transformPointCloud("/base_link", *input, *output, tf_listener);

      // mutex lock cloud_bb
      bb_mutex.lock();

      // calculate bounding box
      getBoundingBox(*output, cloud_bb);
      float bb_zmax = cloud_bb.z_max;
      if (bb_zmax > 0)
      {
        publishMarker(cloud_bb);
      }

      bb_mutex.unlock();

      if (bb_zmax > HUMAN_MIN)
      {
        human_detected = true;
        obstacle_mutex.lock();
        obstacle_detected = true;
        obstacle_mutex.unlock();
        if (obstacle_state != 1)
        {
          ROS_INFO("Obstacle detected (%.2fm): human", bb_zmax);
          obstacle_state = 1;
        }
      }
      else if (bb_zmax > OBS_MIN) // higher than the ground
      {
        human_detected = false;
        obstacle_mutex.lock();
        obstacle_detected = true;
        obstacle_mutex.unlock();
        if (obstacle_state != 2)
        {
          ROS_INFO("Obstacle detected (%.2fm): static object", bb_zmax);
          obstacle_state = 2;
        }
      }//probably no obstacle
      else
      {
        obstacle_mutex.lock();
        obstacle_detected = false;
        obstacle_mutex.unlock();
      }
      last_time = cur_time;
    }
    //}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wait_until_unblocked_action");

  WaitUntilUnblockedAction wait_until_unblocked_action("wait_until_unblocked_action");
  // subscribe to topic
  ros::NodeHandle n;

  // point cloud detector
  ros::Subscriber sub2 = n.subscribe(
     "/pc_detector/rgbd_out",
     10,
     &WaitUntilUnblockedAction::pcCallback,
     &wait_until_unblocked_action);

  ros::spin();

  return 0;
}
