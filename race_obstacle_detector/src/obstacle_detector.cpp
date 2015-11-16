#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <race_obstacle_detector/ObstacleDetectionAction.h> 
#include <race_msgs/BoundingBox.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>

const float HUMAN_MIN = 1.2;
const float OBS_MIN   = 0.1;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class DetectionAction
{
  protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<race_obstacle_detector::ObstacleDetectionAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    race_obstacle_detector::ObstacleDetectionFeedback feedback_;
    race_obstacle_detector::ObstacleDetectionResult result_;

    const std::string *OBJECT_DETECTION_SERVICE_NAME;
    const std::string *COLLISION_PROCESSING_SERVICE_NAME;

    //create service and action clients
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;

    // Point head
    PointHeadClient* point_head_client_;

    // TF
    tf::TransformListener tf_listener;

    // human detection
    bool activated;
    bool obstacle_detected;
    bool human_detected;

    struct BoundingBox { float x_min, x_max, y_min, y_max, z_min, z_max; } cloud_bb;
    boost::mutex bb_mutex;

    ros::Publisher marker_pub;

  public:

    DetectionAction(std::string name,
        const std::string *obj_det_srv,
        const std::string *col_pro_srv) :
      as_(nh_, name, boost::bind(&DetectionAction::executeCB, this, _1), false),
      action_name_(name)
    {
      // object detection
      OBJECT_DETECTION_SERVICE_NAME=obj_det_srv;
      COLLISION_PROCESSING_SERVICE_NAME=col_pro_srv;

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

    ~DetectionAction(void)
    {
      delete point_head_client_;
    }

    void executeCB(const race_obstacle_detector::ObstacleDetectionGoalConstPtr &goal)
    {
      // helper variables
      //ros::Rate r(1);
      bool success = true;
      std::string obstacle = "unknown"; // means also there is NO obstacle

      // publish info to the console for the user
      ROS_INFO("%s: Executing, with pose %4.2f,%4.2f,%4.2f",
          action_name_.c_str(),
          goal->boundingbox.pose_stamped.pose.position.x,
          goal->boundingbox.pose_stamped.pose.position.y,
          goal->boundingbox.pose_stamped.pose.position.z
          );
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }
      else
      {
        //==========Obstacle Decisions here===============
        //bool is_obstacle = false;/*{{{*/
        //TODO add struct
        // do a basic obstacle detection, e.g. in the base scan to verify that
        // there is an obstacle
        //is_obstacle &= detectObstacle(goal->boundingbox);
        // move the head to point
        //success &= pointHead(goal->boundingbox, 1.5);
        // call the face recognition
        //if ( ! detectHuman(goal->boundingbox) )
        //{/*}}}*/
          success &= pointHead(goal->boundingbox, 1.0);
          // call the table detector action
          //if (detectTable(goal->boundingbox) )
          if (obstacle_detected)
          {
            obstacle_detected = false;
            result_.obstacle = true;
            if (human_detected)
            {
              human_detected = false;
              obstacle = "human";
            }
            else
            {
              obstacle = "table";
            }
          }
          else
          {
            result_.obstacle = false;
          }
        //================================================
      }
      // set the result
      if(success)
      {
        //write a 'Blocking' fluent on the blackboard
        //addFluent(obstacle, goal->boundingbox);
        //but only when an obstacle is found
        result_.obstacle_type = obstacle;
        result_.confidence = 91.0;
        result_.boundingbox.pose_stamped.header.frame_id="/base_link";
        result_.boundingbox.pose_stamped.header.stamp=ros::Time::now();
        //mutex lock cloud_bb
        bb_mutex.lock();
        result_.boundingbox.pose_stamped.pose.position.x=(cloud_bb.x_max+cloud_bb.x_min)/2;
        result_.boundingbox.pose_stamped.pose.position.y=(cloud_bb.y_max+cloud_bb.y_min)/2;
        result_.boundingbox.pose_stamped.pose.position.z=(cloud_bb.z_max+cloud_bb.z_min)/2;
        result_.boundingbox.pose_stamped.pose.orientation.x=0;
        result_.boundingbox.pose_stamped.pose.orientation.y=0;
        result_.boundingbox.pose_stamped.pose.orientation.z=0;
        result_.boundingbox.pose_stamped.pose.orientation.w=1;
        result_.boundingbox.dimensions.x=cloud_bb.x_max-cloud_bb.x_min;
        result_.boundingbox.dimensions.y=cloud_bb.y_max-cloud_bb.y_min;
        result_.boundingbox.dimensions.z=cloud_bb.z_max-cloud_bb.z_min;
        bb_mutex.unlock();
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
      else
      {
        result_.obstacle_type = "failure";
        result_.confidence = 0.0;
        ROS_INFO("%s: Failed", action_name_.c_str());
        // set the action state to failed
        //as_.setSucceeded(result_);
        as_.setAborted(result_);
      }
    }
    bool addFluent (std::string & obstacle, race_msgs::BoundingBox bb)
    {
      bool success = true;

      return success;
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
    void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      static uint8_t obstacle_state = 0;
      static ros::Time last_time = ros::Time::now();
      ros::Duration duration(3.0);
      ros::Time cur_time = ros::Time::now();

      //if (activated == true)
      //{
      //TODO never occurs
      //ROS_INFO("cur_time %.2f and last_time %.2f and duration %.2f", cur_time.toSec(), last_time.toSec(), duration.toSec());
      if ((cur_time - last_time) > duration)
      {
        ROS_INFO("Resetting obstacle state");
        obstacle_state = 0;
      }

      //TODO add mutex to all occurences
      obstacle_detected = true;
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
        if (obstacle_state != 1)
        {
          ROS_INFO("Obstacle detected (%.2fm): human", bb_zmax);
          obstacle_state = 1;
        }
      }
      else if (bb_zmax > OBS_MIN) // higher than the ground
      {
        human_detected = false;
        if (obstacle_state != 2)
        {
          ROS_INFO("Obstacle detected (%.2fm): static object", bb_zmax);
          obstacle_state = 2;
        }
      }//probably no obstacle
      //else if (obstacle_state != 0)
      //{
      //  ROS_INFO("Obstacle detected (%.2fm): sensor noise", bb_zmax);
      //  obstacle_state = 0;
      //}
      last_time = cur_time;

      //activated = false;
      //}
    }
    //void humanCallback(const cob_people_detection_msgs::DetectionArray::ConstPtr& msg)/*{{{*/
    //{
    //  if (activated == true)
    //  {
    //    ROS_INFO("Got human detection result..");
    //    if (msg->detections.size() > 0)
    //    {
    //      ROS_INFO("Detected %d potential humans", (int)msg->detections.size());
    //      // only set it to true here, i.e. if at least one msg is positive the
    //      // result is positive
    //      human_detected = true;
    //      activated = false;
    //    }
    //  }
    //}

    //bool detectHuman (race_msgs::BoundingBox bb)
    //{
    //  bool success = false;
    //  // if for 5 seconds at least 2 msgs indicate a face report success
    //  //activate processing
    //  activated = true;
    //  ROS_INFO("Activated human detection");
    //  int timer = 5;

    //  while (timer > 0 && activated == true)
    //  {
    //    --timer;
    //    ros::Duration(1.0).sleep(); // sleep for 1 second
    //  }

    //  activated = false;
    //  ROS_INFO("Deactivated human detection");

    //  success = human_detected;

    //  return success;
    //}
    //bool detectTable (race_msgs::BoundingBox bb)
    //{
    //  bool success = false;

    //  //wait for detection client
    //  while ( !ros::service::waitForService(*OBJECT_DETECTION_SERVICE_NAME,
    //        ros::Duration(2.0)) && nh_.ok() )
    //  {
    //    ROS_INFO("Waiting for object detection service to come up");
    //  }
    //  if (!nh_.ok()) exit(0);
    //  object_detection_srv =
    //    nh_.serviceClient<tabletop_object_detector::TabletopDetection>
    //    (*OBJECT_DETECTION_SERVICE_NAME, true);

    //  //call the tabletop detection
    //  //ROS_INFO("Calling tabletop detector");
    //  tabletop_object_detector::TabletopDetection detection_call;
    //  //we want recognized database objects returned
    //  //set this to false if you are using the pipeline without the database
    //  detection_call.request.return_clusters = true;
    //  //we want the individual object point clouds returned as well
    //  detection_call.request.return_models = true;
    //  detection_call.request.num_models = 1;
    //  if (!object_detection_srv.call(detection_call))
    //  {
    //    ROS_ERROR("Tabletop detection service failed");
    //    //return -1;
    //    return false;
    //  }
    //  if (detection_call.response.detection.result !=
    //      detection_call.response.detection.SUCCESS)
    //  {
    //    ROS_INFO("No table found!");
    //    //ROS_ERROR("Tabletop detection returned error code %d",
    //    //detection_call.response.detection.result);
    //    //return -1;
    //    return false;
    //  }
    //  else
    //  {
    //    //ROS_INFO("Yeah, Detected a table");
    //    ROS_INFO("Table(x,x,y,y): %f,%f,%f,%f at: %4.2f,%4.2f,%4.2f, %s",
    //        detection_call.response.detection.table.x_min,
    //        detection_call.response.detection.table.x_max,
    //        detection_call.response.detection.table.y_min,
    //        detection_call.response.detection.table.y_max,
    //        detection_call.response.detection.table.pose.pose.position.x,
    //        detection_call.response.detection.table.pose.pose.position.y,
    //        detection_call.response.detection.table.pose.pose.position.z,
    //        detection_call.response.detection.table.pose.header.frame_id.c_str()
    //        );

    //    success = true;

    //    //transfrom coordinates into local frame (from map frame)[>{{{<]
    //    //tf::StampedTransform transform;
    //    //try
    //    //{
    //    //  tf_listener.lookupTransform(
    //    //      detection_call.response.detection.table.pose.header.frame_id,
    //    //      bb.pose_stamped.header.frame_id,
    //    //      ros::Time(0),
    //    //      transform
    //    //      );
    //    //}
    //    //catch(tf::TransformException ex)
    //    //{
    //    //  ROS_ERROR("%s", ex.what());
    //    //}
    //    //tf::Transform obstacle;
    //    //tf::Transform boundingbox;
    //    //obstacle.setOrigin(tf::Vector3(
    //    //      detection_call.response.detection.table.pose.pose.position.x,
    //    //      detection_call.response.detection.table.pose.pose.position.y,
    //    //      detection_call.response.detection.table.pose.pose.position.z
    //    //      ));
    //    //obstacle.setRotation(tf::Quaternion(0,0,0,1));
    //    //boundingbox.setOrigin(tf::Vector3(
    //    //      bb.pose_stamped.pose.position.x,
    //    //      bb.pose_stamped.pose.position.y,
    //    //      bb.pose_stamped.pose.position.z
    //    //      ));
    //    //obstacle.setRotation(tf::Quaternion(0,0,0,1));

    //    ////transformation
    //    //boundingbox = boundingbox * transform;
    //    //bounding box is now in local frame, e.g. table frame
    //    //TODO check if overlapping

    //    //float bb_local_x = bb.pose_stamped.pose.position.x + transform.getOrigin().x();
    //    //float bb_local_y = bb.pose_stamped.pose.position.y + transform.getOrigin().y();
    //    ////float bb_local_z = bb.pose_stamped.pose.position.z + transform.getOrigin().z();
    //    //float bb_dim_x   = bb.dimensions.x;
    //    //float bb_dim_y   = bb.dimensions.y;
    //    ////float bb_dim_z   = bb.dimensions.z;
    //    ////TODO transform to Quaternion
    //    ////TODO add as Transform
    //    ////TODO check if inside boundingbox (spatial reasoning)
    //    //// assumption obstacle bb is smaller than bb
    //    //if (detection_call.response.detection.table.x_min < (bb_local_x + bb_dim_x/2) &&
    //    //    detection_call.response.detection.table.x_min > (bb_local_x - bb_dim_x/2) )
    //    //{
    //    //  if (detection_call.response.detection.table.y_min < (bb_local_y + bb_dim_y/2) &&
    //    //      detection_call.response.detection.table.y_min > (bb_local_y - bb_dim_y/2) )
    //    //  {
    //    //    // lower left table 
    //    //    success = true;
    //    //  }
    //    //}
    //    //else if (detection_call.response.detection.table.x_max < (bb_local_x + bb_dim_x/2) &&
    //    //    detection_call.response.detection.table.x_max > (bb_local_x - bb_dim_x/2) )
    //    //{
    //    //  if (detection_call.response.detection.table.y_max < (bb_local_y + bb_dim_y/2) &&
    //    //      detection_call.response.detection.table.y_max > (bb_local_y - bb_dim_y/2) )
    //    //  {
    //    //    // lower left table 
    //    //    success = true;
    //    //  }
    //    //}[>}}}<]
    //  }
    //  return success;
    //}
    //bool detectObstacle (race_msgs::BoundingBox bb)
    //{
    //  bool success = false;

    //  return success;
    //}/*}}}*/
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_detection");

  const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
  const std::string COLLISION_PROCESSING_SERVICE_NAME =
    "/tabletop_collision_map_processing/tabletop_collision_map_processing";

  DetectionAction obstacle_detector("obstacle_detection",
     &OBJECT_DETECTION_SERVICE_NAME,
     &COLLISION_PROCESSING_SERVICE_NAME
     );
  // subscribe to topic
  ros::NodeHandle n;

  // face detector/*{{{*/
  //ros::Subscriber sub = n.subscribe(
  //    "/cob_people_detection/face_recognizer/face_recognitions",
  //    1000,
  //    &DetectionAction::humanCallback,
  //    &obstacle_detector);/*}}}*/

  // point cloud detector
  ros::Subscriber sub2 = n.subscribe(
     "/pc_detector/rgbd_out",
     10,
     &DetectionAction::pcCallback,
     &obstacle_detector);

  ros::spin();

  return 0;
}

