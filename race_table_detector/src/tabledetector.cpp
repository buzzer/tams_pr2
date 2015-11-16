//2013-05-02 Sebastian Rockel
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

class TableDetector
{
  protected:
    ros::NodeHandle * nh;
    ros::Publisher * marker_pub;
    tf::TransformListener * tf_listener;

    const std::string *OBJECT_DETECTION_SERVICE_NAME;
    const std::string *COLLISION_PROCESSING_SERVICE_NAME;

    //create service and action clients
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;

  public:
    TableDetector (ros::NodeHandle * h,
        ros::Publisher * marker,
        tf::TransformListener * listener,
        const std::string *obj_det_srv,
        const std::string *col_pro_srv)
    {
      nh = h;
      marker_pub = marker;
      tf_listener = listener;

      OBJECT_DETECTION_SERVICE_NAME=obj_det_srv;
      COLLISION_PROCESSING_SERVICE_NAME=col_pro_srv;
    }

    ~TableDetector(void) {}

    void publishMarker(geometry_msgs::PoseStamped * pose, tf::StampedTransform * transform)
    {
      visualization_msgs::Marker marker;
      uint32_t shape = visualization_msgs::Marker::ARROW;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      //marker.header.frame_id = "/map";
      marker.header.frame_id = "/base_link";

      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "table_marker";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      //marker.pose.position.x = transform->getOrigin().x();
      //marker.pose.position.y = transform->getOrigin().y();
      //marker.pose.position.z = transform->getOrigin().z();
      //marker.pose.orientation.x = transform->getRotation().x();
      //marker.pose.orientation.y = transform->getRotation().y();
      //marker.pose.orientation.z = transform->getRotation().z();
      //marker.pose.orientation.w = transform->getRotation().w();
      marker.pose.position.x    = pose->pose.position.x;
      marker.pose.position.y    = pose->pose.position.y;
      marker.pose.position.z    = pose->pose.position.z;
      marker.pose.orientation.x = pose->pose.orientation.x;
      marker.pose.orientation.y = pose->pose.orientation.y;
      marker.pose.orientation.z = pose->pose.orientation.z;
      marker.pose.orientation.w = pose->pose.orientation.w;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      // Publish the marker
      marker_pub->publish(marker);
    }

    void transformFrame  (tf::StampedTransform * transform)
    {
      //tf::StampedTransform transform;
      if (transform != NULL)
      {
        try
        {
          tf_listener->waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(2.0) );
          tf_listener->lookupTransform( "/map", "/base_link", ros::Time(0), *transform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
        }
      }
    }

    int8_t detectTable ()
    {
      //wait for detection client
      while ( !ros::service::waitForService(*OBJECT_DETECTION_SERVICE_NAME,
            ros::Duration(2.0)) && nh->ok() )
      {
        ROS_INFO("Waiting for object detection service to come up");
      }
      if (!nh->ok()) exit(0);
      object_detection_srv =
        nh->serviceClient<tabletop_object_detector::TabletopDetection>
        (*OBJECT_DETECTION_SERVICE_NAME, true);

      ////wait for collision map processing client
      //while ( !ros::service::waitForService(*COLLISION_PROCESSING_SERVICE_NAME,
            //ros::Duration(2.0)) && nh->ok() )
      //{
        //ROS_INFO("Waiting for collision processing service to come up");
      //}
      //if (!nh->ok()) exit(0);
      //collision_processing_srv =
        //nh->serviceClient
        //<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
        //(*COLLISION_PROCESSING_SERVICE_NAME, true);

      //call the tabletop detection
      //ROS_INFO("Calling tabletop detector");
      tabletop_object_detector::TabletopDetection detection_call;
      //we want recognized database objects returned
      //set this to false if you are using the pipeline without the database
      detection_call.request.return_clusters = true;
      //we want the individual object point clouds returned as well
      detection_call.request.return_models = true;
      detection_call.request.num_models = 1;
      if (!object_detection_srv.call(detection_call))
      {
        ROS_ERROR("Tabletop detection service failed");
        return -1;
      }
      if (detection_call.response.detection.result !=
          detection_call.response.detection.SUCCESS)
      {
        ROS_INFO("No table found!");
        //ROS_ERROR("Tabletop detection returned error code %d",
        //detection_call.response.detection.result);
        return -1;
      }
      else
      {
        //ROS_INFO("Yeah, Detected a table");
        ROS_INFO("Table(x,x,y,y): %f,%f,%f,%f",
            detection_call.response.detection.table.x_min,
            detection_call.response.detection.table.x_max,
            detection_call.response.detection.table.y_min,
            detection_call.response.detection.table.y_max
            );

        tf::StampedTransform transform;
        //transformFrame(&transform);
        geometry_msgs::PoseStamped pose;
        //pose.pose.position.x    = (detection_call.response.detection.table.x_min + detection_call.response.detection.table.x_max) / 2;
        pose.pose.position.x    = detection_call.response.detection.table.x_min;
        pose.pose.position.y    = (detection_call.response.detection.table.y_min + detection_call.response.detection.table.y_max) / 2;
        pose.pose.position.z    = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        //publishMarker(&detection_call.response.detection.table.pose, &transform);
        publishMarker(&pose, &transform);
      }

      //if (detection_call.response.detection.clusters.empty() && 
          //detection_call.response.detection.models.empty() )
      //{
        //ROS_ERROR("The tabletop detector detected the table, "
            //"but found no objects");
        //return -1;
      //}

      return 1;
    }
};

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "race_tabledetector");
  ros::NodeHandle nh;

  tf::TransformListener listener;

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("race_table_marker", 1);

  const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
  const std::string COLLISION_PROCESSING_SERVICE_NAME =
    "/tabletop_collision_map_processing/tabletop_collision_map_processing";

  TableDetector tabledetector(&nh, &marker_pub, &listener, &OBJECT_DETECTION_SERVICE_NAME,
      &COLLISION_PROCESSING_SERVICE_NAME);

  ros::Rate r(0.5);
  while (ros::ok())
  {
    tabledetector.detectTable();
    r.sleep();
  }

  ros::spin();

  return 0;
}
