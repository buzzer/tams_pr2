#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cisstCommon.h" // for matrix and vector calculation
#include "cisstOSAbstraction.h" // 
#include "cisstVector.h" //
#include <iostream>
 
vct3 transformQuarternionToEular(geometry_msgs::Quaternion input)
{
	vct3 output(0.0);
	double x, y, z, w;
  x = input.x; 
  y = input.y;
  z = input.z;
  w = input.w;
  output.X() = atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
  output.Y() = asin(2*(w*y-z*x));
  output.Z() = atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
  return output;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tray_laser_palms_tf_listener");

  ros::NodeHandle node;

//  ros::service::waitForService("spawn");
//  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
//  turtlesim::Spawn srv;
//  add_turtle.call(srv);

  ros::Publisher r_gripper_posture = node.advertise<geometry_msgs::Pose>("tray_r_gripper_posture", 1);
	ros::Publisher l_gripper_posture = node.advertise<geometry_msgs::Pose>("tray_l_gripper_posture", 1);
  tf::TransformListener r_listener, l_listener;
  tf::StampedTransform r_transform, l_transform;
  ros::Rate rate(10.0);
  ros::Duration(1.0).sleep();
  while (node.ok()){
    try{
			ros::Time now = ros::Time::now();
			l_listener.waitForTransform("/base_tray_laser_link", "/l_wrist_roll_link", now, ros::Duration(1.0));
      l_listener.lookupTransform("/base_tray_laser_link", "/l_wrist_roll_link", now, l_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    try{
			ros::Time now = ros::Time::now();
			r_listener.waitForTransform("/base_tray_laser_link", "/r_wrist_roll_link", now, ros::Duration(1.0));
      r_listener.lookupTransform("/base_tray_laser_link", "/r_wrist_roll_link", now, r_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::Pose r_gripper_pose_msg, l_gripper_pose_msg;
    r_gripper_pose_msg.position.x = r_transform.getOrigin().x();
    r_gripper_pose_msg.position.y = r_transform.getOrigin().y();
    r_gripper_pose_msg.position.z = r_transform.getOrigin().z();
		r_gripper_pose_msg.orientation.x = r_transform.getRotation().x();
    r_gripper_pose_msg.orientation.y = r_transform.getRotation().y();
    r_gripper_pose_msg.orientation.z = r_transform.getRotation().z();
		r_gripper_pose_msg.orientation.w = r_transform.getRotation().w();
    l_gripper_pose_msg.position.x = l_transform.getOrigin().x();
    l_gripper_pose_msg.position.y = l_transform.getOrigin().y();
    l_gripper_pose_msg.position.z = l_transform.getOrigin().z();
		l_gripper_pose_msg.orientation.x = l_transform.getRotation().x();
    l_gripper_pose_msg.orientation.y = l_transform.getRotation().y();
    l_gripper_pose_msg.orientation.z = l_transform.getRotation().z();
		l_gripper_pose_msg.orientation.w = l_transform.getRotation().w();
//		ROS_INFO("current position: %f, %f, %f.", r_gripper_pose_msg.position.x, r_gripper_pose_msg.position.y, r_gripper_pose_msg.position.z);
//		ROS_INFO("current orientation: %f, %f, %f, %f.", r_gripper_pose_msg.orientation.x, r_gripper_pose_msg.orientation.y, r_gripper_pose_msg.orientation.z, r_gripper_pose_msg.orientation.w);
//		std::cout << "eular: " << transformQuarternionToEular(r_gripper_pose_msg.orientation) * cmn180_PI << std::endl;
//		ROS_INFO("current orientation: %f, %f, %f, %f.", r_gripper_pose_msg.orientation.x, r_gripper_pose_msg.orientation.y, r_gripper_pose_msg.orientation.z, r_gripper_pose_msg.orientation.w);
//		std::cout << "eular: " << transformQuarternionToEular(r_gripper_pose_msg.orientation) * cmn180_PI << std::endl;

    r_gripper_posture.publish(r_gripper_pose_msg);
    l_gripper_posture.publish(l_gripper_pose_msg);

    rate.sleep();
  }
  return 0;
};
