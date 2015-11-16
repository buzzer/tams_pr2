#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/tf.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <algorithm>    // std::random_shuffle
#include <vector>       // std::vector
#include <cstdlib>      // std::rand, std::srand

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool initGoals(std::vector<move_base_msgs::MoveBaseGoal> & goals)
{
  move_base_msgs::MoveBaseGoal goal;
  tf::Quaternion rot;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  //home goal
  goal.target_pose.pose.position.x = 13.0;
  goal.target_pose.pose.position.y = 11.3;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 1.0;
  goal.target_pose.pose.orientation.w = 0.0;

  goals.push_back(goal);

  //race:posepreManipulationAreaNorthTable1
  goal.target_pose.pose.position.x = 7.81;
  goal.target_pose.pose.position.y = 12.79;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, -1.57);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaSouthTable1
  goal.target_pose.pose.position.x = 7.68;
  goal.target_pose.pose.position.y = 9.75;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 1.57);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaWestTable2
  //goal.target_pose.pose.position.x = 8.68;
  goal.target_pose.pose.position.x = 9.0;
  goal.target_pose.pose.position.y = 11.49;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 0);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaEastTable2
  goal.target_pose.pose.position.x = 11.78;
  goal.target_pose.pose.position.y = 11.49;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 3.14);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaEastCounter1
  goal.target_pose.pose.position.x = 6.8;
  goal.target_pose.pose.position.y = 10.1;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 3.14);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  return true;
}

bool initGoalsLinear(std::vector<move_base_msgs::MoveBaseGoal> & goals)
{
  move_base_msgs::MoveBaseGoal goal;
  tf::Quaternion rot;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  //home goal
  goal.target_pose.pose.position.x = 13.0;
  goal.target_pose.pose.position.y = 11.3;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 1.0;
  goal.target_pose.pose.orientation.w = 0.0;

  goals.push_back(goal);

  //race:posepreManipulationAreaNorthTable1
  goal.target_pose.pose.position.x = 7.81;
  goal.target_pose.pose.position.y = 12.79;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, -1.57);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaSouthTable1
  goal.target_pose.pose.position.x = 7.68;
  goal.target_pose.pose.position.y = 9.75;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 1.57);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaWestTable2
  //goal.target_pose.pose.position.x = 8.68;
  goal.target_pose.pose.position.x = 9.0;
  goal.target_pose.pose.position.y = 11.49;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 0);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaEastTable2
  goal.target_pose.pose.position.x = 11.78;
  goal.target_pose.pose.position.y = 11.49;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 3.14);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  //posepreManipulationAreaEastCounter1
  goal.target_pose.pose.position.x = 6.8;
  goal.target_pose.pose.position.y = 10.1;
  goal.target_pose.pose.position.z = 0.0;
  rot.setRPY(0, 0, 3.14);
  goal.target_pose.pose.orientation.x = rot.getX();
  goal.target_pose.pose.orientation.y = rot.getY();
  goal.target_pose.pose.orientation.z = rot.getZ();
  goal.target_pose.pose.orientation.w = rot.getW();

  goals.push_back(goal);

  return true;
}

int myrandom (int i)
{
  return std::rand()%i;
}

bool driveTo( MoveBaseClient & ac,
    std::vector<move_base_msgs::MoveBaseGoal> & goals)
{
  for (auto & goal : goals)
  {
    ROS_INFO("Sending goal..");
    ac.sendGoal(goal);

    ROS_INFO("Waiting for result..");
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved to the goal");
    else
      ROS_INFO("The base failed to move to goal. State is %s", ac.getState().toString().c_str());
  }
  return true;
}

bool initVel(std::vector<float> & vel_trans)
{
  float vel = 0.15;
  //vel_trans.push_back(vel);

  vel += 0.1;
  vel_trans.push_back(vel);

  vel += 0.1;
  vel_trans.push_back(vel);

  vel += 0.1;
  vel_trans.push_back(vel);

  vel += 0.1;
  vel_trans.push_back(vel);

  return true;
}

bool setVel(float & vel)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  double_param.name = "max_trans_vel";
  double_param.value = vel;
  conf.doubles.push_back(double_param);

  double_param.name = "max_rot_vel";
  double_param.value = 0.5;
  conf.doubles.push_back(double_param);

  srv_req.config = conf;

  ROS_INFO("Setting to velocity (trans): %4.2f", vel);
  ros::service::call("/move_base_node/DWAPlannerROS/set_parameters", srv_req, srv_resp);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_navigation_goals");
  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(10.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::vector<move_base_msgs::MoveBaseGoal> goals;
  std::vector<float> vel_trans;

  if (argc <= 1)
  {
    initGoals(goals);

    while (ros::ok())
    {
      std::random_shuffle (goals.begin(), goals.end(), myrandom);
      driveTo(ac, goals);
    }
  }
  else
  {
    ROS_INFO("Mode: linear goals, changing velocity");
    initGoalsLinear(goals);
    initVel(vel_trans);

    while (ros::ok())
    {
      for(auto & vel : vel_trans)
      {
        setVel(vel);
        driveTo(ac, goals);
      }
    }
  }

  return 0;
}
