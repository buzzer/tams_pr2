/*
 * 2015-02-06 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 * Moves the robot's arms to a pre-pick up pose with arms to side posture.
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <carryarm_actionlib/CarryarmAction.h>

int main (int argc, char **argv)
{
  int32_t pose = 4; // side pose
  int32_t arm = 2;  // both arms

  ros::init(argc, argv, "move_arms_to_side");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<carryarm_actionlib::CarryarmAction> ac("carryarm", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(ros::Duration(200.0));

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  carryarm_actionlib::CarryarmGoal goal;


  ROS_INFO("carrypose: %d, carryarm: %s", pose, arm ? "left" : "right");
  goal.carrypose = pose;
  goal.carryarm  = arm;

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(200.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}

