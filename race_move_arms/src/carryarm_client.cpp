#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <race_move_arms/CarryarmAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_carryarm");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<race_move_arms::CarryarmAction> ac("carryarm", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  race_move_arms::CarryarmGoal goal;

  uint8_t pose;
  uint8_t arm;
  // parse command line arguments
  if (argc == 3)
  {
    pose = atoi(argv[1]);
    arm = atoi(argv[2]);
  }
  else // default values
  {
    pose = 1;
    arm  = 1; // left arm
  }
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

  //exit
  return 0;
}

