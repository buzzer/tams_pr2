#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tray_pick_place/TrayPickPlaceAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_tray_pickup");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<tray_pick_place::TrayPickPlaceAction> ac("tray_pick_place", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  tray_pick_place::TrayPickPlaceGoal goal;

  uint8_t action;
  uint8_t arm;
  uint8_t slot;
  double height = 0.0;
  // parse command line arguments
  if (argc == 4)
  {
    action = atoi(argv[1]);
    arm = atoi(argv[2]);
    slot = atoi(argv[3]); // slot number
  }
  else if (argc == 5)
  {
    action = atoi(argv[1]);
    arm = atoi(argv[2]);
    slot = atoi(argv[3]); // slot number
    height = atof(argv[4]);
	}
  else // default values
  {
    action = tray_pick_place::TrayPickPlaceGoal::PICK_SIDE;
    arm    = tray_pick_place::TrayPickPlaceGoal::LEFT_ARM;
    slot   = tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT;
  }
  ROS_INFO("action: %d, arm: %s, slot: %d", action, arm ? "left" : "right", slot);

  goal.action = action;
  goal.arm    = arm;
  goal.slot = slot;
	goal.bbox_dims.z = height;
	goal.use_place_pose = false;
	goal.place_pose.pose.orientation.x = 0.0; 
	goal.place_pose.pose.orientation.y = 0.0; 
	goal.place_pose.pose.orientation.z = 0.0; 
	goal.place_pose.pose.orientation.w = 1.0; 
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));

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

