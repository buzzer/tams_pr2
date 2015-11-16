#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <race_move_arms/MovearmAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_movearm");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<race_move_arms::MovearmAction> ac("race_movearm", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  race_move_arms::MovearmGoal goal;

  const uint8_t armposturecount = 4;
  uint8_t armpostures [armposturecount];
  armpostures[0] = race_move_arms::MovearmPosture::ARMTOSIDEPOSTURE;
  armpostures[1] = race_move_arms::MovearmPosture::ARMTUCKEDPOSTURE;
  armpostures[2] = race_move_arms::MovearmPosture::ARMUNTUCKEDPOSTURE;
  armpostures[3] = race_move_arms::MovearmPosture::ARMCARRYPOSTURE;

  uint8_t leftarm;
  uint8_t rightarm;
  // parse command line arguments
  if (argc == 3)
  {
    leftarm = atoi(argv[1]);
    rightarm = atoi(argv[2]);

    assert(leftarm < armposturecount);
    assert(rightarm < armposturecount);
    //assert(leftarm >= 0);
    //assert(rightarm >= 0);
    ROS_INFO("sending leftarm: %d, rightarm: %d", armpostures[leftarm], armpostures[rightarm]);
    goal.leftarm.armposture  = armpostures[leftarm];
    goal.rightarm.armposture = armpostures[rightarm];

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(200.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if (success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
    else
    {
      ac.cancelGoal();
      ROS_INFO("Action did not finish before the time out.");
    }

  }
  else // default values
  {
    ROS_INFO("Sending %d goals", armposturecount * armposturecount);
    for (uint32_t i=0; i < armposturecount; i++)
    {
      for (uint32_t j=0; j < armposturecount; j++)
      {
        goal.leftarm.armposture  = armpostures[i];
        goal.rightarm.armposture = armpostures[j];
        ROS_INFO("sending leftarm: %d, rightarm: %d", armpostures[i], armpostures[j]);

        ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(200.0));

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = ac.getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
        {
          ROS_INFO("Action did not finish before the time out.");
        }
        //ros::Duration(0.5).sleep();
      }
    }
  }
  //exit
  return 0;
}

