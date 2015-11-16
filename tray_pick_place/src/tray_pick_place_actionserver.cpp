#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tray_pick_place/TrayPickPlaceAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <string.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h> // to controll gripper
#include "cisstCommon.h" // for matrix and vector calculation
#include "cisstOSAbstraction.h" //
#include "cisstVector.h" //
#include "race_tray_monitor/race_tray_object_midpoint.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/QuaternionStamped.h"
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace tf;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

vct3 g_pos1(0.0, 0.0, 0.07), g_pos2(0.0, 0.0, 0.07), g_pos3(0.0, 0.0, 0.07); // position of the three objects
// locate to place the objects
vct3 g_place_pos1(0.17, -0.13, 0.08), g_place_pos2(0.17, 0.0, 0.08), g_place_pos3(0.17, 0.13, 0.08);

//vct3 g_l_gripper_eular(0.0, 0.0, 0.0), g_r_gripper_eular(0.0, 0.0, 0.0);

double g_l_elbow_flex_joint_effort = 0.0, g_l_shoulder_pan_joint_effort = 0.0;

double g_r_elbow_flex_joint_effort = 0.0, g_r_shoulder_pan_joint_effort = 0.0;

geometry_msgs::Pose g_r_stamped_pose, g_l_stamped_pose; //for transform the frame between /map and /base_tray_laser_link

class Torso{
private:
  TorsoClient *torso_client_;

public:
	
	bool isUp;
	
  //Action client initialization
  Torso()
  {

    torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

    //wait for the action server to come up
    while(!torso_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the torso action server to come up");
    }
    
    isUp = false;
  }

  ~Torso()
  {
    delete torso_client_;
  }

  //tell the torso to go up
  void up()
  {

    pr2_controllers_msgs::SingleJointPositionGoal up;
    up.position = 0.295;  //all the way up is 0.2
    up.min_duration = ros::Duration(2.0);
    up.max_velocity = 1.0;

    ROS_INFO("Sending up goal %f.", up.position);
    torso_client_->sendGoal(up);
    torso_client_->waitForResult();
    
    isUp = true;
  }

  //tell the torso to go down
  void down()
  {

    pr2_controllers_msgs::SingleJointPositionGoal down;
    down.position = 0.18;
    down.min_duration = ros::Duration(2.0);
    down.max_velocity = 1.0;

    ROS_INFO("Sending down goal %f.", down.position);
    torso_client_->sendGoal(down);
    torso_client_->waitForResult();
    
    isUp = false;
  }
};

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


bool isPosePossible(int num)
{
	bool is_possible = true;
//	std::cout << is_possible << std::endl;
	switch(num)
	{
		case tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT:
		if ((0.5 < g_pos1.X()) || (g_pos1.X() < 0.07))
		{
			is_possible = false;
//			std::cout << "position of object 1: " << g_pos1 << std::endl;
		}
		break;
		case tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT:
//		std::cout << "g_pose2: " << g_pos2 << std::endl;
		if ((0.5 < g_pos2.X()) || (g_pos2.X() < 0.07))
		{
			is_possible = false;
//			std::cout << "position of object 2: " << g_pos2 << std::endl;
		}
		break;
		case tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT:
		if ((0.5 < g_pos3.X()) || (g_pos3.X() < 0.07))
		{
			is_possible = false;
//			std::cout << "position of object 3: " << g_pos3 << std::endl;
		}
		break;
		default:break;
	}

//	std::cout << is_possible << std::endl;
	return is_possible;
}
////////////////////////// open gripper <jinn> ///////////////////////////////////
class RGripper{
private:
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> *gripper_client_;

public:
  //Action client initialization
  RGripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("r_gripper_controller/gripper_action", true);

    //wait for the gripper action server to come up
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up...");
    }
    ROS_INFO("Successfully connected to the r_gripper_controller/gripper_action action server.");
  }

  ~RGripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.12;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult(ros::Duration(3.0));
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
//    else
//      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  bool close(){
		bool success = true;
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult(ros::Duration(3.0));
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {ROS_INFO("The gripper closed!"); success = true;}
//    else
//      {ROS_INFO("The gripper failed to close."); success = false;}
    return success;
  }
};

class LGripper{
private:
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> *gripper_client_;

public:
  //Action client initialization
  LGripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("l_gripper_controller/gripper_action", true);

    //wait for the gripper action server to come up
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
    }
    ROS_INFO("Successfully connected to the r_gripper_controller/gripper_action action server.");
  }

  ~LGripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.12;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult(ros::Duration(3.0));
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
 //   else
//      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  bool close(){
		bool success = true;
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult(ros::Duration(3.0));
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {ROS_INFO("The gripper closed!"); success = true;}
 //   else
 //     {ROS_INFO("The gripper failed to close."); success =  false;}
    return success;
  }
};
//////////////////////////////////////////////////////////////////////////////////

class TrayPickPlaceAction
{
  protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<tray_pick_place::TrayPickPlaceAction> as_;
    std::string action_name_;
    // create messages that are used to published result
    tray_pick_place::TrayPickPlaceResult result_;
    //offset of the object position
    float rightObjectX, rightObjectY, leftObjectX, leftObjectY, middleObjectX, middleObjectY;
    float objectHeight1, objectHeight2, objectHeight3, gripper_offset;
    vct3 m_slot1_offset, m_slot2_offset, m_slot3_offset;
    double side_place_y_angle;
		vct3 current_eular, pre_place_eular;
		tf::TransformListener listener;

    RGripper rgripper;
    LGripper lgripper;
		Torso torso;

  public:

    TrayPickPlaceAction(std::string name, vct3 offsets):
      as_(nh_, name, boost::bind(&TrayPickPlaceAction::executeCB, this, _1), false),
      action_name_(name)
    {
    //offset of the object position
			objectHeight1 = 0.1; objectHeight2 = 0.1; objectHeight3 = 0.1; gripper_offset = 0.2;
			// for simulation, use these parameters: 
			rightObjectX = offsets.X(); leftObjectX = offsets.Y(); middleObjectX = offsets.Z();
			//ROS_INFO("Using parameters for tray_pick_place SIMULATION.");
			//rightObjectX = 0.01; leftObjectX = 0.01; middleObjectX = 0.02;
			// for pr2, use these parameters: 
			//ROS_INFO("Using parameters for tray_pick_place ON REAL PR2.");
			//rightObjectX = -0.015; leftObjectX = -0.015; middleObjectX = 0.005;

			
			rightObjectY = -0.26; leftObjectY = 0.26; middleObjectY = 0.016;
						
			side_place_y_angle = 0.0;
			m_slot1_offset.SetAll(0.0); m_slot2_offset.SetAll(0.0); m_slot3_offset.SetAll(0.0);
			current_eular.SetAll(0.0); pre_place_eular.SetAll(0.0);
			as_.start();
    }

    ~TrayPickPlaceAction(void) { }

    bool initBothArmGoals( arm_navigation_msgs::MoveArmGoal * leftBoth,
                           arm_navigation_msgs::MoveArmGoal * rightBoth)
    {
      //leftBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(leftBoth != NULL);

      //TODO make dynamic
      leftBoth->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      leftBoth->motion_plan_request.goal_constraints.joint_constraints[0].position =  0.145;
      leftBoth->motion_plan_request.goal_constraints.joint_constraints[1].position =  1.265;
      leftBoth->motion_plan_request.goal_constraints.joint_constraints[2].position =  1.836;
      leftBoth->motion_plan_request.goal_constraints.joint_constraints[3].position = -1.510;
      leftBoth->motion_plan_request.goal_constraints.joint_constraints[4].position =  2.812;
      leftBoth->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.415;
      leftBoth->motion_plan_request.goal_constraints.joint_constraints[6].position = -1.585;

      //rightBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(rightBoth != NULL);

      //TODO make dynamic
      rightBoth->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      rightBoth->motion_plan_request.goal_constraints.joint_constraints[0].position = -0.08;
      rightBoth->motion_plan_request.goal_constraints.joint_constraints[1].position =  1.05;
      rightBoth->motion_plan_request.goal_constraints.joint_constraints[2].position = -1.45;
      rightBoth->motion_plan_request.goal_constraints.joint_constraints[3].position = -1.43;
      rightBoth->motion_plan_request.goal_constraints.joint_constraints[4].position = -2.55;
      rightBoth->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.52;
      rightBoth->motion_plan_request.goal_constraints.joint_constraints[6].position =  2.70;

      return true;
    }

    bool initRightArmGoalSide( arm_navigation_msgs::MoveArmGoal * rightSide )
    {
      //rightBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(rightSide != NULL);

      //TODO make dynamic
      rightSide->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      rightSide->motion_plan_request.goal_constraints.joint_constraints[0].position = -2.115;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[1].position = -0.02;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[2].position = -1.64;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[3].position = -2.07;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[4].position =  0.0;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[5].position = -0.5;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[6].position =  1.57;

      return true;
    }

    bool initLeftArmGoalSide( arm_navigation_msgs::MoveArmGoal * leftSide )
    {
      //leftBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(leftSide != NULL);

      //TODO make dynamic
      leftSide->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      leftSide->motion_plan_request.goal_constraints.joint_constraints[0].position =  2.115;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[1].position = -0.02;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[2].position =  1.64;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[3].position = -2.07;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[4].position =  0.0;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[5].position = -0.5;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[6].position =  -1.57;

      return true;
    }

    bool initRightArmGoalAbove( arm_navigation_msgs::MoveArmGoal * rightGoal )
    {
      //rightBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(rightGoal != NULL);

      //TODO make dynamic
      rightGoal->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      rightGoal->motion_plan_request.goal_constraints.joint_constraints[0].position = -0.62;
      rightGoal->motion_plan_request.goal_constraints.joint_constraints[1].position = -0.04;
      rightGoal->motion_plan_request.goal_constraints.joint_constraints[2].position = -1.51;
      rightGoal->motion_plan_request.goal_constraints.joint_constraints[3].position = -1.63;
      rightGoal->motion_plan_request.goal_constraints.joint_constraints[4].position = -1.61;
      rightGoal->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.63;
      rightGoal->motion_plan_request.goal_constraints.joint_constraints[6].position =  0.57;

      return true;
    }

    bool initLeftArmGoalAbove( arm_navigation_msgs::MoveArmGoal * leftGoal )
    {
      //leftBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(leftGoal != NULL);

      //TODO make dynamic
      leftGoal->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      leftGoal->motion_plan_request.goal_constraints.joint_constraints[0].position =  0.62;
      leftGoal->motion_plan_request.goal_constraints.joint_constraints[1].position =  0.04;
      leftGoal->motion_plan_request.goal_constraints.joint_constraints[2].position =  1.51;
      leftGoal->motion_plan_request.goal_constraints.joint_constraints[3].position = -1.63;
      leftGoal->motion_plan_request.goal_constraints.joint_constraints[4].position =  1.61;
      leftGoal->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.63;
      leftGoal->motion_plan_request.goal_constraints.joint_constraints[6].position =  0.57;

      return true;
    }

    bool initRightArmGoalSingle( arm_navigation_msgs::MoveArmGoal * rightSingle )
    {
      //rightBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(rightSingle != NULL);

      //TODO make dynamic
      rightSingle->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      rightSingle->motion_plan_request.goal_constraints.joint_constraints[0].position = -0.08;
      rightSingle->motion_plan_request.goal_constraints.joint_constraints[1].position =  1.05;
      rightSingle->motion_plan_request.goal_constraints.joint_constraints[2].position = -1.45;
      rightSingle->motion_plan_request.goal_constraints.joint_constraints[3].position = -1.43;
      rightSingle->motion_plan_request.goal_constraints.joint_constraints[4].position = -2.55;
      rightSingle->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.52;
      rightSingle->motion_plan_request.goal_constraints.joint_constraints[6].position =  2.70;

      return true;
    }

    bool initLeftArmGoalSingle( arm_navigation_msgs::MoveArmGoal * leftSingle )
    {
      //leftBoth = new arm_navigation_msgs::MoveArmGoal ();
      assert(leftSingle != NULL);

      //TODO make dynamic
      leftSingle->motion_plan_request.goal_constraints.joint_constraints.resize(7);

      leftSingle->motion_plan_request.goal_constraints.joint_constraints[0].position =  0.09;
      leftSingle->motion_plan_request.goal_constraints.joint_constraints[1].position =  0.93;
      leftSingle->motion_plan_request.goal_constraints.joint_constraints[2].position =  1.64;
      leftSingle->motion_plan_request.goal_constraints.joint_constraints[3].position = -1.27;
      leftSingle->motion_plan_request.goal_constraints.joint_constraints[4].position =  2.47;
      leftSingle->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.26;
      leftSingle->motion_plan_request.goal_constraints.joint_constraints[6].position = -0.98;

      return true;
    }

    void executeCB(const tray_pick_place::TrayPickPlaceGoalConstPtr &goal)
    {
      // make sure that the action hasn't been canceled
      if (!as_.isActive())
        return;

      bool success = true;
      bool rightArm = true; // right arm

      ROS_INFO("Executing, creating arm action");

      // start executing the action
////////////////////// code for tray object grasping <jinn> ///////////////////////////////
      if (goal->action == tray_pick_place::TrayPickPlaceGoal::PICK_SIDE) // move arm to grasp object on tray from side
      {
        // check that preempt has not been requested by the client
	ROS_INFO("Tray manipulation action: PICK_SIDE action!");
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
        }
        else
        {
          ROS_INFO("Executing arm trajectory..");
          bool fromSide = true;
          switch (goal->arm)
          {
            case tray_pick_place::TrayPickPlaceGoal::RIGHT_ARM:
              ROS_INFO("use right arm");
              rightArm = true;
              success = trayGrasp(rightArm, fromSide, goal->slot);
              break;
            case tray_pick_place::TrayPickPlaceGoal::LEFT_ARM:
              ROS_INFO("use left arm");
              rightArm = false;
              success = trayGrasp(rightArm, fromSide, goal->slot);
              break;
            default:
              ROS_INFO("Unknown arm command, ignoring");
              break;
          }
        }
      }

      else if (goal->action == tray_pick_place::TrayPickPlaceGoal::PICK_TOP) // move arm to grasp object from top
      {
        // check that preempt has not been requested by the client
	ROS_INFO("Tray manipulation action: PICK_TOP action!");
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
        }
        else
        {
          ROS_INFO("Executing arm trajectory..");
          bool fromSide = false;

          switch (goal->arm)
          {
            case tray_pick_place::TrayPickPlaceGoal::RIGHT_ARM:
              ROS_INFO("use right arm");
              rightArm = true;
              success = trayGrasp(rightArm, fromSide, goal->slot);
              break;
            case tray_pick_place::TrayPickPlaceGoal::LEFT_ARM:
              ROS_INFO("use right arm");
              rightArm = false;
              success = trayGrasp(rightArm, fromSide, goal->slot);
              break;
            default:
              ROS_INFO("Unknown arm command, ignoring");
              break;
          }
        }
      }

      else if (goal->action == tray_pick_place::TrayPickPlaceGoal::PLACE) // put object, posture is determined automatically
      {
        // check that preempt has not been requested by the client
	ROS_INFO("Tray manipulation action: PLACE action!");
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
        }
        else
        {
//					geometry_msgs::PoseStamped stamped_out;

          ROS_INFO("Executing arm trajectory..");
          bool fromSide = true;
					ROS_INFO("height: %f", goal->bbox_dims.z);
          switch (goal->arm)
          {
            case tray_pick_place::TrayPickPlaceGoal::RIGHT_ARM:
              ROS_INFO("use right arm");
              rightArm = true;
              ROS_INFO("Detecting grasping posture..");
//							listener.transformPose ("base_link", g_r_stamped_pose, stamped_out);
							if (!goal->use_place_pose)
							{
								current_eular = transformQuarternionToEular(g_r_stamped_pose.orientation);
								std::cout << "current eular: " << current_eular*cmn180_PI << std::endl;
								pre_place_eular = current_eular; 
								if (abs(current_eular.Y()) < cmnPI_4) {fromSide = true; side_place_y_angle = current_eular.Y(); ROS_INFO("Current posture is grasping from side.");}
								else {fromSide = false; ROS_INFO("Current posture is grasping from top.");}
								success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
							}
							else
							{
								current_eular = transformQuarternionToEular(goal->place_pose.pose.orientation);
								std::cout << "given eular: " << current_eular*cmn180_PI << std::endl;
								pre_place_eular = current_eular; 
								success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
							}
              break;
            case tray_pick_place::TrayPickPlaceGoal::LEFT_ARM:
              ROS_INFO("use right arm");
              rightArm = false;
              ROS_INFO("Detecting grasping posture..");
//							listener.transformPose ("base_link", g_l_stamped_pose, stamped_out);
							if (!goal->use_place_pose)
							{
								current_eular = transformQuarternionToEular(g_l_stamped_pose.orientation);
								std::cout << "current eular: " << current_eular*cmn180_PI << std::endl;
								pre_place_eular = current_eular; 
								if (abs(current_eular.Y()) < cmnPI_4) {fromSide = true; side_place_y_angle = current_eular.Y(); ROS_INFO("Current posture is grasping from side.");}
								else {fromSide = false; ROS_INFO("Current posture is grasping from top.");}
								success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
							}
							else
							{
								current_eular = transformQuarternionToEular(goal->place_pose.pose.orientation);
								std::cout << "given eular: " << current_eular*cmn180_PI << std::endl;
								pre_place_eular = current_eular; 
								success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
							}
              break;
            default:
              ROS_INFO("Unknown arm command, ignoring");
              break;
          }
        }

      }
      else if (goal->action == tray_pick_place::TrayPickPlaceGoal::PLACE_SIDE) // put object from side
      {
	ROS_INFO("Tray manipulation action: PLACE_SIDE action!");
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
          ROS_INFO("Executing arm trajectory..");
          bool fromSide = true;
					ROS_INFO("height: %f", goal->bbox_dims.z);
          switch (goal->arm)
          {
            case tray_pick_place::TrayPickPlaceGoal::RIGHT_ARM:
              ROS_INFO("use right arm");
              rightArm = true;
              side_place_y_angle = 0.0;
							current_eular.Assign(cmnPI, 0.0, cmnPI_2);
              success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
              break;
            case tray_pick_place::TrayPickPlaceGoal::LEFT_ARM:
              ROS_INFO("use left arm");
              rightArm = false;
              side_place_y_angle = 0.0;
							current_eular.Assign(-cmnPI, 0.0, -cmnPI_2);
              success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
              break;
            default:
              ROS_INFO("Unknown arm command, ignoring");
              break;
          }
        }
      }

      else if (goal->action == tray_pick_place::TrayPickPlaceGoal::PLACE_TOP) // put object from top
      {
				ROS_INFO("Tray manipulation action: PLACE_TOP action!");
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
          ROS_INFO("Executing arm trajectory..");
          bool fromSide = false;
					ROS_INFO("height: %f", goal->bbox_dims.z);
					current_eular.Assign(0.0, cmnPI_2, 0.0);
          switch (goal->arm)
          {
            case tray_pick_place::TrayPickPlaceGoal::RIGHT_ARM:
              ROS_INFO("use right arm");
              rightArm = true;
              success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
              break;
            case tray_pick_place::TrayPickPlaceGoal::LEFT_ARM:
              ROS_INFO("use left arm");
              rightArm = false;
              success = trayPlace(rightArm, fromSide, goal->slot, goal->bbox_dims.z);
              break;
            default:
              ROS_INFO("Unknown arm command, ignoring");
              break;
          }
        }

      }
//////////////////////////////////////////////////////////////////////////////////

      if(success)
      {
        result_.result = tray_pick_place::TrayPickPlaceResult::SUCCESS_GRASP;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
      else
      {
        result_.result = tray_pick_place::TrayPickPlaceResult::FAILED_GRASP;
        ROS_INFO("%s: Failed", action_name_.c_str());
        // set the action state to failed
        //as_.setSucceeded(result_);
        //as_.setAborted(result_);
        as_.setAborted(result_);
      }
    }
////////////////////// code for tray object grasping <jinn> ///////////////////////////////
    bool moveArmToTrayObject (bool rightArm, vct3 pos, vct3 eular)
    {
      std::string move_arm_topic;
      if (rightArm) move_arm_topic = "move_right_arm";
      else move_arm_topic = "move_left_arm";
      actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(move_arm_topic,true);
      ROS_INFO("Waiting for %s server...", move_arm_topic.c_str());
      move_arm.waitForServer();
      ROS_INFO("Connected to %s server.", move_arm_topic.c_str());
      arm_navigation_msgs::MoveArmGoal armGoal;
      //TODO make dynamic
      if (rightArm) armGoal.motion_plan_request.group_name = "right_arm";
      else armGoal.motion_plan_request.group_name = "left_arm";
      armGoal.motion_plan_request.num_planning_attempts = 1;
      armGoal.motion_plan_request.planner_id = std::string("");
      armGoal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
      armGoal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

      arm_navigation_msgs::SimplePoseConstraint desired_pose;
      desired_pose.header.frame_id = "base_tray_laser_link";
      //desired_pose.header.frame_id = "torso_lift_link";
      if (rightArm) desired_pose.link_name = "r_wrist_roll_link";
      else desired_pose.link_name = "l_wrist_roll_link";
      //if (rightArm) desired_pose.link_name = "r_gripper_tool_frame";
      //else desired_pose.link_name = "l_gripper_tool_frame";
      desired_pose.pose.position.x = pos.X();
      desired_pose.pose.position.y = pos.Y();
      desired_pose.pose.position.z = pos.Z();

////////////////////// eular ///////////////////////////////
      desired_pose.pose.orientation.x = sin(eular.X()/2)*cos(eular.Y()/2)*cos(eular.Z()/2) - cos(eular.X()/2)*sin(eular.Y()/2)*sin(eular.Z()/2);
      desired_pose.pose.orientation.y = cos(eular.X()/2)*sin(eular.Y()/2)*cos(eular.Z()/2) + sin(eular.X()/2)*cos(eular.Y()/2)*sin(eular.Z()/2);
      desired_pose.pose.orientation.z = cos(eular.X()/2)*cos(eular.Y()/2)*sin(eular.Z()/2) - sin(eular.X()/2)*sin(eular.Y()/2)*cos(eular.Z()/2);
      desired_pose.pose.orientation.w = cos(eular.X()/2)*cos(eular.Y()/2)*cos(eular.Z()/2) + sin(eular.X()/2)*sin(eular.Y()/2)*sin(eular.Z()/2);

      desired_pose.absolute_position_tolerance.x = 0.02;
      desired_pose.absolute_position_tolerance.y = 0.02;
      desired_pose.absolute_position_tolerance.z = 0.02;

      desired_pose.absolute_roll_tolerance = 0.04;
      desired_pose.absolute_pitch_tolerance = 0.04;
      desired_pose.absolute_yaw_tolerance = 0.04;

      arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose, armGoal);

      armGoal.accept_partial_plans = true;
      armGoal.accept_invalid_goals = true;
      armGoal.disable_collision_monitoring = true;

      //Ignore collisions between Gripper and objects
      arm_navigation_msgs::CollisionOperation coll;
      //coll.object1 = "gripper";
      coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
      armGoal.operations.collision_operations.push_back(coll);

      bool result = true;
      bool success = true;

      if (nh_.ok())
      {
        bool finished_within_time = false;
        ROS_INFO("Sending arm goal..");
        move_arm.sendGoal(armGoal);
        finished_within_time = move_arm.waitForResult(ros::Duration(200.0));

        if (!finished_within_time)
        {
          move_arm.cancelGoal();
          ROS_INFO("Timed out achieving arm goal");
          success = false;
        }
        else
        {
          actionlib::SimpleClientGoalState state = move_arm.getState();
          success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
          if(success)
            ROS_INFO("arm moveArmTrayObject finished: %s",state.toString().c_str());
          else
            ROS_INFO("arm moveArmTrayObject failed: %s",state.toString().c_str());
        }
      }

      result = success;
      return result;
    }
//////////////////////////////////////////////// tray grasp //////////////////////////
    bool trayGrasp(bool rightArm, bool fromSide, int object)
    {
			int actual_object;
			actual_object = object;
	    bool success = true;
			// check whether some objects are on the tray
			if (!(isPosePossible(tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT)) 
				&& !(isPosePossible(tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT)) 
				&& !(isPosePossible(tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT)))
				{ROS_INFO("There is nothing detected on the tray!"); return false;}
				// counting object number
			int object_counter = 0;
			if (isPosePossible(tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT)) 
				{ROS_INFO("An object detected on the right slot!"); object_counter++;}
			if (isPosePossible(tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT)) 
				{ROS_INFO("An object detected on the middle slot!"); object_counter++;}
			if (isPosePossible(tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT))
				{ROS_INFO("An object detected on the left slot!"); object_counter++;}
			
	    // to judge if there is already an object on the slot
	    if (!isPosePossible(object))
			{
				switch(object)
				{
					case tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT:
					ROS_INFO("There is no object detected on the right slot!"); 
					break;
					case tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT:
					ROS_INFO("There is no object detected on the middle slot!"); 
					break;
					case tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT:
					ROS_INFO("There is no object detected on the left slot!"); 
					break;
					default:
					break;
				}
				// if only one object on the tray
				if (object_counter == 1)
				{
					if (isPosePossible(tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT)) 
						{ROS_INFO("An object detected on the right slot!"); actual_object = tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT;}
					if (isPosePossible(tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT)) 
						{ROS_INFO("An object detected on the middle slot!"); actual_object = tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT;}
					if (isPosePossible(tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT))
						{ROS_INFO("An object detected on the left slot!"); actual_object = tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT;}
					ROS_INFO("Try to pick the object!");
				}
				else {success = false; return success;}
			}

      vct3 pos(0.0), eular(0.0);
      
      // locate on the objects
			vct3 pos1(0.0), pos2(0.0), pos3(0.0);

      switch(actual_object)
      {
        case tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT:
					pos1 = g_pos1;
          ROS_INFO("Object 1: %f, %f, %f", pos1.X(), pos1.Y(), objectHeight1);
          if (fromSide)
          {
            if (rightArm) pos.Assign(pos1.X() + rightObjectX + m_slot1_offset.X(), pos1.Y() + rightObjectY + m_slot1_offset.Y(), /*pos1.Z()*/ objectHeight1 * 0.5);
            else pos.Assign(pos1.X() + rightObjectX + m_slot1_offset.X(), pos1.Y() + leftObjectY + m_slot1_offset.Y() - 0.01, /*pos1.Z()*/ objectHeight1 * 0.5);
          }
          else
          {
						std::cout << "objectHeight1: " << objectHeight1 << std::endl;
            if (rightArm) pos.Assign(pos1.X() + rightObjectX + m_slot1_offset.X() - 0.005, pos1.Y() - middleObjectY + m_slot1_offset.Y() + 0.016, objectHeight1 + gripper_offset);
						else pos.Assign(pos1.X() + rightObjectX + m_slot1_offset.X() - 0.008, pos1.Y() - middleObjectY + m_slot1_offset.Y() + 0.016, objectHeight1 + gripper_offset);
          }
          break;
        case tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT:
					pos2 = g_pos2;
          ROS_INFO("Object 2: %f, %f, %f", pos2.X(), pos2.Y(), objectHeight2);
          if (fromSide)
          {
            if (rightArm) pos.Assign(pos2.X() + middleObjectX + m_slot2_offset.X(), pos2.Y() + rightObjectY + m_slot2_offset.Y() + 0.02, /*pos2.Z()*/ objectHeight2 * 0.5);
            else pos.Assign(pos2.X() + middleObjectX + m_slot2_offset.X(), pos2.Y() + leftObjectY + m_slot2_offset.Y() - 0.02, /*pos2.Z()*/ objectHeight2 * 0.5);
          }
          else
          {
						std::cout << "objectHeight2: " << objectHeight2 << std::endl;
            if (rightArm) pos.Assign(pos2.X() + middleObjectX + m_slot2_offset.X() - 0.01, pos2.Y() + m_slot2_offset.Y(), objectHeight2 + gripper_offset);
						else pos.Assign(pos2.X() + middleObjectX + m_slot2_offset.X() - 0.01, pos2.Y() + m_slot2_offset.Y() - 0.011, objectHeight2 + gripper_offset);
          }
          break;
        case tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT:
					pos3 = g_pos3;
          ROS_INFO("Object 3: %f, %f, %f", pos3.X(), pos3.Y(), objectHeight3);
          if (fromSide)
          {
            if (rightArm) pos.Assign(pos3.X() + leftObjectX + m_slot3_offset.X(), pos3.Y() + rightObjectY + m_slot3_offset.Y() + 0.01, /*pos3.Z()*/ objectHeight3 * 0.5);
            else pos.Assign(pos3.X() + leftObjectX + m_slot3_offset.X(), pos3.Y() + leftObjectY + m_slot3_offset.Y(), /*pos3.Z()*/ objectHeight3 * 0.5);
          }
          else
          {
						std::cout << "objectHeight3: " << objectHeight3 << std::endl;
            if (rightArm) pos.Assign(pos3.X() + leftObjectX + m_slot3_offset.X() - 0.005, pos3.Y() + middleObjectY + m_slot3_offset.Y()- 0.01, objectHeight3 + gripper_offset);
						else pos.Assign(pos3.X() + leftObjectX + m_slot3_offset.X() - 0.005, pos3.Y() + middleObjectY + m_slot3_offset.Y() - 0.02, objectHeight3 + gripper_offset);
          }
          break;
        default:
					ROS_INFO("Input a wrong slot number!");
          success = false;
          return success;
          break;
      }
			// define pick posture
      if (rightArm && fromSide) eular.Assign(0.0, 0.0, cmnPI_2);
      else if (!rightArm && fromSide) eular.Assign(0.0, 0.0, -cmnPI_2);
      else if (!fromSide) eular.Assign(0.0, cmnPI_2, 0.0);
      
			if (abs(pre_place_eular.X()) <= cmnPI_2) eular.X() = 0.0;
			else if (pre_place_eular.X() < -cmnPI_2) eular.X() = -cmnPI;
			else eular.X() = cmnPI;
      
      
      success = moveArmToTrayObject(rightArm, pos, eular);
      
      int tmp_counter = 0;
      while(!success)
      {
				if (fromSide) 
				{
					if (tmp_counter > 2) 
					{
						torso.down();
						if (!torso.isUp) ROS_INFO("Torso down...");
					}
				}
				tmp_counter++;
				ros::Duration(0.5).sleep();
				ROS_INFO("Moving failed, try again: %d.", tmp_counter);
				success = moveArmToTrayObject(rightArm, pos, eular);
				if (tmp_counter > 6) {ROS_INFO("Failed to locate!"); break;}
			}
			
      if (!success) 
			{
				if (!torso.isUp) torso.up();
				return success;
			}
      
      ros::Duration(0.5).sleep();
      // start to grasp
      ROS_INFO("Moving to Grasp");
      if (rightArm) rgripper.open();
      else lgripper.open();
			
			ros::Duration(0.5).sleep();
      if (rightArm && fromSide) pos.Y() += 0.08;
      else if (!rightArm && fromSide) pos.Y() -= 0.08;
      else if (!fromSide) pos.Z() -= 0.07;
      success = moveArmToTrayObject(rightArm, pos, eular);
      
      tmp_counter = 0;
      while(!success)
      {
				tmp_counter++;
				ros::Duration(0.5).sleep();
				ROS_INFO("Moving failed, try again: %d.", tmp_counter);
				success = moveArmToTrayObject(rightArm, pos, eular);
				if (tmp_counter > 5) {tmp_counter = 0; ROS_INFO("Failed to pick up!"); break;}
			}
			if (!success) 
			{
				if (!torso.isUp) torso.up();
				if (torso.isUp) ROS_INFO("Torso up...");
				return success;
			}
      
//      ros::Duration(0.5).sleep();
      if (rightArm) rgripper.close();
      else lgripper.close();

      // move up
      if (rightArm) pos.Assign(0.3, -0.5, 0.6); // side position in /base_tray_laser_link
      else pos.Assign(0.3, 0.5, 0.6);
      success = moveArmToTrayObject(rightArm, pos, eular);
      if (!torso.isUp) torso.up();
      if (torso.isUp) ROS_INFO("Torso up...");
      if (!success) return success;
      ROS_INFO("Moving back to the side");
			
//    std::cout << "distance decreasing: " << tmp_dec << std::endl;
			
      return success;
    }
////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////// tray place //////////////////////////
    bool trayPlace(bool rightArm, bool fromSide, int object, double height)
    {
	    bool success = true;

	    // to judge if there is already an object on the slot
	    if (isPosePossible(object)) 
	    {
				switch(object)
				{
					case tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT:
					ROS_INFO("There is an object on the right slot!"); 
					break;
					case tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT:
					ROS_INFO("There is an object on the middle slot!"); 
					break;
					case tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT:
					ROS_INFO("There is an object on the left slot!"); 
					break;
					default:
					break;
				}
				success = false; return success;
			}
	    else {success = true;}

      vct3 pos(0.0), current_pos(0.0), eular(0.0);

      switch(object)
      {
        case tray_pick_place::TrayPickPlaceGoal::RIGHT_SLOT:
          ROS_INFO("Place 1: %f, %f, %f", g_place_pos1.X(), g_place_pos1.Y(), g_place_pos1.Z());
          objectHeight1 = height;
          ROS_INFO("The height of object: %f", objectHeight1);
          if (fromSide)
          {
            if (rightArm) pos.Assign(g_place_pos1.X() + rightObjectX, g_place_pos1.Y() + rightObjectY + 0.03, height * 0.7 + 0.05);
            else pos.Assign(g_place_pos1.X() + leftObjectX, g_place_pos1.Y() + leftObjectY - 0.16, height * 0.7 + 0.05);
          }
          else
          {
            if (rightArm) pos.Assign(g_place_pos1.X() + middleObjectX, g_place_pos1.Y() - 0.07, height + 0.05);
            else pos.Assign(g_place_pos1.X() + middleObjectX, g_place_pos1.Y() - 0.07, height + 0.05);
          }
          break;
        case tray_pick_place::TrayPickPlaceGoal::MIDDLE_SLOT:
          ROS_INFO("Place 2: %f, %f, %f", g_place_pos2.X(), g_place_pos2.Y(), g_place_pos2.Z());
					objectHeight2 = height;
					ROS_INFO("The height of object: %f", objectHeight2);
          if (fromSide)
          {
            if (rightArm) pos.Assign(g_place_pos2.X() + rightObjectX, g_place_pos2.Y() + rightObjectY + 0.06, height * 0.7 + 0.05);
            else pos.Assign(g_place_pos2.X() + leftObjectX, g_place_pos2.Y() + leftObjectY - 0.06, height * 0.7 + 0.05);
          }
          else
          {
            pos.Assign(g_place_pos2.X() + middleObjectX, g_place_pos2.Y(), height + 0.05);
          }
          break;
        case tray_pick_place::TrayPickPlaceGoal::LEFT_SLOT:
          ROS_INFO("Place 3: %f, %f, %f", g_place_pos3.X(), g_place_pos3.Y(), g_place_pos3.Z());
					objectHeight3 = height;
					ROS_INFO("The height of object: %f", objectHeight3);
          if (fromSide)
          {
            if (rightArm) pos.Assign(g_place_pos3.X() + rightObjectX, g_place_pos3.Y() + rightObjectY + 0.16, height * 0.7 + 0.05);
            else pos.Assign(g_place_pos3.X() + leftObjectX, g_place_pos3.Y() + leftObjectY - 0.03, height * 0.7 + 0.05);
          }
          else
          {
            if (rightArm) pos.Assign(g_place_pos3.X() + middleObjectX, g_place_pos3.Y() + 0.07, height + 0.05);
						else pos.Assign(g_place_pos3.X() + middleObjectX, g_place_pos3.Y() + 0.07, height + 0.05);
          }
          break;
        default:
          success = false;
          return success;
          break;
      }

      if (rightArm && fromSide) eular.Assign(current_eular.X(), current_eular.Y(), cmnPI_2);
      else if (!rightArm && fromSide) eular.Assign(current_eular.X(), current_eular.Y(), -cmnPI_2);
      else if (!fromSide) eular.Assign(0.0, cmnPI_2, 0.0);

			if (abs(current_eular.X()) <= cmnPI_2) eular.X() = 0.0;
			else if (current_eular.X() < -cmnPI_2) eular.X() = -cmnPI;
			else eular.X() = cmnPI;
			std::cout << "target posture: " << current_eular*cmn180_PI << std::endl;
			std::cout << "posture to place: " << eular*cmn180_PI << std::endl;
      
      current_pos = pos; current_pos.Z() = pos.Z();
      ROS_INFO("move it to pre-placing position.");
      success = moveArmToTrayObject(rightArm, vct3(current_pos.X(), current_pos.Y(), 0.3), eular);
      int tmp_counter = 0;
      while(!success)
      {
				tmp_counter++;
				ros::Duration(0.5).sleep();
				ROS_INFO("Moving failed, try again: %d.", tmp_counter);
				success = moveArmToTrayObject(rightArm, vct3(current_pos.X(), current_pos.Y(), 0.3), eular);
				if (tmp_counter > 5) {tmp_counter = 0; ROS_INFO("Failed to place!"); break;}
			}
			if (!success) return success;
			
			success = moveArmToTrayObject(rightArm, current_pos, eular);
			tmp_counter = 0;
      while(!success)
      {
				tmp_counter++;
				ros::Duration(0.5).sleep();
				ROS_INFO("Moving failed, try again: %d.", tmp_counter);
				success = moveArmToTrayObject(rightArm, current_pos, eular);
				if (tmp_counter > 5) {tmp_counter = 0; ROS_INFO("Failed to place!"); break;}
			}
			if (!success) return success;
      
      ros::Duration(0.5).sleep();

      double begin_elbow_effort = 0, begin_shoulder_effort = 0, cur_effort = 0;
      if (rightArm)
				{begin_elbow_effort = g_r_elbow_flex_joint_effort; begin_shoulder_effort = g_r_shoulder_pan_joint_effort;}
      else
				{begin_elbow_effort = g_l_elbow_flex_joint_effort; begin_shoulder_effort = g_l_shoulder_pan_joint_effort;}

      int counter = 0;
      while(!isPosePossible(object))
      {
				// move down
				current_pos.Z() -= 0.02;
				success &= moveArmToTrayObject(rightArm, current_pos, eular);
				if (!success) {ROS_INFO("Can't reach the height! Release gripper!"); break;}
//				ros::Duration(0.5).sleep();
				counter++;
				ROS_INFO("Detecting object: %d; counter: %d", isPosePossible(object), counter);
//				std::cout << "Current heigth: " << current_pos.Z() << " pos.Z(): " << pos.Z() << std::endl;
				if (rightArm)
					cur_effort = abs(g_r_elbow_flex_joint_effort - begin_elbow_effort) + abs(g_r_shoulder_pan_joint_effort - begin_shoulder_effort);
				else
					cur_effort = abs(g_l_elbow_flex_joint_effort - begin_elbow_effort) + abs(g_l_shoulder_pan_joint_effort - begin_shoulder_effort);
				ROS_INFO("force difference: %f", cur_effort);
				if (cur_effort > 4) {ROS_INFO("Place force is overloaded! %f", cur_effort); break;}
//				if (counter > 15) break;
			}

      // start to release
      ros::Duration(0.5).sleep();
      ROS_INFO("Moving to Release");
      if (rightArm) rgripper.open();
      else lgripper.open();
      //calculate offset of this place slot
      
			ros::Duration(0.5).sleep();
      if (rightArm && fromSide) {current_pos.Y() -= 0.11; current_pos.Z() += 0.1;}
      else if (!rightArm && fromSide) {current_pos.Y() += 0.11; current_pos.Z() += 0.1;}
      else if (!fromSide) current_pos.Z() += 0.11;
      success = moveArmToTrayObject(rightArm, current_pos, eular);
      if (!success) return success;
      // move arm to the side
      ros::Duration(0.5).sleep();
      success = moveArmToSide(rightArm);
      if (!success) return success;
      ROS_INFO("Moving to the side");

      if (rightArm) rgripper.close();
      else lgripper.close();

      return success;
    }

////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////

    bool moveArmToSide ( bool rightArm )
    {
      bool result  = true;

      if (rightArm == true)
      {
        arm_navigation_msgs::MoveArmGoal rightArmGoal;

        initRightArmGoalSide(&rightArmGoal);

        //move right arm
        result = moveRightArm (&rightArmGoal);
      }
      else
      {
        arm_navigation_msgs::MoveArmGoal leftArmGoal;

        initLeftArmGoalSide(&leftArmGoal);

        //move left arm
        result = moveLeftArm (&leftArmGoal);
      }

      return result;
    }

    //TODO rename to moveArmSingle ( bool rightArm )
    bool moveRightArm ( void )
    {
      bool result  = true;

      arm_navigation_msgs::MoveArmGoal rightArmGoal;

      initRightArmGoalSingle(&rightArmGoal);

      //move right arm
      result &= moveRightArm (&rightArmGoal);

      return result;
    }

    //TODO rename to moveArmSingle ( bool rightArm )
    bool moveLeftArm ( void )
    {
      bool result  = true;

      arm_navigation_msgs::MoveArmGoal leftArmGoal;

      initLeftArmGoalSingle(&leftArmGoal);

      //move left arm
      result &= moveLeftArm (&leftArmGoal);

      return result;
    }

    bool moveLeftArm ( const arm_navigation_msgs::MoveArmGoal * newArmGoal )
    {
      actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_left_arm",true);

      ROS_INFO("Waiting for move_left_arm server..");
      move_arm.waitForServer();
      ROS_INFO("Connected to move_left_arm server");

      arm_navigation_msgs::MoveArmGoal armGoal; std::vector<std::string> names(7);

      names[0] = "l_shoulder_pan_joint";
      names[1] = "l_shoulder_lift_joint";
      names[2] = "l_upper_arm_roll_joint";
      names[3] = "l_elbow_flex_joint";
      names[4] = "l_forearm_roll_joint";
      names[5] = "l_wrist_flex_joint";
      names[6] = "l_wrist_roll_joint";

      armGoal.motion_plan_request.group_name = "left_arm";
      armGoal.motion_plan_request.num_planning_attempts = 1;
      armGoal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
      armGoal.motion_plan_request.planner_id= std::string("");
      armGoal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
      armGoal.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

      for (unsigned int i = 0 ; i < armGoal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
      {
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
        // Set the joint values
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].position = newArmGoal->motion_plan_request.goal_constraints.joint_constraints[i].position;

        //// disable collision checking
        //armGoal.planning_scene_diff.allowed_collision_matrix.entries[i].enabled[0] = false;
      }
      armGoal.accept_partial_plans = true;
      armGoal.accept_invalid_goals = true;
      armGoal.disable_collision_monitoring = true;
      //
      //Ignore collisions between Gripper and objects
      arm_navigation_msgs::CollisionOperation coll;
      //coll.object1 = "gripper";
      coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
      armGoal.operations.collision_operations.push_back(coll);

      bool result = true;
      bool success = true;

      if (nh_.ok())
      {
        bool finished_within_time = false;
        ROS_INFO("Sending arm goal..");
        move_arm.sendGoal(armGoal);
        finished_within_time = move_arm.waitForResult(ros::Duration(200.0));

        if (!finished_within_time)
        {
          move_arm.cancelGoal();
          ROS_INFO("Timed out achieving arm goal");
          success = false;
        }
        else
        {
          actionlib::SimpleClientGoalState state = move_arm.getState();
          success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
          if(success)
            ROS_INFO("arm moveLeftArm finished: %s",state.toString().c_str());
          else
            ROS_INFO("arm moveLeftArm failed: %s",state.toString().c_str());
        }
      }
      result = success;

      return result;
    }

    bool moveRightArm ( const arm_navigation_msgs::MoveArmGoal * newArmGoal )
    {
      actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);

      ROS_INFO("Waiting for move_right_arm server..");
      move_arm.waitForServer();
      ROS_INFO("Connected to move_right_arm server");

      arm_navigation_msgs::MoveArmGoal armGoal;
      std::vector<std::string> names(7);

      names[0] = "r_shoulder_pan_joint";
      names[1] = "r_shoulder_lift_joint";
      names[2] = "r_upper_arm_roll_joint";
      names[3] = "r_elbow_flex_joint";
      names[4] = "r_forearm_roll_joint";
      names[5] = "r_wrist_flex_joint";
      names[6] = "r_wrist_roll_joint";

      armGoal.motion_plan_request.group_name = "right_arm";
      armGoal.motion_plan_request.num_planning_attempts = 1;
      armGoal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

      armGoal.motion_plan_request.planner_id= std::string("");
      armGoal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
      armGoal.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

      for (unsigned int i = 0 ; i < armGoal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
      {
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
        // Set the joint values
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].position =
          newArmGoal->motion_plan_request.goal_constraints.joint_constraints[i].position;

        //// disable collision checking
        //armGoal.planning_scene_diff.allowed_collision_matrix.entries[i].enabled[0] = false;
      }
      armGoal.accept_partial_plans = true;
      armGoal.accept_invalid_goals = true;
      armGoal.disable_collision_monitoring = true;

      //Ignore collisions between Gripper and objects
      arm_navigation_msgs::CollisionOperation coll;
      //coll.object1 = "gripper";
      coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
      armGoal.operations.collision_operations.push_back(coll);

      bool result = true;
      bool success = true;

      if (nh_.ok())
      {
        bool finished_within_time = false;
        ROS_INFO("Sending arm goal..");
        move_arm.sendGoal(armGoal);
        finished_within_time = move_arm.waitForResult(ros::Duration(200.0));

        if (!finished_within_time)
        {
          move_arm.cancelGoal();
          ROS_INFO("Timed out achieving arm goal");
          success = false;
        }
        else
        {
          actionlib::SimpleClientGoalState state = move_arm.getState();
          success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
          if(success)
            ROS_INFO("arm moveRightArm finished: %s",state.toString().c_str());
          else
            ROS_INFO("arm moveRightArm failed: %s",state.toString().c_str());
        }
      }
      result = success;

      return result;
    }

  public:
};

void callBackTrayMonitor(const race_tray_monitor::race_tray_object_midpoint & objectposition)
{
    //ROS_INFO("%d objects have been found!", objectposition.pos.size());
    int pos_num = 0;
    pos_num = objectposition.pos.size();

    g_pos1.X() = 0.0; g_pos1.Y() = 0.0; g_pos2.X() = 0.0; g_pos2.Y() = 0.0; g_pos3.X() = 0.0; g_pos3.Y() = 0.0; // initial global positions

    if (pos_num == 0)		return;

//    ROS_INFO("%d object is found by tray monitor!", pos_num);

    vct3 planned_pos[3]; planned_pos[0] = g_place_pos1 - 0.05; planned_pos[1] = g_place_pos2; planned_pos[2] = g_place_pos3 + 0.05;
    int pos_flag[3] = {0};

    for (int i = 0; i < 3; i++)
    {
      double distance = 10, pre_distance = 10;
      for (int j = 0; j < pos_num; j++)
      {
        vct3 dec(planned_pos[i].X() - objectposition.pos[j].x, planned_pos[i].Y() - objectposition.pos[j].y, 0.0);
        distance = dec.Norm();
        if (distance <= pre_distance) {pos_flag[i] = j; pre_distance = distance;}
       }
    }

    vct3 pos1(objectposition.pos[pos_flag[0]].x, objectposition.pos[pos_flag[0]].y, 0.0);
    vct3 pos2(objectposition.pos[pos_flag[1]].x, objectposition.pos[pos_flag[1]].y, 0.0);
    vct3 pos3(objectposition.pos[pos_flag[2]].x, objectposition.pos[pos_flag[2]].y, 0.0);
    vct3 dec11, dec12, dec13; dec11 = pos1 - (g_place_pos1 - 0.05); 
	dec12 = pos1 - g_place_pos2; dec13 = pos1 - (g_place_pos3 + 0.05);
    vct3 dec21, dec22, dec23; dec21 = pos2 - (g_place_pos1 - 0.05); 
	dec22 = pos2 - g_place_pos2; dec23 = pos2 - (g_place_pos3 + 0.05);
    vct3 dec31, dec32, dec33; dec31 = pos3 - (g_place_pos1 - 0.05); 
	dec32 = pos3 - g_place_pos2; dec33 = pos3 - (g_place_pos3 + 0.05);

    if (dec11.Norm() <= dec12.Norm() && dec11.Norm() <= dec13.Norm()
     && dec11.Norm() <= dec21.Norm() && dec11.Norm() <= dec31.Norm()) g_pos1.Assign(pos1.X(), pos1.Y(), g_pos1.Z());
    if (dec12.Norm() <= dec11.Norm() && dec12.Norm() <= dec13.Norm()
     && dec12.Norm() <= dec22.Norm() && dec12.Norm() <= dec32.Norm()) g_pos2.Assign(pos1.X(), pos1.Y(), g_pos2.Z());
    if (dec13.Norm() <= dec11.Norm() && dec13.Norm() <= dec12.Norm()
     && dec13.Norm() <= dec23.Norm() && dec23.Norm() <= dec33.Norm()) g_pos3.Assign(pos1.X(), pos1.Y(), g_pos3.Z());
    if (dec21.Norm() <= dec22.Norm() && dec21.Norm() <= dec23.Norm()
     && dec21.Norm() <= dec11.Norm() && dec21.Norm() <= dec31.Norm()) g_pos1.Assign(pos2.X(), pos2.Y(), g_pos1.Z());
    if (dec22.Norm() <= dec21.Norm() && dec22.Norm() <= dec23.Norm()
     && dec22.Norm() <= dec12.Norm() && dec22.Norm() <= dec32.Norm()) g_pos2.Assign(pos2.X(), pos2.Y(), g_pos2.Z());
    if (dec23.Norm() <= dec21.Norm() && dec23.Norm() <= dec22.Norm()
     && dec23.Norm() <= dec13.Norm() && dec23.Norm() <= dec33.Norm()) g_pos3.Assign(pos2.X(), pos2.Y(), g_pos3.Z());
    if (dec31.Norm() <= dec32.Norm() && dec31.Norm() <= dec33.Norm()
     && dec31.Norm() <= dec11.Norm() && dec31.Norm() <= dec21.Norm()) g_pos1.Assign(pos3.X(), pos3.Y(), g_pos1.Z());
    if (dec32.Norm() <= dec31.Norm() && dec32.Norm() <= dec33.Norm()
     && dec32.Norm() <= dec12.Norm() && dec32.Norm() <= dec22.Norm()) g_pos2.Assign(pos3.X(), pos3.Y(), g_pos2.Z());
    if (dec33.Norm() <= dec31.Norm() && dec33.Norm() <= dec32.Norm()
     && dec33.Norm() <= dec13.Norm() && dec33.Norm() <= dec23.Norm()) g_pos3.Assign(pos3.X(), pos3.Y(), g_pos3.Z());

//	std::cout << "object 1: " << g_pos1 << std::endl;
//	std::cout << "object 2: " << g_pos2 << std::endl;
//	std::cout << "object 3: " << g_pos3 << std::endl;
}

void callBackRGraspingPos(const geometry_msgs::Pose & r_gripper_pos)
{
//	g_r_gripper_eular = transformQuarternionToEular(r_gripper_pos.pose.pose.orientation);
	g_r_stamped_pose = r_gripper_pos;
//	std::cout << "g_r_stamped_pose.orientation" << transformQuarternionToEular(g_r_stamped_pose.orientation)*cmn180_PI << std::endl;
}

void callBackLGraspingPos(const geometry_msgs::Pose & l_gripper_pos)
{
//	g_l_gripper_eular = transformQuarternionToEular(l_gripper_pos.pose.pose.orientation);
	g_l_stamped_pose = l_gripper_pos;
}

void callBackJointsStates(const sensor_msgs::JointState & joint_states)
{
	uint num = 0;
	int /*num_l_wrist_flex_joint = 0,*/ num_l_elbow_flex_joint = 0, num_l_shoulder_pan_joint = 0;
	int /*num_r_wrist_flex_joint = 0,*/ num_r_elbow_flex_joint = 0, num_r_shoulder_pan_joint = 0;
	for (num = 0; num < joint_states.name.size(); num++)
	{
//		if (joint_states.name[num] == "l_wrist_flex_joint") num_l_wrist_flex_joint = num;
		if (joint_states.name[num] == "l_elbow_flex_joint") num_l_elbow_flex_joint = num;
		if (joint_states.name[num] == "l_shoulder_pan_joint") num_l_shoulder_pan_joint = num;
//		if (joint_states.name[num] == "r_wrist_flex_joint") num_r_wrist_flex_joint = num;
		if (joint_states.name[num] == "r_elbow_flex_joint") num_r_elbow_flex_joint = num;
		if (joint_states.name[num] == "r_shoulder_pan_joint") num_r_shoulder_pan_joint = num;
	}
	g_l_elbow_flex_joint_effort = joint_states.effort[num_l_elbow_flex_joint];
	g_r_elbow_flex_joint_effort = joint_states.effort[num_r_elbow_flex_joint];
	g_l_shoulder_pan_joint_effort = joint_states.effort[num_l_shoulder_pan_joint];
	g_r_shoulder_pan_joint_effort = joint_states.effort[num_r_shoulder_pan_joint];
//	ROS_INFO("l_elbow_flex_joint effort: %f", joint_states.effort[num_l_elbow_flex_joint]);
//	ROS_INFO("l_wrist_flex_joint effort: %f", joint_states.effort[num_l_wrist_flex_joint]);
//	ROS_INFO("l_shoulder_pan_joint effort: %f", joint_states.effort[num_l_shoulder_pan_joint]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tray_pick_place");
  ros::NodeHandle n;

  ros::Subscriber tray_object_position = n.subscribe("/tray_monitor/race_tray_object_midpoint", 1, callBackTrayMonitor);
  ros::Subscriber r_grasping_pose = n.subscribe("/tray_r_gripper_posture", 1, callBackRGraspingPos);
  ros::Subscriber l_grasping_pose = n.subscribe("/tray_l_gripper_posture", 1, callBackLGraspingPos);
	ros::Subscriber joints_states = n.subscribe("/joint_states", 1, callBackJointsStates);
	
	vct3 offset(atof(argv[1]), atof(argv[2]), atof(argv[3]));
	
  TrayPickPlaceAction TrayPickPlace(ros::this_node::getName(), offset);

  ros::spin();

  return 0;
}
