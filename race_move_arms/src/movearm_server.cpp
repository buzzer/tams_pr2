#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <race_move_arms/MovearmAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>

class MovearmAction
{
  protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<race_move_arms::MovearmAction> as_;
    std::string action_name_;
    // create messages that are used to published result
    race_move_arms::MovearmResult result_;

    std::string *joint_names; // PR2 arms have 7 joints
    // Declare postures joint target values
    // Names copied from pr2_tuck_arms_action
    float * l_arm_tucked;
    float * r_arm_tucked;
    float * l_arm_untucked;
    float * r_arm_untucked;
    float * r_arm_approach;
    float * r_arm_up;
    float * r_arm_carrying;
    float * l_arm_carrying;
    float * r_arm_side;
    float * l_arm_side;

    float ** r_arm_up_traj;
    float ** r_arm_car_traj;
    float ** l_arm_car_traj;
    float ** r_arm_side_traj;
    float ** l_arm_side_traj;

    // Tuck trajectory
    float ** l_arm_tuck_traj;
    float ** r_arm_tuck_traj;

    // Untuck trajctory
    float ** l_arm_untuck_traj;
    float ** r_arm_untuck_traj;

    // clear trajectory
    float ** l_arm_clear_traj;
    float ** r_arm_clear_traj;


  public:

    MovearmAction(std::string name) :
      as_(nh_, name, boost::bind(&MovearmAction::executeCB, this, _1), false),
      action_name_(name)
    {
      as_.start();

      // Init joint names
      joint_names    = (std::string [7]){
                      "shoulder_pan",
                      "shoulder_lift",
                      "upper_arm_roll",
                      "elbow_flex",
                      "forearm_roll",
                      "wrist_flex",
                      "wrist_roll"};

      // Init joint values
      l_arm_tucked   = (float [7]){0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407};
      r_arm_tucked   = (float [7]){-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436};
      l_arm_untucked = (float [7]){ 0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0};
      r_arm_untucked = (float [7]){-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0};
      r_arm_approach = (float [7]){0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369};
      r_arm_up       = (float [7]){-0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0};

      r_arm_carrying = (float [7]){-0.08, 1.05, -1.45, -1.43, -2.55, -1.52, 2.70};
      l_arm_carrying = (float [7]){0.145, 1.265, 1.836,-1.51,  2.812,-1.415,-1.585};
      r_arm_side     = (float [7]){-2.135,-0.02,-1.64, -2.07,-1.64,  -1.68, 1.40};
      l_arm_side     = (float [7]){2.115, -0.02,1.64,  -2.07, 1.64,  -1.68, 1.40};

      // Init trajectories
      r_arm_up_traj  = (float * [1]){r_arm_up};
      // Tuck trajectory
      l_arm_tuck_traj = (float * [1]){l_arm_tucked};
      r_arm_tuck_traj = (float * [2]){r_arm_approach, r_arm_tucked};

      // Untuck trajctory
      l_arm_untuck_traj = (float * [1]){l_arm_untucked};
      r_arm_untuck_traj = (float * [2]){r_arm_approach, r_arm_untucked};

      // clear trajectory
      l_arm_clear_traj = (float * [1]){l_arm_untucked};
      r_arm_clear_traj = (float * [1]){r_arm_untucked};

      // carrying trajectory
      r_arm_car_traj   = (float * [2]){r_arm_up, r_arm_carrying};
      l_arm_car_traj   = (float * [1]){l_arm_carrying};

      // arm to side trajectory
      r_arm_side_traj  = (float * [1]){r_arm_side};
      l_arm_side_traj  = (float * [1]){l_arm_side};
    }

    ~MovearmAction(void) { }

#if 0/*{{{*/
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

      rightSide->motion_plan_request.goal_constraints.joint_constraints[0].position = -2.135;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[1].position = -0.02;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[2].position = -1.64;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[3].position = -2.07;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[4].position = -1.64;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.68;
      rightSide->motion_plan_request.goal_constraints.joint_constraints[6].position =  1.40;

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
      leftSide->motion_plan_request.goal_constraints.joint_constraints[4].position =  1.64;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[5].position = -1.68;
      leftSide->motion_plan_request.goal_constraints.joint_constraints[6].position =  1.40;

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
#endif/*}}}*/

    void executeCB(const race_move_arms::MovearmGoalConstPtr &goal)
    {
      bool success = true;
      // Left arm goal
      const uint8_t left_armposture = goal->leftarm.armposture;
      // Right arm goal
      const uint8_t right_armposture = goal->rightarm.armposture;
      // Valid postures (Optimization: right and left arm possible poses are equal)
      //const uint8_t armtosideposture   = race_move_arms::MoveArmPosture::ARMTOSIDEPOSTURE;
      //const uint8_t armtuckedposture   = race_move_arms::MoveArmPosture::ARMTUCKEDPOSTURE;
      //const uint8_t armuntuckedposture = race_move_arms::MoveArmPosture::ARMUNTUCKEDPOSTURE;
      //const uint8_t armcarryposture    = race_move_arms::MoveArmPosture::ARMCARRYPOSTURE;

      // make sure that the action hasn't been canceled
      if (!as_.isActive())
      {
        return;
      }
      else
      {
        ROS_WARN("Action has probably be canceled, returning");
      }

      ROS_INFO("Executing, creating movearm action");

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
          //ROS_INFO("Checking left arm goal..");

          //if (left_armposture == armtosideposture)
          //{
              //ROS_INFO("..ARMTOSIDEPOSTURE");
          success = movearm(left_armposture, right_armposture);
          //}
          //else if(left_armposture == armtuckedposture)
          //{
              //ROS_INFO("..ARMTUCKEDPOSTURE");
          //}
          //else if (left_armposture == armuntuckedposture)
          //{
              //ROS_INFO("..ARMUNTUCKEDPOSTURE");
          //}
          //else if (left_armposture == armcarryposture)
          //{
              //ROS_INFO("..ARMCARRYPOSTURE");
          //}
          //else
          //{
              //ROS_INFO("Unknown movearm command for right arm, ignoring");
          //}

          //ROS_INFO("Checking right arm goal..");

          //if (right_armposture == armtosideposture)
          //{
              //ROS_INFO("..ARMTOSIDEPOSTURE");
          //}
          //else if(right_armposture == armtuckedposture)
          //{
              //ROS_INFO("..ARMTUCKEDPOSTURE");
          //}
          //else if (right_armposture == armuntuckedposture)
          //{
              //ROS_INFO("..ARMUNTUCKEDPOSTURE");
          //}
          //else if (right_armposture == armcarryposture)
          //{
              //ROS_INFO("..ARMCARRYPOSTURE");
          //}
          //else
          //{
              //ROS_INFO("Unknown movearm command for right arm, ignoring");
          //}
        }
#if 0/*{{{*/
          ROS_INFO("Executing arm trajectory..");

          switch (goal->movearm)
          {
            case 0:
              ROS_INFO("Using right arm");
              success = moveRightArm();
              break;
            case 1:
              ROS_INFO("Using left arm");
              success = moveLeftArm();
              break;
            case 2:
              ROS_INFO("Using both arms");
              rightArm = true;
              success = moveArmToAbove(rightArm);
              success &= moveBothArms();
              break;
            default:
              ROS_INFO("Unknown movearm command, ignoring");
              break;
          }
        }
      //}
      else if (goal->carrypose == 2) // the other arm is already in carrypose!
      {
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

          switch (goal->movearm)
          {
            case 0:
              ROS_INFO("Using right arm");
              success = moveBothArms(); // trivial case
              break;
            case 1:
              ROS_INFO("Using left arm");
              //move right arm to side first
              rightArm = true;
              success = moveArmToAbove(rightArm);
              success &= moveBothArms(); // trivial case
              break;
            default:
              ROS_INFO("Unknown movearm command, ignoring");
              break;
          }
        }
      }
      else if (goal->carrypose == 3) // move arm to above
      {
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

          switch (goal->movearm)
          {
            case 0:
              ROS_INFO("Using right arm");
              rightArm = true;
              success = moveArmToAbove(rightArm);
              break;
            case 1:
              ROS_INFO("Using left arm");
              rightArm = false;
              success = moveArmToAbove(rightArm);
              break;
            case 2:
              ROS_INFO("Using both arms");
              rightArm = true;
              success = moveArmToAbove(rightArm);
              rightArm = false;
              success &= moveArmToAbove(rightArm);
              break;
            default:
              ROS_INFO("Unknown movearm command, ignoring");
              break;
          }
        }
      }
      else if (goal->carrypose == 4) // move arm to side
      {
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

          switch (goal->movearm)
          {
            case 0:
              ROS_INFO("Using right arm");
              rightArm = true;
              //success = moveArmToAbove(rightArm);
              success = moveArmToSide(rightArm);
              break;
            case 1:
              ROS_INFO("Using left arm");
              rightArm = true;
              success = moveArmToAbove(rightArm);
              rightArm = false;
              //success = moveArmToAbove(rightArm);
              success &= moveArmToSide(rightArm);
              success &= moveRightArm();
              break;
            case 2:
              ROS_INFO("Using both arms");
              rightArm = true;
              //success = moveArmToAbove(rightArm);
              success = moveArmToSide(rightArm);
              rightArm = false;
              //success &= moveArmToAbove(rightArm);
              success &= moveArmToSide(rightArm);
              break;
            default:
              ROS_INFO("Unknown movearm command, ignoring");
              break;
          }
        }
      }
#endif/*}}}*/


      if(success)
      {
        //result_.result = 1;
        //actionlib_msgs::GoalStatus status;
        //result_.result = status.SUCCEEDED;
        //result_.result = actionlib_msgs::GoalStatus::SUCCEEDED;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        //as_.setSucceeded(result_);
        as_.setSucceeded();
      }
      else
      {
        //result_.result = 0;
        //result_.result = actionlib_msgs::GoalStatus::ABORTED;
        ROS_INFO("%s: Failed", action_name_.c_str());
        // set the action state to failed
        //as_.setSucceeded(result_);
        //as_.setAborted(result_);
        //as_.setAborted(result_);
        as_.setAborted();
      }
    }

#if 0/*{{{*/
    // TODO trajectory fails!
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

    bool moveArmToAbove ( bool rightArm )
    {
      bool result  = true;

      if (rightArm == true)
      {
        arm_navigation_msgs::MoveArmGoal rightArmGoal;

        initRightArmGoalAbove(&rightArmGoal);

        //move right arm
        result = moveRightArm (&rightArmGoal);
      }
      else
      {
        arm_navigation_msgs::MoveArmGoal leftArmGoal;

        initLeftArmGoalAbove(&leftArmGoal);

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

      ROS_INFO("Waiting for server..");
      move_arm.waitForServer();
      ROS_INFO("Connected to server");

      arm_navigation_msgs::MoveArmGoal armGoal;
      std::vector<std::string> names(7);

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
        armGoal.motion_plan_request.goal_constraints.joint_constraints[i].position =
          newArmGoal->motion_plan_request.goal_constraints.joint_constraints[i].position;
      }

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
            ROS_INFO("Action finished: %s",state.toString().c_str());
          else
            ROS_INFO("Action failed: %s",state.toString().c_str());

        }
      }
      result = success;

      return result;
    }

#endif/*}}}*/
    //bool moveRightArm ( const arm_navigation_msgs::MoveArmGoal * newArmGoal )
    bool movearm ( const uint8_t leftarm_posture, const uint8_t rightarm_posture)
    {
      actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);

      ROS_INFO("Waiting for server..");
      move_arm.waitForServer();
      ROS_INFO("Connected to server");

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
        //armGoal.motion_plan_request.goal_constraints.joint_constraints[i].position =
          //newArmGoal->motion_plan_request.goal_constraints.joint_constraints[i].position;
      }

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
            ROS_INFO("Action finished: %s",state.toString().c_str());
          else
            ROS_INFO("Action failed: %s",state.toString().c_str());

        }
      }
      result = success;

      return result;
    }
#if 0/*{{{*/


    bool moveBothArms ( void )
    {
      bool result  = true;

      arm_navigation_msgs::MoveArmGoal leftArmGoal;
      arm_navigation_msgs::MoveArmGoal rightArmGoal;

      initBothArmGoals(&leftArmGoal, &rightArmGoal);

      //move left arm first
      result = moveLeftArm (&leftArmGoal);

      //move right arm
      result &= moveRightArm (&rightArmGoal);

      return result;
    }
#endif/*}}}*/
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_movearm");

  MovearmAction movearm(ros::this_node::getName());

  ros::spin();

  return 0;
}
