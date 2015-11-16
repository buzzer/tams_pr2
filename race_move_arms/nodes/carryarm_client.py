#! /usr/bin/env python

import roslib; roslib.load_manifest('carryarm_actionlib')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the carryarm action, including the
# goal message and the result message.
#import actionlib_tutorials.msg
import carryarm_actionlib.msg
import sys

def carryarm_client(whicharm, whichpose):
    # Creates the SimpleActionClient, passing the type of the action
    # (CarryarmAction) to the constructor.
    client = actionlib.SimpleActionClient('carryarm', carryarm_actionlib.msg.CarryarmAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    #goal = carryarm_actionlib.msg.CarryarmGoal(carrypose=1,carryarm=0)
    goal = carryarm_actionlib.msg.CarryarmGoal(carrypose=whichpose,carryarm=whicharm)
    #goal = carryarm_actionlib.msg.CarryarmGoal(carrypose=1,carryarm=2)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CarryarmResult

def usage():
  return "usage: %s [arm: 0|1|2] [pose: 0|1|2]"%sys.argv[0]

if __name__ == '__main__':
    #rospy.myargv(argv=sys.argv)
    if len(sys.argv) == 2:
      whicharm = int(sys.argv[1])
      carrypose = 1
    elif len(sys.argv) == 3:
      whicharm = int(sys.argv[1])
      carrypose = int(sys.argv[2])
    else:
      print usage()
      sys.exit(0)

    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('carryarm_client_py')
        result = carryarm_client(whicharm, carrypose)
        #print "Result:", ', '.join([str(n) for n in result.result])
        if result:
          print "Result:", result.result
        else:
          print "Action aborted"
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
