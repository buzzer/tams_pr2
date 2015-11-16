#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('race_joint_states_listener')
import rospy
from race_joint_states_listener.srv import ReturnJointStates
import time
import sys

def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


if __name__ == "__main__":
    #print out the positions, velocities, and efforts of the left arm joints
    joint_names_left = ["l_shoulder_pan_joint",
        "l_shoulder_lift_joint",
        "l_upper_arm_roll_joint",
        "l_elbow_flex_joint",
        "l_forearm_roll_joint",
        "l_wrist_flex_joint",
        "l_wrist_roll_joint"]
    #print out the positions, velocities, and efforts of the right arm joints
    joint_names_right = ["r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint"]

    while(1):
      (position, velocity, effort) = call_return_joint_states(joint_names_left)
      print "left position:", pplist(position)
      print "left velocity:", pplist(velocity)
      print "left effort:", pplist(effort)
      (position, velocity, effort) = call_return_joint_states(joint_names_right)
      print "right position:", pplist(position)
      print "right velocity:", pplist(velocity)
      print "right effort:", pplist(effort)
      time.sleep(1)
