#!/usr/bin/env python
#
# 2014-08-04 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
# Provides a service interface to changing the robot motion parameters
# uses the dynamic_reconfigure client
#
import roslib; roslib.load_manifest('robot_motion_param')
from robot_motion_param.srv import *
import rospy
import dynamic_reconfigure.client

client = dynamic_reconfigure.client.Client('/move_base_node/DWAPlannerROS')

def handle_request(req):
  vel = req.max_vel
  acc = req.max_acc
  jerk= req.max_jerk
  print "Setting motion parameters (vel, acc, jerk).. {0:4.2f} {1:4.2f} {2:4.2f}".format(vel, acc, jerk)
  params = {
      'max_trans_vel' : vel,
      'max_vel_x' : vel,
      'acc_lim_x' : acc, # TODO consider angular velocity!
      'acc_lim_y' : acc,
      'acc_lim_theta' : acc+0.7, # angular limit is by default higher than linear
      }
  config = client.update_configuration(params)
  return True

def server():
  rospy.init_node('robot_motion_param')
  s = rospy.Service('set_motion_param', MotionParam, handle_request)
  print "Ready to set motion parameters.."
  rospy.spin()

if __name__ == "__main__":
  server()
