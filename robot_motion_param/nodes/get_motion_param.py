#!/usr/bin/env python
#
# 2014-08-04 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
# Provides a service interface to read the robot motion parameters
# uses the dynamic_reconfigure client
#
import roslib; roslib.load_manifest('robot_motion_param')
from robot_motion_param.srv import *
import rospy
import dynamic_reconfigure.client

def get_params(node, timeout=None):
  client = dynamic_reconfigure.client.Client(node, timeout=timeout)
  return client.get_configuration(timeout=timeout)

def handle_request(req):
  config = get_params('/move_base_node/DWAPlannerROS', 1.0)
  max_vel = config['max_trans_vel']
  max_acc = config['acc_lim_x']
  max_jerk = 0
  print "Reading motion parameters (vel, acc, jerk).. {0:4.2f} {1:4.2f} {2:4.2f}".format(max_vel, max_acc, max_jerk)
  #print config
  return GetMotionParamResponse(max_vel, max_acc, max_jerk)

def server():
  rospy.init_node('get_robot_motion_param')
  s = rospy.Service('get_motion_param', GetMotionParam, handle_request)
  print "Ready to read motion parameters.."
  rospy.spin()

if __name__ == "__main__":
  client = dynamic_reconfigure.client.Client('/move_base_node/DWAPlannerROS')
  server()
