#!/bin/sh

# This script records all PR2 base dynamics and marker topics

rosbag record \
  -o "`rospack find race_pr2_markers`/dynamics_obj" \
   /object_monitor/obj_dev \
   /obj_reset
