#!/bin/sh

# This script records all PR2 base dynamics and marker topics

rosbag record \
  -o "`rospack find race_pr2_markers`/dynamics" \
   /speedmarker/pr2_base_dynamics \
   /speedmarker/dynamics \
   /speedmarker/pr2_distance_traveled \
   /move_base/goal
