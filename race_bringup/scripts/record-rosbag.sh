#!/bin/sh

# this script records all blackboard topics, but only works if the blackboard
# is already started. if you want to start this script before the blackboard,
# use record-rosbag-static.sh.

rosbag record \
   /area_markers \
   /attached_objects \
   /attached_objects_array \
   $(rostopic list /blackboard) \
   /base_scan_throttled \
   /display_joint_goal \
   /display_path \
   /gazebo/model_states \
   /generated_shop2_plan \
   /grasp_markers \
   /interpolation_markers \
   /map \
   /move_base_node/DWAPlannerROS/global_plan \
   /move_base_node/DWAPlannerROS/local_plan \
   /move_base_node/NavfnROS/plan \
   /move_base_node/current_goal \
   /move_base_node/local_costmap/inflated_obstacles \
   /move_base_node/local_costmap/obstacles \
   /move_base_node/local_costmap/robot_footprint \
   /move_left_arm_markers \
   /move_right_arm_markers \
   /object_manipulator/grasp_execution_markers \
   /particlecloud \
   /perception/scene_manager/object_detection_array \
   /planning_scene_markers \
   /point_cluster_grasp_planner_markers \
   /tabletop_objects \
   /tabletop_object_recognition_markers \
   /tabletop_segmentation_markers \
   /tf \
   /visualization_marker \
   /head_mount_kinect/rgb/image_raw \
   /head_mount_kinect/rgb/camera_info \
   /head_mount_kinect/depth_registered/image_raw \
   /head_mount_kinect/depth_registered/camera_info \
   /r_gripper_controller/state \
   /r_gripper_controller/command \
   /l_gripper_controller/state \
   /l_gripper_controller/command \
   /object_search_marker \
   /next_best_view_marker
