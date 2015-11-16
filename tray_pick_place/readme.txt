T1:
roslaunch tray_pick_place pr2_tray_pick_place.launch

T2:
roslaunch tray_pick_place spawn_3mugs.launch
roslaunch tray_pick_place race_prerequisites.launch

T3:
// monitor the object position on the tray, and publish topic (/race_tray_object_midpoint) of the positions (from Liwei)
// change the scan range in file: ros/race/race_pr2/race_pr2_description/urdf/base_v0/base.urdf.xacro
// <xacro:tray_hokuyo_lx30_laser_v0 name="${name}_tray_laser" parent="${name}" ros_topic="tray_scan" update_rate="20" min_angle="-1.4" max_angle="1.4" >
rosrun race_tray_monitor race_tray_monitor 

T4:
rosrun tray_pick_place tray_pick_place_actionserver

T4:
/***************** action for pick and place object on the tray ****************
name: tray_pickup_place
// three parameters to choose hand, object and grasping side:
choose grasping side: 
10 --- to grasp from side
11 --- to grasp from top 
choose hand:
0 --- use right hand
1 --- use left hand
choose object:
0 --- for object 1
1 --- for object 2
2 --- for object 3
*******************************************************************************/
// grasp the object on the right (object 3) from side
rosrun tray_pick_place tray_pick_place_actionclient 10 0 2 

// grasp the object on the left (object 2) from side
rosrun tray_pick_place tray_pick_place_actionclient 10 0 1

// grasp the object in the middle (object 1) from top
// PS: there are some bugs for grasping object from top
rosrun tray_pick_place tray_pick_place_actionclient 11 0 0

