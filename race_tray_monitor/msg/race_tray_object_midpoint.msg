# Message to publish objects detected on the robot tray
# Detection is based on clustering points in the laser scan
std_msgs/Header header
#center point of a cluster
geometry_msgs/Point[] pos 
#minimum
geometry_msgs/Point[] min
#maximum
geometry_msgs/Point[] max
