//2014-04-14 Sebastian Rockel (rockel@informatik.uni-hamburg.de)
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TwistCmd
{
  protected:

    ros::Publisher * twist_pub;

  public:

    TwistCmd(ros::Publisher * marker)
    {
      twist_pub = marker;
    }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "pr2_cmd_goals");
  ros::NodeHandle n;

  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/navigation/cmd_vel", 1);

  TwistCmd speedmarker(&twist_pub);

  ros::spin();

  return 0;
}
