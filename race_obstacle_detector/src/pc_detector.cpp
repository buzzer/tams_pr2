#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class PcFilter
{
  protected:

    ros::Publisher pub_;

  public:

    PcFilter(ros::Publisher & pub)
    {
      pub_ = pub;
    }

    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      static ros::Time last = ros::Time::now();
      ros::Time cur_time = ros::Time::now();

      if ( (cur_time - last) > ros::Duration(1.0) )
      {
        last = cur_time;

        // pass trough
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_input (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);

        //grid filter
        sensor_msgs::PointCloud2 grid_output;
        sensor_msgs::PointCloud2 pass_output;

        // GRID FILTER
        //Perform the actual filtering
        pcl::VoxelGrid<sensor_msgs::PointCloud2> sor ;
        sor.setInputCloud(msg);
        sor.setLeafSize (0.1, 0.1, 0.1);
        sor.filter (grid_output);

        //Conversion
        pcl::fromROSMsg(grid_output,*pass_input.get());

        // PASS THROUGH FILTER
        //Create the filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (pass_input);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_pass.get());

        // Conversion
        pcl::toROSMsg(*cloud_pass, pass_output);

        // Is there a point left, then publish it
        //if (pass_output.data.size() > 0)
        // Publish whether or not cloud is empty
        pub_.publish (pass_output);
      }
    }
};

int main(int argc, char **argv)
{
  std::string rgbd_topic;
  ros::init(argc, argv, "pc_detector");

  ros::NodeHandle n;
  ros::NodeHandle private_nh_("~");
  private_nh_.getParam("rgbd_topic", rgbd_topic);

  ROS_INFO("Subscribing to topic %s", rgbd_topic.c_str());
  ros::Publisher filtered_pub = n.advertise<sensor_msgs::PointCloud2>("/pc_detector/rgbd_out", 10);
  PcFilter pc_filter(filtered_pub);
  ros::Subscriber sub = n.subscribe(rgbd_topic.c_str(), 10, &PcFilter::pointsCallback, &pc_filter);

  ros::spin();

  return 0;
}
