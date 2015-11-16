/*@author: Liwei Zhang <lzhang@informatik.uni-hamburg.de>
 * Created on 29.07.2014
 * @author: Bo Sun
 *
 * 2014-12-08:
 * @author: Sebastian Rockel (rockel@informatik.uni-hamburg.de)
 *
 */
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_conversions/pcl_conversions.h>

#include "race_tray_monitor/race_tray_object_midpoint.h"

class LaserScanToPointCloud
{

  protected:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_, object_pos_, scan_filtered_;
    ros::Publisher mark_tray;
    ros::Publisher mark_objects;

  public:

    LaserScanToPointCloud(ros::NodeHandle n) :
      n_(n),
      laser_sub_(n_, "tray_laser_scan", 10),
      laser_notifier_(laser_sub_,listener_, "base_link", 10)
    {
      laser_notifier_.registerCallback(
        boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
      laser_notifier_.setTolerance(ros::Duration(0.01));
      scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/tray_monitor/tray_laser_cloud",1);
      scan_filtered_ = n_.advertise<sensor_msgs::LaserScan>("/tray_monitor/tray_laser_filtered",1);
      object_pos_ = n_.advertise<race_tray_monitor::race_tray_object_midpoint>("/tray_monitor/race_tray_object_midpoint",1);
      mark_tray = n_.advertise<visualization_msgs::Marker>("/tray_monitor/tray_marker", 1);
      mark_objects = n_.advertise<visualization_msgs::MarkerArray>("/tray_monitor/obj_marker", 1);
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
      ros::Time now = scan_in->header.stamp;
      static ros::Time lastTime = now;
      ros::Duration duration = now - lastTime;

      // throttle
      if (duration >= ros::Duration(0.10)) // 5 Hz
      {
        sensor_msgs::PointCloud2 cloud;
        sensor_msgs::PointCloud2 cloud_xy;
        race_tray_monitor::race_tray_object_midpoint tray_objects_position;
        static unsigned int object_cnt = 99;
        // Copy laser scan
        sensor_msgs::LaserScan filtered(*scan_in);
        const double range_cutoff = 0.35; // max langer range of interest
        const double tray_y = 0.64; // in m
        const double tray_x = 0.24; // in m relativ to laser
        const double tray_x_min = 0.02; // in m relativ to laser
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker_tray;

        if ( filtered.ranges.empty() )
          return;

        for (unsigned int r=0; r<filtered.ranges.size(); r++)
          // NaNs and Infs will always return false on a comparison.
          if (filtered.range_min <= filtered.ranges.at(r) &&
              filtered.ranges.at(r) <= range_cutoff )
          {
            // Accept this measurement
          }
          else
          {
            filtered.ranges.at(r) = 0.0;
          }

        // publish tray area marker
        marker_tray.header.frame_id = "base_tray_link";
        marker_tray.header.stamp = scan_in->header.stamp;
        marker_tray.ns = "tray_marker";
        marker_tray.id = 0;
        marker_tray.lifetime = ros::Duration(2.0);
        marker_tray.type = visualization_msgs::Marker::CUBE;
        marker_tray.action = visualization_msgs::Marker::ADD;
        marker_tray.pose.position.x = tray_x/2;
        marker_tray.pose.position.y = 0.0;
        marker_tray.pose.position.z = 0.0;
        marker_tray.pose.orientation.x = 0.0;
        marker_tray.pose.orientation.y = 0.0;
        marker_tray.pose.orientation.z = 0.0;
        marker_tray.pose.orientation.w = 1.0;
        marker_tray.scale.x = tray_x;
        marker_tray.scale.y = tray_y;
        marker_tray.scale.z = 0.004;
        marker_tray.color.r = 0.0f;
        marker_tray.color.g = 0.0f;
        marker_tray.color.b = 1.0f;
        marker_tray.color.a = 0.5f;
        mark_tray.publish(marker_tray);

        // publish filtered laser scan
        scan_filtered_.publish(filtered);

        try
        {
          projector_.transformLaserScanToPointCloud("base_tray_laser_link", filtered, cloud, listener_, range_cutoff);
        }
        catch (tf::TransformException& e)
        {
          std::cout << e.what();
          return;
        }

        // Filtering the data
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(cloud,*cloud_pcl);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(tray_x_min, tray_x);

        pass_x.setInputCloud(cloud_pcl);
        pass_x.filter(*cloud_pcl);

        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-tray_y/2, tray_y/2);

        pass_y.setInputCloud(cloud_pcl);
        pass_y.filter(*cloud_pcl);

        pcl::toROSMsg(*cloud_pcl.get(),cloud_xy);
        scan_pub_.publish(cloud_xy);

        if (cloud_pcl->size() > 0)
        {
          //Creating the KdTree object for the search method of the extraction
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
          tree->setInputCloud(cloud_pcl);


          std::vector<pcl::PointIndices> cluster_indices;
          pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
          ec.setClusterTolerance(0.03); // in m
          ec.setMinClusterSize(10);
          ec.setMaxClusterSize(400);// number of points
          ec.setSearchMethod(tree);
          ec.setInputCloud(cloud_pcl);
          ec.extract(cluster_indices);

          if (object_cnt != cluster_indices.size())
          {
            object_cnt = cluster_indices.size();
            ROS_INFO("%d objects found on the tray!", (int) cluster_indices.size());
          }

          if (cluster_indices.size() > 0)
          {
            unsigned int j = 0;

            tray_objects_position.header.stamp = scan_in->header.stamp;
            tray_objects_position.header.frame_id = "base_tray_link";

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
              {
                cloud_cluster->points.push_back (cloud_pcl->points[*pit]); //*
              }

              cloud_cluster->width = cloud_cluster->points.size ();
              cloud_cluster->height = 1;
              cloud_cluster->is_dense = true;

              pcl::PointXYZ minpt, maxpt, midpt;
              pcl::getMinMax3D<pcl::PointXYZ> (*cloud_cluster,minpt,maxpt);
              minpt.z = 0.0;
              maxpt.z = minpt.z; // 2D
              midpt.x = (minpt.x+maxpt.x)/2;
              midpt.y = (minpt.y+maxpt.y)/2;
              midpt.z = (minpt.z+maxpt.z)/2;

              // add another element
              if (tray_objects_position.pos.size() < j+1)
              {
                geometry_msgs::Point point;
                tray_objects_position.pos.push_back(point);
                tray_objects_position.min.push_back(point);
                tray_objects_position.max.push_back(point);
              }

              tray_objects_position.pos.at(j).x = midpt.x;
              tray_objects_position.pos.at(j).y = midpt.y;
              tray_objects_position.pos.at(j).z = midpt.z;
              tray_objects_position.min.at(j).x = minpt.x;
              tray_objects_position.min.at(j).y = minpt.y;
              tray_objects_position.min.at(j).z = minpt.z;
              tray_objects_position.max.at(j).x = maxpt.x;
              tray_objects_position.max.at(j).y = maxpt.y;
              tray_objects_position.max.at(j).z = maxpt.z;

              ROS_DEBUG("the minimal coordinate of object %d : %4.2f,%4.2f,%4.2f", j+1, minpt.x,minpt.y,minpt.z);
              ROS_DEBUG("the maximal coordinate of object %d : %4.2f,%4.2f,%4.2f", j+1, maxpt.x,maxpt.y,maxpt.z);
              ROS_DEBUG("the midpoint coordinate of object %d : %4.2f,%4.2f,%4.2f", j+1, midpt.x,midpt.y,midpt.z);

              //std::cout << "PointCloud representing the Object" << j+1 <<":"<< cloud_cluster->points.size () << " data points." << std::endl;
              //std::stringstream ss;
              //ss << "cloud_cluster_" << j << ".pcd";
              //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
              j++;
            }
          }
        }
        else if (object_cnt != 0)
        {
          object_cnt = 0;
          ROS_INFO("0 objects found on the tray!");
        }

        object_pos_.publish(tray_objects_position);

        // publish objects marker array
        for (unsigned int i=0; i<tray_objects_position.pos.size(); i++)
        {
          visualization_msgs::Marker marker_object;
          double height=0.05;

          marker_object.header.frame_id = "base_tray_link";
          marker_object.header.stamp = scan_in->header.stamp;
          marker_object.ns = "tray_object"+std::to_string(i);
          marker_object.id = i;
          marker_object.lifetime = ros::Duration(1.0);
          marker_object.type = visualization_msgs::Marker::CUBE;
          marker_object.action = visualization_msgs::Marker::ADD;
          marker_object.pose.position.x = tray_objects_position.pos.at(i).x;
          marker_object.pose.position.y = tray_objects_position.pos.at(i).y;
          marker_object.pose.position.z = tray_objects_position.pos.at(i).z+height/2;
          marker_object.pose.orientation.x = 0.0;
          marker_object.pose.orientation.y = 0.0;
          marker_object.pose.orientation.z = 0.0;
          marker_object.pose.orientation.w = 1.0;
          marker_object.scale.x = std::abs(tray_objects_position.max.at(i).x-tray_objects_position.min.at(i).x);
          marker_object.scale.y = std::abs(tray_objects_position.max.at(i).y-tray_objects_position.min.at(i).y);
          marker_object.scale.z = height;
          marker_object.color.r = 1.0f;
          marker_object.color.g = 0.0f;
          marker_object.color.b = 0.0f;
          marker_object.color.a = 0.8f;

          marker_array.markers.push_back(marker_object);
        }
        mark_objects.publish(marker_array);

        tray_objects_position.pos.clear();

        lastTime = now;
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_tray_monitor");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);

  ros::spin();

  return 0;
}
