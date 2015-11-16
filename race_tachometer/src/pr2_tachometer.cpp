/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013
 *    Denis Klimentjew <klimentjew@informatik.uni-hamburg.de>
 *
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "race_tachometer/tachometer.h"

using std::stringstream;
using std::string;
using std::ifstream;
using std::ofstream;
using std::cout;
/**
 * Tachometer for the PR2
 */

class RobotTachometer
{
  public:

    const char* file_path;

    ros::Time begin;

    stringstream sstr, sstr1, sstr2, sstr3, sstr4;

    float x_pose_old, y_pose_old, time_d, time, speed_av, speed_max;
    float distance, distance_local, time_global;
    float x_pose, y_pose, speed; //x_orient, y_orient, z_orient, w_orient;

    void getSpeed(const geometry_msgs::PoseWithCovarianceStamped& msg);

    void speed_marker_gen();

    void init();

    int write_back();

}; //RobotTachometer


//Initialisation and read the parameters from file
 void RobotTachometer::init()
 {
  int count = 0;
  string line;
  ifstream data (file_path);
  if (data.is_open())
  {
    while (! data.eof() )
    {
      if(count == 0)
      {
        getline (data,line);
        sstr << line;
        sstr >> distance;
      }

      if(count == 1)
      {
        getline (data,line);
        sstr1 << line;
        sstr1 >> speed_av;
      }

      if(count == 2)
      {
        getline (data,line);
        sstr2 << line;
        sstr2 >> speed_max;
      }

      if(count == 3)
      {
        getline (data,line);
        sstr3 << line;
        sstr3 >> time;
      }

      if(count == 4)
      {
        getline (data,line);
        sstr4 << line;
        sstr4 >> time_global;
      }
      // Ignore other lines

      count++;
    }
   data.close();
   }
   else
   {
     cout << "Can't open file";
   }

   ROS_DEBUG("INIT DISTANCE %5.5f speed av %5.5f speed max %5.5f run time %5.5f global time %5.5f", distance, speed_av, speed_max, time, time_global); 

   begin.sec = 0;
   begin.nsec = 0;
   x_pose_old=0;
   y_pose_old=0;
   distance_local=0;
 }

 void RobotTachometer::getSpeed(const geometry_msgs::PoseWithCovarianceStamped& msg)
 {

  if(begin.sec == 0 && begin.nsec == 0)
  {
    speed = 0;
    x_pose = msg.pose.pose.position.x;
    y_pose = msg.pose.pose.position.y;
    //ROS_INFO("\n speed: %3.4f m/s", speed);
  }

  else
  {

    x_pose = msg.pose.pose.position.x;
    y_pose = msg.pose.pose.position.y;
    //ROS_INFO("\n x_pose: %3.4f y_pose: %3.4f \n\n", x_pose, y_pose);

    //Velocity calculation
    distance_local = sqrt(pow(x_pose - x_pose_old, 2) + pow(y_pose - y_pose_old, 2));
    distance += fabs(distance_local);
    time_d = (msg.header.stamp.sec - begin.sec) + ((msg.header.stamp.nsec - begin.nsec)*pow(10, -9));
    speed = (distance_local / time_d);

    //Global time
    time_global += time_d;

    if(speed > 0.01)  // running time of the robot
      time += time_d;

    if(speed > speed_max) //max velocity
      speed_max = speed;


    ROS_DEBUG("\n speed: %3.4f m/s distance: %5.5f m run_time: %5.5f global_time: %5.5f \n\n", speed, distance, time, time_global);

    //Robot orientation (quaternion)
    //x_orient = msg.pose.pose.orientation.x;
    //y_orient = msg.pose.pose.orientation.y;
    //z_orient = msg.pose.pose.orientation.z;
    //w_orient = msg.pose.pose.orientation.w;
    //ROS_INFO("\n x_orient: %3.4f \n y_orient: %3.4f \n z_orient: %3.4f \n w_orient: %3.4f", x_orient, y_orient, z_orient, w_orient);
  }
    //Postprocessing
    x_pose_old = x_pose;
    y_pose_old = y_pose;
    begin.sec = msg.header.stamp.sec;
    begin.nsec = msg.header.stamp.nsec;

}

int RobotTachometer::write_back()
{
      ofstream write_file;
      write_file.open (file_path);
      write_file << distance << std::endl;
      write_file << distance / time << std::endl;
      write_file << speed_max << std::endl;
      write_file << time << std::endl;
      write_file << time_global << std::endl;
      write_file << "Format is: distance [m], average speed [m/s], max. speed [m/s], driving time [s], total time [s]"
        << std::endl;
      write_file.close();
      return 0;
}

int main(int argc, char **argv)
{
  race_tachometer::tachometer tachometer_msg;
  string path;
  ros::init(argc, argv, "tachometer");

  ros::NodeHandle n("~");
  ros::Publisher pub = n.advertise<race_tachometer::tachometer>("tachometer", 1);
  RobotTachometer robotTachometer;

  n.getParam("file_path", path);
  robotTachometer.file_path = path.c_str();

  ROS_INFO("PR2 tachometer: data will be read and written to file: %s", path.c_str());

  robotTachometer.init();

  ros::Subscriber sub = n.subscribe(
      "/robot_pose_ekf/odom_combined",
      1000,
      &RobotTachometer::getSpeed,
      &robotTachometer);

  ros::Rate s(0.5);
  s.sleep();

  ros::Rate r(1); // 1 hz

  while (ros::ok())
  {
    tachometer_msg.distance = robotTachometer.distance;
    tachometer_msg.speed_av= robotTachometer.speed_av;
    tachometer_msg.speed_max = robotTachometer.speed_max;
    tachometer_msg.time_mv = robotTachometer.time;
    tachometer_msg.time_global = robotTachometer.time_global;

    pub.publish(tachometer_msg);

    ros::spinOnce();
    r.sleep();
  }

  robotTachometer.write_back();
  //TODO never reaches here
  ROS_INFO("\nPR2 tachometer: Write back successful \n");
  return 0;
}

