/*
 * @Author: fish 973841082@qq.com
 * @Date: 2023-02-17 15:49:00
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-17 18:17:58
 * @FilePath: /water-slam/global_fusion/src/globalGnssNode.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <thread>

using namespace std;


void GPS_callback(const nav_msgs::Odometry::ConstPtr &gps_msg)
{
    
    // //printf("gps_callback! \n");
    // m_buf.lock();
    // gpsQueue.push(GPS_msg);
    // m_buf.unlock();

    // write result to file
    std::ofstream foutC("/home/fish/vin_ws/src/water-slam/relust/gt.txt", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(9);
    foutC << gps_msg->header.stamp.toSec()/* * 1e9 */<< " ";
    foutC.precision(6);
    foutC   << gps_msg->pose.pose.position.x << " "
                << gps_msg->pose.pose.position.y << " "
                << gps_msg->pose.pose.position.z << " "
                << gps_msg->pose.pose.orientation.x << " "
                << gps_msg->pose.pose.orientation.y << " "
                << gps_msg->pose.pose.orientation.z << " "
                << gps_msg->pose.pose.orientation.w << endl;
    foutC.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_sub");
    ros::NodeHandle n("~");

    ros::Subscriber sub_gt = n.subscribe("/globalEstimator/gps_write", 1000, GPS_callback);

    ros::spin();
    return 0;
}
