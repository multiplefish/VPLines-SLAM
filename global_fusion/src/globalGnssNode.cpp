/*
 * @Author: fish 973841082@qq.com
 * @Date: 2023-02-17 15:49:00
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-17 18:10:43
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
#include "globalOpt.h"
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

GlobalOptimization globalEstimator;
ros::Publisher pub_gps;
nav_msgs::Path path;
double last_vio_t = -1;
std::queue<nav_msgs::Odometry::ConstPtr> gpsQueue;
std::mutex m_buf;

void GPS_callback(const nav_msgs::Odometry::ConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void process()
{

    while(true)
    {
        double t = 0;
        m_buf.lock();
        while(!gpsQueue.empty())
        {
            nav_msgs::Odometry::ConstPtr gps_msg = gpsQueue.front();
            double gps_t = gps_msg->header.stamp.toSec();
            t = gps_t;
            double latitude = gps_msg->pose.pose.position.x;
            double longitude = gps_msg->pose.pose.position.y;
            double altitude = gps_msg->pose.pose.position.z;
            
            globalEstimator.inputGPS(gps_t,latitude, longitude, altitude, 0.5);
            double xyz[3];
                
            globalEstimator.GPS2XYZ1(latitude, longitude, altitude, xyz);
            vector<double> tmp{xyz[0], xyz[1], xyz[2]};

            ROS_INFO("gps msg: %f %f %f ",xyz[0],xyz[1],xyz[2]);
            nav_msgs::Odometry gps_xyz; 
            gps_xyz.header = gps_msg->header;
            gps_xyz.header.frame_id = "world";
            gps_xyz.child_frame_id = "world";
            gps_xyz.pose.pose.position.x = xyz[0];
            gps_xyz.pose.pose.position.y = xyz[1];
            gps_xyz.pose.pose.position.z = xyz[2];

            gps_xyz.pose.pose.orientation.x = gps_msg->pose.pose.orientation.x;
            gps_xyz.pose.pose.orientation.y = gps_msg->pose.pose.orientation.y;
            gps_xyz.pose.pose.orientation.z = gps_msg->pose.pose.orientation.z;
            gps_xyz.pose.pose.orientation.w = gps_msg->pose.pose.orientation.w;
            
            pub_gps.publish(gps_xyz);
            // write result to file
            // std::ofstream foutC("/home/fish/vin_ws/src/water-slam/relust/gt.txt", ios::app);
            // foutC.setf(ios::fixed, ios::floatfield);
            // foutC.precision(9);
            // foutC << gps_xyz.header.stamp.toSec()/* * 1e9 */<< " ";
            // foutC.precision(6);
            // foutC   << gps_xyz.pose.pose.position.x << " "
            //         << gps_xyz.pose.pose.position.y << " "
            //         <<gps_xyz.pose.pose.position.z << " "
            //         << gps_xyz.pose.pose.orientation.x << " "
            //         << gps_xyz.pose.pose.orientation.y << " "
            //         << gps_xyz.pose.pose.orientation.z<< " "
            //         << gps_xyz.pose.pose.orientation.w << endl;
            // foutC.close();
            gpsQueue.pop();
            break;

        }
        m_buf.unlock();
        Eigen::Vector3d global_t;
        Eigen:: Quaterniond global_q;
        globalEstimator.getGlobalOdom(global_t, global_q);


            // write result to file
        std::ofstream foutC("/home/fish/vin_ws/src/water-slam/relust/gt.txt", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << t/* * 1e9 */<< " ";
        foutC.precision(6);
        foutC   << global_t.x() << " "
                << global_t.y() << " "
                << global_t.z() << " "
                << global_q.x() << " "
                << global_q.y() << " "
                << global_q.z() << " "
                << global_q.w() << endl;
        foutC.close();



            
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    // global_path = &globalEstimator.global_path;

    ros::Subscriber sub_GPS = n.subscribe("/g410_ins/gnss", 1000, GPS_callback);
    pub_gps = n.advertise<nav_msgs::Odometry>("gps_write", 100);
    std::thread gnss_thread{process};
    ros::spin();
    return 0;
}
