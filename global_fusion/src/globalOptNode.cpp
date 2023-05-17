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

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
ros::Publisher pub_gps,pub_global_gps_path;
nav_msgs::Path *global_path;
nav_msgs::Path path;
double last_vio_t = -1;
std::queue<nav_msgs::Odometry::ConstPtr> gpsQueue;
std::mutex m_buf;
void pulish_odom(double time,Eigen::Vector3d global_t,Eigen::Quaterniond global_q)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp.fromSec(time);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();

    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();

    ROS_INFO("gps orientation msg: %f %f %f  %f ",global_q.x(),global_q.y(),global_q.z(),global_q.w());

    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);

        // write result to file
    std::ofstream foutC("/home/fish/vin_ws/src/water-slam/relust/global_fusion.txt", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(9);
    foutC << time/* * 1e9 */<< " ";
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
void pulish_gps(double time,Eigen::Vector3d t,Eigen::Quaterniond q)
{



    nav_msgs::Odometry gps_xyz; 
    gps_xyz.header.stamp.fromSec(time);
    gps_xyz.header.frame_id = "world";
    gps_xyz.child_frame_id = "world";
    gps_xyz.pose.pose.position.x = t.x();
    gps_xyz.pose.pose.position.y = t.y();
    gps_xyz.pose.pose.position.z = t.z();

    if(t(0)>500) return;
    gps_xyz.pose.pose.orientation.x = q.x();
    gps_xyz.pose.pose.orientation.y = q.y();
    gps_xyz.pose.pose.orientation.z = q.z();
    gps_xyz.pose.pose.orientation.w = q.w();


    // pub_gps.publish(gps_xyz);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = gps_xyz.header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = gps_xyz.pose.pose;
    path.header = gps_xyz.header;
    path.header.frame_id = "world";
    path.poses.push_back(pose_stamped);


    pub_gps.publish(gps_xyz);
    pub_global_gps_path.publish(path);
        // write result to file
    std::ofstream foutC("/home/fish/vin_ws/src/water-slam/relust/gnss.txt", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(9);
    foutC << time/* * 1e9 */<< " ";
    foutC.precision(6);
    foutC   << t.x() << " "
            << t.y() << " "
            << t.z() << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << endl;
    foutC.close();
    ROS_INFO("gps orientation msg: %f %f %f  %f ",q.x(),q.y(),q.z(),q.w());
    ROS_INFO("gps position msg: %f %f %f",t.x(),t.y(),t.z());


}

void GPS_callback(const nav_msgs::Odometry::ConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    // vio
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);

    double xyz[3]={0};
    double gps_t=0;
    Eigen:: Quaterniond gps_orientation;

    m_buf.lock();
    while(!gpsQueue.empty())
    {

        nav_msgs::Odometry::ConstPtr GPS_msg = gpsQueue.front();
        gps_t = GPS_msg->header.stamp.toSec();

        
        double latitude = GPS_msg->pose.pose.position.x;
        double longitude = GPS_msg->pose.pose.position.y;
        double altitude = GPS_msg->pose.pose.position.z;
        
        gps_orientation.x() = GPS_msg->pose.pose.orientation.x;
        gps_orientation.y() = GPS_msg->pose.pose.orientation.y;
        gps_orientation.z() = GPS_msg->pose.pose.orientation.z;
        gps_orientation.w() = GPS_msg->pose.pose.orientation.w;

 
        globalEstimator.GPS2XYZ1(latitude, longitude, altitude, xyz);
        


        if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        {
         
            //int numSats = GPS_msg->status.service;
            double pos_accuracy = 0.85;

            
            
            // if(pos_accuracy <= 0)
            //     pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            // if(GPS_msg->status.status > 8)
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 0.01)
            gpsQueue.pop();
        else if(gps_t > t + 0.01)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    pulish_odom(pose_msg->header.stamp.toSec(),global_t,global_q);
    pulish_gps(gps_t,Eigen::Vector3d{xyz[0],xyz[1],xyz[2]},gps_orientation);




}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;

    ros::Subscriber sub_GPS = n.subscribe("/g410_ins/gnss", 1000, GPS_callback);
    // ros::Subscriber sub_GPS = n.subscribe("/gnss", 1000, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 1000, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_gps_path = n.advertise<nav_msgs::Path>("global_gps_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_gps = n.advertise<nav_msgs::Odometry>("gps", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}
