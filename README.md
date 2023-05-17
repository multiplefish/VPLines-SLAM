# VPLines-SLAM
## 1.Prerequisites
- Ubuntu 20.04 LTS (or later) installed on your system
- ROS1 (Melodic or Noetic) installed. Refer to the ROS1 documentation for installation instructions.
- OpenCV 3.4.2 installed. Refer to the OpenCV documentation for installation instructions.

## 2.Installation
- Install ROS1: Follow the instructions provided in the ROS1 documentation to install ROS1 (Melodic or Noetic) on your Ubuntu 20 system.

- Install OpenCV 3.4.2: Follow the instructions provided in the OpenCV documentation to install OpenCV 3.4.2 on your Ubuntu 20 system.

- This repository contains the SLAM of Algorithm, which is designed to address the challenges of shadow point removal, attention mechanism, time synchronization with multi-sensor fusion, and GNSS integration.

## 3.Features

- **Vanishing Point Integration**: Algorithm Title incorporates the Vanishing Point technique to analyze and utilize the geometric convergence points in the scene. By utilizing this information, the algorithm achieves a substantial improvement in the precision of localization.

- **Attention Mechanism**: The algorithm utilizes an attention mechanism to selectively focus on important features, improving the overall performance and robustness of the system.

- **Time Synchronization with Multi-Sensor Fusion**: Algorithm Title incorporates time calibration techniques to synchronize data from multiple sensors, enabling precise fusion and alignment of information.

- **GNSS Integration**: The algorithm seamlessly integrates Global Navigation Satellite System (GNSS) data, providing accurate positioning and aiding in the overall fusion process.

## 4.Set up your workspace
- Create a catkin workspace: Open a terminal and run the following commands to create a catkin workspace:
bash
```
$ mkdir -p ~/catkin_ws/src
$ git clone https://github.com/multiplefish/VPLines-SLAM.git
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```
## 5. Visual-Inertial Odometry and Pose Graph Reuse on Public datasets
- Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera. The system also works with [ETH-asl cla dataset](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/). We take EuRoC as the example.
```
$ roslaunch vins_estimator euroc.launch
```
- Replace <SLAM_PACKAGE_NAME> with the name of the SLAM package and <SLAM_LAUNCH_FILE> with the name of the launch file.
## 6.Demo
- [Euroc Demo]([https://www.bilibili.com/video/BV1Ns4y1M76b/?share_source=copy_web&vd_source=ee24d784b726ef1b7e66ebb6e3cfe4fc](https://www.bilibili.com/video/BV1tk4y1L7fN/?vd_source=1cea8d2ef05dd4472ff8577febae73a7))
Click the image above to watch the video.

