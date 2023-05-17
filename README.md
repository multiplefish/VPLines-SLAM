##1.Prerequisites
Ubuntu 20.04 LTS (or later) installed on your system
ROS1 (Melodic or Noetic) installed. Refer to the ROS1 documentation for installation instructions.
OpenCV 3.4.2 installed. Refer to the OpenCV documentation for installation instructions.
## 2.Installation
Install ROS1: Follow the instructions provided in the ROS1 documentation to install ROS1 (Melodic or Noetic) on your Ubuntu 20 system.

Install OpenCV 3.4.2: Follow the instructions provided in the OpenCV documentation to install OpenCV 3.4.2 on your Ubuntu 20 system.

## 3.Set up your workspace
Create a catkin workspace: Open a terminal and run the following commands to create a catkin workspace:
bash
```
$ mkdir -p ~/catkin_ws/src
$ git clone https://github.com/multiplefish/VPLines-SLAM.git
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```
## 4. Visual-Inertial Odometry and Pose Graph Reuse on Public datasets
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera. The system also works with [ETH-asl cla dataset](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/). We take EuRoC as the example.
```
$ roslaunch vins_estimator euroc.launch
```
Replace <SLAM_PACKAGE_NAME> with the name of the SLAM package and <SLAM_LAUNCH_FILE> with the name of the launch file.

Visualize the SLAM output: Use RViz or any other visualization tool supported by the SLAM package to visualize the SLAM output.


