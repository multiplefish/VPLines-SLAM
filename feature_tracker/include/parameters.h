/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-22 18:27:49
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-18 19:03:03
 * @FilePath: /VINS-Mono/feature_tracker/include/parameters.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

extern float MIN_LINE_LENGTH;
extern float line_fit_err;
extern int max_h_lines;
extern int max_v_lines; 

std::string readParameters(ros::NodeHandle &n);
