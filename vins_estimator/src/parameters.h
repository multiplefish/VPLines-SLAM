/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-28 19:14:28
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-16 19:52:56
 * @FilePath: /vin_ws/src/water-slam/vins_estimator/src/parameters.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
const int LINE_MIN_OBS = 5;
const double LOOP_INFO_VALUE = 50.0;
//#define DEPTH_PRIOR
//#define GT
#define UNIT_SPHERE_ERROR



extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string VINS_FOLDER_PATH;

extern int LOOP_CLOSURE;
extern int MIN_LOOP_NUM;
extern int MAX_KEYFRAME_NUM;
extern std::string PATTERN_FILE;
extern std::string VOC_FILE;
extern std::string CAM_NAMES;
extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;



extern int USE_FEATURE_SELECTOR;
extern int MAX_FEATURES;
extern int INIT_THREASHOLD;
extern bool USE_GROUND_TRUTH;


extern int min_length;
extern float vp_factor;
extern float line_factor;


std::string readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_LINE = 4
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
