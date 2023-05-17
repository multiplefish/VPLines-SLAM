/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-28 19:14:28
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-05-17 10:08:43
 * @FilePath: /water-slam/vins_estimator/src/initial/initial_ex_rotation.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once 

#include <vector>
#include "../parameters.h"
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <ros/console.h>

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
	InitialEXRotation();
    bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
	Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);
private:
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    int frame_count;

    vector< Matrix3d > Rc;
    vector< Matrix3d > Rimu;
    vector< Matrix3d > Rc_g;
    Matrix3d ric;
};


