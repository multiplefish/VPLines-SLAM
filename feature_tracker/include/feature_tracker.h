/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-22 18:27:49
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2022-12-25 13:18:00
 * @FilePath: /VINS-Mono/feature_tracker/src/feature_tracker.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

// void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
template <class T>
void reduceVector(vector<T> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();
    void detect_features(const cv::Mat& grey,
                                    std::vector<cv::Point2f>& features,
                                    std::vector<float>& scores,
                                    int maxCorners,
                                    const cv::Mat& mask);
    void undistortedPoints();

    vector<cv::Point2f> undistortedPoints1();
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;

    vector<float> n_scores;
    
    
    vector<float> forw_scores;

    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    vector<float> forw_un_scores;

    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
};
