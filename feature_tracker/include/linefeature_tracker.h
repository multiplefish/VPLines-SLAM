/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-24 15:45:00
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-15 13:47:29
 * @FilePath: /PLVINS_ws/src/PL-VINS/feature_tracker/src/linefeature_tracker.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#pragma once

#include <iostream>
#include <queue>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "parameters.h"
#include "tic_toc.h"

#include <eigen3/Eigen/Dense>
#include <opencv2/features2d.hpp>
// #include "ELSED.h"?
#include "line.h"
#include "edline_detector.h"
#include "line_matching.h"
#include "../include/vanishing_point_detection.h"

// #include "../src/line_match/src/line.h"
// #include "../src/line_match/src/edline_detector.h"
// #include "../src/line_match/src/line_matching.h"
// #include "../include/vanishing_point_detection.h"


// using namespace cv::line_descriptor;
using namespace std;
using namespace cv;
using namespace camodocal;



class FrameLines
{
public:
    Mat img;
    vector<Line> vecLine;
    vector< int > lineID;
    vector<Eigen::Vector4d> vps;
    vector<int> vp_idx;

    vector< int >  t_cnt;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;

class LineFeatureTracker
{
  public:
    LineFeatureTracker();

    void readIntrinsicParameter(const string &calib_file);

    vector<Line> undistortedLineEndPoints();
    void readImage(const cv::Mat &_img);
    FrameLinesPtr curframe_, forwframe_;
    vanishing_point_detection vpdetect;

    bool lines_exit ;
    LineMatching line_matching ;  
    EDLineDetector line_detctor;
    private:

    void match_line_match(const cv::Mat &cur_img,const cv::Mat &prev_img,
    std::vector<Line>&cur_lsd,std::vector<Line>&prev_lsd,std::vector<int> &line_prev_to_line_cur);
    double segAngle(const Line &s);
    void edline_detect(cv::Mat &image,
                           std::vector<Line> &lines,
                           const bool &smoothed);
    cv::Mat undist_map1_, undist_map2_ , K_;
    camodocal::CameraPtr m_camera;       // pinhole camera

    int frame_cnt;
    int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值

    

};
