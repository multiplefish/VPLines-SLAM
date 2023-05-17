/*
 * @Description: 
 * @Version: 
 * @Author: 
 * @Date: 2022-11-24 09:39:49
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-14 14:03:18
 */
#pragma once
#include <string>
#include <algorithm>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_models/CameraFactory.h>

#include "line.h"
// #include "edline_detector.h"
// #include "line_matching.h"

using namespace std;
using namespace Eigen;

// Hypotheses
struct Line_info
{
    Eigen::Vector3d para;
    double length;
	double orientation;
};
struct Param
{
    float fx;
    float cx;
    float cy;
    float fy;
};


class vanishing_point_detection
{
private:
    camodocal::CameraPtr m_camera; ///< geometric camera model
    void getVPHypVia2Lines(vector<Line> cur_keyLine, vector<Eigen::Vector3d> &para_vector, vector<double> &length_vector, 
        vector<double> &orientation_vector, std::vector<std::vector<Eigen::Vector3d> > &vpHypo );
    void getSphereGrids(vector<Line> cur_keyLine, vector<Vector3d> &para_vector, 
        vector<double> &length_vector, vector<double> &orientation_vector, std::vector<std::vector<double> > &sphereGrid );
    void getBestVpsHyp( std::vector<std::vector<double> > &sphereGrid, 
                            std::vector<std::vector<Vector3d> >  &vpHypo,
                            std::vector<Vector3d> &vps  );
    void lines2Vps(vector<Line> cur_keyLine, double thAngle, std::vector<Vector3d> &vps, std::vector<std::vector<int> > &clusters, vector<int> &vp_idx);
    void drawClusters( cv::Mat &img, std::vector<Line> &lines, std::vector<std::vector<int> > &clusters);
    void lineinfo(vector<Line> cur_keyLine, vector<Eigen::Vector3d> &para_vector, vector<double> &length_vector, vector<double> &orientation_vector);
    double segAngle(const Line &s);
    Eigen::Matrix3d R0;
    Eigen::Matrix3d R1;
    int frame_count = 0;
    vector<Line> lx;
    vector<Line> ly;
    vector<Line> lz;

public:
    void run_vanishing_point_detection(const cv::Mat &img, std::vector<Line > &lines, std::vector<Line > &all_lines,  
            std::vector<Eigen::Vector3d> &vps,vector<int> &local_vp_ids );
    vanishing_point_detection(){};
    ~vanishing_point_detection() = default;
    void init(float _f,float _cx,float _cy,double _noiseRatio);

    cv::Point2d pp;
	double f;
	double noiseRatio;
    



};


