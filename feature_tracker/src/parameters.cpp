/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-22 18:27:49
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-18 19:03:23
 * @FilePath: /VINS-Mono/feature_tracker/src/parameters.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "../include/parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

float MIN_LINE_LENGTH;
float line_fit_err;
int max_h_lines;
int max_v_lines; 

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

std::string readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];

    max_h_lines = fsSettings["max_h_lines"];
    max_v_lines = fsSettings["max_v_lines"];

    // max_h_lines: 35 
    // max_v_lines: 35 

    MIN_LINE_LENGTH = std::move(fsSettings["min_line_length"]);
    line_fit_err = std::move(fsSettings["line_fit_err"]);
    std::cout<<MIN_LINE_LENGTH<<" "<<line_fit_err<<std::endl;
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();

    return config_file;


}
