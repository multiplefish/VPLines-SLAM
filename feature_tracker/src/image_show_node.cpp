#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 
#include <nav_msgs/Odometry.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <sstream>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>




#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "../include/parameters.h"

#include "line.h"
using namespace std;



camodocal::CameraPtr m_camera;
cv::Mat undist_map1_, undist_map2_ , K_;

ros::Publisher pub_point_line;
sensor_msgs::PointCloudConstPtr linefeature;

vector<Line> undistortedLineEndPoints(const vector<Line> &un_lines)
  {
        vector<Line> lines(un_lines.size());
        // un_lines = curframe_->vecLine;
        float fx = K_.at<float>(0, 0);
        float fy = K_.at<float>(1, 1);
        float cx = K_.at<float>(0, 2);
        float cy = K_.at<float>(1, 2);
        for (unsigned int i = 0; i < un_lines.size(); i++)
        {
            lines[i].line_endpoint[0] = (un_lines[i].line_endpoint[0]*fx + cx);
            lines[i].line_endpoint[1] = (un_lines[i].line_endpoint[1]*fy + cy);
            lines[i].line_endpoint[2] = (un_lines[i].line_endpoint[2]*fx + cx);
            lines[i].line_endpoint[3] = (un_lines[i].line_endpoint[3]*fy + cy);
        }
        return lines;
}

void callback(const sensor_msgs::PointCloudConstPtr &point_feature_msg,
              const sensor_msgs::PointCloudConstPtr &line_feature_msg,
              const sensor_msgs::ImageConstPtr& img_msg) 
{
    //在这里把点和线画出来就可以了
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
    cv::Mat img1;
    cv::cvtColor(show_img, img1, cv::COLOR_GRAY2BGR);


    for(int i=0; i<point_feature_msg->points.size(); i++)
    { 
        // m_camera->
        cv::Point endPoint = cv::Point(point_feature_msg->channels[1].values[i], point_feature_msg->channels[2].values[i]);

        float prob = point_feature_msg->channels[5].values[i];
        // cout<<"prob"<<prob<<std::endl;
        auto scoreColor = cv::Scalar(255 * (1 - prob), 0, 255 * prob);
        cv::circle(img1, endPoint, 2, scoreColor, 2);
    }

    cv::remap(img1, show_img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    float vx,vy,vz;
    vector<Line> un_lines;
    for(int i=0; i<line_feature_msg->points.size(); i++)
    { 
      Line l;

      l.line_endpoint[0] = line_feature_msg->points[i].x;
      l.line_endpoint[1] = line_feature_msg->points[i].y;
      l.line_endpoint[2] = line_feature_msg->channels[1].values[i];
      l.line_endpoint[3] = line_feature_msg->channels[2].values[i];

      if(line_feature_msg->channels[5].values[i] != NULL){
        vx = line_feature_msg->channels[3].values[i];
        vy = line_feature_msg->channels[4].values[i];
        vz = line_feature_msg->channels[5].values[i];
        // cout<<"vx vy vz "<<vx<<" "<<vy<<" "<<vz<<std::endl;
      }
 


      un_lines.emplace_back(l);
    }
    vector<Line> lines = undistortedLineEndPoints(un_lines);
    for(int i=0; i<lines.size(); i++)
    { 
      cv::Point sp(lines[i].line_endpoint[0], lines[i].line_endpoint[1]);
      cv::Point ep(lines[i].line_endpoint[2], lines[i].line_endpoint[3]);
      auto scoreColor = cv::Scalar(0, 0, 255 );
      cv::line(show_img,sp,ep,scoreColor,2,8,0);
    }
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show_img).toImageMsg();
    pub_point_line.publish(output_msg);

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "imshow");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);                 
  
  string calib_file = readParameters(n);
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

  K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    

  message_filters::Subscriber<sensor_msgs::PointCloud> point_feature_sub(n, "/feature_tracker/feature",1000); 
  message_filters::Subscriber<sensor_msgs::PointCloud> line_feature_sub(n, "/linefeature_tracker/linefeature", 1000);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/cam0/image_raw", 1000);

  pub_point_line = n.advertise<sensor_msgs::Image>("/pointline_image",1000);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud,sensor_msgs::PointCloud,sensor_msgs::Image> MySyncPolicy;
  
 
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_feature_sub, line_feature_sub,image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  
  ros::spin();

  return 0;
}
