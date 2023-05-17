#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "../include/linefeature_tracker.h"

// #include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;

LineFeatureTracker trackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;
bool init_pub = 0;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
    }

    // frequency control, 如果图像频率低于一个值
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    TicToc t_r;

    if (PUB_THIS_FRAME)
    {
        cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat show_img = ptr->image;

        
        frame_cnt++;
        trackerData.readImage(ptr->image.rowRange(0 , ROW));   // rowRange(i,j) 取图像的i～j行

        pub_count++;
        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v

        sensor_msgs::ChannelFloat32 vp_x;
        sensor_msgs::ChannelFloat32 vp_y;
        sensor_msgs::ChannelFloat32 vp_z;
        sensor_msgs::ChannelFloat32 vp_z_inv;

        feature_lines->header = img_msg->header;
        feature_lines->header.frame_id = "world";

        auto &vp = trackerData.curframe_->vps;
            
        vector<set<int>> hash_ids(NUM_OF_CAM);
        
        if(trackerData.lines_exit == true) 
        {

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                auto un_lines = trackerData.undistortedLineEndPoints();
                auto &ids = trackerData.curframe_->lineID;

                        

                for (unsigned int j = 0; j < ids.size(); j++)
                    {

                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::Point32 p;
                        p.x = un_lines[j].line_endpoint[0];
                        p.y = un_lines[j].line_endpoint[1];
                        p.z = 1;

                        feature_lines->points.push_back(p);
                        id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
                        
                        u_of_endpoint.values.push_back(un_lines[j].line_endpoint[2]);
                        v_of_endpoint.values.push_back(un_lines[j].line_endpoint[3]);
                        if(!vp.empty())
                        {
                            vp_x.values.push_back(vp[i](0));
                            vp_y.values.push_back(vp[i](1));
                            vp_z.values.push_back(vp[i](2));
                            vp_z_inv.values.push_back(vp[i](3));
                        }
                        else
                        {
                            vp_x.values.push_back(0);
                            vp_y.values.push_back(0);
                            vp_z.values.push_back(0); 
                            vp_z_inv.values.push_back(0); 
                        }
            
                    }
            }
                
        }
        else
        {

            geometry_msgs::Point32 p;
            p.x = 0;
            p.y = 0;
            p.z = 0;
            feature_lines->points.push_back(p);

            id_of_line.values.push_back(0);

            u_of_endpoint.values.push_back(0);
            v_of_endpoint.values.push_back(0);

            vp_x.values.push_back(0);
            vp_y.values.push_back(0);
            vp_z.values.push_back(0); 
            vp_z_inv.values.push_back(0); 
    
        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);

            
        feature_lines->channels.push_back(vp_x);
        feature_lines->channels.push_back(vp_y);
        feature_lines->channels.push_back(vp_z);
        feature_lines->channels.push_back(vp_z_inv);
            


        ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());


        if (!init_pub)
            init_pub = 1;
        else
            pub_img.publish(feature_lines);

        if (SHOW_TRACK)
        {
            cv::Mat tmp_img;
            cv::cvtColor(trackerData.curframe_->img, tmp_img, CV_GRAY2BGR);

            for(size_t i=0; i<trackerData.curframe_->vecLine.size(); ++i)
            {
                    
            Line l = trackerData.curframe_->vecLine[i];
            int cnt = trackerData.curframe_->t_cnt[i];
            cv::Point2f sp(l.line_endpoint[0],l.line_endpoint[1]);
            cv::Point2f ep(l.line_endpoint[2],l.line_endpoint[3]);

            if(cnt == 1)
                cv::line(tmp_img,sp,ep,cv::Scalar(0,255,0),2,8,0);
            else if(cnt >= 2)
                cv::line(tmp_img,sp,ep,cv::Scalar(0,0,255),2,8,0);
            else if(cnt == 0)
                cv::line(tmp_img,sp,ep,cv::Scalar(255,0,0),2,8,0);
  
            }
            sensor_msgs::ImagePtr msg_lines = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tmp_img).toImageMsg();
            pub_match.publish(*msg_lines);
        }

    }
    sum_time += t_r.toc();
    mean_time = sum_time/frame_cnt;
    ROS_INFO("whole Line feature tracker processing costs: %f", mean_time);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linefeature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    EDLineParam param = {5, 1,30, 5, 2, std::move(MIN_LINE_LENGTH), std::move(line_fit_err)};
    trackerData.line_detctor = EDLineDetector(param);
    trackerData.line_matching = LineMatching();
    
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData.readIntrinsicParameter(CAM_NAMES[i]);

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    pub_img = n.advertise<sensor_msgs::PointCloud>("linefeature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("linefeature_img",1000);

    ros::spin();
    return 0;
}