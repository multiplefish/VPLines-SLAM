#include "../include/linefeature_tracker.h"

#include "../include/parameters.h"
// upm::ELSED elsed;
using namespace std;

LineFeatureTracker::LineFeatureTracker()
{

    allfeature_cnt = 0;
    lines_exit = true;
   
    // EDLineParam param = {5, 1,30, 5, 2, std::move(MIN_LINE_LENGTH), std::move(line_fit_err)};
    // line_detctor = EDLineDetector(param);
    // line_matching = LineMatching();

}


double LineFeatureTracker::segAngle(const Line &s) {
  if (s.line_endpoint[2] > s.line_endpoint[0])
    return std::atan2(s.line_endpoint[3] - s.line_endpoint[1], s.line_endpoint[2] - s.line_endpoint[0]);
  else
    return std::atan2(s.line_endpoint[1]- s.line_endpoint[3] , s.line_endpoint[0] - s.line_endpoint[2]);
}
void LineFeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());

    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    

    vpdetect.init(K_.at<float>(0, 0),K_.at<float>(0, 2),K_.at<float>(1, 2),0.5);
}

vector<Line> LineFeatureTracker::undistortedLineEndPoints()
{
    vector<Line> un_lines;
    un_lines = curframe_->vecLine;
    float fx = K_.at<float>(0, 0);
    float fy = K_.at<float>(1, 1);
    float cx = K_.at<float>(0, 2);
    float cy = K_.at<float>(1, 2);
    for (unsigned int i = 0; i <curframe_->vecLine.size(); i++)
    {
        un_lines[i].line_endpoint[0] = (curframe_->vecLine[i].line_endpoint[0] - cx)/fx;
        un_lines[i].line_endpoint[1] = (curframe_->vecLine[i].line_endpoint[1] - cy)/fy;
        un_lines[i].line_endpoint[2] = (curframe_->vecLine[i].line_endpoint[2] - cx)/fx;
        un_lines[i].line_endpoint[3] = (curframe_->vecLine[i].line_endpoint[3] - cy)/fy;
    }
    return un_lines;
}



void LineFeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_p;

    lines_exit = true;
    cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);

    if (EQUALIZE)   // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img = img;
    }
    TicToc t_li;


    edline_detect(forwframe_->img,forwframe_->vecLine,true);

    if(forwframe_->vecLine.empty()){
        ROS_WARN("no lines");
        lines_exit = false;
        return; 

    }
    for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
    {
        if(first_img)
        {
            forwframe_->lineID.emplace_back(allfeature_cnt++);
            forwframe_->t_cnt.emplace_back(0);
        }
        else
        {
            forwframe_->lineID.emplace_back(-1);   // give a negative id
            forwframe_->t_cnt.emplace_back(0);
        }
    }
    

    if(curframe_->vecLine.size() > 0)
    {
        /* compute matches */
        TicToc t_match;
        std::vector<int> line_prev_to_line_cur;
        match_line_match(forwframe_->img,curframe_->img,forwframe_->vecLine,curframe_->vecLine,line_prev_to_line_cur);
    
        //把没追踪到的线存起来
        for (int k = 0; k < line_prev_to_line_cur.size(); ++k) {
            int mt = line_prev_to_line_cur[k];

            if(mt>0){
                forwframe_->lineID[mt] = curframe_->lineID[k];
                forwframe_->t_cnt[mt] = curframe_->t_cnt[mt] +1;
            }
            
        }

        vector<Line> vecLine_tracked, vecLine_new,verticalLine;
        vector<Eigen::Vector3d> vecLine_vps;
        vector< int > lineID_tracked, lineID_new;
   
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
        {
            
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.emplace_back(forwframe_->vecLine[i]);
                lineID_new.emplace_back(forwframe_->lineID[i]);
                
            }
            
            else
            {
                vecLine_tracked.emplace_back(forwframe_->vecLine[i]);
                lineID_tracked.emplace_back(forwframe_->lineID[i]);

                double angle = segAngle(forwframe_->vecLine[i]);

                if((((angle < 3.14/4.0 && angle > 3*3.14/4.0))||(angle > -3.14/4.0 && angle < -3*3.14/4.0)))
                    verticalLine.push_back(forwframe_->vecLine[i]);
   
            }
        }

        vector<Line> h_Line_new, v_Line_new;
        vector< int > h_lineID_new,v_lineID_new;

        for (size_t i = 0; i < vecLine_new.size(); ++i)
        {
            double angle = segAngle(vecLine_new[i]);
            if((((angle >= 3.14/4.0 && angle <= 3*3.14/4.0))||(angle <= -3.14/4.0 && angle >= -3*3.14/4.0)))
            {
                h_Line_new.emplace_back(vecLine_new[i]);
                h_lineID_new.emplace_back(lineID_new[i]);

            }
            else
            {
                v_Line_new.emplace_back(vecLine_new[i]);
                v_lineID_new.emplace_back(lineID_new[i]);
                verticalLine.push_back(vecLine_new[i]);

            }      
        }
        int h_line,v_line;
        h_line = v_line =0;

        for (size_t i = 0; i < vecLine_tracked.size(); ++i)
        {   
            double angle = segAngle(vecLine_tracked[i]);

            if((((angle >= 3.14/4.0 && angle <= 3*3.14/4.0))||(angle <= -3.14/4 && angle >= -3*3.14/4.0)))
            {
                h_line ++;
            }
            else
            {
                v_line ++;
            }
        }
    //         max_h_lines = fsSettings["max_h_lines"];
    // max_v_lines = fsSettings["max_v_lines"];

        int diff_h = max_h_lines - h_line;
        int diff_v = max_v_lines - v_line;
        if( diff_h > 0)    // 补充线条
        {

            if(diff_h > h_Line_new.size())
                diff_h = h_Line_new.size();

            for (int k = 0; k < diff_h; ++k) 
            {
                vecLine_tracked.emplace_back(h_Line_new[k]);
                lineID_tracked.emplace_back(h_lineID_new[k]);
             
            }
           
        }

        if( diff_v > 0)    // 补充线条
        {

            if(diff_v > v_Line_new.size())
                diff_v = v_Line_new.size();

            for (int k = 0; k < diff_v; ++k)  
            {
                vecLine_tracked.emplace_back(v_Line_new[k]);
                lineID_tracked.emplace_back(v_lineID_new[k]);
                
                
            }            
        }
        
 
        forwframe_->vecLine.swap(vecLine_tracked);
        forwframe_->lineID.swap(lineID_tracked);


        forwframe_->vps.clear();

        if(forwframe_->vecLine.size() > 2)
        {
            vector<Eigen::Vector3d> _vps;
            vector<int> local_vp_ids;
            
            if(verticalLine.size()>2)
                vpdetect.run_vanishing_point_detection(forwframe_->img,verticalLine,forwframe_->vecLine,_vps,local_vp_ids);
            else
                vpdetect.run_vanishing_point_detection(forwframe_->img,forwframe_->vecLine,forwframe_->vecLine,_vps,local_vp_ids);
                
            for(int i=0; i<forwframe_->vecLine.size(); i++)
            {   
                if(local_vp_ids.size()>0)
                {

                    if(local_vp_ids[i] == 3 )
                    {
                        forwframe_->vps.push_back(Vector4d(0.0,0.0,0.0,0.0));
                    } 
                    else
                    {   
                        Vector4d tmp_vp(_vps[local_vp_ids[i]].x(),_vps[local_vp_ids[i]].y(),_vps[local_vp_ids[i]].z(),_vps[local_vp_ids[i]].z()/_vps[local_vp_ids[i]].z());
                        forwframe_->vps.push_back(tmp_vp);
                    }

                }
                else
                {
                    forwframe_->vps.push_back(Vector4d(0.0,0.0,0.0,0.0));
                }

            }

        }
        else
        {
            cout<<"no vp lines"<<std::endl;
            for(int i=0; i<forwframe_->vecLine.size(); i++)
            {  
                forwframe_->vps.push_back(Vector4d(0.0,0.0,0.0,0.0));

            }
        }
    
        assert(forwframe_->vps.size() == forwframe_->vecLine.size() );

            

    }

    curframe_ .swap(forwframe_);


}


void LineFeatureTracker::match_line_match(const cv::Mat &cur_img,const cv::Mat &prev_img,
    std::vector<Line>&cur_lsd,std::vector<Line>&prev_lsd,std::vector<int> &line_prev_to_line_cur)
    {
        static int prev_idx_debug =0;
        static int cur_idx_debug =0;


        int debug_show = 0;
        line_matching.Matching(prev_img,
                                    cur_img,
                                    prev_lsd,
                                    cur_lsd,
                                    line_prev_to_line_cur,
                                    *(cv::Mat*)NULL,
                                    *(cv::Mat*)NULL,
                                    *(cv::Mat*)NULL,
                                    true,//自适应
                                    true,//<滤波
                                    debug_show,//<debug
                                    prev_idx_debug++, cur_idx_debug++);
        if(debug_show > 0)
            cv::waitKey(10);
    }

void LineFeatureTracker::edline_detect(cv::Mat &image,
                           std::vector<Line> &lines,
                           const bool &smoothed)
{

    // cv::Mat img = image.clone();
    line_detctor.EDline(image, lines, smoothed);
    // std::vector<Line> tmp_lines;

    // for(auto l : lines){
    //     // cout<<"line length: "<<l.length <<" ";
    //     if(l.length > MIN_LINE_LENGTH)
    //         tmp_lines.emplace_back(l);
    // }
    // cout<<std::endl;

    // if(!tmp_lines.empty())
    //     lines.swap(tmp_lines);


 }