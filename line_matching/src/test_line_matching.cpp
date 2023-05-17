/*
 * @Description: 
 * @Version: 
 * @Author: 
 * @Date: 2022-11-20 12:17:20
 * @LastEditors: fish
 * @LastEditTime: 2022-11-20 22:07:25
 */
#include <sys/time.h>

#include <iostream>

#include "line_matching.h"
using namespace std;

void test() {
    // provide the reference transform matrix between two frames. It will get better result if given.
    bool use_reference_T = false;
    // 0: close  1: show debug information 2: show more debug information
    int debug_show = 2;
    // illumination adapt for KLT tracker
    bool illumination_adapt = true;
    // remove outlider
    bool topological_filter = true;

    int idx_cout = 0;
    int idx_ref = 1;
    int idx_cur = idx_ref + 1;

    // ksize, sigma, gradientThreshold, anchorThreshold, scanIntervals, minLineLen, lineFitErrThreshold
    // EDLineParam param = {5, 1.0, 30, 5, 2, 25, 2.0};  //1.4
    EDLineParam param = {5, 1.0, 30, 5, 2, 30, 1.8};
    EDLineDetector line_detctor = EDLineDetector(param);
    LineMatching line_matching = LineMatching();
    float filter_distance = 3.0;

    cv::Mat img_cur,prev_image;
    std::vector<Line> lines_cur,line_prev;
    std::vector<int> line_prev_to_line_cur;


    std::vector<std::string> image_path;
    for(int i=0; i<3418;++i){
        std::string path = "/home/fish/code/data/data_image/"+to_string(i)+".jpg";
        image_path.emplace_back(path);
    }

    char key;
    for(int i=0; i<image_path.size(); ++i){

        img_cur = cv::imread(image_path[i], 0);
        // cv::imshow("s",img_cur);
        if(!prev_image.empty()){
            line_detctor.EDline(img_cur, lines_cur, false);
            if(!lines_cur.empty()){

            line_matching.LineFilter(lines_cur, filter_distance);
            line_matching.Matching(prev_image,
                                    img_cur,
                                    line_prev,
                                    lines_cur,
                                    line_prev_to_line_cur,
                                    *(cv::Mat*)NULL,
                                    *(cv::Mat*)NULL,
                                    *(cv::Mat*)NULL,
                                    illumination_adapt,
                                    topological_filter,
                                    debug_show,
                                    (idx_ref++)-1, idx_cur++);
                            
            } 
        }
        else{
            line_detctor.EDline(img_cur, lines_cur, false);
            line_matching.LineFilter(lines_cur, filter_distance);
        }

        prev_image = img_cur;
        line_prev = lines_cur;

        cv::waitKey(70);
    }
}

int main(int argc, char** argv) {
    test();
    return 0;
}