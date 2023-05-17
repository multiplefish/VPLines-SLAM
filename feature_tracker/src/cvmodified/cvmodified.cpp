#include <cstdio>
#include <vector>
#include <iostream>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "cvmodified.h"

using namespace cv;

namespace cvmodified {

struct greaterThanPtr
{
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

// ----------------------------------------------------------------------------
/**
 * @description: 计算harris角点和shi-tomasi角点
 * @param _image：8位或32位浮点型输入图像，单通道
 *              _corners：保存检测出的角点
 *              qualityLevel：角点的品质因子
 *              minDistance：对于初选出的角点而言，如果在其周围minDistance范围内存在其他更强角点，则将此角点删除
 *              _mask 指定感兴趣区，如不需在整幅图上寻找角点，则用此参数指定ROI
 *              blockSize：计算协方差矩阵时的窗口大小
 *              指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
 *              harrisK：Harris角点检测需要的k值
 * @return {*}
 * @author: fish
 */

void goodFeaturesToTrack( InputArray _image, OutputArray _corners, OutputArray _scores,
                              int maxCorners, double qualityLevel, double minDistance,
                              InputArray _mask, int blockSize, int gradientSize,
                              bool useHarrisDetector, double harrisK )
{

    // 对image进行全图操作还是进行一个感兴趣区域操作
    CV_Assert( qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0 );
    CV_Assert( _mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(_image)) );

    // std::cout << std::endl << "** calling modified version **" << std::endl;
    // eig存储的是每个像素协方差的最小特征值,tmp是存储用来保存的膨胀后的eig

    Mat image = _image.getMat(), eig, tmp;
    if (image.empty())
    {
        _corners.release();
        _scores.release();
        return;
    }
    //blockSize是计算2*2协方差矩阵的窗口大小，sobel算子窗口为3，harrisK是计算Harris角点时需要的值
    // //计算每个像素对应的协方差矩阵的最小特征值，保存在eig中
    if( useHarrisDetector )
        cornerHarris( image, eig, blockSize, gradientSize, harrisK );
    else
        cornerMinEigenVal( image, eig, blockSize, gradientSize );

    double maxVal = 0;
    minMaxLoc( eig, 0, &maxVal, 0, 0, _mask );
    threshold( eig, eig, maxVal*qualityLevel, 0, THRESH_TOZERO );
    //默认用3*3的核膨胀，膨胀之后，除了局部最大值点和原来相同，其它非局部最大值点被  
    //3*3邻域内的最大值点取代，如不理解，可看一下灰度图像的膨胀原理 
    dilate( eig, tmp, Mat());

    Size imgsize = image.size();
    //存放粗选出的角点地址
    std::vector<const float*> tmpCorners;

    // collect list of pointers to features - put them into temporary image
    Mat mask = _mask.getMat();
    for( int y = 1; y < imgsize.height - 1; y++ )
    {   
        //获得eig第y行的首地址
        const float* eig_data = (const float*)eig.ptr(y);
        //获得tmp第y行的首地址
        const float* tmp_data = (const float*)tmp.ptr(y);
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;

        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )
                //保存其位置
                tmpCorners.push_back(eig_data + x);
        }
    }

    std::vector<Point2f> corners;
    std::vector<float> scores;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0)
    {
        _corners.release();
        _scores.release();
        return;
    }

    std::sort( tmpCorners.begin(), tmpCorners.end(), greaterThanPtr() );
    //-----------此分割线以上是根据特征值粗选出的角点，我们称之为弱角点----------//
	//-----------此分割线以下还要根据minDistance进一步筛选角点，仍然能存活下来的我们称之为强角点----------//
    if (minDistance >= 1)
    {
         // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;
        //向最近的整数取整
        const int cell_size = cvRound(minDistance);
        //这里根据cell_size构建了一个矩形窗口grid(虽然下面的grid定义的是vector<vector>，而并不是我们这里说的矩形窗口，
        // 但为了便于理解,还是将grid想象成一个grid_width * grid_height的矩形窗口比较好)，
        // 除以cell_size说明grid窗口里相差一个像素相当于_image里相差minDistance个像素，至于为什么加上cell_size - 1后面会讲

        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;
        // //vector里面是vector，grid用来保存获得的强角点坐标
        std::vector<std::vector<Point2f> > grid(grid_width*grid_height);
        //vector里面是vector，grid用来保存获得的强角点坐标
        minDistance *= minDistance;
        // // 刚刚粗选的弱角点，都要到这里来接收新一轮的考验

        for( i = 0; i < total; i++ )
        {
            //tmpCorners中保存了角点的地址，eig.data返回eig内存块的首地址
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            //角点在原图像中的行
            int y = (int)(ofs / eig.step);
            //角点在原图像中的列
            int x = (int)((ofs - y*eig.step)/sizeof(float));
            //先认为当前角点能接收考验，即能被保留下来
            bool good = true;
            //x_cell，y_cell是角点（y,x）在grid中的对应坐标
            int x_cell = x / cell_size;
            int y_cell = y / cell_size;
            // (y_cell，x_cell）的4邻域像素
            //现在知道为什么前面grid_width定义时要加上cell_size - 1了吧，
            // 这是为了使得（y,x）在grid中的4邻域像素都存在，也就是说(y_cell，x_cell）不会成为边界像素
            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
             // boundary check，再次确认x1,y1,x2或y2不会超出grid边界
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width-1, x2);
            y2 = std::min(grid_height-1, y2);
            //记住grid中相差一个像素，相当于_image中相差了minDistance个像素
            for( int yy = y1; yy <= y2; yy++ )
            {
                for( int xx = x1; xx <= x2; xx++ )
                {   
                    std::vector <Point2f> &m = grid[yy*grid_width + xx];
                    
                    //如果(y_cell，x_cell)的4邻域像素，也就是(y,x)的minDistance邻域像素中已有被保留的强角点
                    if( m.size() )
                    {
                        for(j = 0; j < m.size(); j++)
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;
                             //注意如果(y,x)的minDistance邻域像素中已有被保留的强角点，则说明该强角点是在(y,x)之前就被测试过的，
                            //  又因为tmpCorners中已按照特征值降序排列（特征值越大说明角点越好），这说明先测试的一定是更好的角点，
                            //  也就是已保存的强角点一定好于当前角点，所以这里只要比较距离，如果距离满足条件，可以立马扔掉当前测试的角点
                            if( dx*dx + dy*dy < minDistance )
                            {
                                good = false;
                                goto break_out;
                            }
                        }
                    }
                }
            }

            break_out:

            if (good)
            {   
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

                corners.push_back(Point2f((float)x, (float)y));
                scores.push_back(*tmpCorners[i]);
                // std::cout<<"scores:"<<scores.back()<<std::endl;
                ++ncorners;

                if( maxCorners > 0 && (int)ncorners == maxCorners )
                    break;
            }
        }
    }
    else//除了像素本身，没有哪个邻域像素能与当前像素满足minDistance < 1,因此直接保存粗选的角点
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            corners.push_back(Point2f((float)x, (float)y));
            scores.push_back(*tmpCorners[i]);

            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )
                break;
        }
    }

    Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
    Mat(scores).convertTo(_scores, _scores.fixedType() ? _scores.type() : CV_32F);
    // for(int i=0; i<_scores.rows();++i){
    //     for(int j=0; j<_scores.cols(); ++j){
    //         std::cout<<_scores(i,j)<<std::endl;
    //     }
    // } 
}

// ----------------------------------------------------------------------------

void goodFeaturesToTrack( InputArray _image, OutputArray _corners, OutputArray _scores,
                          int maxCorners, double qualityLevel, double minDistance,
                          InputArray _mask, int blockSize,
                          bool useHarrisDetector, double harrisK )
{
    cvmodified::goodFeaturesToTrack(_image, _corners, _scores, maxCorners, qualityLevel, minDistance,
                        _mask, blockSize, 3, useHarrisDetector, harrisK );
    
}

} // namespace cvmodified
