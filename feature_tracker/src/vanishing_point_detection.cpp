/*
 * @Description: 
 * @Version: 
 * @Author: 
 * @Date: 2022-11-24 09:39:33
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-02-16 13:44:25
 */
/**
 * @description: 先由getVPHypVia2Lines穷举所有的灭点，
 * 	在get_sphere_grids来构建极坐标系搜索网络便于验证灭点，
 * 	在get_best_vps_hyp进行所有的假设进行验证，获取响应的ps
 * 	在lines2Vps根据阈值1.0 / 180.0 * CV_PI进行分类，对线特征进行分类
 * @return {*}
 * @author: fish
 */
#include "../include/vanishing_point_detection.h"
#include "../include/tic_toc.h"
#include <glog/logging.h>
#include <thread>

using namespace std;
using namespace Eigen;
double vanishing_point_detection::segAngle(const Line &s) {
  if (s.line_endpoint[2] > s.line_endpoint[0])
    return std::atan2(s.line_endpoint[3] - s.line_endpoint[1], s.line_endpoint[2] - s.line_endpoint[0]);
  else
    return std::atan2(s.line_endpoint[1]- s.line_endpoint[3] , s.line_endpoint[0] - s.line_endpoint[2]);
}
void vanishing_point_detection::init(float _f,float _cx,float _cy,double _noiseRatio){
	this->f = _f;
	this->pp.x = _cx;
	this->pp.y = _cy;
	noiseRatio = _noiseRatio;
}

void vanishing_point_detection::run_vanishing_point_detection(const cv::Mat &img,std::vector<Line > &lines,std::vector<Line > &all_lines,  std::vector<Eigen::Vector3d> &vps,vector<int> &local_vp_ids){
	cv::Mat image = img;

	std::vector<std::vector<int> > clusters;
	vector<Vector3d> para_vector;
	vector<double> length_vector;
    vector<double> orientation_vector;
	vector<std::vector<Vector3d> > vpHypo;
	vector<std::vector<Vector3d> > tmp_vps;
    vector<std::vector<double> > sphereGrid;

	lineinfo(lines, para_vector, length_vector, orientation_vector);

	std::thread get_vpthread([&](){
		getVPHypVia2Lines(lines, para_vector, length_vector, orientation_vector, vpHypo);
	});
	std::thread get_sphere_gridsthread([&](){
		getSphereGrids(lines, para_vector, length_vector, orientation_vector, sphereGrid );
	});
	get_vpthread.join();
	get_sphere_gridsthread.join();

    getBestVpsHyp(sphereGrid, vpHypo, vps);
	double thAngle = 1.0 / 180.0 * CV_PI;
    lines2Vps(all_lines, thAngle, vps, clusters, local_vp_ids);		

	// drawClusters(image,all_lines,clusters);	

    frame_count++;
}

void vanishing_point_detection::lineinfo(vector<Line> cur_keyLine, vector<Eigen::Vector3d> &para_vector, vector<double> &length_vector, vector<double> &orientation_vector){
    // get the parameters of each line
	int num = cur_keyLine.size();
    for ( int i = 0; i < num; ++i )
    {
        Vector3d p1(cur_keyLine[i].line_endpoint[0], cur_keyLine[i].line_endpoint[1], 1.0);
        Vector3d p2(cur_keyLine[i].line_endpoint[2], cur_keyLine[i].line_endpoint[3], 1.0);

        para_vector.push_back(p1.cross( p2 ));

        double dx = cur_keyLine[i].line_endpoint[0] - cur_keyLine[i].line_endpoint[1];
        double dy = cur_keyLine[i].line_endpoint[2] - cur_keyLine[i].line_endpoint[3];
        length_vector.push_back(sqrt( dx * dx + dy * dy ));

        double orientation = atan2( dy, dx );
        if ( orientation < 0 )
        {
            orientation += CV_PI;
        }
        orientation_vector.push_back(orientation);
    }
}
void vanishing_point_detection::getVPHypVia2Lines(vector<Line> cur_keyLine, vector<Eigen::Vector3d> &para_vector, vector<double> &length_vector, vector<double> &orientation_vector, std::vector<std::vector<Eigen::Vector3d> > &vpHypo )
{
    int num = cur_keyLine.size();
    
    double noiseRatio = 0.5;
    double p = 1.0 / 3.0 * pow( 1.0 - noiseRatio, 2 );

    double confEfficience = 0.9999;
    int it = log( 1 - confEfficience ) / log( 1.0 - p );

    int numVp2 = 360;
    double stepVp2 = 2.0 * CV_PI / numVp2;


    // get vp hypothesis for each iteration
    vpHypo = std::vector<std::vector<Vector3d> > ( it * numVp2, std::vector<Vector3d>(4) );
    int count = 0;
    srand((unsigned)time(NULL));
    for ( int i = 0; i < it; ++ i )
    {
        int idx1 = rand() % num;
        int idx2 = rand() % num;
        //  std::cout<<"idx "<<idx1<<" idx2 "<<idx2<< std::endl;
        while ( idx1 < 0 || idx2 < 0)
        {
            idx1 = rand() % num;
            idx2 = rand() % num;
        }     
        while ( idx2 == idx1 )
        {
            idx2 = rand() % num;
        }
        
 
        // get the vp1
        Vector3d vp1_Img = para_vector[idx1].cross( para_vector[idx2] );
        if ( vp1_Img(2) == 0 )
        {
            i --;
            continue;
        }

        Vector3d vp1(vp1_Img(0) / vp1_Img(2) - pp.x,
                     vp1_Img(1) / vp1_Img(2) - pp.y,
                     f );
        if ( vp1(2) == 0 ) { vp1(2) = 0.0011; }
        double N = sqrt( vp1(0) * vp1(0) + vp1(1) * vp1(1) + vp1(2) * vp1(2) );
        vp1 *= 1.0 / N;

        // get the vp2 and vp3
        Vector3d vp2( 0.0, 0.0, 0.0 );
        Vector3d vp3( 0.0, 0.0, 0.0 );
        Vector3d vp4( 0.0, 0.0, 0.0 );

        for ( int j = 0; j < numVp2; ++ j )
        {
            // vp2
            double lambda = j * stepVp2;

            double k1 = vp1(0) * sin( lambda ) + vp1(1) * cos( lambda );
            double k2 = vp1(2);
            double phi = atan( - k2 / k1 );

            double Z = cos( phi );
            double X = sin( phi ) * sin( lambda );
            double Y = sin( phi ) * cos( lambda );

            vp2(0) = X;  vp2(1) = Y;  vp2(2) = Z;
            if ( vp2(2) == 0.0 ) { vp2(2) = 0.0011; }
            N = sqrt( vp2(0) * vp2(0) + vp2(1) * vp2(1) + vp2(2) * vp2(2) );
            vp2 *= 1.0 / N;
            if ( vp2(2) < 0 ) { vp2 *= -1.0; }

            // vp3
            vp3 = vp1.cross( vp2 );
            if ( vp3(2) == 0.0 ) { vp3(2) = 0.0011; }
            N = sqrt( vp3(0) * vp3(0) + vp3(1) * vp3(1) + vp3(2) * vp3(2) );
            vp3 *= 1.0 / N;
            if ( vp3(2) < 0 ) { vp3 *= -1.0; }
            //
            vpHypo[count][0] = Vector3d( vp1(0), vp1(1), vp1(2) );
            vpHypo[count][1] = Vector3d( vp2(0), vp2(1), vp2(2) );
            vpHypo[count][2] = Vector3d( vp3(0), vp3(1), vp3(2) );

            count ++;
        }
    }
}


void vanishing_point_detection::getSphereGrids(vector<Line> cur_keyLine, vector<Vector3d> &para_vector, vector<double> &length_vector, vector<double> &orientation_vector, std::vector<std::vector<double> > &sphereGrid )
{
    // build sphere grid with 1 degree accuracy
    double angelAccuracy = 1.0 / 180.0 * CV_PI;
    double angleSpanLA = CV_PI / 2.0;
    double angleSpanLO = CV_PI * 2.0;
    int gridLA = angleSpanLA / angelAccuracy;
    int gridLO = angleSpanLO / angelAccuracy;

    sphereGrid = std::vector< std::vector<double> >( gridLA, std::vector<double>(gridLO) );
    for ( int i=0; i<gridLA; ++i )
    {
        for ( int j=0; j<gridLO; ++j )
        {
            sphereGrid[i][j] = 0.0;
        }
    }

    // put intersection points into the grid
    double angelTolerance = 60.0 / 180.0 * CV_PI;
    Vector3d ptIntersect;
    double x = 0.0, y = 0.0;
    double X = 0.0, Y = 0.0, Z = 0.0, N = 0.0;
    double latitude = 0.0, longitude = 0.0;
    int LA = 0, LO = 0;
    double angleDev = 0.0;
    for ( int i=0; i<cur_keyLine.size()-1; ++i )
    {
        for ( int j=i+1; j<cur_keyLine.size(); ++j )
        {
            ptIntersect = para_vector[i].cross( para_vector[j] );

            if ( ptIntersect(2) == 0 )
            {
                continue;
            }

            x = ptIntersect(0) / ptIntersect(2);
            y = ptIntersect(1) / ptIntersect(2);

            X = x - pp.x;
            Y = y - pp.y;
            Z = f;
            N = sqrt( X * X + Y * Y + Z * Z );

            latitude = acos( Z / N );
            longitude = atan2( X, Y ) + CV_PI;

            LA = int( latitude / angelAccuracy );
            if ( LA >= gridLA )
            {
                LA = gridLA - 1;
            }

            LO = int( longitude / angelAccuracy );
            if ( LO >= gridLO )
            {
                LO = gridLO - 1;
            }

            //
            angleDev = abs( orientation_vector[i] - orientation_vector[j] );
            angleDev = min( CV_PI - angleDev, angleDev );
            if ( angleDev > angelTolerance )
            {
                continue;
            }

            sphereGrid[LA][LO] += sqrt( length_vector[i] * length_vector[j] ) * ( sin( 2.0 * angleDev ) + 0.2 ); // 0.2 is much robuster
        }
    }

    //
    int halfSize = 1;
    int winSize = halfSize * 2 + 1;
    int neighNum = winSize * winSize;

    // get the weighted line length of each grid
    std::vector< std::vector<double> > sphereGridNew = std::vector< std::vector<double> >( gridLA, std::vector<double>(gridLO) );
    for ( int i=halfSize; i<gridLA-halfSize; ++i )
    {
        for ( int j=halfSize; j<gridLO-halfSize; ++j )
        {
            double neighborTotal = 0.0;
            for ( int m=0; m<winSize; ++m )
            {
                for ( int n=0; n<winSize; ++n )
                {
                    neighborTotal += sphereGrid[i-halfSize+m][j-halfSize+n];
                }
            }

            sphereGridNew[i][j] = sphereGrid[i][j] + neighborTotal / neighNum;
        }
    }
    sphereGrid = sphereGridNew;
}

void vanishing_point_detection::getBestVpsHyp( std::vector<std::vector<double> > &sphereGrid, std::vector<std::vector<Vector3d> >  &vpHypo, std::vector<Vector3d> &vps  )
{
    int num = vpHypo.size();
    double oneDegree = 1.0 / 180.0 * CV_PI;

    // get the corresponding line length of every hypotheses
    std::vector<double> lineLength( num, 0.0 );
    for ( int i = 0; i < num; ++ i )
    {
        std::vector<cv::Point2d> vpLALO( 3 );
        for ( int j = 0; j < 3; ++ j )
        {
            if ( vpHypo[i][j](2) == 0.0 )
            {
                continue;
            }

            if ( vpHypo[i][j](2) > 1.0 || vpHypo[i][j](2) < -1.0 )
            {
                cout<<1.0000<<endl;
            }
            double latitude = acos( vpHypo[i][j](2) );
            double longitude = atan2( vpHypo[i][j](0), vpHypo[i][j](1) ) + CV_PI;

            int gridLA = int( latitude / oneDegree );
            if ( gridLA == 90 )
            {
                gridLA = 89;
            }

            int gridLO = int( longitude / oneDegree );
            if ( gridLO == 360 )
            {
                gridLO = 359;
            }

            lineLength[i] += sphereGrid[gridLA][gridLO];
        }
    }

    // get the best hypotheses
    int bestIdx = 0;
    double maxLength = 0.0;
    for ( int i = 0; i < num; ++ i )
    {
        if ( lineLength[i] > maxLength )
        {
            maxLength = lineLength[i];
            bestIdx = i;
        }
    }

    vps = vpHypo[bestIdx];
    static std::vector<Vector3d> fisrt_vps = vps;
    int row_f = 1;
    int row_v = 1;

    // for(int )
    if(frame_count == 0)
    {
        
        if(abs(fisrt_vps[1](1) ) > 0.8)
            row_f = 1;
        else    
            row_f = 2;
    }
    else{
        
        if(abs(vps[1](1) ) > 0.8)
            row_v = 1;
        else    
            row_v = 2;
        if(row_f != row_v )
            swap(vps[1],vps[2]);
        
    }

   

    // // 不平行
    // if( abs(fisrt_vps[1].dot(vps[1])) < 0.6){
    //     swap(vps[1],vps[2]);
    // }
    // for(auto i: fisrt_vps)
    //     cout<<"fisrt_vps: "<<i.transpose()<<std::endl;
    // for(auto i: vps)
    //     cout<<"vps: "<<i.transpose()<<std::endl;
//    cout << vps.size() << endl;
}

void vanishing_point_detection::lines2Vps(vector<Line> cur_keyLine, double thAngle, std::vector<Vector3d> &vps, std::vector<std::vector<int> > &clusters, vector<int> &vp_idx)
{
    clusters.clear();
    clusters.resize( 3 );
    lx.clear();
    ly.clear();
    lz.clear();
    int vps_size = 3;
    //get the corresponding vanish points on the image plane
    std::vector<cv::Point2d> vp2D( vps_size );
    for ( int i = 0; i < vps_size; ++ i )
    {
        vp2D[i].x =  vps[i](0) * f /
                     vps[i](2) + pp.x;
        vp2D[i].y =  vps[i](1) * f /
                     vps[i](2) + pp.y;
    }

    for ( int i = 0; i < cur_keyLine.size(); ++ i )
    {   
        // cout<<cur_keyLine[i].angle<<std::endl;
        double x1 = cur_keyLine[i].line_endpoint[0];
        double y1 = cur_keyLine[i].line_endpoint[1];

        double x2 = cur_keyLine[i].line_endpoint[2];
        double y2 = cur_keyLine[i].line_endpoint[3];

        double xm = ( x1 + x2 ) / 2.0;
        double ym = ( y1 + y2 ) / 2.0;

        double v1x = x1 - x2;
        double v1y = y1 - y2;
        double N1 = sqrt( v1x * v1x + v1y * v1y );
        v1x /= N1;   v1y /= N1;

        double minAngle = 1000;
        int bestIdx = 0;
        for ( int j = 0; j < vps_size; ++ j )
        {
            double v2x = vp2D[j].x - xm;
            double v2y = vp2D[j].y - ym;
            double N2 = sqrt( v2x * v2x + v2y * v2y );
            v2x /= N2;  v2y /= N2;

            double crossValue = v1x * v2x + v1y * v2y;
            if ( crossValue > 1.0 )
            {
                crossValue = 1.0;
            }
            if ( crossValue < -1.0 )
            {
                crossValue = -1.0;
            }
            double angle = acos( crossValue );
            angle = min( CV_PI - angle, angle );

            bool flag = false;
            if ( angle < minAngle )
            {   
                if(j == 0)
                {
                    if(ly.size() > 1){

                    // int idx1 = rand() % num;
                    int idx = rand()% ly.size();
                    Line cur_l = cur_keyLine[i];
                    Line query_l = lx[idx];

                    float cur_angle = segAngle(cur_l);
                    float query_angle = segAngle(query_l);

                    float delta_angle = abs(cur_angle - query_angle);

                    if(delta_angle < 0.175)
                        flag = true;
                    // if(delta_angle < 1)
                    // cout<<"delta_angle x: "<<delta_angle<<std::endl;
                    }


                }
                else if(j == 1 )
                {
                    if(lz.size() > 1){
                    int idx = rand()% lz.size();
                    Line cur_l = cur_keyLine[i];
                    Line query_l = lx[idx];

                    float cur_angle = segAngle(cur_l);
                    float query_angle = segAngle(query_l);

                    float delta_angle = abs(cur_angle - query_angle);
                    if(delta_angle < 0.175)
                        flag = true;
                    // cout<<"delta_angle y: "<<delta_angle<<std::endl;
                    }
                }
                else if(j == 2)
                {
                    if(lx.size() > 1){
                    int idx = rand()% lx.size();
                    Line cur_l = cur_keyLine[i];
                    Line query_l = lx[idx];

                    float cur_angle = segAngle(cur_l);
                    float query_angle = segAngle(query_l);

                    float delta_angle = abs(cur_angle - query_angle);

                    if(delta_angle < 0.175)
                        flag = true;
                    // cout<<"delta_angle z: "<<delta_angle<<std::endl;  
                     }
                }

                if(flag == false){
                    minAngle = angle;
                    bestIdx = j;   
                    if(j == 0 )
                    lx.push_back(cur_keyLine[i]);
                    else if(j ==1 )
                        ly.push_back(cur_keyLine[i]);
                    else if(j == 2)
                        lz.push_back(cur_keyLine[i]);                 
                }

            }
        }

        //
        if ( minAngle < thAngle)
        {
            clusters[bestIdx].push_back( i );
            vp_idx.push_back(bestIdx);
        }
        else
            vp_idx.push_back(3);
    }
}

void vanishing_point_detection::drawClusters( cv::Mat &img, std::vector<Line> &lines, std::vector<std::vector<int> > &clusters )
{
    cv::Mat vp_img ;
    cv::Mat line_img;
    int cols = img.cols;
    int rows = img.rows;

    //draw lines
    cv::cvtColor(img.clone(), vp_img, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img.clone(), line_img, cv::COLOR_GRAY2BGR);

    std::vector<cv::Scalar> lineColors( 3 );
    lineColors[0] = cv::Scalar( 0, 0, 255 );
    lineColors[1] = cv::Scalar( 0, 255, 0 );
    lineColors[2] = cv::Scalar( 255, 0, 0 );
//    lineColors[3] = cv::Scalar( 0, 255, 255 );

    for ( int i=0; i<lines.size(); ++i )
    {
        int idx = i;
        cv::Point2f pt_s(lines[i].line_endpoint[0],lines[i].line_endpoint[1]);
        cv::Point2f pt_e(lines[i].line_endpoint[2],lines[i].line_endpoint[3]);
        cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

        // cv::line( vp_img, pt_s, pt_e, cv::Scalar(0,0,0), 2, CV_AA );
        cv::line( line_img, pt_s, pt_e, cv::Scalar(0,255,255), 2, CV_AA );
    }

    for ( int i = 0; i < clusters.size(); ++i )
    {
        for ( int j = 0; j < clusters[i].size(); ++j )
        {
            int idx = clusters[i][j];

            cv::Point2f pt_s(lines[idx].line_endpoint[0],lines[idx].line_endpoint[1]);
            cv::Point2f pt_e(lines[idx].line_endpoint[2],lines[idx].line_endpoint[3]);
            cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

            cv::line( vp_img, pt_s, pt_e, lineColors[i], 2, CV_AA );
        }
    }
    cv::imshow("img", img);
    cv::imshow("line img", line_img);
    cv::imshow("vp img", vp_img);
    cv::waitKey(1);
}