/*
 * @Description: 
 * @Version: 
 * @Author: 
 * @Date: 2022-12-06 10:06:23
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-05-17 10:05:00
 */
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"
#include "../initial/initial_ex_rotation.h"
#include <glog/logging.h>
#include "../utility/nanoflann/nanoflann.hpp"
// #include "../sophus/se3.hpp"
#include <ceres/ceres.h>
using namespace std;
using namespace Eigen;

enum : int { Rc=0, Ri=1, tc=2, ti=3};

struct PointToLineDistanceFactor
{
    PointToLineDistanceFactor(
                              Eigen::Vector3d curr_point_,
                              Eigen::Vector3d  closed_point_a_,
                              Eigen::Vector3d  closed_point_b_)
        : curr_point(curr_point_),
          closed_point_a(closed_point_a_),
          closed_point_b(closed_point_b_) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        // SE3<T> T_cw(pose_t, pose_q);
        Eigen::Quaternion<T> q_wc{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_wc{t[0],t[1], t[2]};

		Eigen::Matrix<T, 3, 1> cur{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(closed_point_a.x()), T(closed_point_a.y()), T(closed_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(closed_point_b.x()), T(closed_point_b.y()), T(closed_point_b.z())};

        Eigen::Matrix<T, 3, 1>  point_w = q_wc * cur + t_wc;

        Eigen::Matrix<T, 3, 1>  nu = (point_w - lpa).cross(point_w - lpb); // 向量OA 叉乘 向量OB
        T de = (lpa - lpb).norm();                // 向量AB
        T norm = T(nu.norm());

        if(norm < 1e-6)
            norm = nu.x() + nu.y() +nu.z();
            
        residual[0] = norm / de;


        return true;
    }

    static ceres::CostFunction *Create( Eigen::Vector3d  curr_point_,
                                       Eigen::Vector3d closed_point_a_,
                                       Eigen::Vector3d  closed_point_b_)
    {
        return (new ceres::AutoDiffCostFunction<PointToLineDistanceFactor, 1, 4, 3>(
                    new PointToLineDistanceFactor(curr_point_, closed_point_a_,
                                                  closed_point_b_)));
    }

    Eigen::Vector3d  curr_point, closed_point_a, closed_point_b;
};

struct PointCloudAdaptor {
    PointCloudAdaptor(Eigen::MatrixXd _cloud)
        : cloud(_cloud){}
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const {
        return cloud.cols();
    }

    inline double kdtree_get_pt(const size_t idx, int dim) const {
        return cloud(idx * cloud.rows() + dim);  
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const {
        return false;
    }

    mutable Eigen::MatrixXd cloud;
};
using kdtree_ = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, -1>;

Eigen::MatrixXd transform(Eigen::MatrixXd map_cloud, Eigen::Quaterniond & q,Eigen::Vector3d &t);

class InitialEXTime{
    public:
    InitialEXTime() = default;
    ~InitialEXTime() = default;
    bool CalibrationExTime(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, const double image_time,const double imu_time,double &dt);
    double calib_time(Eigen::MatrixXd& map_cloud,Eigen::MatrixXd& input_cloud,Eigen::Matrix4d & prior_pose,std::string &config_files);
    inline void setRic(Matrix3d ric_){
        ric = ric_;
    }
    private:
    bool flag = false;
    InitialEXRotation ex_rotation;
    using measurement_t = std::tuple<Matrix3d, Matrix3d, Vector3d,
                                  Vector3d>;
    double t_i = 0;
    Matrix3d ric;
    vector<Vector3d> cam ;
    vector<Vector3d> imu ;
    
};
