/*
 * @Author: fish 973841082@qq.com
 * @Date: 2022-12-28 19:14:28
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2022-12-28 20:51:08
 * @FilePath: /vin_ws/src/VINS-Mono/vins_estimator/src/factor/line_projection_factor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
class vpProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 4>
{
  public:
    vpProjectionFactor(const Eigen::Vector3d &_pts_i):obs_i(_pts_i){};
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class lineProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 4>
{
  public:
    lineProjectionFactor(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

///////////////////////////////line in camera frame///////////////////////////////////////////
class lineProjectionFactor_incamera : public ceres::SizedCostFunction<2, 7, 7, 7, 4>
{
public:
    lineProjectionFactor_incamera(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
class lineProjectionFactor_instartframe : public ceres::SizedCostFunction<2, 4>
{
public:
    lineProjectionFactor_instartframe(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};