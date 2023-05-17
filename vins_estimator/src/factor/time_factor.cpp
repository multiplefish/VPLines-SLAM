/*
 * @Description: 
 * @Version: 
 * @Author: 
 * @Date: 2022-12-06 10:10:08
 * @LastEditors: fish 973841082@qq.com
 * @LastEditTime: 2023-05-17 11:18:24
 */
#include "time_factor.h"
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

bool InitialEXTime::CalibrationExTime(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, const double image_time,const double imu_time,double &dt){
    // vector< Matrix3d > Rc;
    // vector< Matrix3d > Rimu;
    
    static Matrix3d Rc01 = ex_rotation.solveRelativeR(corres);
    static double tc0 = image_time;
    Matrix3d Rc12 = ex_rotation.solveRelativeR(corres);
    Matrix3d R_c = Rc01*Rc12;
    double t_c = image_time - tc0;
    // ric <<0.0148655429818, -0.999880929698, 0.00414029679422,
    //        0.999557249008, 0.0149672133247, 0.025715529948, 
    //        -0.0257744366974, 0.00375618835797, 0.999660727178;
    // 根据特征关联求解两个连续帧相机的旋转R12
    static Matrix3d Ri01 = (ric.inverse() * delta_q_imu * ric);
    Matrix3d Ri12 = (ric.inverse() * delta_q_imu * ric);
    Matrix3d R_i = Ri01*Ri12;
    t_i = t_i + imu_time;
    static size_t count_time_frame = 0;
    ++count_time_frame;


    Eigen::Vector3d r_cam = R_c.eulerAngles(2,1,0);
    Eigen::Vector3d r_imu = R_i.eulerAngles(2,1,0);

    LOG(INFO)<<"CAM angle "<<r_cam(0)<<"time:"<<t_c<<std::endl;
    LOG(INFO)<<"IMU angle "<<r_imu(0)<<"time:"<<t_i<<std::endl;


    cam.emplace_back(Vector3d{t_c/100.0,r_cam(0),0.0});
    imu.emplace_back(Vector3d{t_i/100.0,r_imu(0),0.0});

    if(t_i > 7.0 && flag == false){
        flag = true;
        Eigen::MatrixXd measurement_cam(3,cam.size());
        Eigen::MatrixXd measurement_imu(3,cam.size());
    
        for(int i=0; i<cam.size(); ++i){
            measurement_cam.col(i) = cam[i];
            measurement_imu.col(i) = imu[i];
        }
        Eigen::Matrix4d prior_pose;
        std::string config_files;

       double offset = calib_time(measurement_imu,measurement_cam,prior_pose,config_files);
       LOG(INFO)<<"offest "<<offset*100.0<<std::endl;
 
    }
    return flag;



}

Eigen::MatrixXd transform(Eigen::MatrixXd map_cloud, Eigen::Quaterniond & q,Eigen::Vector3d &t){
    Eigen::MatrixXd ret = map_cloud;
    for(int y=0;y<ret.cols();y++){
        double*  ptr = & ret(y*ret.rows());
        Eigen::Vector3d pt = q * Eigen::Vector3d(ptr[0], ptr[1], ptr[2]) + t;
        ptr[0] = pt[0]; ptr[1] = pt[1]; ptr[2] = pt[2];
    }
    return ret;
}
double InitialEXTime::calib_time(Eigen::MatrixXd& map_cloud,Eigen::MatrixXd& input_cloud,Eigen::Matrix4d & prior_pose,std::string &config_files){

    int max_iteration = 30;
    double huber = 0.1;
    int min_residuals = 20;
    double line_search_radius = 1.0;
    double progress_threshold = 0.01; // converaged
    double min_line_dis   = 0.001;
    bool   const_rotation = true;

    PointCloudAdaptor adaptor(map_cloud);
    kdtree_ kdtree(2,adaptor); 

    kdtree.buildIndex();

    std::vector<size_t> tmp_indices(2);
    std::vector<float>  tmp_dists(2);



    //time angle
    double paramter[2] ={0.0,0.0};

    for(int i=0; i<max_iteration; ++i){
        // 对齐点云
        Eigen::MatrixXd cloud_aligned_tmp = transform(input_cloud, q_w_curr,t_w_curr);
        // 转换以下
        Eigen::MatrixXf cloud_aligned = cloud_aligned_tmp.cast<float>();

        // 定义一下ceres的核函数
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        // 由于旋转不满足一般意义的加法，因此这里使用ceres自带的local param
        ceres::LocalParameterization *q_parameterization =new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        // 待优化的变量是帧间位姿，平移和旋转，这里旋转使用四元数来表示
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);

        // 不优化旋转
        problem.SetParameterBlockConstant(para_q);

        for(auto y = 0; y < cloud_aligned.cols(); y++){
            float* c = &cloud_aligned(y*cloud_aligned.rows());
            // LOG(INFO)<<"cloud_aligned:"<<std::endl<<c[0]<<" "<<c[1]<<" "<<c[2]<<" end"<<std::endl;
            kdtree.knnSearch(c, 2u, &tmp_indices[0], &tmp_dists[0]);

            if(tmp_dists[1] > line_search_radius) continue;// abort

            double* a = &map_cloud(cloud_aligned.rows() * tmp_indices[0]);
            double* b = &map_cloud(cloud_aligned.rows() * tmp_indices[1]);

            Eigen::Vector3d pc(c[0], c[1], c[2]), pa(a[0],a[1],a[2]), pb(b[0], b[1], b[2]);
            // LOG(INFO)<<"input cloud:"<<std::endl<<pc.transpose()<<std::endl;
            // LOG(INFO)<<"map cloud:"<<std::endl<<pa.transpose()<<" "<<pb.transpose()<<std::endl;
            if((pa-pb).norm() < min_line_dis) continue;// too close

            ceres::CostFunction* cost = PointToLineDistanceFactor::Create(pc, pa, pb);

            problem.AddResidualBlock(cost, loss_function, para_q, para_t);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 30;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;
        // LOG(INFO)<<"t_w_curr"<<t_w_curr<<std::endl;
        // LOG(INFO)<<"q_w_curr"<<q_w_curr.x()<<" "<<q_w_curr.y()<<" "<<q_w_curr.z()<<" "<<q_w_curr.w()<<std::endl;      
        if(t_w_curr.norm() + q_w_curr.toRotationMatrix().norm() < progress_threshold)
            break;
    }
    
    return t_w_curr.x();
}