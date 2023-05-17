#include "feature_selector.h"
#include <ros/ros.h>
FeatureSelector::FeatureSelector(ros::NodeHandle nh, Estimator& estimator, 
                                 const std::string& calib_file)
: nh_(nh), estimator_(estimator)
{
  hgen_ = std::unique_ptr<HorizonGenerator>(new HorizonGenerator(nh_,calib_file));
  hgen_->setParameters(estimator_.ric[0], estimator_.tic[0]);

  // save extrinsics (for feature info step)
  q_IC_ = estimator_.ric[0];
  t_IC_ = estimator_.tic[0];
  m_camera_ = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

// ----------------------------------------------------------------------------

/**
 * @description: 参数设置
 * @return {*}
 * @author: fish
 */
void FeatureSelector::setParameters(double accVar, double accBiasVar,
                                    bool enable, int maxFeatures,
                                    int initThresh, bool useGT)
{
  accVarDTime_ = accVar;
  accBiasVarDTime_ = accBiasVar;
  enable_ = enable;
  maxFeatures_ = maxFeatures;
  initThresh_ = initThresh;
  horizonGeneration_ = (useGT) ? GT : IMU;
}



/**
 * @description: 状态传播
 * @return {*}
 * @author: fish
 */
void FeatureSelector::setNextStateFromImuPropagation(
    double imageTimestamp,
    const Eigen::Vector3d& P, const Eigen::Quaterniond& Q,
    const Eigen::Vector3d& V, const Eigen::Vector3d& a,
    const Eigen::Vector3d& w, const Eigen::Vector3d& Ba)
{
  state_k_.first.coeffRef(xTIMESTAMP) = state_k1_.first.coeff(xTIMESTAMP);
  state_k_.first.segment<3>(xPOS) = estimator_.Ps[WINDOW_SIZE];
  state_k_.first.segment<3>(xVEL) = estimator_.Vs[WINDOW_SIZE];
  state_k_.first.segment<3>(xB_A) = estimator_.Bas[WINDOW_SIZE];
  state_k_.second = estimator_.Rs[WINDOW_SIZE];

  state_k1_.first.coeffRef(xTIMESTAMP) = imageTimestamp;
  state_k1_.first.segment<3>(xPOS) = P;
  state_k1_.first.segment<3>(xVEL) = V;
  state_k1_.first.segment<3>(xB_A) = Ba;
  state_k1_.second = Q;

  ak1_ = a;
  wk1_ = w;
}

/**
 * @description: *
 * @parm {*}image：输入的特征点
 * {*}header：时间戳
 *  {*}imu：预计分的数量
 * @author: fish
 */
std::pair<std::vector<int>, std::vector<int>> FeatureSelector::select(image_t& image,
          const std_msgs::Header& header, int nrImuMeasurements)
{
   if (!enable_) return {};

  //
  // Timing information
  //

  // frame time of previous image
  static double frameTime_k = header.stamp.toSec();

  // time difference between last frame and current frame
  double deltaF = header.stamp.toSec() - frameTime_k;

  // calculate the IMU sampling rate of the last frame-to-frame meas set
  double deltaImu = deltaF / nrImuMeasurements;


  //
  // Decide which features are new and which are already being used
  //

  // remove new features from image and put into image_new.
  // Image will only contain features that are currently being tracked.
  // TODO: Is this true? Does VINS-Mono use *all* features given to it?
  image_t image_new;
  splitOnFeatureId(lastFeatureId_, image, image_new);

  // updated the largest feature_id for the next iteration (if different).
  if (!image_new.empty()) lastFeatureId_ = image_new.crbegin()->first;

  // the subset of features to pass to VINS-Mono back end
  image_t subset;

  // add in previously tracked features
  for (auto fid : trackedFeatures_) {
    // attempt to retrieve this feature from image
    auto feature = image.find(fid);
    if (feature != image.end()) {
      subset[fid] = feature->second;
    }

    // NOTE: We are not removing not found features because they could
    // pop up again (i.e., loop-closure (?), missed detections, etc.)
  }
  // ROS_WARN_STREAM("Feature subset initialized with " << subset.size() << " out"
                  // " of " << trackedFeatures_.size() << " known features");

  //
  // Future State Generation
  //

  // We will need to know the state at each frame in the horizon, k:k+H.
  // Note that this includes the current optimized state, xk
  auto state_kkH = generateFutureHorizon(header, nrImuMeasurements, deltaImu, deltaF);
  hgen_->visualize(header, state_kkH);


  //
  // Anticipation: Compute the Expected Information over the Horizon
  //

  // Calculate the information content from motion over the horizon (eq 15)
  auto Omega_kkH = calcInfoFromRobotMotion(state_kkH, nrImuMeasurements, deltaImu);

  // Add in prior information to OmegaIMU_kkH (eq 16)
  addOmegaPrior(Omega_kkH);

  // Calculate the information content of each of the new features
  auto Delta_ells = calcInfoFromFeatures(image_new, state_kkH);

  // Calculate the information content of each of the currently used features
  std::map<int, omega_horizon_t> Delta_used_ells;
  Delta_used_ells = calcInfoFromFeatures(subset, state_kkH);


  //
  // Attention: Select a subset of features that maximizes expected information
  //

  // Has VINS-Mono initialized?
  bool initialized = estimator_.solver_flag == Estimator::SolverFlag::NON_LINEAR;

  // We would like to only track N features total (including currently tracked
  // features). Therefore, we will calculate how many of the new features should
  // be selected (kappa).
  int kappa = std::max(0, maxFeatures_ - static_cast<int>(subset.size()));

  // so we can keep track of the new features we chose
  std::vector<int> selectedIds;
  selectedIds.reserve(kappa);

  // Only select features if VINS-Mono is initialized
  if (initialized) {
    selectedIds = selectInformativeFeatures(subset, image_new, kappa, Omega_kkH,
                                                Delta_ells, Delta_used_ells);
  } else if (!initialized && firstImage_) {
    // use the whole image to initialize!
    subset.swap(image_new);

    // consider all of these features as tracked by VINS-Mono
    for (const auto& fpair : subset) trackedFeatures_.push_back(fpair.first);

    // these features will be added by the trackedFeatures_ mechanism
    firstImage_ = false;
  }

  // if we still aren't initialized, but there aren't many features then add
  // all of the current image's features to the subset
  if (!initialized && static_cast<int>(subset.size()) < initThresh_) {
    subset.insert(image.begin(), image.end());
  }

  // ROS_WARN_STREAM("Feature selector chose " << subset.size() << " features");

  // return best features to use for VINS-Mono
  image.swap(subset);

  // keep track of which features have been passed to the back end. If we see
  // these features again, we need to let them through unharrassed.
  trackedFeatures_.insert(trackedFeatures_.end(), selectedIds.begin(), selectedIds.end());

  // for next iteration
  frameTime_k = header.stamp.toSec();

  return std::make_pair(trackedFeatures_, selectedIds);
}

//
/**
 * @description: 特征点分割老旧
 * @param {int} k
 * @param {image_t&} image
 * @param {image_t&} image_new
 * @return {*}
 * @author: fish
 */
void FeatureSelector::splitOnFeatureId(int k, image_t& image, image_t& image_new)
{
  // pick up after feature_id k
  auto it = image.upper_bound(k);
  bool found = (it != image.end());

  // if found, copy new features to image_new and remove from image
  if (found) {
    image_new.insert(it, image.end());
    image.erase(it, image.end());
  }
}



/**
 * @description: 产生状态
 * @return {*}
 * @author: fish
 */
state_horizon_t FeatureSelector::generateFutureHorizon(
                                        const std_msgs::Header& header,
                                        int nrImuMeasurements,
                                        double deltaImu, double deltaFrame)
{

  // generate the horizon based on the requested scheme
  if (horizonGeneration_ == IMU) {
    return hgen_->imu(state_k_, state_k1_, ak1_, wk1_, nrImuMeasurements, deltaImu);
  } else { //if (horizonGeneration_ == GT) {
    
    return hgen_->groundTruth(state_k_, state_k1_, deltaFrame);
  }

}
// ----------------------------------------------------------------------------
/**
 * @description: 计算线特征的信息
 * @return {*}
 * @author: fish
 */
std::map<int, omega_horizon_t> FeatureSelector::calcInfoFromLineFeatures(
                                             image_t& image,
                                            const state_horizon_t& state_kkH)
{
  int row = 4;
  std::map<int, omega_horizon_t> Delta_ells;  

  // Eigen::MatrixXd cloud(row,estimator_.f_manager.line_feature.size());
  // // convenience: (yet-to-be-corrected) transformation
  // // of camera frame w.r.t world frame at time k+1
  // const auto& t_WC_k1 = state_k1_.first.segment<3>(xPOS) + state_k1_.second * t_IC_;
  // const auto& q_WC_k1 = state_k1_.second * q_IC_;
  // // copied from visualization.cpp, pubPointCloud
  // int col = 0;
  // auto line_feature = estimator_.f_manager.line_feature;
  // // LOG(INFO)<<"line_feature Number"<<line_feature.size()<<std::endl;
  // for (auto& it_per_id : line_feature) {

  //     it_per_id.used_num = it_per_id.line_feature_per_frame.size();
  //     if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation))
  //       continue;
  //     int imu_i = it_per_id.start_frame;

  //     Eigen::Vector3d twc = estimator_.Ps[imu_i] + estimator_.Rs[imu_i] * estimator_.tic[0];   // twc = Rwi * tic + twi
  //     Eigen::Matrix3d Rwc = estimator_.Rs[imu_i] * estimator_.ric[0];               // Rwc = Rwi * Ric

  //     Vector6d line_w = plk_to_pose(it_per_id.line_plucker, Rwc, twc);  // transfrom to world frame 
  //     // LOG(INFO)<<"line_w: "<<plk_to_orth(line_w)<<std::endl;
  //     // transfrom to k1 frame 
  //     Eigen::Matrix3d Rwk1 = state_k1_.second.toRotationMatrix();
  //     Eigen::Vector3d twk1 = state_k1_.first.segment<3>(xPOS);

  //     Vector6d line_k1 = plk_from_pose(line_w, Rwk1, twk1);  // transfrom to k1 frame 
  //     Eigen::Matrix3d Rbc = q_IC_.toRotationMatrix();
  //     Eigen::Vector3d tbc = t_IC_;
  //     Vector6d line_c = plk_from_pose(line_k1, Rbc, tbc);
  //     // input cloud
  //     // Eigen::Vector4d tmp = plk_to_point(line_c);
  //     cloud.col(col++) = plk_to_orth(line_c);
  // }
  // // // LOG(INFO)<<"col: "<<col<<std::endl;
  // Eigen::MatrixXd map_cloud(row,col);

  // for(int i=0; i<col; ++i)
  //   map_cloud.col(i) = cloud.col(i);

  // LineCloudAdaptor adaptor(map_cloud);

  // line_kdtree_ line_kdtree(row,adaptor); 
  // line_kdtree.buildIndex();


  // std::vector<size_t> tmp_indices(2);
  // std::vector<float>  tmp_dists(2);

  // int number_all_lines = image.size();
  // // ROS_INFO("number_all_lines: %d",number_all_lines);
  // int count = 0;

  // // image_t image_new;
  // for(const auto& fpair: image){
    
  //   auto feature_id = fpair.first;
  //   Eigen::Vector4d feature = fpair.second[0].second.head<4>(); // calibrated [x1 y1 x2,y2]
  //   Vector6d line_c_image = pp_plk(Eigen::Vector3d(feature(0,0),feature(1,0),1.0),Eigen::Vector3d(feature(2,0),feature(3,0),1.0));
  
  //   Vector4d orth_tmp = plk_to_orth(line_c_image);

  //   Vector4f orth_c = orth_tmp.cast<float>();

  //   float *c = &orth_c[0];


  //   line_kdtree.knnSearch(c, 2u, &tmp_indices[0], &tmp_dists[0]);

  //   if(tmp_dists[1] > 1.0) continue;// abort

  //   double* a = &map_cloud(orth_c.rows() * tmp_indices[0]);
  //   Eigen::Vector4d pc(a[0], a[1], a[2],a[3]);

  //   Vector6d line_c_map = orth_to_plk(pc);
  //   Eigen::Matrix3d Rwc(q_WC_k1);
  //   Vector6d line_w = plk_to_pose(line_c_map, Rwc, t_WC_k1);  // transfrom to world frame   


  //   // ROS_INFO("query: %f %f %f %f map: %f %f %f %f",orth_c(0),orth_c(1),orth_c(2),orth_c(3),pc(0),pc(1),pc(2),pc(3));

  //   Eigen::Matrix<double, 3, 3> EtE; // ET*E
  //   Eigen::Matrix<double, 3, 3*HORIZON> FtE_tmp; // ET*E
  //   Eigen::Matrix<double, 3, 3*HORIZON> FtF_tmp; // 存储以下FT*F的结果

  //   // NOTE: start forward-simulating the landmark projection from k+2
  //   for (int h=2; h<=HORIZON; ++h) { 

  //     const auto& t_WC_h = state_kkH[h].first.segment<3>(xPOS) + state_kkH[h].second * t_IC_;
  //     const auto& q_WC_h = state_kkH[h].second * q_IC_;

  //     Eigen::Matrix3d R_WC_h = q_WC_h.toRotationMatrix();

  //     Vector6d line_c_h = plk_from_pose(line_w, R_WC_h, t_WC_h);// transfrom to camera frame

  //     Eigen::Vector3d nw = line_w.head(3);
  //     Eigen::Vector3d vc = line_c_h.tail(3);

  //     Eigen::Matrix3d A = Utility::skewSymmetric(nw)*Utility::skewSymmetric(vc);  //<F
  //     Eigen::Matrix3d B = Utility::skewSymmetric(nw)*R_WC_h.transpose();          //<E
  //     Eigen::Matrix3d ch = B.transpose()*B;
  //     EtE += ch;
  //     FtE_tmp.block<3, 3>(0, 3*(h-1)) = A.transpose()*B;
  //     FtF_tmp.block<3, 3>(0, 3*(h-1)) = A.transpose()*A;

  //     // LOG(INFO)<<"ETE: "<<std::endl<<ch<<std::endl;
  //     // LOG(INFO)<<"F_tmp: "<<std::endl<<F_tmp.block<3, 3>(0, 3*(h-1))<<std::endl;

  //   }
  //   //这是k+1帧的
  //   Eigen::Matrix3d A = Utility::skewSymmetric(line_w.head(3))*Utility::skewSymmetric(line_c_map.tail(3));  //<F
  //   Eigen::Matrix3d B = Utility::skewSymmetric(line_w.head(3))*Rwc.transpose();          //<E
  //   Eigen::Matrix3d ch = B.transpose()*B;
  //   EtE += ch;
  //   FtE_tmp.block<3, 3>(0, 0) = A.transpose()*B;
  //   FtF_tmp.block<3, 3>(0, 0) = A.transpose()*A;

  //   Eigen::Matrix3d W = EtE.inverse();
  //   // 构造以下这个点的信息矩阵,是边缘化后的,它是一个稀疏的矩阵
  //   omega_horizon_t Delta_ell = omega_horizon_t::Zero();

  //   // 这里有错?
  //   // 先遍历列
  //   for (int j=1; j<=HORIZON; ++j) {

  //     Eigen::Ref<Eigen::Matrix3d> FtEcol = FtE_tmp.block<3, 3>(0, 3*(j-1));

  //     for (int i=j; i<=HORIZON; ++i) { // NOTE: i=j for lower triangle
  //       Eigen::Ref<Eigen::Matrix3d> FtFrow = FtE_tmp.block<3, 3>(0, 3*(i-1));
  //       Eigen::Matrix3d Dij = FtFrow*W*FtEcol.transpose();
  //       if (i == j) {
  //         // diagonal
  //         Eigen::Ref<Eigen::Matrix3d> FtF = FtF_tmp.block<3, 3>(0, 3*(i-1));
  //         Delta_ell.block<3, 3>(9*i, 9*j) = FtF - Dij;
  //       } else {
  //         // FlT*El
  //         // lower triangle
  //         Delta_ell.block<3, 3>(9*i, 9*j) = -Dij;
  //         // ElT*Fl
  //         // upper triangle
  //         Delta_ell.block<3, 3>(9*j, 9*i) = -Dij.transpose();
  //       }
  //     }


  //   }
  //   Delta_ells[feature_id] = Delta_ell;
  //   // LOG(INFO)<<std::endl<<Delta_ell<<std::endl;
  // }
  // // image.swap(image_new);
  // ROS_INFO("number_select_lines: %d",count);
  return Delta_ells;
}
// ----------------------------------------------------------------------------
/**
 * @description: 计算点特征的信息
 * @return {*}
 * @author: fish
 */
std::map<int, omega_horizon_t> FeatureSelector::calcInfoFromFeatures(
                                            const image_t& image,
                                            const state_horizon_t& state_kkH)
{
  std::map<int, omega_horizon_t> Delta_ells;

  const auto& t_WC_k1 = state_k1_.first.segment<3>(xPOS) + state_k1_.second * t_IC_;
  const auto& q_WC_k1 = state_k1_.second * q_IC_;

  auto depthsByIdx = initKDTree();

  for (const auto& fpair : image) {

    // 相机数目
    constexpr int c = 0;

    // 特征点id
    int feature_id = fpair.first;
    Eigen::Vector3d feature = fpair.second[c].second.head<3>(); // calibrated [u v 1]

    // 深度
    double d = findNNDepth(depthsByIdx, feature.coeff(0), feature.coeff(1));
    feature = feature.normalized() * d;

    auto pell = t_WC_k1 + q_WC_k1 * feature;

    int numVisible = 1;

    // 构造信息矩阵
    Eigen::Matrix<double, 3, 3*HORIZON> Ch; 
    Ch.setZero();

    // Also sum up the Ch blocks for EtE;
    Eigen::Matrix3d EtE = Eigen::Matrix3d::Zero();

    // NOTE: start forward-simulating the landmark projection from k+2
    // (we have the frame k+1 projection, since that's where it came from)
    for (int h=2; h<=HORIZON; ++h) { 

      // convenience: future camera frame (k+h) w.r.t world frame
      const auto& t_WC_h = state_kkH[h].first.segment<3>(xPOS) + state_kkH[h].second * t_IC_;
      const auto& q_WC_h = state_kkH[h].second * q_IC_;

      // create bearing vector of feature w.r.t camera pose h
      Eigen::Vector3d uell = (q_WC_h.inverse() * (pell - t_WC_h)).normalized();

      Eigen::Vector2d pixels;
      m_camera_->spaceToPlane(uell, pixels);

      // If not visible from this pose, skip
      if (!inFOV(pixels)) continue;

      Eigen::Matrix3d Bh = Utility::skewSymmetric(uell)*((q_WC_h*q_IC_).inverse()).toRotationMatrix();
      Ch.block<3, 3>(0, 3*(h-1)) = Bh.transpose()*Bh;
      // Ft
      // Sum up block for EtE
      EtE += Ch.block<3, 3>(0, 3*(h-1));

      ++numVisible;
    }


    if (numVisible == 1) continue;


    Eigen::Matrix3d Bh = Utility::skewSymmetric(feature.normalized())
                                                *((q_WC_k1*q_IC_).inverse()).toRotationMatrix();
    Ch.block<3, 3>(0, 0) = Bh.transpose()*Bh;

    // add information to EtE
    EtE += Ch.block<3, 3>(0, 0);

    // Compute landmark covariance (should be invertible)
    Eigen::Matrix3d W = EtE.inverse();


    
    omega_horizon_t Delta_ell = omega_horizon_t::Zero();

    
    // 构造边缘化信息矩阵
    for (int j=1; j<=HORIZON; ++j) {
      // for convenience

      Eigen::Ref<Eigen::Matrix3d> Cj = Ch.block<3, 3>(0, 3*(j-1));

      for (int i=j; i<=HORIZON; ++i) { // NOTE: i=j for lower triangle
        // for convenience
        // FlT*Fl
        Eigen::Ref<Eigen::Matrix3d> Ci = Ch.block<3, 3>(0, 3*(i-1));
        Eigen::Matrix3d Dij = Ci*W*Cj.transpose();
        // 舒尔不
        // ElT*El
        if (i == j) {
          // diagonal
          Delta_ell.block<3, 3>(9*i, 9*j) = Ci - Dij;
        } else {
          // FlT*El
          // lower triangle
          Delta_ell.block<3, 3>(9*i, 9*j) = -Dij;
          // ElT*Fl
          // upper triangle
          Delta_ell.block<3, 3>(9*j, 9*i) = -Dij.transpose();
        }

      }
    }
  //  LOG(INFO)<<std::endl<<Delta_ell<<std::endl;
    // Store this information matrix with its associated feature ID
    Delta_ells[feature_id] = Delta_ell;
  }
  return Delta_ells;
}

// ----------------------------------------------------------------------------

/**
 * @description: 特征点有效值
 * @param {Vector2d&} 
 * @return {*}
 * @author: fish
 */
bool FeatureSelector::inFOV(const Eigen::Vector2d& p)
{
  constexpr int border = 0; // TODO: Could be good to have a border here
  int u = std::round(p.coeff(0));
  int v = std::round(p.coeff(1));
  return (border <= u && u < m_camera_->imageWidth() - border) && 
         (border <= v && v < m_camera_->imageHeight() - border);
}


/**
 * @description: kd树的构造
 * @return {*}
 * @author: fish
 */
std::vector<double> FeatureSelector::initKDTree()
{
  // setup dataset
  static std::vector<std::pair<double, double>> dataset;
  dataset.clear(); dataset.reserve(estimator_.f_manager.feature.size());

  std::vector<double> depths;
  depths.reserve(estimator_.f_manager.feature.size());

  for (const auto& it_per_id : estimator_.f_manager.feature) {


    int used_num = it_per_id.feature_per_frame.size();
    if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2)) continue;
    if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1) continue;


    int imu_i = it_per_id.start_frame;
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i = estimator_.Rs[imu_i] * (estimator_.ric[0] * pts_i + estimator_.tic[0]) + estimator_.Ps[imu_i];

    Eigen::Vector3d p_IL_k1 = state_k1_.second.inverse() * (w_pts_i - state_k1_.first.segment<3>(xPOS));
    Eigen::Vector3d p_CL_k1 = q_IC_.inverse() * (p_IL_k1 - t_IC_);
    

    w_pts_i = p_CL_k1 / p_CL_k1.coeff(2);
    double x = w_pts_i.coeff(0), y = w_pts_i.coeff(1);

    dataset.push_back(std::make_pair(x, y));
    depths.push_back(it_per_id.estimated_depth);
  }

  static PointCloud cloud(dataset);

  kdtree_.reset(new my_kd_tree_t(2/* dim */, cloud,
                nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
  kdtree_->buildIndex();

  return depths;
}


/**
 * @description: kd树的搜索
 * @return {*}
 * @author: fish
 */
double FeatureSelector::findNNDepth(const std::vector<double>& depths, 
                                    double x, double y)
{

  if (depths.size() == 0) return 1.0;

  // build query
  double query_pt[2] = { x, y };

  // do a knn search
  // TODO: Considering avg multiple neighbors?
  const size_t num_results = 1;
  size_t ret_index = 0;
  double out_dist_sqr;
  nanoflann::KNNResultSet<double> resultSet(num_results);
  resultSet.init(&ret_index, &out_dist_sqr);
  kdtree_->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

  return depths[ret_index];
}

// ----------------------------------------------------------------------------

omega_horizon_t FeatureSelector::calcInfoFromRobotMotion(
                                    const state_horizon_t& state_kkH,
                                    double nrImuMeasurements, double deltaImu)
{
  // ** Build the large information matrix over the horizon (eq 15).
  //
  // There is a sparse structure to the information matrix that we can exploit.
  // We can calculate the horizon info. matrix in blocks. Notice that each
  // pair of consecutive frames in the horizon create four 9x9 sub-blocks.
  // For example, for a horizon of H=3, the first pair of images (h=1) creates
  // a large information matrix like the following (each block is 9x9):
  //
  //         |------------------------------------
  //         | At*Ω*A |  At*Ω  |    0   |    0   |
  //         |------------------------------------
  //         |   Ω*A  |    Ω   |    0   |    0   |
  //         |------------------------------------
  //         |    0   |    0   |    0   |    0   |
  //         |------------------------------------
  //         |    0   |    0   |    0   |    0   |
  //         |------------------------------------
  //
  // The four non-zero sub-blocks shift along the diagonal as we loop through
  // the horizon (i.e., for h=2 there are zeros on all the edges and for h=3
  // the Ω is in the bottom-right corner). Note that the Ai matrix must be
  // recomputed for each h. The Ω matrix is calculated as the inverse of
  // the covariance in equation (52) and characterizes the noise in a
  // preintegrated set of IMU measurements using the linear IMU model.

  // NOTE: We are even more clever and only calculate the upper-triangular
  // and then transpose since this is a symmetric PSD matrix

  omega_horizon_t Omega_kkH = omega_horizon_t::Zero();

  for (int h=1; h<=HORIZON; ++h) { // for consecutive frames in horizon

    // convenience: frames (i, j) are a consecutive pair in horizon
    const auto& Qi = state_kkH[h-1].second;
    const auto& Qj = state_kkH[h].second;

    // Create Ablk and Ω as explained in the appendix
    auto mats = createLinearImuMatrices(Qi, Qj, nrImuMeasurements, deltaImu);

    // convenience: select sub-blocks to add to, based on h
    Eigen::Ref<omega_t> block1 = Omega_kkH.block<STATE_SIZE, STATE_SIZE>((h-1)*STATE_SIZE, (h-1)*STATE_SIZE);
    Eigen::Ref<omega_t> block2 = Omega_kkH.block<STATE_SIZE, STATE_SIZE>((h-1)*STATE_SIZE, h*STATE_SIZE);
    Eigen::Ref<omega_t> block3 = Omega_kkH.block<STATE_SIZE, STATE_SIZE>(h*STATE_SIZE, (h-1)*STATE_SIZE);
    Eigen::Ref<omega_t> block4 = Omega_kkH.block<STATE_SIZE, STATE_SIZE>(h*STATE_SIZE, h*STATE_SIZE);

    // At*Ω*A (top-left sub-block)
    block1 += mats.second.transpose()*mats.first*mats.second;

    // At*Ω (top-right sub-block)
    auto tmp = mats.second.transpose()*mats.first;
    block2 += tmp;

    // Ω*A (bottom-left sub-block)
    block3 += tmp.transpose();

    // Ω (bottom-right sub-block)
    block4 += mats.first;
  }

  return Omega_kkH;
}


/**
 * @description: 线性化IMU
 * @return {*}
 * @author: fish
 */
std::pair<omega_t, ablk_t> FeatureSelector::createLinearImuMatrices(
      const Eigen::Quaterniond& Qi, const Eigen::Quaterniond& Qj,
      double nrImuMeasurements, double deltaImu)
{
  //
  // "Pre-integrate" future IMU measurements over horizon
  //

  // helper matrices, equations (47) and (48)
  Eigen::Matrix3d Nij = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Mij = Eigen::Matrix3d::Zero();

  // initialize block coefficients
  double CCt_11 = 0;
  double CCt_12 = 0;

  // This is an IMU-rate for loop
  for (int i=0; i<nrImuMeasurements; ++i) {

    // slerp from Qi toward Qj by where we are in between the frames
    // (this will never slerp all the way to Qj)
    auto q = Qi.slerp(i/static_cast<double>(nrImuMeasurements), Qj);

    // so many indices...
    double jkh = (nrImuMeasurements - i - 0.5);
    Nij += jkh * q.toRotationMatrix();
    Mij += q.toRotationMatrix();

    // entries of CCt
    CCt_11 += jkh*jkh;
    CCt_12 += jkh;
  }

  // powers of IMU sampling period
  const double deltaImu_2 = deltaImu*deltaImu;
  const double deltaImu_3 = deltaImu_2*deltaImu;
  const double deltaImu_4 = deltaImu_3*deltaImu;

  //
  // Build cov(eta^imu_ij) -- see equation (52)
  //

  // NOTE: In paper, bottom right entry of CCt should have (j-k), not (j-k-1).
  omega_t covImu = omega_t::Zero();
  covImu.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity()
          * nrImuMeasurements * CCt_11 * deltaImu_4 * accVarDTime_;
  covImu.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity()
          * CCt_12 * deltaImu_3 * accVarDTime_;
  covImu.block<3, 3>(3, 0) = covImu.block<3, 3>(0, 3).transpose();
  covImu.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity()
          * nrImuMeasurements * deltaImu_2 * accVarDTime_;
  covImu.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity()
          * nrImuMeasurements * accBiasVarDTime_;

  //
  // Build Ablk -- see equation (50)
  //

  Nij *= deltaImu_2;
  Mij *= deltaImu;

  ablk_t Ablk = -ablk_t::Identity();
  Ablk.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * nrImuMeasurements*deltaImu;
  Ablk.block<3, 3>(0, 6) = Nij;
  Ablk.block<3, 3>(3, 6) = Mij;

  return std::make_pair(covImu.inverse(), Ablk);
}

// ----------------------------------------------------------------------------

void FeatureSelector::addOmegaPrior(Eigen::Ref<omega_horizon_t> Omega)
{
  // upper-left sub-block -- use identity for now to keep det(OmegakkH) > 0
  omega_t OmegaPrior = omega_t::Identity();

  // Add the prior to (upper-left) OmegaIMU (input)
  Omega.block<STATE_SIZE, STATE_SIZE>(0, 0) += OmegaPrior;
}

// ----------------------------------------------------------------------------

/**
 * @description: 选择特征点
 * @return {*}
 * @author: fish
 */
std::vector<int> FeatureSelector::selectInformativeFeatures(image_t& subset,
            const image_t& image, int kappa, const omega_horizon_t& Omega_kkH,
            const std::map<int, omega_horizon_t>& Delta_ells,
            const std::map<int, omega_horizon_t>& Delta_used_ells)
{
  // Combine motion information with information from features that are already
  // being used in the VINS-Mono optimization backend
  omega_horizon_t Omega = Omega_kkH;
  for (const auto& Delta : Delta_used_ells) {
    Omega += Delta.second;
  }

  // blacklist of already selected features (by id)
  std::vector<int> blacklist;
  blacklist.reserve(kappa);

  // combined information of subset
  omega_horizon_t OmegaS = omega_horizon_t::Zero();

  // select the indices of the best features
  for (int i=0; i<kappa; ++i) {

    // compute upper bounds in form of <UB, featureId> descending by UB
    auto upperBounds = sortedlogDetUB(Omega, OmegaS, Delta_ells, blacklist, image);

    // initialize the best cost function value and feature ID to worst case
    double fMax = -1.0;
    int lMax = -1;

    // iterating through upperBounds in descending order, check each feature
    for (const auto& fpair : upperBounds) {
      int feature_id = fpair.second;
      double ub = fpair.first;

      // lazy evaluation: break if UB is less than the current best cost
      if (ub < fMax) break;
 
      // convenience: the information matrix corresponding to this feature
      const auto& Delta_ell = Delta_ells.at(feature_id);

      // find probability of this feature being tracked
      double p = image.at(feature_id)[0].second.coeff(fPROB);
      // LOG(INFO)<<"p:"<<p<<std::endl;

      // calculate logdet efficiently
      double fValue = Utility::logdet(Omega + OmegaS + p*Delta_ell, true);

      // nan check
      if (std::isnan(fValue)) ROS_ERROR_STREAM("logdet returned nan!");

      // store this feature/reward if better than before
      if (fValue > fMax) {
        fMax = fValue;
        lMax = feature_id;
      }
    }

    // if lMax == -1 there was likely a nan (probably because roundoff error
    // caused det(M) < 0). I guess there just won't be a feature this iter.
    if (lMax > -1) {
      // Accumulate combined feature information in subset
      double p = image.at(lMax)[0].second.coeff(fPROB);
      OmegaS += p*Delta_ells.at(lMax);

      // add feature that returns the most information to the subset
      subset[lMax] = image.at(lMax);

      // mark as used
      blacklist.push_back(lMax);
    }
  }

  // which new features were selected
  return blacklist;
}

/**
 * @description: 选择线特征
 * @return {*}
 * @author: fish
 */
std::vector<int> FeatureSelector::selectInformativeFeaturesLines(image_t& subset,
            const image_t& image, int kappa, const omega_horizon_t& Omega_kkH,
            const std::map<int, omega_horizon_t>& Delta_ells,
            const std::map<int, omega_horizon_t>& Delta_used_ells)
{
  // Combine motion information with information from features that are already
  // being used in the VINS-Mono optimization backend
  omega_horizon_t Omega = Omega_kkH;
  for (const auto& Delta : Delta_used_ells) {
    Omega += Delta.second;
  }

  // blacklist of already selected features (by id)
  std::vector<int> blacklist;
  blacklist.reserve(kappa);

  // combined information of subset
  omega_horizon_t OmegaS = omega_horizon_t::Zero();

  // select the indices of the best features
  for (int i=0; i<kappa; ++i) {

    // compute upper bounds in form of <UB, featureId> descending by UB
    auto upperBounds = sortedlogDetUB(Omega, OmegaS, Delta_ells, blacklist, image);

    // initialize the best cost function value and feature ID to worst case
    double fMax = -1.0;
    int lMax = -1;

    // iterating through upperBounds in descending order, check each feature
    for (const auto& fpair : upperBounds) {
      int feature_id = fpair.second;
      double ub = fpair.first;

      // lazy evaluation: break if UB is less than the current best cost
      if (ub < fMax) break;
 
      // convenience: the information matrix corresponding to this feature
      const auto& Delta_ell = Delta_ells.at(feature_id);

      // find probability of this feature being tracked
      double p = image.at(feature_id)[0].second.coeff(fPROB);
      // LOG(INFO)<<"lines p:"<<p<<std::endl;

      // calculate logdet efficiently
      double fValue = Utility::logdet(Omega + OmegaS + p*Delta_ell, true);

      // nan check
      if (std::isnan(fValue)) ROS_ERROR_STREAM("logdet returned nan!");

      // store this feature/reward if better than before
      if (fValue > fMax) {
        fMax = fValue;
        lMax = feature_id;
      }
    }

    // if lMax == -1 there was likely a nan (probably because roundoff error
    // caused det(M) < 0). I guess there just won't be a feature this iter.
    if (lMax > -1) {
      // Accumulate combined feature information in subset
      double p = image.at(lMax)[0].second.coeff(fPROB);
      OmegaS += p*Delta_ells.at(lMax);

      // add feature that returns the most information to the subset
      subset[lMax] = image.at(lMax);

      // mark as used
      blacklist.push_back(lMax);
    }
  }

  // which new features were selected
  return blacklist;
}
// ----------------------------------------------------------------------------

/**
 * @description: 阈值计算
 * @return {*}
 * @author: fish
 */
std::map<double, int, std::greater<double>> FeatureSelector::sortedlogDetUB(
  const omega_horizon_t& Omega, const omega_horizon_t& OmegaS,
  const std::map<int, omega_horizon_t>& Delta_ells,
  const std::vector<int>& blacklist, const image_t& image)
{
  // returns a descending sorted map with upper bound as the first key,
  // and feature id as the value for all features in image
  std::map<double, int, std::greater<double>> UBs;

  // Partially create argument to UB function (see eq 9). The only thing
  // missing from this is the additive and expected information from the
  // l-th feature. Each is added one at a time (independently) to calc UB
  const omega_horizon_t M = Omega + OmegaS;

  // Find the upper bound of adding each Delta_ell to M independently
  for (const auto& fpair : Delta_ells) {
    int feature_id = fpair.first;

    // if a feature was already selected, do not calc UB. Not including it
    // in the UBs prevents it from being selected again.
    bool in_blacklist = std::find(blacklist.begin(), blacklist.end(),
                                  feature_id) != blacklist.end();
    if (in_blacklist) continue;

    // find probability of this feature being tracked
    double p = image.at(feature_id)[0].second.coeff(fPROB);

    // construct the argument to the logdetUB function
    omega_horizon_t A = M + p*Delta_ells.at(feature_id);

    // calculate upper bound (eq 29)
    double ub = A.diagonal().array().log().sum();

    // store in map for automatic sorting (desc) and easy lookup
    UBs[ub] = feature_id;
  }

  return UBs;
}