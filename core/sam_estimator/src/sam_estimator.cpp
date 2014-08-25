/*
 * sam_estimator.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#include <sam_estimator/sam_estimator.hpp>
#include <iostream>

#include <ros/ros.h>

using namespace gtsam;

namespace galt {
namespace sam_estimator {

/// gaussian noise model from NxN covariance matrix
template <int N, typename T>
static noiseModel::Gaussian::shared_ptr gaussianNoiseModel(const kr::mat<T,N,N>& m) {
  return noiseModel::Gaussian::Covariance(m.template cast<double>());
}

/// initialize an isotropic covariance matrix
template <int N>
static kr::mat<double,N,N> isotropicMat(const double& std) {
  kr::mat<double,N,N> mat;
  mat.setZero();
  for (int i=0; i < N; i++) {
    mat(i,i) = std*std;
  }
  return mat;
}

SamEstimator::SamEstimator() : has_vo_pose_(false), rotation_set_(false), meas_index_(0), initialized_(false) {}

void SamEstimator::AddImu(const ImuMeasurement &measurement) {
  if (!IsInitialized()) {
    throw exception("Estimator must be initialized before calling AddImu");
  }
  imu_buffer_.push_back(measurement);
}

void SamEstimator::AddVo(const VoMeasurement& measurement) {
  
  //  convert to IMU frame
  const auto iRc = Eigen::AngleAxisd(M_PI, kr::vec3d(0,1,0));
  const auto iTc = Eigen::Vector3d(0.09,-0.045,0);
  const kr::Posed iPc(kr::quatd(iRc.matrix()),iTc);  
  const kr::Posed imu_vo_pose = measurement.pose.composeInBody(iPc);
  
  if (has_vo_pose_) {
    //  convert into a relative measurement
    const kr::Posed inc_pose = imu_vo_pose.expressedIn(last_vo_pose_);
    VoMeasurement rec = measurement;
    rec.pose = inc_pose;
    vo_buffer_.push_back(rec);
  }
  last_vo_pose_ = imu_vo_pose;
  has_vo_pose_ = true;
  
  //  run through queued measurements
  while(ProcessQueues());
}

void SamEstimator::InitializeGraph(const kr::Posed& first_pose,
                                   const Vector6& sigmas, Timestamp start_time) {
  if (!initialized_) {
    
    const Pose3 initial_pose = static_cast<gtsam::Pose3>(first_pose);
    current_pose_ = initial_pose;
    estimates_.insert(CurPoseKey(), current_pose_);

    //  prior on pose
    auto noise_model = noiseModel::Diagonal::Sigmas(sigmas);
    graph_.add(PriorFactor<Pose3>(CurPoseKey(), current_pose_, noise_model));
    
    ROS_INFO("Graph is initialized");
    initialized_ = true;
    meas_index_++;
  }
}

bool SamEstimator::ProcessQueues() {
  //  determine which measurement to process next
  bool have_imu = !imu_buffer_.empty();
  bool have_vo = !vo_buffer_.empty();
  
  if (!have_imu && !have_vo) {
    return false;
  }
  
  const Timestamp imu_time = have_imu ? imu_buffer_.front().time :
        std::numeric_limits<double>::infinity();
  const Timestamp vo_time = have_vo ? vo_buffer_.front().time :
        std::numeric_limits<double>::infinity();
  
  if (imu_time <= vo_time) {
    HandleImu(imu_buffer_.front());
    imu_buffer_.pop_front();
  } else {
    HandleVo(vo_buffer_.front());
    vo_buffer_.pop_front();
  }
  
  return true;
}

void SamEstimator::HandleImu(const ImuMeasurement& imu) {
  //  last orientation read from filter
  last_rotation_ = imu.wQb;
  last_rotation_cov_ = imu.cov;
  has_rotation_ = true;
}

void SamEstimator::HandleVo(const VoMeasurement& vo) {
  if (!has_rotation_) {
    return;
  }
  
  const kr::Posed inc_pose = vo.pose;
  const kr::quatd inc_rot = last_vo_rotation_.conjugate() * last_rotation_;
  last_vo_rotation_ = last_rotation_;
  
  if (!rotation_set_) {
    rotation_set_ = true;
    return; //  wait until next one...
  }
  
  const Pose3 inc_vo_pose = static_cast<gtsam::Pose3>(inc_pose);
  const Pose3 previous_pose_ = current_pose_; //  last pose estimate
  
  //  some made up bullshit noise on the vo
  Vector6 vo_noise_sigmas;
  vo_noise_sigmas << 0.7,0.7,0.7,0.2,0.2,0.2;
  const auto vo_noise = noiseModel::Diagonal::Sigmas(vo_noise_sigmas);
  
  graph_.add(BetweenFactor<Pose3>(PrevPoseKey(),
                                  CurPoseKey(),
                                  inc_vo_pose,
                                  vo_noise));
  //  more hacky crap
  const Pose3 inc_rot_pose(Rot3(inc_rot),Point3(0,0,0));
  
  kr::mat<double,6,6> inc_rot_cov;
  inc_rot_cov.setZero();
  inc_rot_cov.block<3,3>(0,0) = last_rotation_cov_;
  for (int i=0; i < 3; i++) {
    inc_rot_cov(i+3,i+3) = 10000.0; //  huge variance on incorrect position
  }
  graph_.add(BetweenFactor<Pose3>(PrevPoseKey(),
                                 CurPoseKey(),
                                 inc_rot_pose,
                                 gaussianNoiseModel(inc_rot_cov)));
  
  estimates_.insert(CurPoseKey(), previous_pose_);
  
  //  optimize
  PerformUpdate();
}

void SamEstimator::PerformUpdate() {  
  //  perform ISAM2 update and reset the graph
  isam_.update(graph_, estimates_);
  
  estimates_ = isam_.calculateEstimate();
  current_pose_ = estimates_.at<Pose3>(CurPoseKey());
  
  //  get variance
  //gtsam::Marginals marginals(graph_,estimates_);
  //current_marginals_ = marginals.marginalCovariance(CurPoseKey());
  
  all_poses_.clear();
  for (int i=0; i < meas_index_; i++) {
    all_poses_.push_back(kr::Posed(estimates_.at<Pose3>(PoseKey(i))));
  }
  
  estimates_.clear();
  graph_.resize(0); 
  
  //  advance one index
  meas_index_++;
}

gtsam::noiseModel::Diagonal::shared_ptr SamEstimator::BiasNoiseModel() const {
  //  note: currently unused
  Vector6 noise;
  auto accel_var = config_.accel_bias_std*config_.accel_bias_std;
  auto gyro_var = config_.gyro_bias_std*config_.gyro_bias_std;
  noise << accel_var,accel_var,accel_var,gyro_var,gyro_var,gyro_var;
  return noiseModel::Diagonal::Sigmas(noise);
}

gtsam::Symbol SamEstimator::PoseKey(int index) const {
  assert(index >= 0 && index <= meas_index_);
  return gtsam::Symbol('x', index);
}

gtsam::Symbol SamEstimator::VelKey(int index) const {
  assert(index >= 0 && index <= meas_index_);
  return gtsam::Symbol('v', index);  
}

gtsam::Symbol SamEstimator::BiasKey(int index) const {
   assert(index >= 0 && index <= meas_index_);
  return gtsam::Symbol('b', index);
}

}  // namespace sam_estimator
}  // namespace galt
