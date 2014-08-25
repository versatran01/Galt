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
  imu_buffer_.push_back(measurement);
}

void SamEstimator::AddGps(const GpsMeasurement &measurement) {
  if (!IsInitialized()) {
    throw exception("Estimator must be initialized before calling AddGps");
  }
  gps_buffer_.push_back(measurement);
  //while(ProcessQueues());
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
    
    //  set zero initial velocity and bias
    current_velocity_ = LieVector(Vector3(0,0,0));
    estimates_.insert(CurVelKey(), current_velocity_);
    current_bias_ = imuBias::ConstantBias(Vector3(0,0,0),Vector3(0,0,0));
    estimates_.insert(CurBiasKey(), current_bias_);
    
    //  corresponding priors
    auto vel_noise = noiseModel::Isotropic::Sigma(3, 0.3);
    graph_.add(PriorFactor<LieVector>(CurVelKey(),current_velocity_,vel_noise));
    graph_.add(PriorFactor<imuBias::ConstantBias>(CurBiasKey(),current_bias_,
                                                  BiasNoiseModel()));
    
    pre_imu_ = ImuFactor::PreintegratedMeasurements(current_bias_,
                                           isotropicMat<3>(Config().accel_std),
                                           isotropicMat<3>(Config().gyro_std),
                                           isotropicMat<3>(Config().integration_std));
    
    //  get rid of anything before this time
    while (!imu_buffer_.empty()) {
      if (imu_buffer_.front().time < start_time) {
        imu_buffer_.pop_front();
      } else {
        break;
      }
    }
    while (!vo_buffer_.empty()) {
      if (vo_buffer_.front().time < start_time) {
        vo_buffer_.pop_front();
      } else {
        break;
      }
    }
    
    ROS_INFO("Graph is initialized");
    initialized_ = true;
    meas_index_++;
  }
}

double SamEstimator::OldestTimestamp() const {
  double TS = std::numeric_limits<double>::infinity();
  
  if (!imu_buffer_.empty()) {
    TS = std::min(imu_buffer_.back().time, TS);
  }
  if (!vo_buffer_.empty()) {
    TS = std::min(vo_buffer_.back().time, TS);
  }
  if (!gps_buffer_.empty()) {
    TS = std::min(gps_buffer_.back().time, TS);
  }
  
  return TS;
}

bool SamEstimator::ProcessQueues() {
  //  determine which measurement to process next
  bool have_imu = !imu_buffer_.empty();
  bool have_vo = !vo_buffer_.empty();
  bool have_gps = !gps_buffer_.empty();
  
  if (!have_imu && !have_vo && !have_gps) {
    return false;
  }
  
  const Timestamp imu_time = have_imu ? imu_buffer_.front().time :
        std::numeric_limits<double>::infinity();
  const Timestamp vo_time = have_vo ? vo_buffer_.front().time :
        std::numeric_limits<double>::infinity();
  const Timestamp gps_time = have_gps ? gps_buffer_.front().time :
        std::numeric_limits<double>::infinity();
  
  /// @todo: refactor this into something less ugly/stupid...
  struct order {
    int type;
    Timestamp time;
    order(int type, Timestamp time) : type(type), time(time) {}
    bool operator < (const order& rhs) const { return time < rhs.time; }
  };
  std::vector<order> meas({order(0,imu_time),order(1,vo_time),order(2,gps_time)});
  std::sort(meas.begin(),meas.end());
  
  switch(meas.front().type) {
  case 0:
    HandleImu(imu_buffer_.front());
    imu_buffer_.pop_front();
    break;
  case 1:
    HandleVo(vo_buffer_.front());
    vo_buffer_.pop_front();
    break;
  case 2:
    HandleGps(gps_buffer_.front());
    gps_buffer_.pop_front();
    break;
  default:
    assert(false);
  }
  
  return true;
}

void SamEstimator::HandleImu(const ImuMeasurement& imu) {
  //  last orientation read from filter
  last_rotation_ = imu.wQb;
  last_rotation_cov_ = imu.cov;
  has_rotation_ = true;
  
  //  integrate IMU model
  imu_int_count_++;
  pre_imu_.integrateMeasurement(imu.z.head(3), imu.z.tail(3), imu.dt);
}

void SamEstimator::HandleVo(const VoMeasurement& vo) {
  if (!has_rotation_) {
    return;
  }
  
  if (!imu_int_count_) {
    ROS_WARN("Skipping vo...");
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
  vo_noise_sigmas << 0.3,0.3,0.3,0.2,0.2,0.2;
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
  //estimates_.insert(CurVelKey(), current_velocity_);
  
  //  optimize
  PerformUpdate();
}

void SamEstimator::HandleGps(const GpsMeasurement& gps) {
  
  const Pose3 gps_pose = static_cast<gtsam::Pose3>(gps.pose);
  const LieVector gps_vel(gps.vel.cast<double>());
  
  //  some hacky nonsense...
  if (!imu_int_count_) {
    ROS_WARN("Skipping gps...");
    return;
  }
  
  ROS_INFO("Current: %f, %f, %f", current_pose_.translation().x(),
           current_pose_.translation().y(), current_pose_.translation().z());
  
  ROS_INFO("New: %f, %f, %f", gps_pose.translation().x(),
           gps_pose.translation().y(), gps_pose.translation().z());
    
  const double fudge_factor = 1000;
  
  //  noise on GPS pose
  //  scale to handle awful smoothing problem
  const kr::mat<double,6,6> prcov = gps.cov.block<6,6>(0,0) * fudge_factor;
  auto pose_noise = gaussianNoiseModel(prcov);
  
  const kr::mat<double,3,3> velcov = gps.cov.block<3,3>(6,6)*fudge_factor;
  auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(velcov(0,0),
                                                        velcov(1,1),
                                                        velcov(2,2)));
  
  graph_.add(PriorFactor<Pose3>(CurPoseKey(),gps_pose,pose_noise));
  graph_.add(PriorFactor<LieVector>(CurVelKey(),gps_vel,vel_noise));
      
  //  insert initial guesses for GPS measurement
  estimates_.insert(CurPoseKey(), gps_pose);
  estimates_.insert(CurVelKey(), gps_vel);
  
  PerformUpdate();
}

void SamEstimator::AddImuFactor() {
  //  add imu factors
  ImuFactor imu_factor(PrevPoseKey(),PrevVelKey(),
                       CurPoseKey(),CurVelKey(),CurBiasKey(),
                       pre_imu_,Vector3(0,0,-Config().gravity_mag),Vector3(0,0,0));
  graph_.add(imu_factor);
  
  ROS_INFO("Pre-int count: %i",  imu_int_count_);
  pre_imu_.print("preintegrated:");
  
  auto bias_noise = BiasNoiseModel();
  imuBias::ConstantBias zeroBias(Vector3(0,0,0),Vector3(0,0,0));
  BetweenFactor<imuBias::ConstantBias> bias_factor(PrevBiasKey(),CurBiasKey(),
                                                   zeroBias,bias_noise);
  graph_.add(bias_factor);
  
  //  initial guesses
  estimates_.insert(CurBiasKey(), current_bias_);
  imu_int_count_=0;
}

void SamEstimator::PerformUpdate() {
  //AddImuFactor();
  
  //  perform ISAM2 update and reset the graph
  isam_.update(graph_, estimates_);
  
  estimates_ = isam_.calculateEstimate();
  current_pose_ = estimates_.at<Pose3>(CurPoseKey());
  
  if (estimates_.find(CurBiasKey()) != estimates_.end()) {
    current_bias_ = estimates_.at<imuBias::ConstantBias>(CurBiasKey());
    Vector6 vec = current_bias_.vector();
    ROS_INFO("Biases: %f, %f, %f, %f, %f, %f",
             vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
  }
  if (estimates_.find(CurVelKey()) != estimates_.end()) {
    current_velocity_ = estimates_.at<LieVector>(CurVelKey());
    Vector3 vec = current_velocity_.vector();
    ROS_INFO("Velocity: %f, %f, %f", vec[0], vec[1], vec[2]);
  }
  
  //  reset IMU integrator
  pre_imu_ = ImuFactor::PreintegratedMeasurements(current_bias_,
                                         isotropicMat<3>(Config().accel_std),
                                         isotropicMat<3>(Config().gyro_std),
                                         isotropicMat<3>(Config().integration_std));
  
  all_poses_.clear();
  for (int i=0; i < meas_index_; i++) {
    all_poses_.push_back(kr::Posed(estimates_.at<Pose3>(PoseKey(i))));
  }
  //all_poses_.push_back(kr::Posed(current_pose_));
  
  estimates_.clear();
  graph_.resize(0); 
  
  //  advance one index
  meas_index_++;
}

gtsam::noiseModel::Diagonal::shared_ptr SamEstimator::BiasNoiseModel() const {
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
