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

SamEstimator::SamEstimator() : meas_index_(0), initialized_(false) {}

void SamEstimator::AddImu(const ImuMeasurement &measurement) {
  
  if (!IsInitialized()) {
    throw exception("AddImu may only be called after initialization");
  }
  
  imu_buffer_.push_back(measurement);
}

void SamEstimator::AddGps(const GpsMeasurement &measurement) {
  
  //  convert to gtsam pose
  const Pose3 gps_pose = static_cast<gtsam::Pose3>(measurement.pose);
  const LieVector gps_vel(measurement.vel.cast<double>());
  
  //  noise on GPS pose
  const kr::mat<double,6,6> prcov = measurement.cov.block<6,6>(0,0);
  auto pose_noise = gaussianNoiseModel(prcov);
  
  //  noise on velocity (assume diagonal)
  const kr::mat<double,3,3> velcov = measurement.cov.block<3,3>(6,6);
  auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(velcov(0,0),
                                                        velcov(1,1),
                                                        velcov(2,2)));
  
  //  noise on the bias prior
  const auto avar = Config().accel_bias_std*Config().accel_bias_std;
  const auto gvar = Config().gyro_bias_std*Config().gyro_bias_std;
  Vector6 bias_sig;
  for (int i=0; i < 3; i++) {
    bias_sig[i] = avar;
    bias_sig[i+3] = gvar;
  }
  //  initial guess for imu bias
  const imuBias::ConstantBias zero_bias(Vector3(0,0,0),Vector3(0,0,0));
  
  if (!initialized_) {
    std::cout << "Initializing SAM estimator from GPS pose" << std::endl;
    currentPose_ = gps_pose;
    currentVelocity_ = gps_vel;
    currentBias_ = zero_bias;
    
    //  initialize the graph using the first GPS measurement
    estimates_.insert(CurPoseKey(), currentPose_);
    estimates_.insert(CurVelKey(), currentVelocity_);
    //  initial bias estimate is all zero
    estimates_.insert(CurBiasKey(), currentBias_);

    //  prior on pose, velocity and bias
    graph_.add(PriorFactor<Pose3>(CurPoseKey(),
                                  currentPose_, pose_noise));
    graph_.add(PriorFactor<LieVector>(CurVelKey(),currentVelocity_,vel_noise));
    
    auto bias_noise = noiseModel::Diagonal::Sigmas(bias_sig);
    graph_.add(PriorFactor<imuBias::ConstantBias>(CurBiasKey(),
                                                  currentBias_,bias_noise));
    initialized_ = true;
  } else {
    //  process backlog of IMU measurements
    ImuFactor imu_factor;
    int imu_count;
    if (CreateImuFactor(measurement.time, imu_factor, imu_count)) {
      graph_.add(imu_factor);
      /// @todo: why is this zero bias?
      auto bias_noise = noiseModel::Diagonal::Sigmas(bias_sig * std::sqrt(imu_count));
      BetweenFactor<imuBias::ConstantBias> bias_factor(PrevBiasKey(),CurBiasKey(),
                                                       zero_bias,bias_noise);
      graph_.add(bias_factor);
    }
    
    //  add a factor for the new GPS measurement
    graph_.add(PriorFactor<Pose3>(CurPoseKey(),gps_pose,pose_noise));
    graph_.add(PriorFactor<LieVector>(CurVelKey(),gps_vel,vel_noise));
    
    //  insert initial guesses for GPS measurement
    estimates_.insert(CurPoseKey(), gps_pose);
    estimates_.insert(CurVelKey(), currentVelocity_);
    estimates_.insert(CurBiasKey(), currentBias_);
    
    if (meas_index_ > 10) {
      //  perform optimizationa after the graph is built out a bit
      isam_.update(graph_, estimates_);
     
      //  update estimates
      estimates_ = isam_.calculateEstimate();
      currentPose_ = estimates_.at<Pose3>(CurPoseKey());
      currentVelocity_ = estimates_.at<LieVector>(CurVelKey());
      currentBias_ = estimates_.at<imuBias::ConstantBias>(CurBiasKey());
      
      estimates_.clear();
      graph_.resize(0);
    } else {
      //  until first optimization just consider these as truth
      currentPose_ = gps_pose;
      currentVelocity_ = gps_vel;
    }
  }
 
  allPoses_.push_back(currentPose_);
  
  //  only at the very end do we increment the pose key
  meas_index_++;
}

void SamEstimator::Initialize() {
  
}

void SamEstimator::Optimize() {
  
}

bool SamEstimator::CreateImuFactor(Timestamp time, 
                                   gtsam::ImuFactor& factor,
                                   int& count) {
  //  consume current buffer if IMU measurements
  if (imu_buffer_.empty()) {
    return false;
  } else if (imu_buffer_.front().time > time) {
    return false;
  }
  
  ImuFactor::PreintegratedMeasurements meas(currentBias_,
                                            isotropicMat<3>(Config().accel_std),
                                            isotropicMat<3>(Config().gyro_std),
                                            isotropicMat<3>(Config().integration_std));
  
  count = 0;
  while (!imu_buffer_.empty()) {
    const ImuMeasurement& imu_meas = imu_buffer_.front();
    if (imu_meas.time > time) {
      break;  //  should be integrated after this time
    }
    
    meas.integrateMeasurement(imu_meas.z.head(3), imu_meas.z.tail(3), imu_meas.dt);
    imu_buffer_.pop_front();
    count++;
  }
  
  //  create factor from combined measurements
  //  for now set coriolis velocity to zero
  factor = ImuFactor(PrevPoseKey(),PrevVelKey(),
                     CurPoseKey(),CurVelKey(),CurBiasKey(),
                     meas,Vector3(0,0,-Config().gravity_mag),Vector3(0,0,0));
  return true;
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
