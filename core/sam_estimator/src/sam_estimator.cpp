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
  
  if (!initialized_) {
    //  initialize the graph using the first GPS measurement
    estimates_.insert(PoseKey(), gps_pose);
    estimates_.insert(VelocityKey(), gps_vel);
    //  initial bias estimate is all zero
    constantBias_ = imuBias::ConstantBias(Vector3(0,0,0), Vector3(0,0,0));
    estimates_.insert(BiasKey(), constantBias_);

    //  prior on pose
    const kr::mat<double,6,6> prcov = measurement.cov.block<6,6>(0,0);
    auto pose_noise = gaussianNoiseModel(prcov);
    graph_.add(PriorFactor<Pose3>(PoseKey(),
                                  gps_pose, pose_noise));
    
    //  prior on velocity (assume diagonal for now)
    const kr::mat<double,3,3> velcov = measurement.cov.block<3,3>(6,6);
    auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(velcov(0,0),velcov(1,1),velcov(2,2)));
    graph_.add(PriorFactor<LieVector>(VelocityKey(),gps_vel,vel_noise));
    
    //  prior on bias
    const auto avar = Config().accel_bias_std*Config().accel_bias_std;
    const auto gvar = Config().gyro_bias_std*Config().gyro_bias_std;
    Vector6 bias_sig;
    for (int i=0; i < 3; i++) {
      bias_sig[i] = avar;
      bias_sig[i+3] = gvar;
    }
    auto bias_noise = noiseModel::Diagonal::Sigmas(bias_sig);
    graph_.add(PriorFactor<imuBias::ConstantBias>(BiasKey(),
                                                  constantBias_,bias_noise));
    
    initialized_ = true;
  } else {    
    //  process backlog of IMU measurements
    ImuFactor imu_factor;
    if (CreateImuFactor(measurement.time, imu_factor)) {
      graph_.add(imu_factor);
    }
    
    
  }
 
  //  only at the very end, increment the pose key
  meas_index_++;
}

void SamEstimator::Initialize() {
  
}

void SamEstimator::Optimize() {
  
}

bool SamEstimator::CreateImuFactor(Timestamp time, gtsam::ImuFactor& factor) {
  //  consume current buffer if IMU measurements
  if (imu_buffer_.empty()) {
    return false;
  } else if (imu_buffer_.front().time > time) {
    return false;
  }
  
  ImuFactor::PreintegratedMeasurements meas(constantBias_,
                                            isotropicMat<3>(Config().accel_std),
                                            isotropicMat<3>(Config().gyro_std),
                                            isotropicMat<3>(Config().integration_std));
  while (!imu_buffer_.empty()) {
    const ImuMeasurement& imu_meas = imu_buffer_.front();
    if (imu_meas.time > time) {
      break;  //  should be integrated after this time
    }
    
    meas.integrateMeasurement(imu_meas.z.head(3), imu_meas.z.tail(3), imu_meas.dt);
    imu_buffer_.pop_front();
  }
  
  //  create factor from combined measurements
  //  for now set coriolis velocity to zero
  factor = ImuFactor(PoseKey(-1),VelocityKey(-1),
                     PoseKey(),VelocityKey(),BiasKey(),
                     meas,Vector3(0,0,-Config().gravity_mag),Vector3(0,0,0));
  return true;
}

gtsam::Symbol SamEstimator::PoseKey(int rel) const {
  assert(meas_index_ + rel >= 0);
  return gtsam::Symbol('x', meas_index_ + rel);
}

gtsam::Symbol SamEstimator::VelocityKey(int rel) const {
  assert(meas_index_ + rel >= 0);
  return gtsam::Symbol('v', meas_index_ + rel);  
}

gtsam::Symbol SamEstimator::BiasKey(int rel) const {
  assert(meas_index_ + rel >= 0);
  return gtsam::Symbol('b', meas_index_ + rel);
}

}  // namespace sam_estimator
}  // namespace galt
