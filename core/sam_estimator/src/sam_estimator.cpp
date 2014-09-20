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

SamEstimator::SamEstimator() : meas_index_(0), initialized_(false) {}

void SamEstimator::AddFrame(const kr::Posed& pose, 
                            const kr::mat<double,6,6>& pose_cov,
                const std::vector<stereo_vo::FeatureMsg> &feat_left,
                const std::vector<stereo_vo::FeatureMsg> &feat_right) {
  
  if (!calib_) {
    return; //  waiting for calibration
  }
  
  //  gaussian model of noise is taken from GPS-KF
  auto odom_noise_model = noiseModel::Gaussian::Covariance(pose_cov);
  if (!IsInitialized()) {
    //  check difference
    const kr::Posed diff = pose.expressedIn(previous_odom_);
    if (diff.p().norm() < 0.3) { /// @todo: make this a parameter
      //  wait until we've moved a bit for the first pose
      return;
    }
    
    //  add first pose
    const Pose3 initial_pose = static_cast<gtsam::Pose3>(pose);
    estimates_.insert(CurPoseKey(), initial_pose);
    
    //  fixed prior factor 
    graph_.add(PriorFactor<Pose3>(CurPoseKey(), initial_pose, odom_noise_model));
    ROS_INFO("Adding initialization factor");
  } else {
    const kr::Posed inc_pose = pose.expressedIn(previous_odom_);
    const Pose3 p3increment = static_cast<Pose3>(inc_pose);
    
    //  insert GPS/IMU data as 'fake odometry'
    graph_.add(BetweenFactor<Pose3>(PrevPoseKey(),
                                    CurPoseKey(),
                                    p3increment, odom_noise_model));
    //  initial guess
    estimates_.insert(CurPoseKey(), static_cast<Pose3>(pose));
  }
  
  const auto pixel_noise = noiseModel::Isotropic::Sigma(2, 1.0);
  gtsam::Cal3_S2::shared_ptr single_calib;
  single_calib.reset(new gtsam::Cal3_S2(calib_->calibration()));
  if (feat_right.empty()) {
    //  normal frame, add regular reprojection factors for the left camera
    for (const stereo_vo::FeatureMsg& f : feat_left) {
      
      Point2 point(f.point.x, f.point.y);
      ProjectionFactor factor(point, pixel_noise, CurPoseKey(),
                              Symbol('l', f.id), single_calib, cam_pose_in_body_);
      graph_.add(factor);
    }
  } else {
    assert(feat_left.size() == feat_right.size());
    
    //  key-frame, add stereo factors
    for (size_t i=0; i < feat_left.size(); i++) {
      const stereo_vo::FeatureMsg& f_left = feat_left[i];
      const stereo_vo::FeatureMsg& f_right = feat_right[i];
      
      StereoPoint2 point(f_left.point.x,
                         f_right.point.x,
                         f_right.point.y);
      
      Symbol feature_sym('l', f_left.id);
      StereoProjectionFactor factor(point, pixel_noise,
                                    CurPoseKey(), feature_sym, 
                                    calib_, cam_pose_in_body_);
      graph_.add(factor);
    }
  }
  
  if (IsInitialized()) {
    //  start optimization after a certain number of poses are collected
    PerformUpdate();
  }
  
  previous_odom_ = pose;
  meas_index_++;
}

void SamEstimator::PerformUpdate() {  
  //  perform ISAM2 update and reset the graph
  isam_.update(graph_, estimates_);
  
  estimates_ = isam_.calculateEstimate();
  Pose3 optimized_pose = estimates_.at<Pose3>(CurPoseKey());
  
  //  get variance
  gtsam::Marginals marginals(graph_, estimates_);
  //current_marginals_ = marginals.marginalCovariance(CurPoseKey());
  
//  all_poses_.clear();
//  for (int i=0; i < meas_index_; i++) {
//    all_poses_.push_back(kr::Posed(estimates_.at<Pose3>(PoseKey(i))));
//  }
  
  estimates_.clear();
  graph_.resize(0);
}

void SamEstimator::SetCalibration(double fx, double fy, double cx, 
                                  double cy, double b) {
  calib_.reset(new Cal3_S2Stereo(fx,fy,0,cx,cy,b));
}

void SamEstimator::SetSensorPose(const kr::Posed& cam_pose) {
  cam_pose_in_body_ = static_cast<Pose3>(cam_pose);
}

gtsam::Symbol SamEstimator::PoseKey(int index) const {
  assert(index >= 0 && index <= meas_index_);
  return gtsam::Symbol('x', index);
}

}  // namespace sam_estimator
}  // namespace galt
