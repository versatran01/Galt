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
#include <kr_math/feature.hpp>
#include <iostream>

#include <ros/ros.h>

using namespace gtsam;

namespace galt {
namespace sam_estimator {

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
    } else {
      ROS_INFO("Inserting another init pose");
    }
    
    //  add first pose
    const Pose3 initial_pose = static_cast<gtsam::Pose3>(pose);
    estimates_.insert(CurPoseKey(), initial_pose);
    
    //  fixed prior factor 
    graph_.add(PriorFactor<Pose3>(CurPoseKey(), initial_pose, odom_noise_model));
    ROS_INFO("Adding initialization factor");
  } else {
    ROS_INFO("Adding between factors");
    const kr::Posed inc_pose = pose.expressedIn(previous_odom_);
    const Pose3 p3increment = static_cast<Pose3>(inc_pose);
    
    //  insert GPS/IMU data as 'fake odometry'
    graph_.add(BetweenFactor<Pose3>(PrevPoseKey(),
                                    CurPoseKey(),
                                    p3increment, odom_noise_model));
    //  initial guess
    estimates_.insert(CurPoseKey(), static_cast<Pose3>(pose));
  }
  
  triangulated_points_.clear();
  
  const auto pixel_noise = noiseModel::Isotropic::Sigma(2, 1.0);
  gtsam::Cal3_S2::shared_ptr single_calib;  //  calib of left camera
  single_calib.reset(new gtsam::Cal3_S2(calib_->calibration()));
  
  //  iterate through features
  for (const stereo_vo::FeatureMsg& f : feat_left) {
    //  old feature
    if (!f.fresh) {
      if (current_ids_.find(f.id) != current_ids_.end()) {
        //  this point was triangulated and an estimate was inserted
        Point2 point(f.point.x, f.point.y);
        ProjectionFactor factor(point, pixel_noise, CurPoseKey(),
                                Symbol('l', f.id), single_calib, 
                                static_cast<Pose3>(cam_pose_in_body_));
        graph_.add(factor);   
      } else {
        ROS_WARN("Ignoring feature %u, it was not triangulated", f.id); 
      }
    } else {
      //  new feature
      //  find the right feature, lazy method for now
      auto ite = std::find_if(feat_right.cbegin(), feat_right.cend(),
                              [&](const stereo_vo::FeatureMsg& msg) -> bool {
        return msg.id == f.id;
      });
      assert(ite != feat_right.end());  //  MUST have a match
      
      const stereo_vo::FeatureMsg& f_left = f;
      const stereo_vo::FeatureMsg& f_right = *ite;
      
      if (current_ids_.find(f_left.id) == current_ids_.end()) {
        ROS_INFO("New feature: %u", f_left.id);
        
        //  insert initial estimate here
        gtsam::Point3 tri_point;
        kr::mat3d tri_cov;
        if (triangulate(f_left.point,f_right.point,pose,tri_point,tri_cov)) {
          ROS_INFO("Point was triangulated");
          triangulated_points_.push_back(tri_point);
          estimates_.insert(Symbol('l', f_left.id), tri_point);
          current_ids_.insert(f_left.id);
          
          Point2 point(f_left.point.x, f_left.point.y);
          ProjectionFactor factor(point, pixel_noise, CurPoseKey(),
                                  Symbol('l', f_left.id), single_calib, 
                                  static_cast<Pose3>(cam_pose_in_body_));
          graph_.add(factor);   
        }
      }
    }
  }
  
  if (IsInitialized()) {
    //  start optimization after a certain number of poses are collected
    //PerformUpdate();
  } else {
    //  input pose
    current_pose_ = static_cast<gtsam::Pose3>(pose);
  }
  
  previous_odom_ = pose;
  meas_index_++;
}

bool SamEstimator::triangulate(
                          const geometry_msgs::Point& left,
                          const geometry_msgs::Point& right,
                          const kr::Posed& odom_pose,
                          gtsam::Point3& output_point,
                          kr::mat3d& covariance) {

  kr::vec2d vleft, vright;  
  vleft[0] = (left.x - model_.left().cx()) / model_.left().fx();
  vleft[1] = (left.y - model_.left().cy()) / model_.left().fy();
  vright[0] = (right.x - model_.right().cx()) / model_.right().fx();
  vright[1] = (right.y - model_.right().cy()) / model_.right().fy();

  //  left camera pose is identity, right is shifted by baseline
  const kr::Posed left_cam_pose;
  kr::Posed right_cam_pose;
  right_cam_pose.p()[0] += model_.baseline();
  
  //  triangulate in the left camera frame
  kr::vec3d position;
  double ratio;
  kr::triangulate(left_cam_pose, vleft,
                  right_cam_pose, vright,
                  position, ratio);
  //  convert back to world
  const kr::Posed cam_in_world = odom_pose.composeInBody(cam_pose_in_body_);
  position = cam_in_world.transformFromBody(position);
  
  /// @todo:  generate a covariance estimate
  covariance.setIdentity();
  covariance *= 0.3;
  
  output_point = Point3(position[0], position[1], position[2]);
  
  /// @todo: make this ratio an option
  return (ratio < 1e4);
}


void SamEstimator::PerformUpdate() {  
  //  perform ISAM2 update and reset the graph
  isam_.update(graph_, estimates_);
  
  estimates_ = isam_.calculateEstimate();
  const Pose3 optimized_pose = estimates_.at<Pose3>(CurPoseKey());
  current_pose_ = optimized_pose;
  
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

void 
SamEstimator::SetCameraModel(const image_geometry::StereoCameraModel& model) {
  model_ = model;
  calib_.reset(new Cal3_S2Stereo(model.left().fx(),model.left().fy(),0,
                                 model.left().cx(),model.left().cy(),
                                 model.baseline()));  
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
