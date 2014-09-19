/*
 * sam_estimator_node.cpp
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

#include <sam_estimator/sam_estimator_node.hpp>
#include <sam_estimator/sam_estimator.hpp>
#include <nav_msgs/Odometry.h>
#include <memory>

using std::make_shared;

namespace galt {
namespace sam_estimator {

SamEstimatorNode::SamEstimatorNode(const ros::NodeHandle &nh) : nh_(nh) {
  //  create an estimator
  estimator_ = make_shared<SamEstimator>();
  visualizer_ = make_shared<Visualizer>(nh_);
  
  //  subscribe to topics
  sub_features_.subscribe(nh_, "features", kROSQueueSize);
  sub_odom_.subscribe(nh_, "odometry_in", kROSQueueSize);
  
  sync_ = make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(
             TimeSyncPolicy(kROSQueueSize), sub_features_, sub_odom_);
  sync_->registerCallback(boost::bind(&SamEstimatorNode::odomFeaturesCallback,
                                      this,_1,_2));
  
  ROS_INFO("Subscribing to ~features and ~odometry_in");
  
  pub_odometry_ =
      nh_.advertise<nav_msgs::Odometry>("odometry", 1);
}

void SamEstimatorNode::GpsCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  const double time = odometry_msg->header.stamp.toSec();
  
  SamEstimator::GpsMeasurement meas;
  meas.time = time;
  meas.pose = kr::Posed(odometry_msg->pose.pose);
  meas.vel[0] = odometry_msg->twist.twist.linear.x;
  meas.vel[1] = odometry_msg->twist.twist.linear.y;
  meas.vel[2] = odometry_msg->twist.twist.linear.z;
  
  meas.cov.setZero();
  //  copy pose covariance
  for (int i=0; i < 6; i++) {
    for (int j=0; j < 6; j++) {
      meas.cov(i,j) = odometry_msg->pose.covariance[(i*6) + j];
    }
  }
  //  copy velocity covariance
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      meas.cov(i+6,j+6) = odometry_msg->twist.covariance[(i*6) + j];
    }
  }
  
  if (!estimator_->IsInitialized() && (init_height_ > 0)) {
    //  overwrite height with laser estimate
    meas.pose.p()[2] = init_height_;
    
    gtsam::Vector6 sigmas;
    for (int i=0; i < 3; i++) {
      sigmas[i] = meas.cov(i+3,i+3);
      sigmas[i+3] = meas.cov(i,i);  //  swap order
    }
    ROS_WARN("Initializing sam_estimator from GPS + laser height");
    estimator_->InitializeGraph(meas.pose, sigmas, meas.time);
  }
}

void SamEstimatorNode::VisualOdometryCallback(
    const geometry_msgs::PoseStampedConstPtr &pose_msg) {
  
  SamEstimator::VoMeasurement vo;
  vo.pose = kr::Posed(pose_msg->pose);
  vo.time = pose_msg->header.stamp.toSec();
  
  if (estimator_->IsInitialized()) {
    estimator_->AddVo(vo);
  }
  visualizer_->SetTrajectory(estimator_->AllPoses());
  
  //  publish odometery
  nav_msgs::Odometry odo;
  odo.header.stamp = pose_msg->header.stamp;
  odo.header.frame_id = "/world"; /// @todo: make params later...
  odo.child_frame_id ="/body";
  
  kr::Posed output_pose = kr::Posed(estimator_->CurrentPose());
  kr::mat<double,6,6> output_cov = estimator_->CurrentMarginals();
  
  /// @todo: for now just use diagonal blocks, fix this later
  odo.pose.pose = static_cast<geometry_msgs::Pose>(output_pose);
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      odo.pose.covariance[(i)*6 + j] = output_cov(i+3,j+3);
      odo.pose.covariance[(i+3)*6 + j+3] = output_cov(i,j);
    }
  }
  
  pub_odometry_.publish(odo);
}

void 
SamEstimatorNode::odomFeaturesCallback(
    const nav_msgs::OdometryConstPtr& odom_msg,
    const stereo_vo::FeaturesStampedConstPtr& feat_msg) {
  
  
  
}

}  // namespace sam_estimator
}  // namespace galt
