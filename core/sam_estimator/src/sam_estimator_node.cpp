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

SamEstimatorNode::SamEstimatorNode(const ros::NodeHandle &nh) : nh_(nh),
 init_height_(-1), prev_imu_time_(0.0) {
  //  create an estimator
  estimator_ = make_shared<SamEstimator>();
  visualizer_ = make_shared<Visualizer>(nh_);
  
  //  subscribe to all relevant topics
  sub_gps_ = nh_.subscribe("gps_odom", kROSQueueSize,
                           &SamEstimatorNode::GpsCallback, this);
  sub_imu_ =
      nh_.subscribe("imu", kROSQueueSize, &SamEstimatorNode::ImuCallback, this);
  sub_vo_ = nh_.subscribe("vo_pose", kROSQueueSize,
                              &SamEstimatorNode::VisualOdometryCallback, this);
  sub_laser_height_ = nh_.subscribe("laser_height", kROSQueueSize,
                             &SamEstimatorNode::LaserHeightCallback, this);
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

void SamEstimatorNode::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {  
  const double time = imu_msg->header.stamp.toSec();
  if (prev_imu_time_ == 0) {
    //  fudge the first IMU sample...
    prev_imu_time_ = time;
  }
  
  SamEstimator::ImuMeasurement meas;
  meas.time = time;
  meas.dt = time - prev_imu_time_;
  meas.z[0] = imu_msg->linear_acceleration.x * 9.80665; /// @todo: Fix IMU node
  meas.z[1] = imu_msg->linear_acceleration.y * 9.80665;
  meas.z[2] = imu_msg->linear_acceleration.z * 9.80665;
  meas.z[3] = imu_msg->angular_velocity.x;
  meas.z[4] = imu_msg->angular_velocity.y;
  meas.z[5] = imu_msg->angular_velocity.z;
  meas.wQb = kr::quatd(imu_msg->orientation.w,
                       imu_msg->orientation.x,
                       imu_msg->orientation.y,
                       imu_msg->orientation.z);
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      meas.cov(i,j) = imu_msg->orientation_covariance[i*3 + j];
    }
  }
  if (estimator_->IsInitialized()) {
    estimator_->AddImu(meas);
  }
  prev_imu_time_ = time;
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

void SamEstimatorNode::LaserHeightCallback(
    const laser_altimeter::HeightConstPtr& height_msg) {
 
  const double height = height_msg->max;
  if (init_height_ < 0) {
    /// @todo: use a moving average filter here for initialization
    init_height_ = height;
    ROS_INFO("Laser initialization height: %.3f", height);
  }
}

}  // namespace sam_estimator
}  // namespace galt
