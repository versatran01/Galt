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

using std::make_shared;

namespace galt {
namespace sam_estimator {

SamEstimatorNode::SamEstimatorNode(const ros::NodeHandle &nh) : nh_(nh) {
  //  create an estimator
  estimator_ = make_shared<SamEstimator>();
  visualizer_ = make_shared<Visualizer>(nh_);
  
  //  subscribe to all relevant topics
  sub_gps_ = nh_.subscribe("gps_odom", kROSQueueSize,
                           &SamEstimatorNode::GpsCallback, this);
  sub_imu_ =
      nh_.subscribe("imu", kROSQueueSize, &SamEstimatorNode::ImuCallback, this);
  sub_stereo_ = nh_.subscribe("vo_pose", kROSQueueSize,
                              &SamEstimatorNode::StereoCallback, this);
  sub_laser_ = nh_.subscribe("laser_scan", kROSQueueSize,
                             &SamEstimatorNode::LaserCallback, this);
  //pub_pose_ =
  //    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
}

void SamEstimatorNode::GpsCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  const double time = odometry_msg->header.stamp.toSec();
  
  static int gps_cb_count=0;
  
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
  
  if (gps_cb_count++ == 30) {
    gtsam::Vector6 sigmas;
    for (int i=0; i < 3; i++) {
      sigmas[i] = meas.cov(i+3,i+3);
      sigmas[i+3] = meas.cov(i,i);  //  swap order
    }
    ROS_WARN("Warning: Performing garbage gps initialization");
    estimator_->InitializeGraph(meas.pose,sigmas);
  }
  
  //  currently does nothing...  
  estimator_->AddGps(meas);
}

void SamEstimatorNode::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {  
  const double time = imu_msg->header.stamp.toSec();
  static double prevTime = time;  //  temporary
  
  SamEstimator::ImuMeasurement meas;
  meas.time = time;
  meas.dt = time - prevTime;
  meas.z[0] = imu_msg->linear_acceleration.x;
  meas.z[1] = imu_msg->linear_acceleration.y;
  meas.z[2] = imu_msg->linear_acceleration.z;
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
  
  if ( estimator_->IsInitialized() ) {
    estimator_->AddImu(meas);
  }
  
  prevTime = time;
}

void SamEstimatorNode::LaserCallback(
    const sensor_msgs::LaserScanConstPtr &laser_msg) {
  //ROS_INFO("laser callback!");
}

void SamEstimatorNode::StereoCallback(
    const geometry_msgs::PoseStampedConstPtr &pose_msg) {
  
  SamEstimator::VoMeasurement vo;
  vo.pose = kr::Posed(pose_msg->pose);
  vo.time = pose_msg->header.stamp.toSec();
  if (estimator_->IsInitialized()) {
    estimator_->AddVo(vo);
  }
  
  visualizer_->SetTrajectory(estimator_->AllPoses());
}

}  // namespace sam_estimator
}  // namespace galt
