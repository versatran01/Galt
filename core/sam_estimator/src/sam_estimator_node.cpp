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
  
  //  subscribe to all relevant topics
  sub_gps_ = nh_.subscribe("gps_odom", kROSQueueSize,
                           &SamEstimatorNode::GpsCallback, this);
  sub_imu_ =
      nh_.subscribe("imu", kROSQueueSize, &SamEstimatorNode::ImuCallback, this);
  sub_stereo_ = nh_.subscribe("vo_pose", kROSQueueSize,
                              &SamEstimatorNode::StereoCallback, this);
  sub_laser_ = nh_.subscribe("laser_scan", kROSQueueSize,
                             &SamEstimatorNode::LaserCallback, this);
  pub_pose_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
}

void SamEstimatorNode::GpsCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  const double time = odometry_msg->header.stamp.toSec();
  
  SamEstimator::GpsMeasurement meas;
  meas.time = time;
  //meas.pose();
  
  estimator_->AddGps(meas);
  
  /// @todo: send to visualizer here
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
  //ROS_INFO("stereo callback!");
}

}  // namespace sam_estimator
}  // namespace galt
