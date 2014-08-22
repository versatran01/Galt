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
  //ROS_INFO("gps callback!"); 
}

void SamEstimatorNode::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  //ROS_INFO("imu callback!");
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
