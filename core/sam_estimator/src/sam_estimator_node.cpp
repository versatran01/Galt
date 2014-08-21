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

namespace galt {
namespace sam_estimator {

SamEstimatorNode::SamEstimatorNode(const ros::NodeHandle &nh) : nh_(nh) {}

void SamEstimatorNode::GpsCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {}

void SamEstimatorNode::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {}

void SamEstimatorNode::LaserCallback(
    const sensor_msgs::LaserScanConstPtr &laser_msg) {}

void SamEstimatorNode::StereoCallback(
    const geometry_msgs::PoseStampedConstPtr &pose_msg) {}

}  // namespace sam_estimator
}  // namespace galt
