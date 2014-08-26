/*
 * sam_estimator_node.hpp
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

#ifndef GALT_SAM_ESTIMATOR_NODE_HPP_
#define GALT_SAM_ESTIMATOR_NODE_HPP_

#include <sam_estimator/sam_estimator.hpp>
#include <sam_estimator/visualizer.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <laser_altimeter/Height.h>

namespace galt {
namespace sam_estimator {

class SamEstimatorNode {
 public:
  SamEstimatorNode(const ros::NodeHandle& nh);
  
private:
  
  constexpr static int kROSQueueSize = 1;

  SamEstimator::Ptr estimator_;
  Visualizer::Ptr visualizer_;
    
  //  ROS objects
  ros::NodeHandle nh_;
  ros::Subscriber sub_gps_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_vo_;
  ros::Subscriber sub_laser_height_;
  ros::Publisher pub_odometry_;

  //  State
  double init_height_;
  double prev_imu_time_;
  
  //  ROS callbacks
  void GpsCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  void VisualOdometryCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void LaserHeightCallback(const laser_altimeter::HeightConstPtr& height_msg);
};
}
}

#endif
