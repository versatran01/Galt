/*
 * node.hpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of imu_covariance.
 *
 *	Created on: 31/08/2014
 */

#ifndef GALT_IMU_COVARIANCE_NODE_HPP_
#define GALT_IMU_COVARIANCE_NODE_HPP_

#include <ros/ros.h>
#include <kr_math/base_types.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

namespace galt {
namespace imu_covariance {

class Node {
 public:
  Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  //  number of items in the input queues
  static constexpr int kROSQueueSize = 10;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber subImu_, subMagneticField_, subPressure_;
  ros::Publisher pubImu_, pubMagneticField_, pubPressure_;

  //  standard deviations on noise for each sensor
  Eigen::Vector3d accelStd_;
  Eigen::Vector3d gyroStd_;
  Eigen::Vector3d fieldStd_;
  double pressureStd_;

  void imuCallback(const sensor_msgs::ImuConstPtr &imuMsg);
  void magneticFieldCallback(const sensor_msgs::MagneticFieldConstPtr &magMsg);
  void fluidPressureCallback(
      const sensor_msgs::FluidPressureConstPtr &pressureMsg);
};

}  //  imu_covariance
}  //  galt

#endif  //  GALT_IMU_COVARIANCE_NODE_HPP_
