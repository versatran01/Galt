/*
 * node.cpp
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

#include <imu_covariance/node.hpp>
#include <kr_math/yaml.hpp>
#include <sstream>

namespace galt {
namespace imu_covariance {

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  //  load settings
  std::string configPath;
  if (!pnh.hasParam("config")) {
    ROS_ERROR("You must specify the config_path option");
    throw std::invalid_argument("No config_path specified");
  }
  pnh.getParam("config", configPath);

  YAML::Node settings;
  try {
    settings = YAML::LoadFile(configPath);
  }
  catch (std::exception &e) {
    std::stringstream ss;
    ss << "Failed to load " << configPath;
    ss << "\n" << e.what();
    throw std::runtime_error(ss.str());
  }
  YAML::Node noiseStd = settings["noise_std"];
  if (!noiseStd) {
    throw std::invalid_argument("Config is missing the noise_std node");
  }

  try {
    accelStd_ = noiseStd["accel"].as<Eigen::Vector3d>();
    gyroStd_ = noiseStd["gyro"].as<Eigen::Vector3d>();
    fieldStd_ = noiseStd["mag"].as<Eigen::Vector3d>();
    pressureStd_ = noiseStd["pressure"].as<double>();
  }
  catch (std::exception &e) {
    std::stringstream ss;
    ss << "Config has invalid setting.\n";
    ss << e.what();
    throw std::runtime_error(ss.str());
  }

  // inputs, subscribe to topics under imu namespace
  subImu_ = nh_.subscribe("imu", kROSQueueSize, &Node::imuCallback, this);
  subMagneticField_ = nh_.subscribe("magnetic_field", kROSQueueSize,
                                    &Node::magneticFieldCallback, this);
  subPressure_ = nh_.subscribe("pressure", kROSQueueSize,
                               &Node::fluidPressureCallback, this);
  // outputs, advertice topics with same names under imu/imu_covariance
  // namespace
  pubImu_ = pnh_.advertise<sensor_msgs::Imu>("imu", 1);
  pubMagneticField_ =
      pnh_.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  pubPressure_ = pnh_.advertise<sensor_msgs::FluidPressure>("pressure", 1);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr &imuMsg) {
  sensor_msgs::Imu imu = *imuMsg;
  imu.header.seq = 0;
  imu.linear_acceleration_covariance[0] = accelStd_[0] * accelStd_[0];
  imu.linear_acceleration_covariance[4] = accelStd_[1] * accelStd_[1];
  imu.linear_acceleration_covariance[8] = accelStd_[2] * accelStd_[2];
  imu.angular_velocity_covariance[0] = gyroStd_[0] * gyroStd_[0];
  imu.angular_velocity_covariance[4] = gyroStd_[1] * gyroStd_[1];
  imu.angular_velocity_covariance[8] = gyroStd_[2] * gyroStd_[2];
  pubImu_.publish(imu);
}

void Node::magneticFieldCallback(
    const sensor_msgs::MagneticFieldConstPtr &magMsg) {
  sensor_msgs::MagneticField field = *magMsg;
  field.header.seq = 0;
  field.magnetic_field_covariance[0] = fieldStd_[0] * fieldStd_[0];
  field.magnetic_field_covariance[4] = fieldStd_[1] * fieldStd_[1];
  field.magnetic_field_covariance[8] = fieldStd_[2] * fieldStd_[2];
  pubMagneticField_.publish(field);
}

void Node::fluidPressureCallback(
    const sensor_msgs::FluidPressureConstPtr &pressureMsg) {
  sensor_msgs::FluidPressure pressure = *pressureMsg;
  pressure.header.seq = 0;
  pressure.variance = pressureStd_ * pressureStd_;
  pubPressure_.publish(pressure);
}

}  //  imu_covariance
}  //  galt
