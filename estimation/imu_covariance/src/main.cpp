/*
 * main.cpp
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_covariance");
  ros::NodeHandle nh, pnh("~");
  try {
    galt::imu_covariance::Node node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
