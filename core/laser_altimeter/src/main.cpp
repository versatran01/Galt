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
 *  This file is part of laser_altimeter.
 *
 *	Created on: 24/08/2014
 */

#include <ros/ros.h>
#include <laser_altimeter/node.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_altimeter");
  ros::NodeHandle nh("~");
  galt::laser_altimeter::Node node(nh);
  ros::spin();
  return 0;
}
