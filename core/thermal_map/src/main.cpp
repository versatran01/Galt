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
 *  This file is part of thermal_map.
 *
 *	Created on: 23/08/2014
 */

#include "thermal_map/thermal_map_node.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "thermal_map");
  ros::NodeHandle nh("~");

  galt::thermal_map::ThermalMapNode thermal_map_node(nh);
  ros::spin();
}
