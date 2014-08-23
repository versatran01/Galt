/*
 * flir_gige_node.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of flir_gige.
 *
 *	Created on: 21/08/2014
 */

#include <ros/ros.h>

#include "flir_gige/flir_gige.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "flir_node");
  ros::NodeHandle nh("~");

  try {
    flir_gige::FlirGige flir_gige(nh);
    flir_gige.Run();
    ros::spin();
    flir_gige.End();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
