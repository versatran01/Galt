/*
 * graph_recorder_node.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of graph_recorder.
 *
 *	Created on: 23/09/2014
 */

#include <ros/ros.h>
#include <graph_recorder/node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"graph_recorder");
  ros::NodeHandle pnh("~");
  try {
    galt::graph_recorder::Node node(pnh);
    node.Initialize();
    ros::spin();
  } catch (std::exception& e) {
    ROS_ERROR("Fatal: %s", e.what());
  }
  return 0;
}
