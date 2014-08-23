/*
 * visualizer.cpp
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

#include <sam_estimator/visualizer.hpp>
#include <nav_msgs/Path.h>

namespace galt {
namespace sam_estimator {

Visualizer::Visualizer(const ros::NodeHandle& nh) : nh_(nh) {
  pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1);
}

void Visualizer::SetTrajectory(const std::vector<kr::Posed>& poses) {
  nav_msgs::Path path;
  
  auto time = ros::Time::now();
  for (const kr::Posed& pose : poses) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = time;
    poseStamped.header.frame_id = "/world";
    poseStamped.pose = static_cast<geometry_msgs::Pose>(pose);
    path.poses.push_back(poseStamped);
  }
  path.header.stamp = time;
  path.header.frame_id = "/world";
  pub_path_.publish(path);
}

} // namespace sam_estimator
}// namespace galt
