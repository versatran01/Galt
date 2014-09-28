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
 *  This file is part of graph_recorder.
 *
 *	Created on: 23/09/2014
 */

#ifndef GALT_GRAPH_RECORDER_NODE_HPP_
#define GALT_GRAPH_RECORDER_NODE_HPP_

#include <iostream>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

//  message types we will record
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stereo_vo/FeaturesStamped.h>

//  using Qt to save
#include <QFile>

namespace galt {
namespace graph_recorder {

class Node {
public:
  Node(const ros::NodeHandle& nh);
  
  /// Subscribe to all topics
  void Initialize();
  
private:
  template <typename T>
  using Subscriber = message_filters::Subscriber<T>;
  
  constexpr static unsigned int kROSQueueSize = 300;
  
  //  output streams
  QFile gps_odom_out_;
  QFile vo_poses_out_;
  QFile vo_features_out_;
  
  ros::NodeHandle nh_;
  bool initialized_ = false;
  unsigned int gps_odom_id_ = 0;
  unsigned int vo_pose_id_ = 0;
  
  //  subscribed topics
  Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_vo_pose_;
  Subscriber<stereo_vo::FeaturesStamped> sub_features_;
  Subscriber<nav_msgs::Odometry> sub_gps_odom_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry,
    geometry_msgs::PoseWithCovarianceStamped,
    stereo_vo::FeaturesStamped> TimeSyncPolicy;
  
  //  time synchronizers
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
  
  void Callback(const nav_msgs::OdometryConstPtr&,
      const geometry_msgs::PoseWithCovarianceStampedConstPtr&,
      const stereo_vo::FeaturesStampedConstPtr&);
};

} //  graph_recorder
} //  galt

#endif
