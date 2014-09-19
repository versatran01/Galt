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
#include <nav_msgs/Odometry.h>
#include <stereo_vo/FeaturesStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace galt {
namespace sam_estimator {

class SamEstimatorNode {
 public:
  SamEstimatorNode(const ros::NodeHandle& nh);
  
private:
  
  constexpr static int kROSQueueSize = 100;

  SamEstimator::Ptr estimator_;
  Visualizer::Ptr visualizer_;
    
  //  ROS objects
  ros::NodeHandle nh_;
  ros::Publisher pub_odometry_;

  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
  message_filters::Subscriber<stereo_vo::FeaturesStamped> sub_features_;
    
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, stereo_vo::FeaturesStamped> TimeSyncPolicy;
  //  time synchronized
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
 
  //  synchronized callback
  void odomFeaturesCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                            const stereo_vo::FeaturesStampedConstPtr& feat_msg);
};

} //  namespace sam_estimator
} //  namespace galt

#endif
