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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <stereo_vo/StereoFeaturesStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <rviz_helper/visualizer.h> //  for covariance and pose estimates

namespace galt {
namespace sam_estimator {

class SamEstimatorNode {
 public:
  SamEstimatorNode(const ros::NodeHandle& nh);
  
private:
  
  constexpr static int kROSQueueSize = 100;

  SamEstimator::Ptr estimator_;
    
  //  ROS objects
  ros::NodeHandle nh_;
  ros::Publisher pub_odometry_;

  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
  message_filters::Subscriber<stereo_vo::StereoFeaturesStamped> sub_features_;
    
  typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, stereo_vo::StereoFeaturesStamped> TimeSyncPolicy;
  //  time synchronized
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
 
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_r_info_;
  
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> InfoTimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<InfoTimeSyncPolicy>> sync_info_;
  
  //  camera model
  image_geometry::StereoCameraModel model_;
  
  //  transforms
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
  
  //  synchronized callbacks
  void odomFeaturesCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                            const stereo_vo::StereoFeaturesStampedConstPtr& feat_msg);
  
  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& l_info,
                       const sensor_msgs::CameraInfoConstPtr& r_info);
};

} //  namespace sam_estimator
} //  namespace galt

#endif
