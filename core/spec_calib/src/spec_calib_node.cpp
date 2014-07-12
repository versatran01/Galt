/*
 * spec_calib_node.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "calibrate_line.hpp"

// spectrometer output
#include <ocean_optics/Spectrum.h>

//  circle tracker
#include <circle_tracker/Circles.h>

// kr_math
#include <kr_math/SO3.hpp>

//  pose estimator
#include <monocular_pose_estimator/PixelArray.h>

ros::NodeHandlePtr nh;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "spec_calib_node");
  nh = ros::NodeHandlePtr( new ros::NodeHandle("~") );  
  
  //  subscribe to circles, pose, pixels and image
  image_transport::SubscriberFilter imgSub("image");
  message_filters::Subscriber <sensor_msgs::CameraInfo> camInfoSub("camera_info");
  message_filters::Subscriber <monocular_pose_estimator::PixelArray> pixSub("pixel_array");
  message_filters::Subscriber <circle_tracker::Circles> circSub("circles");
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo, monocular_pose_estimator::PixelArray,
      circle_tracker::Circles> TimeSyncPolicy;
  
  message_filters::Synchronizer<TimeSyncPolicy> 
  
  ros::spin();
  return 0;
}
