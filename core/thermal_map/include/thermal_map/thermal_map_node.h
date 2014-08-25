/*
 * thermal_map_node.h
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

#ifndef GALT_THERMAL_MAP_NODE_H_
#define GALT_THERMAL_MAP_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "thermal_map/rviz_helper.h"

namespace galt {
namespace thermal_map {

class ThermalMapNode {

 public:
  ThermalMapNode(const ros::NodeHandle &nh);

 private:
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;

  void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg);
  void CameraLaserCb(const sensor_msgs::ImageConstPtr &image_msg,
                     const sensor_msgs::CameraInfoConstPtr &cinfo_msg,
                     const sensor_msgs::LaserScanConstPtr &scan_msg);
//  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
//                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
//  void LaserCb(const sensor_msgs::LaserScanConstPtr &scan_msg);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
//  image_transport::CameraSubscriber sub_camera_;
  image_transport::SubscriberFilter sub_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cinfo_;
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_;
  std::unique_ptr<ApproximateSync> approximate_sync_;
  ros::Subscriber sub_odom_;
//  ros::Subscriber sub_laser_;
  ros::Publisher pub_traj_;
  TrajectoryVisualizer viz_traj_;
};

}  // namespace thermal_map
}  // namespace galt
#endif  // GALT_THERMAL_MAP_NODE_H_
