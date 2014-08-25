/*
 * thermal_map_node.cpp
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

#include <geometry_msgs/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {
namespace thermal_map {

ThermalMapNode::ThermalMapNode(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  sub_image_.subscribe(it_, "color_map", 1, hints);
  sub_cinfo_.subscribe(nh_, "camera_info", 1);
  sub_laser_.subscribe(nh_, "scan", 1);
  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(5), sub_image_,
                                              sub_cinfo_, sub_laser_));
  approximate_sync_->registerCallback(
      boost::bind(&ThermalMapNode::CameraLaserCb, this, _1, _2, _3));

  //  sub_camera_ = it_.subscribeCamera("color_map", 1,
  // &ThermalMapNode::CameraCb,
  //                                    this, hints);
  //  sub_odom_ = nh_.subscribe("odometry", 1, &ThermalMapNode::OdomCb, this);
  //  sub_laser_ = nh_.subscribe("scan", 1, &ThermalMapNode::LaserCb, this);
  pub_traj_ = nh_.advertise<visualization_msgs::Marker>("trajectory", 1);
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("thermal_cloud", 1);

  std_msgs::ColorRGBA traj_color;
  traj_color.r = 1;
  traj_color.g = 1;
  traj_color.a = 1;
  viz_traj_ = TrajectoryVisualizer(pub_traj_, traj_color, 0.05, "line");
}

void ThermalMapNode::OdomCb(const nav_msgs::OdometryConstPtr &odom_msg) {
  // Publish trajectory
  viz_traj_.PublishTrajectory(odom_msg->pose.pose.position, odom_msg->header);
}

void ThermalMapNode::CameraLaserCb(
    const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::CameraInfoConstPtr &cinfo_msg,
    const sensor_msgs::LaserScanConstPtr &scan_msg) {
  // Transfomr laser scan to point cloud
  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("thermal", *scan_msg, cloud,
                                            listener_);
  // Project point cloud back into thermal image and change color
  pub_cloud_.publish(cloud);
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
}

// void ThermalMapNode::LaserCb(const sensor_msgs::LaserScanConstPtr &scan_msg)
// {
//  ROS_INFO_THROTTLE(2, "in laser");
//}

// void ThermalMapNode::CameraCb(
//    const sensor_msgs::ImageConstPtr &image_msg,
//    const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
//  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
//}

}  // namespace thermal_map
}  // namespace galt
