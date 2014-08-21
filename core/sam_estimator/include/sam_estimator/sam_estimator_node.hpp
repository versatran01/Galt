/*
 * sam_estimator_node.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#ifndef GALT_SAM_ESTIMATOR_NODE_HPP_
#define GALT_SAM_ESTIMATOR_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace galt {
namespace sam_estimator {

class SamEstimatorNode {
public:
  
  SamEstimatorNode(const ros::NodeHandle& nh);
  
private:
  
  ros::NodeHandle nh_;
  ros::Subscriber sub_gps_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_stereo_;
  ros::Subscriber sub_laser_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_marker_;
  
  //  callbacks for sam_estimator, imu, stereo pose callback, lidar callback
  
  void GpsOdomCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  
  void StereoCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  
  void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& laser_msg);
  
};

}
}


#endif
