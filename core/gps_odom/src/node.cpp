/*
 * node.cpp
 *
 *  Copyright (c) 2013 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#include <ros/package.h>
#include <gps_odom/node.hpp>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <kr_math/SO3.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace gps_odom {

Node::Node() : nh_("~"), pkgPath_(ros::package::getPath("gps_odom")) {
  if (pkgPath_.empty()) {
    ROS_WARN("Failed to find path for package");
  }

  /// @todo: figure out what these should actually be...
  nh_.param<std::string>("child_frame_id", childFrameId_, "imu");
  nh_.param<std::string>("frame_id", frameId_, "world");
  
  refSet_ = false;
  currentDeclination_ = 0.0;
}

void Node::initialize() {
 
  //  IMU and GPS are synchronized separately
  subImu_.subscribe(nh_, "imu", kROSQueueSize);  
//  syncImu_ = std::make_shared<SynchronizerIMU>(TimeSyncIMU(kROSQueueSize), 
//                                               subImu_, subPressure_);
//  syncImu_->registerCallback( boost::bind(&Node::imuCallback, this, _1, _2) );
  
  subFix_.subscribe(nh_, "fix", kROSQueueSize);
  subFixTwist_.subscribe(nh_, "fix_velocity", kROSQueueSize);
  
  syncGps_ = std::make_shared<SynchronizerGPS>(TimeSyncGPS(kROSQueueSize),
                                               subFix_, subFixTwist_, subImu_);
  syncGps_->registerCallback( boost::bind(&Node::gpsCallback, this, _1, _2, _3) );
  
  //  advertise odometry as output
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  //pubImu_ = nh_.advertise<sensor_msgs::Imu>("corrected_imu", 1);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  
  //  current orientation
  kr::quatd wQb = kr::quatd(imu->orientation.w, imu->orientation.x,
                            imu->orientation.y, imu->orientation.z);
  
  //  adjust for declination
  Eigen::AngleAxisd aa;
  aa.angle() = -currentDeclination_;
  aa.axis() = Eigen::Vector3d(0.0,0.0,1.0);
  
  wQb = aa * wQb;
  
  //  re-publish IMU message
  sensor_msgs::Imu outImu;
  outImu = *imu;
  outImu.header.seq = 0;
  outImu.orientation.w = wQb.w();
  outImu.orientation.x = wQb.x();
  outImu.orientation.y = wQb.y();
  outImu.orientation.z = wQb.z();
  //pubImu_.publish(outImu);
}

void Node::gpsCallback(const sensor_msgs::NavSatFixConstPtr& navSatFix,
                       const geometry_msgs::TwistWithCovarianceStampedConstPtr &navSatTwist, 
                       const sensor_msgs::ImuConstPtr& imu) {
  
  const double lat = navSatFix->latitude;
  const double lon = navSatFix->longitude;
  const double hWGS84 = navSatFix->altitude;
  const double tYears = navSatFix->header.stamp.toSec() / (365*24*3600.0) + 1970; 
  
  //  load geoid if required
  if (!geoid_) {
    try {
      geoid_ = std::make_shared<GeographicLib::Geoid>("egm84-15", pkgPath_ + "/geoids");
    } catch (GeographicLib::GeographicErr& e) {
      ROS_ERROR("Failed to load geoid. Reason: %s", e.what());
      return;
    }
  }
  //  load magnetic model, if required
  if (!magneticModel_) {
    try {
      magneticModel_ = std::make_shared<GeographicLib::MagneticModel>("wmm2010", pkgPath_ + "/magnetic_models");
    } catch (GeographicLib::GeographicErr& e) {
      ROS_ERROR("Failed to load model. Reason: %s", e.what());
      return;
    }
  }
  
  //  convert to height above sea level
  const double hMSL = geoid_->ConvertHeight(lat,lon,hWGS84,GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  
  //  generate linear coordinates
  if (!refSet_) {
    refPoint_ = GeographicLib::LocalCartesian(lat,lon,0);
    refSet_ = true;
  }
  
  double locX,locY,locZ;
  refPoint_.Forward(lat,lon,hMSL,locX,locY,locZ);
    
  //  determine magnetic declination
  double bEast, bNorth, bUp;
  magneticModel_->operator()(tYears,lat,lon,hWGS84,bEast,bNorth,bUp);
  currentDeclination_ = std::atan2(bEast,bNorth);
    
  //  calculate corrected yaw angle
  //  matrix composition is of the form wRb = Rz * Ry * Rx
  Eigen::AngleAxisd aa;
  aa.angle() = -currentDeclination_;
  aa.axis() = Eigen::Vector3d(0.0,0.0,1.0);
  
  kr::quatd wQb = kr::quatd(imu->orientation.w,imu->orientation.x,
                            imu->orientation.y,imu->orientation.z);
  wQb = aa * wQb;
  
  nav_msgs::Odometry odometry;
  odometry.header.stamp = navSatFix->header.stamp;
  odometry.header.frame_id = frameId_;
  odometry.child_frame_id = childFrameId_;
  odometry.pose.pose.orientation.w = wQb.w();
  odometry.pose.pose.orientation.x = wQb.x();
  odometry.pose.pose.orientation.y = wQb.y();
  odometry.pose.pose.orientation.z = wQb.z();
  odometry.pose.pose.position.x = locX;
  odometry.pose.pose.position.y = locY;
  odometry.pose.pose.position.z = locZ;
  
  //  generate covariance (6x6 with order: x,y,z,rot_x,rot_y,rot_z)
  kr::mat<double,6,6> poseCovariance;
  poseCovariance.setZero();
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      poseCovariance(i,j) = navSatFix->position_covariance[i*3 + j];   
    }
  }
  
  //  orientation: copy from IMU
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      poseCovariance(3+i,3+j) = imu->orientation_covariance[i*3 + j];
    }
  }
  
  //  linear velocity from GPS, no angular velocity
  odometry.twist.twist.linear = navSatTwist->twist.twist.linear;
  odometry.twist.twist.angular.x = 0;
  odometry.twist.twist.angular.y = 0;
  odometry.twist.twist.angular.z = 0;
  
  //  velocity covariance [x,y,z,rot_x,rot_y,rot_z]
  //  linear from GPS, no angular
  kr::mat<double,6,6> velCovariance;
  velCovariance.setZero();
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      velCovariance(i,j) = navSatTwist->twist.covariance[(i*6) + j];
      velCovariance(i+3,j+3) = -1;  //  unsupported
    }
  }
  //  copy covariance to output
  for (int i=0; i < 6; i++) {
    for (int j=0; j < 6; j++) {
      odometry.pose.covariance[i*6 + j] = poseCovariance(i,j);
      odometry.twist.covariance[i*6 + j] = velCovariance(i,j);
    }
  }
  pubOdometry_.publish(odometry); 
}
} //  namespace gps_odom
