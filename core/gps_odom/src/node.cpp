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

namespace gps_odom {

template <typename T, std::size_t N>
bool allZero(const boost::array<T,N>& array) {
  for (std::size_t i=0; i < array.size(); i++) {
    if (array[i] != 0) {
      return false;
    }
  }
  return true;
}

Node::Node() : nh_("~"), pkgPath_(ros::package::getPath("gps_odom")) {
  if (pkgPath_.empty()) {
    ROS_WARN("Failed to find path for package");
  }
  
  nh_.param("horz_accuracy", hAcc_, 10.0);
  nh_.param("vert_accuracy", vAcc_, 12.0);
  nh_.param("speed_accuracy", sAcc_, 1.0);
}

void Node::initialize() {
 
  //  IMU and GPS are synchronized separately
  subImu_.subscribe(nh_, "imu", kROSQueueSize);
  subPressure_.subscribe(nh_, "pressure",  kROSQueueSize);
  
  syncImu_ = std::make_shared<SynchronizerIMU>(TimeSyncIMU(kROSQueueSize), 
                                               subImu_, subPressure_);
  syncImu_->registerCallback( boost::bind(&Node::imuCallback, this, _1, _2) );
  
  subFix_.subscribe(nh_, "fix", kROSQueueSize);
  subFixVelocity_.subscribe(nh_, "fix_velocity", kROSQueueSize);
  
  syncGps_ = std::make_shared<SynchronizerGPS>(TimeSyncGPS(kROSQueueSize),
                                               subFix_, subFixVelocity_, subImu_);
  syncGps_->registerCallback( boost::bind(&Node::gpsCallback, this, _1, _2, _3) );
  
  //  advertise odometry as output
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu,
                       const sensor_msgs::FluidPressureConstPtr& fluidPressure) {
  
  //  current orientation
  /*const kr::quatd wQb = kr::quatd(imu->orientation.w, imu->orientation.x,
                            imu->orientation.y, imu->orientation.z);
  
  //  convert fluid pressure to height in meters
  //  refer to wikipedia for this formula
  
  const double kP0 = 101325;    //  Pressure at sea level (Pa)
  const double kL = 0.0065;     //  Temperature lapse rate (K/m)
  const double kT0 = 288.15;    //  Sea level standard temperature (K)
  const double kG = 9.80665;    //  Gravitational acceleration (m/s^2)
  const double kM = 0.0289644;  //  Molar mass of dry air (kg / mol)
  const double kR = 8.31447;    //  Universal gas constant J/(mol * K)
  
  const double c = kG*kM/(kR*kL);
  
  //  convert from millibar to pascals
  const double pressurePA = fluidPressure->fluid_pressure * 100;
  
  //  calculate height from barometer
  const double lhs = std::log(pressurePA / kP0) * (1 / c);
  const double h = (1 - std::exp(lhs)) * kT0 / kL;  */
  
  //  convert to units of m/s^2
//  const kr::vec3d aB = kr::vec3d(imu->linear_acceleration.x,
//                           imu->linear_acceleration.y,
//                           imu->linear_acceleration.z) * kG;
  
//  const kr::vec3d aW = (wQb.matrix() * aB);
//  const double accZ = aW[2] - kG;
}

void Node::gpsCallback(const sensor_msgs::NavSatFixConstPtr& navSatFix,
                       const geometry_msgs::Vector3StampedConstPtr& velVec, 
                       const sensor_msgs::ImuConstPtr& imu) {
  
  const double lat = navSatFix->latitude;
  const double lon = navSatFix->longitude;
  const double hWGS84 = navSatFix->altitude;
  const double tYears = navSatFix->header.stamp.toSec() / (365*24*3600.0) + 1970; 
  
  //  get path to geoids folder and load geoid if required
  if (!geoid_) {
    try {
      geoid_ = std::make_shared<GeographicLib::Geoid>("egm84-15", pkgPath_ + "/geoids");
    } catch (GeographicLib::GeographicErr& e) {
      ROS_ERROR("Failed to load geoid. Reason: %s", e.what());
      return;
    }
  }
  
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
  const double declination = std::atan2(bEast,bNorth);
  
  //  calculate corrected yaw angle
  //  matrix composition is of the form wRb = Rz * Ry * Rx
  //  left multiply declination adjustment
  kr::quatd wQb = kr::quatd(imu->orientation.w,imu->orientation.x,
                            imu->orientation.y,imu->orientation.z);
  wQb = kr::rotationQuat(-declination,0.0,0.0,1.0) * wQb;
  
  nav_msgs::Odometry odometry;
  odometry.header.stamp = navSatFix->header.stamp;
  odometry.header.frame_id = "0";
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
  
  //  incoming covariance is valid
  if (!allZero(navSatFix->position_covariance)) {
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        poseCovariance(i,j) = navSatFix->position_covariance[i*3 + j];   
      }
    }
  } else {
    //  pick a moderatly safe default
    poseCovariance(0,0) = (hAcc_/3)*(hAcc_/3);
    poseCovariance(1,1) = (hAcc_/3)*(hAcc_/3);
    poseCovariance(2,2) = (vAcc_/3)*(vAcc_/3);
  }
  
  //  copy from IMU
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      poseCovariance(3+i,3+j) = imu->orientation_covariance[i*3 + j];
    }
  }
  
  //  linear velocity from GPS, angular velocity from IMU
  odometry.twist.twist.linear = velVec->vector;
  odometry.twist.twist.angular = imu->angular_velocity;
  
  //  velocity covariance [x,y,z,rot_x,rot_y,rot_z]
  //  rotation component is unused so leave as zero
  kr::mat<double,6,6> velCovariance;
  velCovariance.setZero();
  velCovariance(0,0) = (sAcc_/3)*(sAcc_/3);
  velCovariance(1,1) = (sAcc_/3)*(sAcc_/3);
  velCovariance(2,2) = (sAcc_/3)*(sAcc_/3);
  
  for (int i=0; i < 6; i++) {
    for (int j=0; j < 6; j++) {
      odometry.pose.covariance[i*6 + j] = poseCovariance(i,j);
      odometry.twist.covariance[i*6 + j] = velCovariance(i,j);
    }
  }
  pubOdometry_.publish(odometry); 
}
} //  namespace gps_odom
