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

namespace gps_odom {

Node::Node() : nh_("~"), pkgPath_(ros::package::getPath("gps_odom")) {
  if (pkgPath_.empty()) {
    ROS_WARN("Failed to find path for package");
  }
  
  lastAttitude_ = kr::quatd(1,0,0,0);
  prevImuTime_ = 0.0;  
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
                                               subFix_, subFixVelocity_);
  syncGps_->registerCallback( boost::bind(&Node::gpsCallback, this, _1, _2) );
  
  //  advertise odometry as output
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu,
                       const sensor_msgs::FluidPressureConstPtr& fluidPressure) {
  
  //  current orientation
  const kr::quatd wQb = kr::quatd(imu->orientation.w, imu->orientation.x,
                            imu->orientation.y, imu->orientation.z);
  lastAttitude_ = wQb;
  
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
  const double h = (1 - std::exp(lhs)) * kT0 / kL;  
  
  //  convert to units of m/s^2
  const kr::vec3d aB = kr::vec3d(imu->linear_acceleration.x,
                           imu->linear_acceleration.y,
                           imu->linear_acceleration.z) * kG;
  
  const kr::vec3d aW = (wQb.matrix() * aB);
  const double accZ = aW[2] - kG;
  
  const double tNow = ros::Time::now().toSec();
  double delta = 1/200.0;
  if (prevImuTime_) {
    delta = tNow - prevImuTime_;
  }
  prevImuTime_ = tNow;
  
  //  filter acceleration and pressure height
  altFilterPressure_.predict(accZ, 1.0, delta);
  altFilterPressure_.updateHeight(h, 0.5);
  
  //  filter acceleration and GPS
  altFilterGps_.predict(accZ, 1.0, delta);

  
  /*static ros::Publisher hPub = nh_.advertise<std_msgs::Float64>("height", 1);
  std_msgs::Float64 f;
  f.data = altFilterPressure_.getHeight();
  hPub.publish(f);
  
  static ros::Publisher hPub2 = nh_.advertise<std_msgs::Float64>("height_gps", 1);
  f.data = altFilterGps_.getHeight();
  hPub2.publish(f);*/
  
  
  //ROS_INFO("Filtered height: %f (%f)", altFilterPressure_.getHeight(), accZ);
}

void Node::gpsCallback(const sensor_msgs::NavSatFixConstPtr& navSatFix,
                       const geometry_msgs::Vector3StampedConstPtr& velVec) {
  
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
  kr::quatd wQb = lastAttitude_;
  wQb = kr::rotationQuat(-declination,0,0,1) * wQb;
  
  //  generate odometry message
  nav_msgs::Odometry odometry;
  odometry.header.stamp = navSatFix->header.stamp;
  odometry.pose.pose.orientation.w = wQb.w();
  odometry.pose.pose.orientation.x = wQb.x();
  odometry.pose.pose.orientation.y = wQb.y();
  odometry.pose.pose.orientation.z = wQb.z();
  
  odometry.pose.pose.position.x = locX;
  odometry.pose.pose.position.y = locY;
  odometry.pose.pose.position.z = locZ;
  
  odometry.twist.twist.linear.x = velVec->vector.x; //  linear velocity
  odometry.twist.twist.linear.y = velVec->vector.y;
  odometry.twist.twist.linear.z = velVec->vector.z;
  
  //odometry.twist.twist.angular.x = 
  
  //  calculate difference between pressure and GPS
  //altFilterGps_.updateHeightAndVelocity(hMSL, 5.0, velVec->vector.z, 1.0);
  
  double delta = altFilterGps_.getHeight() - altFilterPressure_.getHeight();
  
  ROS_INFO("delta: %f", delta);
}
} //  namespace gps_odom
