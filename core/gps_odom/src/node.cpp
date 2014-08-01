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

#include <gps_odom/node.hpp>
#include <nav_msgs/Odometry.h>

namespace gps_odom {

Node::Node() : nh_("~")
{
}

void Node::initialize() {
 
  //  IMU and GPS are synchronized separately
  subImu_.subscribe(nh_, "imu", kROSQueueSize);
  subPressure_.subscribe(nh_, "pressure",  kROSQueueSize);
  
  syncImu_ = std::make_shared<SynchronizerIMU>(TimeSyncIMU(kROSQueueSize), 
                                               subImu_, subPressure_);
  syncImu_->registerCallback( boost::bind(&Node::imuCallback, this, _1, _2) );
  
  subNavPos_.subscribe(nh_, "navposllh", kROSQueueSize);
  subNavStatus_.subscribe(nh_, "navstatus", kROSQueueSize);
  subNavVel_.subscribe(nh_, "navvelned", kROSQueueSize);
  
  syncGps_ = std::make_shared<SynchronizerGPS>(TimeSyncGPS(kROSQueueSize),
                                               subNavPos_, subNavStatus_,
                                               subNavVel_);
  syncGps_->registerCallback( boost::bind(&Node::gpsCallback, this, _1, _2, _3) );
  
  //  advertise odometry as output
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr&,
                       const sensor_msgs::FluidPressureConstPtr&) {
  
  
  
}

void Node::gpsCallback(const ublox_msgs::NavPOSLLHConstPtr&,
                       const ublox_msgs::NavSTATUSConstPtr&,
                       const ublox_msgs::NavVELNEDConstPtr&) {
  
  
  
}

} //  namespace gps_odom
