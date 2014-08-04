/*
 * node.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_kf.
 *
 *	Created on: 03/08/2014
 *		  Author: gareth
 */

#include <gps_kf/node.hpp>

namespace gps_kf {

Node::Node() : nh_("~") {
  
  //  load noise parameters
  double stdGyro[3],stdAccel[3];
  
  nh_.param("noise_std/accel/x",stdAccel[0], 10.0);
  nh_.param("noise_std/accel/y",stdAccel[1], 10.0);
  nh_.param("noise_std/accel/z",stdAccel[2], 10.0);
  
  nh_.param("noise_std/gyro/x",stdGyro[0], 0.01);
  nh_.param("noise_std/gyro/y",stdGyro[1], 0.01);
  nh_.param("noise_std/gyro/z",stdGyro[2], 0.01);
  
  varAccel_.setZero();
  varGyro_.setZero();
  for (int i=0; i < 3; i++) {
   varAccel_(i,i) = stdAccel[i]*stdAccel[i];
   varGyro_(i,i) = stdGyro[i]*stdGyro[i];
  }
  
  nh_.param("world_frame_id", worldFrameId_, std::string("/world"));
  nh_.param("body_frame_id", bodyFrameId_, std::string("/body"));
}

void Node::initialize() {
  //  configure topics
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  
  subImu_ = nh_.subscribe("imu", 1, &Node::imuCallback, this);
  subOdometry_ = nh_.subscribe("gps_odom", 1, &Node::odoCallback, this);
  
  predictTime_ = ros::Time(0,0);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  
  //  TODO: change underlying node to use m/s^2
  kr::vec3d accel;
  accel[0] = imu->linear_acceleration.x * kOneG;
  accel[1] = imu->linear_acceleration.y * kOneG;
  accel[2] = imu->linear_acceleration.z * kOneG;
  
  kr::vec3d gyro;
  gyro[0] = imu->angular_velocity.x;
  gyro[1] = imu->angular_velocity.y;
  gyro[2] = imu->angular_velocity.z;
  
  kr::quatd wQb;
  wQb.w() = imu->orientation.w;
  wQb.x() = imu->orientation.x;
  wQb.y() = imu->orientation.y;
  wQb.z() = imu->orientation.z;
  
  double delta = 0.0;
  if (predictTime_.sec != 0) {
    delta = (imu->header.stamp - predictTime_).toSec();
  }
  predictTime_ = imu->header.stamp;
  
  //  predict
  positionKF_.predict(gyro,varGyro_,accel,varAccel_,delta);
      
  auto P = positionKF_.getCovariance();
  
  //  publish odometry
  nav_msgs::Odometry odo;
  odo.header.stamp = imu->header.stamp;
  odo.header.frame_id = worldFrameId_;
  odo.child_frame_id = bodyFrameId_;
  
  odo.pose.pose.position.x = positionKF_.getPosition()[0];
  odo.pose.pose.position.y = positionKF_.getPosition()[1];
  odo.pose.pose.position.z = positionKF_.getPosition()[2];
  odo.pose.pose.orientation.w = positionKF_.getOrientation().w();
  odo.pose.pose.orientation.x = positionKF_.getOrientation().x();
  odo.pose.pose.orientation.y = positionKF_.getOrientation().y();
  odo.pose.pose.orientation.z = positionKF_.getOrientation().z();
  
  //  top left 3x3 (filter) and bottom right 3x3 (from imu)
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      odo.pose.covariance[(i*6) + j] = P(i+12,j+12);
      odo.pose.covariance[(i+3)*6 + (j+3)] = P(i,j);
    }
  }
  
  odo.twist.twist.linear.x = positionKF_.getVelocity()[0];
  odo.twist.twist.linear.y = positionKF_.getVelocity()[1];
  odo.twist.twist.linear.z = positionKF_.getVelocity()[2];
  odo.twist.twist.angular = imu->angular_velocity;
  
  //  same as above copy
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      odo.twist.covariance[(i*6) + j] = P(i+6,j+6);
      odo.twist.covariance[(i+3)*6 + (j+3)] = imu->angular_velocity_covariance[(i*3) + j];
    }
  }
  
  pubOdometry_.publish(odo);
}

void Node::odoCallback(const nav_msgs::OdometryConstPtr& odometry) {
  
  kr::vec3d p,v;
  kr::quatd q = kr::quatd(odometry->pose.pose.orientation.w,
                          odometry->pose.pose.orientation.x,
                          odometry->pose.pose.orientation.y,
                          odometry->pose.pose.orientation.z);
  p[0] = odometry->pose.pose.position.x;
  p[1] = odometry->pose.pose.position.y;
  p[2] = odometry->pose.pose.position.z;
  v[0] = odometry->twist.twist.linear.x;
  v[1] = odometry->twist.twist.linear.y;
  v[2] = odometry->twist.twist.linear.z;
  
  kr::mat3d varP,varV,varQ;
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      varP(i,j) = odometry->pose.covariance[(i*6) + j];
      varQ(i,j) = odometry->pose.covariance[((i+3)*6) + j+3];
      varV(i,j) = odometry->twist.covariance[(i*6) + j];
    }
  }
  
  //  scale up velocity variance as velocity decreases
  const double magv = v.norm();
  const double f = std::min(std::max(1-magv,0.0),1.0);
  const double scale = 1.0 + 15*(3*f*f - 2*f*f*f);
  varV *= scale;
    
  if (!positionKF_.update(q,varQ,p,varP,v,varV)) {
    ROS_WARN("Warning: Kalman gain was singular in update");
  }
}

}
