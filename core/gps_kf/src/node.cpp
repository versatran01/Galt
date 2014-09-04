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
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

namespace gps_kf {

Node::Node() : nh_("~") {
  /// @todo: figureo out to with these...
  nh_.param<std::string>("frame_id", frameId_, "imu");
  nh_.param<std::string>("child_frame_id", childFrameId_, "world");
}

void Node::initialize() {
  //  configure topics
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  
  subImu_ = nh_.subscribe("imu", 1, &Node::imuCallback, this);
  subOdometry_ = nh_.subscribe("gps_odom", 1, &Node::odoCallback, this);
  
  predictTime_ = ros::Time(0,0);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
  if (!initialized_) {
    return; //  wait for first GPS
  }
  
  kr::vec3d accel;
  kr::mat3d varAccel;
  tf::vectorMsgToEigen(imu->linear_acceleration, accel);
  
  kr::vec3d gyro;
  kr::mat3d varGyro;
  tf::vectorMsgToEigen(imu->angular_velocity, gyro);
  
  kr::quatd wQb;
  kr::mat3d varRot;
  tf::quaternionMsgToEigen(imu->orientation, wQb);
  
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      varAccel(i,j) = imu->linear_acceleration_covariance[i*3 + j];
      varGyro(i,j) = imu->angular_velocity_covariance[i*3 + j];
      varRot(i,j) = imu->orientation_covariance[i*3 + j];
    }
  }
  
  double delta = 0.0;
  if (predictTime_.sec != 0) {
    delta = (imu->header.stamp - predictTime_).toSec();
  }
  predictTime_ = imu->header.stamp;
  
  //  predict
  positionKF_.setBiasUncertainties(1e-4, 1e-2);
  positionKF_.predict(gyro,varGyro,accel,varAccel,delta);
  
  static ros::Publisher pubBias = nh_.advertise<geometry_msgs::Vector3>("bias", 1);
  geometry_msgs::Vector3 bias;
  tf::vectorEigenToMsg(positionKF_.getAccelBias(), bias);
  pubBias.publish(bias);
  
  //  output covariance
  const kr::mat<double,15,15>& P = positionKF_.getCovariance();
    
  //  publish odometry
  nav_msgs::Odometry odo;
  odo.header.stamp = imu->header.stamp;
  odo.header.frame_id = frameId_;
  odo.child_frame_id = childFrameId_;
  
  tf::pointEigenToMsg(positionKF_.getPosition(), odo.pose.pose.position);
  tf::quaternionEigenToMsg(positionKF_.getOrientation(), odo.pose.pose.orientation);
  
  static ros::Publisher pubPoseStamped = nh_.advertise<geometry_msgs::PoseStamped>("debug_pose", 1);
   geometry_msgs::PoseStamped ps;
  ps.pose = odo.pose.pose;
  ps.header.stamp = imu->header.stamp;
  ps.header.frame_id = frameId_;
  pubPoseStamped.publish(ps);
  
  //  top left 3x3 (filter) and bottom right 3x3 (from imu)
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      odo.pose.covariance[(i*6) + j] = P(i+12,j+12);
      odo.pose.covariance[(i+3)*6 + (j+3)] = P(i,j);
    }
  }
  
  tf::vectorEigenToMsg(positionKF_.getVelocity(), odo.twist.twist.linear);
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
  
  static ros::Time firstTs = odometry->header.stamp;
  
  kr::vec3d p,v;
  kr::quatd q;
  tf::quaternionMsgToEigen(odometry->pose.pose.orientation, q);
  tf::pointMsgToEigen(odometry->pose.pose.position, p);
  tf::vectorMsgToEigen(odometry->twist.twist.linear, v);
  
  kr::mat3d varP,varV,varQ;
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      varP(i,j) = odometry->pose.covariance[(i*6) + j];
      varQ(i,j) = odometry->pose.covariance[(i+3)*6 + j+3];
      varV(i,j) = odometry->twist.covariance[(i*6) + j];
    }
  }
  
  if (!initialized_ && (odometry->header.stamp - firstTs).toSec() > 0.25) {
    initialized_ = true;
    positionKF_.initState(q, p, v);
    positionKF_.initCovariance(1e-1, 1e-4, 0.8, 1.0, 5.0);
  } else {
    //std::cout << "P: " << positionKF_.getCovariance().block<3,3>(12,12) << std::endl;
    //std::cout << "Q: " << positionKF_.getCovariance().block<3,3>(0,0) << std::endl;
    
    if (!positionKF_.update(q,varQ,p,varP,v,varV)) {
      ROS_WARN("Warning: Kalman gain was singular in update");
    }
  }
}

}
