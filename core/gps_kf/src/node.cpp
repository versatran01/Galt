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
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <gps_kf/Vector3WithCovarianceStamped.h>

using ::gps_kf::Vector3WithCovarianceStamped;

namespace gps_kf {

Node::Node() : nh_("~") {}

void Node::initialize() {
  //  configure topics
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  pubAccelBias_ = nh_.advertise<Vector3WithCovarianceStamped>("accel_bias", 1);
  pubGyroBias_ = nh_.advertise<Vector3WithCovarianceStamped>("gyro_bias", 1);
  pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);

  subImu_ = nh_.subscribe("imu", 1, &Node::imuCallback, this);
  subOdometry_ = nh_.subscribe("gps_odom", 1, &Node::odoCallback, this);

  predictTime_ = ros::Time(0, 0);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr &imu) {
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

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      varAccel(i, j) = imu->linear_acceleration_covariance[i * 3 + j];
      varGyro(i, j) = imu->angular_velocity_covariance[i * 3 + j];
      varRot(i, j) = imu->orientation_covariance[i * 3 + j];
    }
  }

  double delta = 0.0;
  if (predictTime_.sec != 0) {
    delta = (imu->header.stamp - predictTime_).toSec();
  }
  predictTime_ = imu->header.stamp;

  //  predict
  kr::mat3d Qbg = kr::mat3d::Identity();
  Qbg *= 1e-4;
  kr::mat3d Qba = kr::mat3d::Identity();
  Qba *= 1e-2;
  positionKF_.setBiasUncertainties(Qbg, Qba);
  positionKF_.predict(gyro, varGyro, accel, varAccel, delta);

  //  output covariance
  const kr::mat<double, 15, 15> &P = positionKF_.getCovariance();
  const auto &rotCov = P.block<3, 3>(0, 0);
  const auto &gBiasCov = P.block<3, 3>(3, 3);
  const auto &velCov = P.block<3, 3>(6, 6);
  const auto &aBiasCov = P.block<3, 3>(9, 9);
  const auto &posCov = P.block<3, 3>(12, 12);

  //  publish odometry
  nav_msgs::Odometry odo;
  odo.header.stamp = imu->header.stamp;
  odo.header.frame_id = worldFrameId_;
  odo.child_frame_id = worldFrameId_;
  //  NOTE: angular velocity is still in body frame, but we use worldFrameId_
  //  anyways
  tf::pointEigenToMsg(positionKF_.getPosition(), odo.pose.pose.position);
  tf::quaternionEigenToMsg(positionKF_.getOrientation(),
                           odo.pose.pose.orientation);

  geometry_msgs::PoseStamped pose;
  pose.pose = odo.pose.pose;
  pose.header = odo.header;
  pubPose_.publish(pose);

  //  top left 3x3 (filter) and bottom right 3x3 (from imu)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      odo.pose.covariance[(i * 6) + j] = posCov(i, j);
      odo.pose.covariance[(i + 3) * 6 + (j + 3)] = rotCov(i, j);
    }
  }

  tf::vectorEigenToMsg(positionKF_.getVelocity(), odo.twist.twist.linear);

  //  subtract our bias estimate and propagate covariance
  kr::vec3d angRate;
  tf::vectorMsgToEigen(imu->angular_velocity, angRate);
  angRate.noalias() -= positionKF_.getGyroBias();
  varGyro.noalias() += gBiasCov;
  tf::vectorEigenToMsg(angRate, odo.twist.twist.angular);

  //  output velocity covariances
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      odo.twist.covariance[(i * 6) + j] = velCov(i, j);
      odo.twist.covariance[(i + 3) * 6 + (j + 3)] = varGyro(i, j);
    }
  }
  pubOdometry_.publish(odo);

  //  publish bias messages
  Vector3WithCovarianceStamped aBiasVector, gBiasVector;

  aBiasVector.header.stamp = odo.header.stamp;
  aBiasVector.header.frame_id = imu->header.frame_id;
  tf::vectorEigenToMsg(positionKF_.getAccelBias(), aBiasVector.vector);

  gBiasVector.header.stamp = odo.header.stamp;
  gBiasVector.header.frame_id = imu->header.frame_id;
  tf::vectorEigenToMsg(positionKF_.getGyroBias(), gBiasVector.vector);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      aBiasVector.covariance[i * 3 + j] = aBiasCov(i, j);
      gBiasVector.covariance[i * 3 + j] = gBiasCov(i, j);
    }
  }

  pubAccelBias_.publish(aBiasVector);
  pubGyroBias_.publish(gBiasVector);
}

void Node::odoCallback(const nav_msgs::OdometryConstPtr &odometry) {

  static ros::Time firstTs = odometry->header.stamp;

  kr::vec3d p, v;
  kr::quatd q;
  tf::quaternionMsgToEigen(odometry->pose.pose.orientation, q);
  tf::pointMsgToEigen(odometry->pose.pose.position, p);
  tf::vectorMsgToEigen(odometry->twist.twist.linear, v);

  kr::mat3d varP, varV, varQ;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      varP(i, j) = odometry->pose.covariance[(i * 6) + j];
      varQ(i, j) = odometry->pose.covariance[(i + 3) * 6 + j + 3];
      varV(i, j) = odometry->twist.covariance[(i * 6) + j];
    }
  }

  if (!initialized_ && (odometry->header.stamp - firstTs).toSec() > 0.25) {
    initialized_ = true;
    //  take our output frame ID from the gps_odom
    worldFrameId_ = odometry->header.frame_id;
    positionKF_.initState(q, p, v);
    positionKF_.initCovariance(1e-1, 1e-4, 0.8, 1.0, 5.0);
  } else {
    // std::cout << "P: " << positionKF_.getCovariance().block<3,3>(12,12) <<
    // std::endl;
    // std::cout << "Q: " << positionKF_.getCovariance().block<3,3>(0,0) <<
    // std::endl;

    if (!positionKF_.update(q, varQ, p, varP, v, varV)) {
      ROS_WARN("Warning: Kalman gain was singular in update");
    }
  }
}
}
