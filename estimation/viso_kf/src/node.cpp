/*
 * node.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of viso_kf.
 *
 *	Created on: 03/08/2014
 *		  Author: gareth
 */

#include <viso_kf/node.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <viso_kf/Vector3WithCovarianceStamped.h>
#include <kr_math/pose.hpp>

using ::viso_kf::Vector3WithCovarianceStamped;

namespace viso_kf {

Node::Node() : nh_("~"), tfListener_(tfCore_) {}

void Node::initialize() {
  //  configure topics
  pubOdometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  pubAccelBias_ = nh_.advertise<Vector3WithCovarianceStamped>("accel_bias", 1);
  pubGyroBias_ = nh_.advertise<Vector3WithCovarianceStamped>("gyro_bias", 1);
  pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);

  subImu_ = nh_.subscribe("imu", 1, &Node::imuCallback, this);
  subOdometry_ = nh_.subscribe("odom", 1, &Node::odoCallback, this);

  nh_.param("gyro_bias_drift_std", gyroBiasDriftStd_, 1.0e-3);
  nh_.param("accel_bias_drift_std", accelBiasDriftStd_, 1.0e-2);

  //  amount of time in seconds to wait before initializing filter
  nh_.param("init_dead_time", initDeadTime_, 0.25);

  ROS_INFO("Gyro bias drift uncertainty: %f", gyroBiasDriftStd_);
  ROS_INFO("Accel bias drift uncertainty: %f", accelBiasDriftStd_);
  ROS_INFO("Initialization dead time: %f", initDeadTime_);

  predictTime_ = ros::Time(0, 0);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr &imu) {
  if (!initialized_) {
    return;  //  wait for first visual odometry
  }
  //tfPub_.set_child_frame_id("imu");

  Eigen::Vector3d accel;
  Eigen::Matrix3d varAccel;
  tf::vectorMsgToEigen(imu->linear_acceleration, accel);

  Eigen::Vector3d gyro;
  Eigen::Matrix3d varGyro;
  tf::vectorMsgToEigen(imu->angular_velocity, gyro);

  Eigen::Quaterniond wQb;
  Eigen::Matrix3d varRot;
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
  Eigen::Matrix3d Qbg;
  Qbg.setIdentity();
  Qbg *= gyroBiasDriftStd_ * gyroBiasDriftStd_;
  Eigen::Matrix3d Qba;
  Qba.setIdentity();
  Qba *= accelBiasDriftStd_ * accelBiasDriftStd_;
  positionKF_.setBiasUncertainties(Qbg, Qba);
  positionKF_.predict(gyro, varGyro, accel, varAccel, delta);

  //  output covariance
  const Eigen::Matrix<double, 15, 15> &P = positionKF_.getCovariance();
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
  
  const kr::Pose<double> imuToWorld(positionKF_.getOrientation(), 
                                    positionKF_.getPosition());
  
  kr::Pose<double> baseToImu;
  try {
    //  oh god what does this even do???
    //  hard-code ALL the things!
    const geometry_msgs::TransformStamped transform =
        tfCore_.lookupTransform("imu", "base_link", ros::Time(0));
    const geometry_msgs::Vector3 &t = transform.transform.translation;
    const geometry_msgs::Quaternion &r = transform.transform.rotation;
    const Eigen::Vector3d p(t.x, t.y, t.z);
    const Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    baseToImu.q() = q;
    baseToImu.p() = p;
  }
  catch (const tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
  }
  kr::Pose<double> baseToWorld = imuToWorld.composeInBody(baseToImu);
  odo.pose.pose = static_cast<geometry_msgs::Pose>(baseToWorld);
  
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
  Eigen::Vector3d angRate;
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
  tfPub_.set_child_frame_id("base_link");
  tfPub_.PublishTransform(odo.pose.pose, odo.header);

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

  //  do conversion here:
  kr::Pose<double> imu_in_child;
  try {
    /// @todo: don't hardcode this...
    const geometry_msgs::TransformStamped transform =
        tfCore_.lookupTransform(odometry->child_frame_id, "imu", ros::Time(0));
    const geometry_msgs::Vector3 &t = transform.transform.translation;
    const geometry_msgs::Quaternion &r = transform.transform.rotation;
    const Eigen::Vector3d p(t.x, t.y, t.z);
    const Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    imu_in_child.q() = q;
    imu_in_child.p() = p;
  }
  catch (const tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
  }
  kr::Pose<double> odomPose(odometry->pose.pose);
  odomPose = odomPose.composeInBody(imu_in_child);

  Eigen::Matrix3d varP, varQ;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      varP(i, j) = odometry->pose.covariance[(i * 6) + j];
      varQ(i, j) = odometry->pose.covariance[(i + 3) * 6 + j + 3];
    }
  }

  if (!initialized_ &&
      (odometry->header.stamp - firstTs).toSec() > initDeadTime_) {
    initialized_ = true;
    //  take our output frame ID from the gps_odom
    worldFrameId_ = odometry->header.frame_id;
    positionKF_.initState(odomPose.q(), odomPose.p(), Eigen::Vector3d(0, 0, 0));
    positionKF_.initCovariance(1e-2, 1e-4, 0.25, 0.2, 0.5);
  } else {
    if (!positionKF_.update(odomPose.q(), varQ, odomPose.p(), varP)) {
      ROS_WARN("Warning: Kalman gain was singular in update");
    }
  }
}
}
