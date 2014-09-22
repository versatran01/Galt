/*
 * sam_estimator_node.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#include <sam_estimator/sam_estimator_node.hpp>
#include <sam_estimator/sam_estimator.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <memory>

using std::make_shared;

namespace galt {
namespace sam_estimator {

SamEstimatorNode::SamEstimatorNode(const ros::NodeHandle &nh) : 
  nh_(nh), tf_listener_(core_), traj_viz_(nh_) {
  //  create an estimator
  estimator_ = make_shared<SamEstimator>();
  
  //  subscribe to topics
  sub_features_.subscribe(nh_, "features", kROSQueueSize);
  sub_odom_.subscribe(nh_, "odometry_in", kROSQueueSize);
  
  sync_ = make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(
             TimeSyncPolicy(kROSQueueSize), sub_odom_, sub_features_);
  sync_->registerCallback(boost::bind(&SamEstimatorNode::odomFeaturesCallback,
                                      this,_1,_2));
  
  sub_l_info_.subscribe(nh_, "left_info", kROSQueueSize);
  sub_r_info_.subscribe(nh_, "right_info", kROSQueueSize);
  sync_info_ = make_shared<message_filters::Synchronizer<InfoTimeSyncPolicy>>(
                  InfoTimeSyncPolicy(kROSQueueSize), sub_l_info_, sub_r_info_);
  
  ROS_INFO("Subscribing to ~features and ~odometry_in");
  ROS_INFO("Subscribing to ~left_info and ~right_info");
  
  pub_odometry_ =
      nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  pub_points_ = 
      nh_.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
  
  traj_viz_.set_color(kr::rviz_helper::colors::BLUE);
  traj_viz_.set_alpha(1);
  traj_viz_.set_num_skip(12);
  
  /// @todo: make this an option
  tf_pub_.set_child_frame_id("sam_estimator");
}

void 
SamEstimatorNode::odomFeaturesCallback(
    const nav_msgs::OdometryConstPtr& odom_msg,
    const stereo_vo::StereoFeaturesStampedConstPtr &feat_msg) {
  
  //  extract odometry pose
  const kr::Posed odom_pose(odom_msg->pose.pose);
  kr::mat<double,6,6> odom_cov_in;
  for (int i=0; i < 6; i++) {
    for (int j=0; j < 6; j++) {
      odom_cov_in(i,j) = odom_msg->pose.covariance[(i*6) + 6];
    }
  }
  //  swap order of parameters for gtsam
  kr::mat<double,6,6> odom_cov;
  odom_cov.block<3,3>(0,0) = odom_cov_in.block<3,3>(3,3);
  odom_cov.block<3,3>(3,3) = odom_cov_in.block<3,3>(0,0);
  odom_cov.block<3,3>(0,3) = odom_cov_in.block<3,3>(0,3).transpose();
  odom_cov.block<3,3>(3,0) = odom_cov.block<3,3>(0,3).transpose();
  
  //  get transform from IMU to stereo
  try {
    /// @todo: don't hard-code these strings
    const geometry_msgs::TransformStamped transform = core_.lookupTransform(
        "stereo", "imu", ros::Time(0));
    const geometry_msgs::Vector3& t = transform.transform.translation;
    const geometry_msgs::Quaternion& r = transform.transform.rotation;
    const kr::vec3d p(t.x, t.y, t.z);
    const kr::quatd q(r.w, r.x, r.y, r.z);
    const kr::Posed stereo_in_imu(q,p);
    
    estimator_->SetSensorPose(stereo_in_imu);
  }
  catch (const tf2::TransformException& e) {
    ROS_WARN("%s", e.what());
  }
  
  estimator_->AddFrame(odom_pose,odom_cov,feat_msg->left,feat_msg->right);
  if (estimator_->IsInitialized()) {
    
    //  publish odometry
    nav_msgs::Odometry odo;
    odo.header.stamp = odom_msg->header.stamp;
    odo.header.frame_id = odom_msg->header.frame_id;
    odo.child_frame_id = "sam_estimator";

    //  current pose
    gtsam::Pose3 pose = estimator_->current_pose();
    odo.pose.pose = static_cast<geometry_msgs::Pose>(kr::Posed(pose));
    
    tf_pub_.PublishTransform(odo.pose.pose, odo.header);
    traj_viz_.PublishTrajectory(odo.pose.pose, odo.header);
    pub_odometry_.publish(odo);
  }
  
  std::vector<gtsam::Point3> points = estimator_->triangulated_points();
  if (!points.empty()) {
    //  publish a point cloud also...
    
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = odom_msg->header.stamp;
    cloud.header.frame_id = odom_msg->header.frame_id;
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "rgb";
  
    union {
      uint8_t rgb[4];
      float val;
    } color;
  
    for (const gtsam::Point3& pt : points) {
      //  find corresponding point 3d  
      //if (!point3d.is_inlier()) {
        color.rgb[0] = 255;
        color.rgb[1] = 255;
        color.rgb[2] = 0;
        color.rgb[3] = 0;
      //} else {
//        color.rgb[0] = 0;
//        color.rgb[1] = 255;
//        color.rgb[2] = 255;
//        color.rgb[3] = 0;
      //}
  
      geometry_msgs::Point32 p32;
      p32.x = pt.x();
      p32.y = pt.y();
      p32.z = pt.z();
  
      cloud.points.push_back(p32);
      channel.values.push_back(color.val);
    }
  
    cloud.channels.push_back(channel);
    pub_points_.publish(cloud);
  }
}

void 
SamEstimatorNode::camInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& l_info,
    const sensor_msgs::CameraInfoConstPtr& r_info) {
  
  model_.fromCameraInfo(l_info,r_info);
  ROS_INFO("Initialized baseline: %f", model_.baseline());
  estimator_->SetCameraModel(model_);
  sub_l_info_.unsubscribe();
  sub_r_info_.unsubscribe();
  sync_info_.reset();
} 

}  // namespace sam_estimator
}  // namespace galt
