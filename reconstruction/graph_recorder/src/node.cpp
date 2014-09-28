/*
 * node.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of graph_recorder.
 *
 *	Created on: 23/09/2014
 */

#include <graph_recorder/node.hpp>
#include <QDir>
#include <QIODevice>
#include <QString>

static const std::string kDel = ",";

void writePose(std::stringstream& ss,
               const std::string& del,
               const geometry_msgs::PoseWithCovariance& pose) {
  ss << pose.pose.orientation.w << del;
  ss << pose.pose.orientation.x << del;
  ss << pose.pose.orientation.y << del;
  ss << pose.pose.orientation.z << del;
  ss << pose.pose.position.x << del;
  ss << pose.pose.position.y << del;
  ss << pose.pose.position.z << del;
  //  row-major covariance
  for (int i=0; i < 6; i++) {
    for (int j=0; j < 6; j++) {
      ss << pose.covariance[i*6 + j] << del;
    }
  }
}

namespace galt {
namespace graph_recorder {

Node::Node(const ros::NodeHandle& nh) : nh_(nh) {}

void Node::Initialize() {
  if (initialized_) {
    throw std::runtime_error("Node is already initialized");
  }
  initialized_ = true;
  
  //  get user output destination
  std::string output_path;
  if (!nh_.hasParam("output")) {
    throw std::runtime_error("Please specify the 'output' argument");
  }
  nh_.getParam("output", output_path);
  
  QDir dir(QString::fromStdString(output_path));
  if (!dir.exists()) {
    if (!dir.mkpath(QString::fromStdString(output_path))) {
      throw std::runtime_error("Failed to create path " + output_path);
    }
  }
  
  // generate subpaths
  const std::string gps_odom_path(output_path + "/gps_odom.csv");
  const std::string vo_pose_path(output_path + "/vo_pose.csv");
  const std::string vo_features_path(output_path + "/vo_features.csv");
  
  //  open all necessary files
  gps_odom_out_.setFileName(QString::fromStdString(gps_odom_path));
  if (!gps_odom_out_.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    throw std::runtime_error("Failed to create " + gps_odom_path);
  }
  vo_poses_out_.setFileName(QString::fromStdString(vo_pose_path));
  if (!vo_poses_out_.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    throw std::runtime_error("Failed to create " + vo_pose_path);
  }
  vo_features_out_.setFileName(QString::fromStdString(vo_features_path));
  if (!vo_features_out_.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    throw std::runtime_error("Failed to create " + vo_features_path);
  }
  sub_gps_odom_.subscribe(nh_,"gps_odom", kROSQueueSize);
  sub_vo_pose_.subscribe(nh_,"vo_pose", kROSQueueSize);
  sub_features_.subscribe(nh_,"vo_features", kROSQueueSize);
  
  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(
             TimeSyncPolicy(kROSQueueSize), sub_gps_odom_, 
             sub_vo_pose_, sub_features_);
  
  sync_->registerCallback(boost::bind(&Node::Callback,this,_1,_2,_3));
}

void 
Node::Callback(const nav_msgs::OdometryConstPtr &odom_msg,
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg,
    const stereo_vo::FeaturesStampedConstPtr& feature_msg) {
  
  //  output pose first
  {
    std::stringstream ss;
    ss << std::setprecision(16);  //  high precision output
    ss << vo_pose_id_ << kDel;
    ss << pose_msg->header.stamp.sec << kDel;
    ss << pose_msg->header.stamp.nsec << kDel;
    writePose(ss, kDel, pose_msg->pose);
    ss << std::endl;
    
    const std::string pose_csv = ss.str();
    vo_poses_out_.write(pose_csv.c_str(), pose_csv.length());
    vo_poses_out_.flush();
    vo_pose_id_++;
  }
  
  /// @todo: output features
  
  //  output gps_odom pose
  {
    std::stringstream ss;
    ss << std::setprecision(16);
    ss << gps_odom_id_ << kDel;
    ss << odom_msg->header.stamp.sec << kDel;
    ss << odom_msg->header.stamp.nsec << kDel;
    writePose(ss, kDel, odom_msg->pose);
    ss << std::endl;
    //  write only the pose to disk, ignore twist
    const std::string pose_csv = ss.str();
    gps_odom_out_.write(pose_csv.c_str(), pose_csv.length());
    gps_odom_out_.flush();
    gps_odom_id_++;
  }
}

}
}
