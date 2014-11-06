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
 *  This file is part of laser_altimeter.
 *
 *	Created on: 24/08/2014
 */

#include <laser_altimeter/node.hpp>
#include <laser_altimeter/Height.h>  //  published message
#include <stdexcept>
#include <memory>

#include <kr_math/base_types.hpp>
#include <kr_math/pose.hpp>
#include <sensor_msgs/PointCloud.h>

using std::make_shared;

namespace galt {
namespace laser_altimeter {

Node::Node(const ros::NodeHandle &nh) : nh_(nh), tfListener_(tfCore_) {
  //  determine queue size to use when synchronizing
  int queue_size;
  nh.param("queue_size", queue_size, 200);
  if (queue_size <= 0) {
    throw std::runtime_error("queue_size must be greater than zero");
  }

  //  load transform between laser and IMU
  //  if (nh.hasParam("laser_transform")) {
  //    nh_.param("laser_transform/orientation/w", iQl_.w(), 1.0);
  //    nh_.param("laser_transform/orientation/x", iQl_.x(), 0.0);
  //    nh_.param("laser_transform/orientation/y", iQl_.y(), 0.0);
  //    nh_.param("laser_transform/orientation/z", iQl_.z(), 0.0);
  //    nh_.param("laser_transform/position/x", iPl_[0], 0.0);
  //    nh_.param("laser_transform/position/y", iPl_[1], 0.0);
  //    nh_.param("laser_transform/position/z", iPl_[2], 0.0);

  //    ROS_INFO_STREAM("Loaded transform from yaml:\n" << iQl_.matrix());
  //  } else {
  //    ROS_WARN("No laser/imu transform provided - assuming identity");
  //    iQl_ = kr::Quatd(1, 0, 0, 0);
  //    iPl_.setZero();
  //  }
  nh.param("world_frame_id", worldFrameId_, std::string("world"));

  //  min/max angles to use on lidar, default to [-inf, inf]
  nh.param("angle_min", angleMin_, -std::numeric_limits<double>::infinity());
  nh.param("angle_max", angleMax_, std::numeric_limits<double>::infinity());
  ROS_INFO("Min/max angles: %f, %f", angleMin_, angleMax_);

  subImu_.subscribe(nh_, "imu", queue_size);
  subScan_.subscribe(nh_, "laser_scan", queue_size);
  sync_ = std::make_shared<Synchronizer>(TimeSyncPolicy(queue_size), subImu_,
                                         subScan_);

  sync_->registerCallback(boost::bind(&Node::syncCallback, this, _1, _2));

  //  publish a height message
  pubHeight_ = nh_.advertise<::laser_altimeter::Height>("height", 1);
}

void Node::syncCallback(const sensor_msgs::ImuConstPtr &imuMsg,
                        const sensor_msgs::LaserScanConstPtr &laserMsg) {

  //  IMU to world rotation
  const kr::Quatd wQi(imuMsg->orientation.w, imuMsg->orientation.x,
                      imuMsg->orientation.y, imuMsg->orientation.z);

  const double ang_start = laserMsg->angle_min;
  const double ang_inc = laserMsg->angle_increment;

  const kr::Mat3d wRi = wQi.matrix();

  std::vector<double> heights;
  heights.reserve(laserMsg->ranges.size());

#ifdef DEBUG_PUB_CLOUD
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "laser";
#endif

  //  get laser to imu transform
  kr::Posed laser_to_imu;
  try {
    const geometry_msgs::TransformStamped transform =
        tfCore_.lookupTransform("imu", "laser", ros::Time(0));
    const geometry_msgs::Vector3 &t = transform.transform.translation;
    const geometry_msgs::Quaternion &r = transform.transform.rotation;
    const kr::Vec3d p(t.x, t.y, t.z);
    const kr::Quatd q(r.w, r.x, r.y, r.z);
    laser_to_imu.q() = q;
    laser_to_imu.p() = p;
  }
  catch (const tf2::TransformException &e) {
    ROS_WARN("%s", e.what());
  }

  double angle = ang_start;
  for (const float &range : laserMsg->ranges) {
    if (range > laserMsg->range_min && range < laserMsg->range_max) {
      //  consider only angles in the specified range
      if (angle >= angleMin_ && angle <= angleMax_) {
        //  rotation from beam to laser
        const Eigen::AngleAxisd lRotb(angle, kr::Vec3d(0, 0, 1));

        const kr::Vec3d v_b(range, 0, 0);  //  vector in the beam frame
        const kr::Vec3d v_l = lRotb.matrix() * v_b;  //  laser frame

        const kr::Vec3d v_i = laser_to_imu.transformToBody(v_l);  //  IMU frame

        //  rotate the vector to world frame (without translating)
        const kr::Vec3d v_w = wRi * v_i;

#ifdef DEBUG_PUB_CLOUD
        geometry_msgs::Point32 p32;
        p32.x = v_w[0];
        p32.y = v_w[1];
        p32.z = v_w[2];
        cloud.points.push_back(p32);
#endif

        const double height = v_w[2];
        if (height < 0) {
          //  consider only those vectors pointed downwards
          heights.push_back(-height);
        }
      }
    }
    angle += ang_inc;
  }

#ifdef DEBUG_PUB_CLOUD
  static ros::Publisher pubCloud =
      nh_.advertise<sensor_msgs::PointCloud>("pc", 1);
  pubCloud.publish(cloud);
#endif

  //  determine height stats
  /// @todo: Add a ransac based technique here for added robustness
  double max = 0;
  double mean = 0;
  double mean2 = 0;
  for (const double &h : heights) {
    max = std::max(max, h);
    mean += h;
    mean2 += h * h;
  }
  mean /= heights.size();
  mean2 /= heights.size();
  const double var = mean2 - mean * mean;

  //  publish
  ::laser_altimeter::Height msg;
  msg.header.stamp = laserMsg->header.stamp;
  msg.header.frame_id = worldFrameId_;
  if (heights.empty()) {
    msg.max = msg.mean = 0;
    msg.variance = -1;  //  leave other fields zero
  } else {
    msg.max = max;
    msg.mean = mean;
    msg.variance = var;
  }
  pubHeight_.publish(msg);
}

}  //  laser_altimeter
}  //  galt
