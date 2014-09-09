/*
 * node.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#ifndef GPS_ODOM_NODE_HPP
#define GPS_ODOM_NODE_HPP

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pressure_altimeter/Height.h>

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <rviz_helper/rviz_helper.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace gps_odom {

class Node {
public:
  Node();

  void initialize();

private:
  static constexpr int kROSQueueSize = 200;

  ros::NodeHandle nh_;
  std::string pkgPath_;
  std::string worldFrameId_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubRefPoint_;

  message_filters::Subscriber<sensor_msgs::Imu> subImu_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subFix_;
  message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>
  subFixTwist_;
  message_filters::Subscriber<pressure_altimeter::Height> subHeight_;

  //  time sync policy for GPS data
  using TimeSyncGPS = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped,
      sensor_msgs::Imu, pressure_altimeter::Height>;
  using SynchronizerGPS = message_filters::Synchronizer<TimeSyncGPS>;
  std::shared_ptr<SynchronizerGPS> syncGps_;

  void gpsCallback(
      const sensor_msgs::NavSatFixConstPtr &,
      const geometry_msgs::TwistWithCovarianceStampedConstPtr &navSatTwist,
      const sensor_msgs::ImuConstPtr &,
      const pressure_altimeter::HeightConstPtr &height);

  //  geographic lib objects
  std::shared_ptr<GeographicLib::Geoid> geoid_;
  std::shared_ptr<GeographicLib::MagneticModel> magneticModel_;

  bool refSet_;
  GeographicLib::LocalCartesian refPoint_;
  double refHeight_;
  double currentDeclination_;

  tf2_ros::TransformBroadcaster broadcaster_;
  rviz_helper::TrajectoryVisualizer trajViz_;
  rviz_helper::CovarianceVisualizer covViz_;
};

} //  namespace_gps_odom

#endif // GPS_ODOM_NODE_HPP
