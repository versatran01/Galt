/*
 * calib_proc.h
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of flir_gige.
 *
 *	Created on: 22/08/2014
 */

#ifndef FLIR_GIGE_CALIB_PROC_H_
#define FLIR_GIGE_CALIB_PROC_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <flir_gige/CalibProcDynConfig.h>

namespace flir_gige {

class CalibProc {
 public:
  /**
   * @brief CalibProc Constructor
   * @param nh Common ros node handle
   * @param pnh Private ros node handle
   */
  CalibProc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  void ImageCallback(const sensor_msgs::ImageConstPtr &image_msg);
  void ReconfigureCallback(const flir_gige::CalibProcDynConfig &config,
                           int level);

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  dynamic_reconfigure::Server<flir_gige::CalibProcDynConfig> server_;
};
}  // namespace flir_gig
#endif  // FLIR_GIGE_CALIB_PROC_H_
