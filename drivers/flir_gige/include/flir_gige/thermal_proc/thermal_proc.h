/*
 * thermal_proc.h
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
 *	Created on: 21/08/2014
 */

#ifndef FLIR_GIGE_THERMAL_PROC_H_
#define FLIR_GIGE_THERMAL_PROC_H_

// In case flir_gige/camera_info has no timestamp
#define NO_TIMESTAMP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "flir_gige/ProcDynConfig.h"

namespace flir_gige {

class ThermalProc {
 public:
  ThermalProc(const ros::NodeHandle &nh);

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Publisher heat_pub_;
  sensor_msgs::ImagePtr heat_map_;
  ros::Publisher color_pub_;
  sensor_msgs::ImagePtr color_map_;
  dynamic_reconfigure::Server<ProcDynConfig> server_;
  double celsius_min_{20.0};
  double celsius_max_{40.0};
  const double kT0{273.15};

  uint16_t CelsiusToRaw(double t, double B, double F, double O, double R, double T0);
  double RawToCelsius(uint16_t S, double B, double F, double O, double R, double T0);
  void ReconfigureCallback(const ProcDynConfig &config, int level);

#ifdef NO_TIMESTAMP
  image_transport::Subscriber image_sub_;
  void ImageCallback(const sensor_msgs::ImageConstPtr &image);
#else
  image_transport::CameraSubscriber camera_sub_;
  void CameraCallback(const sensor_msgs::ImageConstPtr &image,
                      const sensor_msgs::CameraInfoConstPtr &cinfo);
#endif

};  // class ThermalProc

}  // namespace flir_gige

#endif  // FLIR_GIGE_THERMAL_PROC_H_
