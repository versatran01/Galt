/*
 * thermal_proc.cpp
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

#include "flir_gige/thermal_proc/thermal_proc.h"

#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <iterator>

#include <opencv2/contrib/contrib.hpp>

namespace flir_gige {

ThermalProc::ThermalProc(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
#ifdef NO_TIMESTAMP
  image_sub_ = it_.subscribe("image_raw", 1, &ThermalProc::ImageCallback, this);
#else
  camera_sub_ =
      it_.subscribeCamera("image_raw", 1, &ThermalProc::CameraCallback, this);

#endif
  heat_pub_ = nh_.advertise<sensor_msgs::Image>("temperature", 1);
  heat_map_ = sensor_msgs::ImagePtr(new sensor_msgs::Image);

  color_pub_ = nh_.advertise<sensor_msgs::Image>("color_map", 1);
  color_map_ = sensor_msgs::ImagePtr(new sensor_msgs::Image);

  server_.setCallback(
      boost::bind(&ThermalProc::ReconfigureCallback, this, _1, _2));
}

#ifdef NO_TIMESTAMP
void ThermalProc::ImageCallback(const sensor_msgs::ImageConstPtr &image) {
  double B = 1428.0;
  double F = 1.0;
  double O = 118.126;
  double R = 377312.0;
//  double f = 329.3387;
//  int cx = 160, cy = 128;
#else
void ThermalProc::CameraCallback(const sensor_msgs::ImageConstPtr &image,
                                 const sensor_msgs::CameraInfoConstPtr &cinfo) {
  double B = cinfo->P[0];
  double F = cinfo->P[1];
  double O = cinfo->P[2];
  double R = cinfo->P[3];
#endif
  // Get image using cv_bridge
  cv_bridge::CvImagePtr raw_ptr;
  try {
    raw_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO16);
  }
  catch (const cv_bridge::Exception &e) {
    ROS_ERROR_STREAM("cv_bridge: " << e.what());
    return;
  }

  // Convert each pixle form 16-bit raw value to temperature
  const auto height = raw_ptr->image.rows;
  const auto width = raw_ptr->image.cols;
  // Caluculate temperature for each pixels
  cv::Mat heat(raw_ptr->image.size(), CV_32FC1);
  for (int i = 0; i < height; ++i) {
    auto *pheat = heat.ptr<float>(i);
    auto *praw = raw_ptr->image.ptr<uint16_t>(i);
    for (int j = 0; j < width; ++j) {
      pheat[j] = RawToCelsius(praw[j], B, F, O, R, kT0);
    }
  }

  // Use cv_bridge to publish
  cv_bridge::CvImage heat_cvimg(image->header,
                                sensor_msgs::image_encodings::TYPE_32FC1, heat);
  heat_pub_.publish(heat_cvimg.toImageMsg());

  // Convert to jet color map if there's any subscriber
  if (color_pub_.getNumSubscribers() > 0) {
    cv::Mat image_jet;
    // Calculate corresponding raw value from temperature
    uint16_t raw_min = CelsiusToRaw(celsius_min_, B, F, O, R, kT0);
    uint16_t raw_max = CelsiusToRaw(celsius_max_, B, F, O, R, kT0);
    auto alpha = 255.0 / (raw_max - raw_min);
    auto beta = -alpha * raw_min;
    raw_ptr->image.convertTo(image_jet, CV_8UC1, alpha, beta);
    cv::applyColorMap(image_jet, image_jet, cv::COLORMAP_JET);
    // Use cv_bridge to publish
    cv_bridge::CvImage color_cvimg(
        image->header, sensor_msgs::image_encodings::BGR8, image_jet);
    color_pub_.publish(color_cvimg.toImageMsg());
  }

  // Display image for now
  //  cv::imshow("raw", raw_ptr->image);
  //  cv::waitKey(3);
}

uint16_t ThermalProc::CelsiusToRaw(double t, double B, double F, double O,
                                  double R, double T0) {
  return static_cast<uint16_t>(R / (std::exp(B / (t + T0)) - F) + O);
}

double ThermalProc::RawToCelsius(uint16_t S, double B, double F, double O,
                                double R, double T0) {
  return (B / std::log(R / (S - O) + F) - kT0);
}

void ThermalProc::ReconfigureCallback(const ProcDynConfig &config, int level) {
  // Do nothing when first starting
  if (level < 0) {
    ROS_INFO(
        "flir_gige: thermal_proc: Initializiting dynamic reconfigure server");
    return;
  }
  celsius_min_ = config.celcius_min;
  celsius_max_ = config.celcius_max;
}

}  // namespace flir_gige
