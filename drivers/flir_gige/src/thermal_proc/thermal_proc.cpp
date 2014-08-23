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

#include <algorithm>
#include <cmath>
#include <iterator>

#include <opencv2/contrib/contrib.hpp>

namespace flir_gige {

ThermalProc::ThermalProc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_{nh}, pnh_{pnh}, it_{nh} {
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&ThermalProc::ConnectCb, this);
  pub_heat_ = it_.advertise("temperature", 1, connect_cb, connect_cb);
  pub_color_ = it_.advertise("color_map", 1, connect_cb, connect_cb);

  server_.setCallback(boost::bind(&ThermalProc::ConfigCb, this, _1, _2));
}

void ThermalProc::ConnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!pub_heat_.getNumSubscribers() && !pub_color_.getNumSubscribers())
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
    sub_camera_ = it_.subscribeCamera("image_raw", 1, &ThermalProc::CameraCb,
                                      this, hints);
  }
}

void ThermalProc::CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                           const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  // Verify camera is actually calibrated
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->D[0] == 0.0) {
    ROS_ERROR_THROTTLE(5,
                       "Topic '%s' requested but "
                       "camera publishing '%s' is uncalibrated",
                       pub_heat_.getTopic().c_str(),
                       sub_camera_.getInfoTopic().c_str());
    return;
  }

  // Get planck constants
  const Planck planck = GetPlanck(*cinfo_msg);

  // Get image using cv_bridge
  cv_bridge::CvImagePtr raw_ptr =
      cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO16);

  // Convert each pixel from 16-bit raw data to temperature
  if (pub_heat_.getNumSubscribers()) {
    cv::Mat heat(raw_ptr->image.size(), CV_32FC1);
    RawToHeat(raw_ptr->image, planck, &heat);
    cv_bridge::CvImage heat_cvimg(
        image_msg->header, sensor_msgs::image_encodings::TYPE_32FC1, heat);
    pub_heat_.publish(heat_cvimg.toImageMsg());
  }

  // Convert to jet color map
  if (pub_color_.getNumSubscribers()) {
    cv::Mat color;
    RawToJet(raw_ptr->image, planck, &color);
    cv_bridge::CvImage color_cvimg(image_msg->header,
                                   sensor_msgs::image_encodings::BGR8, color);
    pub_color_.publish(color_cvimg.toImageMsg());
    cv::imshow("color", color);
    cv::waitKey(3);
  }
}

void ThermalProc::RawToJet(const cv::Mat &raw, const Planck &planck,
                           cv::Mat *color) const {
  const int raw_min = CelsiusToRaw(config_.celcius_min, planck);
  const int raw_max = CelsiusToRaw(config_.celcius_max, planck);
  ROS_INFO_STREAM("spot: " << raw.at<uint16_t>(128, 160));
  ROS_ASSERT_MSG(raw_max > raw_min, "max is less than min");
  const double alpha = 255.0 / (raw_max - raw_min);
  const double beta = -alpha * raw_min;
  ROS_INFO_STREAM("alpha: " << alpha << " beta: " << beta);
  raw.convertTo(*color, CV_8UC1, alpha, beta);
  cv::applyColorMap(*color, *color, cv::COLORMAP_JET);
}

void ThermalProc::RawToHeat(const cv::Mat &raw, const Planck &planck,
                            cv::Mat *heat) const {
  for (int i = 0; i < raw.rows; ++i) {
    float *pheat = heat->ptr<float>(i);
    const uint16_t *praw = raw.ptr<uint16_t>(i);
    for (int j = 0; j < raw.cols; ++j) {
      pheat[j] = static_cast<uint16_t>(RawToCelsius(praw[j], planck));
    }
  }
}

void ThermalProc::ConfigCb(const Config &config, int level) {
  if (level < 0) {
    ROS_INFO(
        "flir_gige: thermal_proc: Initializiting dynamic reconfigure server");
  }
  config_ = config;
}

Planck GetPlanck(const sensor_msgs::CameraInfo &cinfo_msg) {
  Planck planck;
  planck.B = cinfo_msg.R[0];
  planck.F = cinfo_msg.R[1];
  planck.O = cinfo_msg.R[2];
  planck.R = cinfo_msg.R[3];
  return planck;
}

int CelsiusToRaw(const double t, const Planck &plk) {
  return plk.R / (std::exp(plk.B / (t + plk.kT0)) - plk.F) + plk.O;
}

double RawToCelsius(const int S, const Planck &plk) {
  return (plk.B / std::log(plk.R / (S - plk.O) + plk.F) - plk.kT0);
}

}  // namespace flir_gige
