/*
 * calib_proc.cpp
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

#include "flir_gige/calib_proc/calib_proc.h"

#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace flir_gige {

CalibProc::CalibProc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_{nh}, pnh_{pnh}, it_{nh} {
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  image_sub_ = it_.subscribe(ros::names::resolve("image_raw"), 1,
                             &CalibProc::ImageCallback, this, hints);
  image_pub_ = it_.advertise("image_calib", 1);
  server_.setCallback(
      boost::bind(&CalibProc::ReconfigureCallback, this, _1, _2));
  cv::namedWindow("calib_result",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
}

void CalibProc::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  cv::Mat image = cv_bridge::toCvCopy(
                      image_msg, sensor_msgs::image_encodings::MONO8)->image;
  cv::Mat image_calib;

  // Basic processing
  cv::threshold(image, image_calib, config_.thresh, 250, 1);
//  cv::GaussianBlur(image_calib, image_calib, cv::Size(5, 5), 5);

  // Publish image_calib
  cv_bridge::CvImage cv_image(image_msg->header, image_msg->encoding,
                              image_calib);
  image_pub_.publish(cv_image.toImageMsg());

  // Try detecting circles
  // Find all contours
  //  std::vector<std::vector<cv::Point>> contours;
  //  cv::findContours(image_calib.clone(), contours, CV_RETR_EXTERNAL,
  //                   CV_CHAIN_APPROX_NONE);
  //  cv::Mat image_contour = cv::Mat::zeros(image_calib.size(), CV_8UC3);
  //  for (unsigned i = 0; i < contours.size(); i++) {
  //    cv::drawContours(image_contour, contours, i, cv::Scalar(0, 0, 255), 2);
  //  }
  //  cv::imshow("contour", image_contour);

  // Remove contours based on size
  //  image_calib = cv::Mat::zeros(image.size(), CV_8UC1);
  //  for (const auto &contour : contours) {
  //    double area = cv::contourArea(contour);
  //    // Draw a black circle on new image
  //    if (area < 200 && area > 100) {
  //      cv::Moments mu = cv::moments(contour);
  //      cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
  //      cv::circle(image_calib, mc, 3, cv::Scalar(255, 255, 255), 5);
  //    }
  //  }
  //  cv::threshold(image_calib, image_calib, 200, 255, 1);

  // Testing
  cv::SimpleBlobDetector::Params params;
  params.blobColor = 0;
  params.minArea = 100;
  params.maxArea = 200;
  std::vector<cv::Point2f> centers;
  cv::Size pattern_size = cv::Size(4, 3);
  bool found = cv::findCirclesGrid(image_calib, pattern_size, centers,
                                   cv::CALIB_CB_ASYMMETRIC_GRID,
                                   new cv::SimpleBlobDetector(params));
  if (!centers.empty()) {
    ROS_INFO("%d corners", (int)centers.size());
  }
  cv::Mat image_calib_color;
  cv::cvtColor(image_calib, image_calib_color, CV_GRAY2BGR);
  cv::drawChessboardCorners(image_calib_color, pattern_size, cv::Mat(centers),
                            found);

  // Visualization
  cv::imshow("calib_result", image_calib_color);
  cv::waitKey(1);
}

void CalibProc::InitializeConfig(const ros::NodeHandle &nh,
                                 CalibProcDynConfig *config) {
  // Not implemented
}

void CalibProc::ReconfigureCallback(const CalibProcDynConfig &config,
                                    int level) {
  config_ = config;
}

}  // namespace flir_gige
