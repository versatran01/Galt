/*
 * ndvi_preview_node.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 29/7/2014
 *      Author: gareth
 */

#include <iostream>
#include <string>
#include <algorithm>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define QUEUE_SIZE (10)

int save_number = 0;

void image_callback(const sensor_msgs::ImageConstPtr &irImage,
                    const sensor_msgs::CameraInfoConstPtr &irInfo,
                    const sensor_msgs::ImageConstPtr &redImage,
                    const sensor_msgs::CameraInfoConstPtr &redInfo) {

  cv_bridge::CvImageConstPtr bridgedIrPtr =
      cv_bridge::toCvShare(irImage, "mono8");
  cv_bridge::CvImageConstPtr bridgedRedPtr =
      cv_bridge::toCvShare(redImage, "mono8");

  if (!bridgedIrPtr || !bridgedRedPtr) {
    ROS_ERROR("Failed to convert image to mono8 with cv_bridge");
    return;
  }

  cv::Mat outputIr(bridgedIrPtr->image.size(), CV_8UC1);
  cv::Mat adjustedIr(bridgedIrPtr->image.size(), CV_8UC1, cv::Scalar(0, 0, 0));
  cv::Mat adjustedRed(bridgedIrPtr->image.size(), CV_8UC1, cv::Scalar(0, 0, 0));
  cv::Mat NDVI(bridgedIrPtr->image.size(), CV_8UC1, cv::Scalar(0, 0, 0));

  //  NOTE: This is all a harcoded mess for now, just for demo purposes...
  const int dispx = 8;
  const int dispy = 18;

  //  correct the IR image for circular error, then convert to reflectance
  //  and calculate an NDVI
  for (int y = 0; y < bridgedIrPtr->image.rows; y++) {
    for (int x = 0; x < bridgedIrPtr->image.cols; x++) {
      const double delx = x - 674;
      const double dely = y - 459;

      const double rad = std::sqrt(delx * delx + dely * dely);
      const double scl = -2e-6 * (rad * rad) + 0.0001716 * rad + 0.9508;

      const uchar irIn = bridgedIrPtr->image.at<uchar>(y, x);

      int red_x = x + dispx;
      int red_y = y + dispy;

      red_x = std::min(std::max(red_x, 0), bridgedRedPtr->image.cols);
      red_y = std::min(std::max(red_y, 0), bridgedRedPtr->image.rows);

      const uchar redIn = bridgedRedPtr->image.at<uchar>(red_y, red_x);

      uchar irOut;

      if (rad < 450) {
        irOut = static_cast<uchar>(std::min(255.0, irIn / scl));

        //  convert to reflectance
        double ir = irIn / scl;
        ir = ir / 255.0;
        ir = std::min(std::max(0.6759 * ir - 0.00971, 0.0), 1.0);

        double red = redIn / 255.0;
        red = std::min(std::max(1.296 * red - 0.07744, 0.0), 1.0);

        double ndvi = (ir - red) / (ir + red);
        ndvi = (ndvi + 1) / 2;
        // ndvi = std::min(std::max(ndvi,0.0),1.0);

        if (ndvi > 1.0 || ndvi < 0.0) {
          ROS_ERROR("bad calc!");
        }

        adjustedIr.at<uchar>(y, x) = static_cast<uchar>(ir * 255);
        adjustedRed.at<uchar>(y, x) = static_cast<uchar>(red * 255);
        NDVI.at<uchar>(y, x) = static_cast<uchar>(ndvi * 255);

      } else {
        irOut = 0;
      }

      outputIr.at<uchar>(y, x) = irOut;
    }
  }

  cv::imshow("IR", outputIr);
  cv::imshow("Red", bridgedRedPtr->image);
  cv::imshow("Calibrated IR", adjustedIr);
  cv::imshow("Calibrated Red", adjustedRed);
  cv::imshow("NDVI", NDVI);

  if (cv::waitKey(30) == static_cast<int>('s')) {
    //  save images
    cv::imwrite("output_ir_" + std::to_string(save_number) + ".png",
                bridgedIrPtr->image);
    cv::imwrite("output_red_" + std::to_string(save_number) + ".png",
                bridgedRedPtr->image);

    save_number++;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndvi_preview_node");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport imgTransport(nh);

  image_transport::SubscriberFilter subIRImage;
  image_transport::SubscriberFilter subRedImage;

  message_filters::Subscriber<sensor_msgs::CameraInfo> subIRInfo;
  message_filters::Subscriber<sensor_msgs::CameraInfo> subRedInfo;

  subIRImage.subscribe(imgTransport, "infrared/image_raw", QUEUE_SIZE);
  subRedImage.subscribe(imgTransport, "red/image_raw", QUEUE_SIZE);

  subIRInfo.subscribe(nh, "infrared/camera_info", QUEUE_SIZE);
  subRedInfo.subscribe(nh, "red/camera_info", QUEUE_SIZE);

  //  time sync policy
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> TimeSyncPolicy;

  message_filters::Synchronizer<TimeSyncPolicy> sync(TimeSyncPolicy(QUEUE_SIZE),
                                                     subIRImage, subIRInfo,
                                                     subRedImage, subRedInfo);
  sync.registerCallback(boost::bind(&image_callback, _1, _2, _3, _4));

  cv::namedWindow("IR", cv::WINDOW_NORMAL);
  cv::namedWindow("Red", cv::WINDOW_NORMAL);
  cv::namedWindow("Calibrated IR", cv::WINDOW_NORMAL);
  cv::namedWindow("Calibrated Red", cv::WINDOW_NORMAL);
  cv::namedWindow("NDVI", cv::WINDOW_NORMAL);

  ros::spin();
  return 0;
}
