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

#include "ndvi_preview/ndvi_preview_node.hpp"

#include <boost/thread/lock_guard.hpp>

#include <iostream>
#include <string>
#include <algorithm>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

namespace ndvi_preview {

NdviPreviewNode::NdviPreviewNode(const ros::NodeHandle &pnh)
    : pnh_(pnh), it_(pnh) {
  pnh_.param<int>("queue_size", queue_size_, 5);

  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                              sub_image_nir_, sub_cinfo_nir_,
                                              sub_image_red_, sub_cinfo_red_));
  approximate_sync_->registerCallback(
      boost::bind(&NdviPreviewNode::SyncedCameraCb, this, _1, _2, _3, _4));

  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&NdviPreviewNode::ConnectCb, this);

  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_image_ndvi_ = it_.advertise("ndvi", 1, connect_cb, connect_cb);
}

void NdviPreviewNode::SubscribeSyncedTopics() {
  SubscribeSingleCamera("nir", sub_image_nir_, sub_cinfo_nir_);
  SubscribeSingleCamera("red", sub_image_red_, sub_cinfo_red_);
}

void NdviPreviewNode::SubscribeSingleCamera(const std::string &camera,
                                            ImageSubscriberFilter &sub_image,
                                            CinfoSubscriberFilter &sub_cinfo) {
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  ros::NodeHandle cnh(pnh_, camera);
  sub_image.subscribe(it_, ros::names::append(camera, "image"), queue_size_,
                      hints);
  sub_cinfo.subscribe(cnh, "camera_info", queue_size_);
}

void NdviPreviewNode::ConnectCb() {
  /// @todo: something's wrong with this connect callback
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_image_ndvi_.getNumSubscribers() == 0) {
    sub_image_nir_.unsubscribe();
    sub_cinfo_nir_.unsubscribe();
    sub_image_red_.unsubscribe();
    sub_cinfo_red_.unsubscribe();
  } else if (!sub_image_nir_.getSubscriber()) {
    SubscribeSyncedTopics();
  }
}

void NdviPreviewNode::SyncedCameraCb(const ImageConstPtr &nir_image_msg,
                                     const CameraInfoConstPtr &nir_cinfo_msg,
                                     const ImageConstPtr &red_image_msg,
                                     const CameraInfoConstPtr &red_cinfo_msg) {
  ROS_WARN_THROTTLE(1, "IN!!");
  const cv::Mat nir = cv_bridge::toCvShare(nir_image_msg)->image;
  const cv::Mat red = cv_bridge::toCvShare(red_image_msg)->image;
  cv::Mat ndvi = ComputeNdvi(nir, red);
  cv::Mat ndvi_jet;
  cv::applyColorMap(ndvi, ndvi_jet, cv::COLORMAP_JET);

  cv::imshow("ndvi", ndvi_jet);
  cv::waitKey(1);
}

cv::Mat ComputeNdvi(const cv::Mat &nir, const cv::Mat &red) {
  cv::Mat nir_float, red_float;
  nir.convertTo(nir_float, CV_32FC1);
  red.convertTo(red_float, CV_32FC1);

  // Calculate ndvi
  cv::Mat num, den;
  num = nir_float - red_float;
  den = nir_float + red_float;

  cv::Mat ndvi_float, ndvi;
  cv::divide(num, den, ndvi_float, 255);

  // Convert back to unit8
  ndvi_float.convertTo(ndvi, CV_8UC1);
  return ndvi;
}

}  // namespace ndvi_preview

//#define QUEUE_SIZE (10)

// int save_number = 0;

// void image_callback(const sensor_msgs::ImageConstPtr &irImage,
//                    const sensor_msgs::CameraInfoConstPtr &irInfo,
//                    const sensor_msgs::ImageConstPtr &redImage,
//                    const sensor_msgs::CameraInfoConstPtr &redInfo) {
//  cv_bridge::CvImageConstPtr bridgedIrPtr =
//      cv_bridge::toCvShare(irImage, "mono8");
//  cv_bridge::CvImageConstPtr bridgedRedPtr =
//      cv_bridge::toCvShare(redImage, "mono8");

//  if (!bridgedIrPtr || !bridgedRedPtr) {
//    ROS_ERROR("Failed to convert image to mono8 with cv_bridge");
//    return;
//  }

//  cv::Mat outputIr(bridgedIrPtr->image.size(), CV_8UC1);
//  cv::Mat adjustedIr(bridgedIrPtr->image.size(), CV_8UC1, cv::Scalar(0, 0,
//  0));
//  cv::Mat adjustedRed(bridgedIrPtr->image.size(), CV_8UC1, cv::Scalar(0, 0,
//  0));
//  cv::Mat NDVI(bridgedIrPtr->image.size(), CV_8UC1, cv::Scalar(0, 0, 0));

//  //  NOTE: This is all a harcoded mess for now, just for demo purposes...
//  const int dispx = 8;
//  const int dispy = 18;

//  //  correct the IR image for circular error, then convert to reflectance
//  //  and calculate an NDVI
//  for (int y = 0; y < bridgedIrPtr->image.rows; y++) {
//    for (int x = 0; x < bridgedIrPtr->image.cols; x++) {
//      const double delx = x - 674;
//      const double dely = y - 459;

//      const double rad = std::sqrt(delx * delx + dely * dely);
//      const double scl = -2e-6 * (rad * rad) + 0.0001716 * rad + 0.9508;

//      const uchar irIn = bridgedIrPtr->image.at<uchar>(y, x);

//      int red_x = x + dispx;
//      int red_y = y + dispy;

//      red_x = std::min(std::max(red_x, 0), bridgedRedPtr->image.cols);
//      red_y = std::min(std::max(red_y, 0), bridgedRedPtr->image.rows);

//      const uchar redIn = bridgedRedPtr->image.at<uchar>(red_y, red_x);

//      uchar irOut;

//      if (rad < 450) {
//        irOut = static_cast<uchar>(std::min(255.0, irIn / scl));

//        //  convert to reflectance
//        double ir = irIn / scl;
//        ir = ir / 255.0;
//        ir = std::min(std::max(0.6759 * ir - 0.00971, 0.0), 1.0);

//        double red = redIn / 255.0;
//        red = std::min(std::max(1.296 * red - 0.07744, 0.0), 1.0);

//        double ndvi = (ir - red) / (ir + red);
//        ndvi = (ndvi + 1) / 2;
//        // ndvi = std::min(std::max(ndvi,0.0),1.0);

//        if (ndvi > 1.0 || ndvi < 0.0) {
//          ROS_ERROR("bad calc!");
//        }

//        adjustedIr.at<uchar>(y, x) = static_cast<uchar>(ir * 255);
//        adjustedRed.at<uchar>(y, x) = static_cast<uchar>(red * 255);
//        NDVI.at<uchar>(y, x) = static_cast<uchar>(ndvi * 255);

//      } else {
//        irOut = 0;
//      }

//      outputIr.at<uchar>(y, x) = irOut;
//    }
//  }

//  cv::imshow("IR", outputIr);
//  cv::imshow("Red", bridgedRedPtr->image);
//  cv::imshow("Calibrated IR", adjustedIr);
//  cv::imshow("Calibrated Red", adjustedRed);
//  cv::imshow("NDVI", NDVI);

//  if (cv::waitKey(30) == static_cast<int>('s')) {
//    //  save images
//    cv::imwrite("output_ir_" + std::to_string(save_number) + ".png",
//                bridgedIrPtr->image);
//    cv::imwrite("output_red_" + std::to_string(save_number) + ".png",
//                bridgedRedPtr->image);

//    save_number++;
//  }
//}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndvi_preview_node");
  ros::NodeHandle pnh("~");

  try {
    ndvi_preview::NdviPreviewNode ndvi_preview_node(pnh);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
