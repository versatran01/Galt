/*
 * flir_gige.h
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

#ifndef FLIR_GIGE_FLIR_GIGE_H_
#define FLIR_GIGE_FLIR_GIGE_H_

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "flir_gige/planck.h"
#include "flir_gige/gige_camera.h"
#include "flir_gige/FlirDynConfig.h"

namespace flir_gige {

class FlirGige {
 public:
  using CameraInfoManagerPtr =
      std::shared_ptr<camera_info_manager::CameraInfoManager>;
  using DynConfig = ::flir_gige::FlirDynConfig;

  FlirGige(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  /**
   * @brief Run Start camera
   */
  void Run();
  /**
   * @brief End Stop camera
   */
  void End();

 private:
  void PublishImage(const cv::Mat &image, const Planck &planck);
  void PublishTemperature(const std::pair<double, double> &spot);
  std::string GetImageEncoding(const cv::Mat &image) const;
  void ConfigCb(DynConfig &config, int level);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string frame_id_;
  std::unique_ptr<ros::Rate> rate_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher pub_camera_;
  sensor_msgs::ImagePtr image_;
  sensor_msgs::CameraInfoPtr cinfo_;
  CameraInfoManagerPtr cinfo_manager_;
  ros::Publisher pub_temperature_;
  dynamic_reconfigure::Server<DynConfig> server_;
  std::unique_ptr<GigeCamera> gige_camera_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_FLIR_GIGE_H_
