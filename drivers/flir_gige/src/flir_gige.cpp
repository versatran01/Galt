#include "flir_gige/flir_gige.h"

#include <memory>
#include <functional>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Temperature.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "flir_gige/gige_camera.h"
#include "flir_gige/FlirConfig.h"

namespace flir_gige {

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;
using camera_info_manager::CameraInfoManager;

FlirGige::FlirGige(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
  // Get ros parameteres
  nh_.param<std::string>("frame_id", frame_id_, std::string("flir_a5"));
  double fps;
  nh_.param<double>("fps", fps, 20.0);
  ROS_ASSERT_MSG(fps > 0, "FlirGige: fps must be greater than 0");
  rate_.reset(new ros::Rate(fps));

  // Create a camera
  std::string ip_address;
  nh_.param<std::string>("ip_address", ip_address, std::string(""));
  camera_.reset(new GigeCamera(ip_address));
  camera_->use_image = std::bind(&FlirGige::PublishImage, this,
                                 std::placeholders::_1, std::placeholders::_2);
  camera_->use_temperature =
      std::bind(&FlirGige::PublishTemperature, this, std::placeholders::_1);

  // Setup camera publisher and dynamic reconfigure callback
  std::string calib_url;
  nh_.param<std::string>("calib_url", calib_url, "");
  CameraInfoManager cinfo_manager(nh_, frame_id_, calib_url);
  if (!cinfo_manager.isCalibrated()) {
    ROS_WARN_STREAM("FlirGige: " << frame_id_ << " not calibrated");
  }
  cinfo_ = CameraInfoPtr(new CameraInfo(cinfo_manager.getCameraInfo()));

  camera_pub_ = it_.advertiseCamera("image_raw", 1);
  temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("spot", 1);
  server_.setCallback(
      boost::bind(&FlirGige::ReconfigureCallback, this, _1, _2));
}

void FlirGige::Run() {
  GigeConfig config;
  nh_.param<bool>("color", config.color, false);
  nh_.param<int>("width", config.width, 320);
  nh_.param<int>("height", config.height, 256);
  nh_.param<int>("bit", config.bit, 2);
  camera_->Connect();
  camera_->Configure(config);
  camera_->Start();
}

void FlirGige::End() {
  camera_->Stop();
  camera_->Disconnect();
}

void FlirGige::PublishImage(const cv::Mat &image,
                            const std::vector<double> &planck) {
  // Construct a cv image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
  std_msgs::Header header;
  cv_ptr->header.stamp = ros::Time::now();
  cv_ptr->header.frame_id = frame_id_;
  cv_ptr->image = image;
  cinfo_->header = header;
//  cinfo_->D = planck;
  // Since D maybe useful, we instead put planck constants into P
  // The orders are B F O R
  std::copy(planck.cbegin(), planck.cend(), cinfo_->P.begin());
  cv_ptr->encoding = GetImageEncoding(image);
  // Convert to ros image msg and publish camera
  image_ = cv_ptr->toImageMsg();
  camera_pub_.publish(image_, cinfo_);
  rate_->sleep();
}

void FlirGige::PublishTemperature(const std::pair<double, double> &spot) {
  // Construct a temperature mesage
  sensor_msgs::Temperature temperature;
  temperature.header.stamp = ros::Time::now();
  temperature.header.frame_id = frame_id_;
  temperature.temperature = spot.first;
  temperature.variance = spot.second;
  temp_pub_.publish(temperature);
}

std::string FlirGige::GetImageEncoding(const cv::Mat &image) {
  std::string encoding;
  switch (image.type()) {
    case CV_8UC1:
      encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case CV_8UC3:
      encoding = sensor_msgs::image_encodings::BGR8;
      break;
    case CV_16UC1:
      encoding = sensor_msgs::image_encodings::MONO16;
      break;
    default:
      encoding = sensor_msgs::image_encodings::MONO8;
  }
  return encoding;
}

void FlirGige::ReconfigureCallback(FlirConfig &config, int level) {
  // Do nothing when first starting
  if (level < 0) {
    return;
  }
  // Get config
  GigeConfig gige_config;
  gige_config.color = config.color;
  gige_config.width = config.width;
  gige_config.height = config.height;
  gige_config.bit = config.bit;
  // Stop the camera if in acquisition
  if (camera_->IsAcquire()) {
    // Stop the image thread if camera is running
    camera_->Stop();
    camera_->Disconnect();
  }
  // Reconnect to the camera
  camera_->Connect();
  // Reconfigure the camera
  camera_->Configure(gige_config);
  // Restart the camera
  camera_->Start();
}

}  // namespace flir_gige