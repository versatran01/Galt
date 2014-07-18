#include "flir_gige/flir_gige.h"

#include <memory>
#include <functional>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "flir_gige/gige_camera.h"
#include "flir_gige/FlirConfig.h"

namespace flir_gige {

FlirGige::FlirGige(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
  // Get ros parameteres
  nh_.param<std::string>("frame_id", frame_id_, std::string("flir"));
  double fps;
  nh_.param<double>("fps", fps, 20.0);
  ROS_ASSERT_MSG(fps > 0, "FlirGige: fps must be greater than 0");
  rate_.reset(new ros::Rate(fps));
  // Create a camera
  std::string ip_address;
  nh_.param<std::string>("ip_address", ip_address, std::string(""));
  camera_.reset(new flir_gige::GigeCamera(ip_address));
  camera_->use_image =
      std::bind(&FlirGige::PublishImage, this, std::placeholders::_1);
  // Setup image publisher and dynamic reconfigure callback
  image_pub_ = it_.advertise("image_raw", 1);
  server_.setCallback(
      boost::bind(&FlirGige::ReconfigureCallback, this, _1, _2));
}

void FlirGige::Run() {
  bool color;
  nh_.param<bool>("color", color, false);
  camera_->Connect();
  camera_->Configure(color);
  camera_->Start();
}

void FlirGige::PublishImage(const cv::Mat &image) {
//  if (!ros::ok()) {
//    camera_->Stop();
//    camera_->Disconnect();
//    ROS_WARN("I am here!");
    // Not sure if this shutdown is needed, seems to work fine without it
//    ros::shutdown();
//  }
  // Construct a cv image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
  cv_ptr->header.stamp = ros::Time::now();
  cv_ptr->header.frame_id = frame_id_;
  cv_ptr->header.seq = seq++;
  cv_ptr->image = image;
  if (image.channels() == 1) {
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
  } else if (image.channels() == 3) {
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  }
  // Convert to ros image msg and publish
  image_msg_ = cv_ptr->toImageMsg();
  image_pub_.publish(image_msg_);
  rate_->sleep();
}

void FlirGige::ReconfigureCallback(flir_gige::FlirConfig &config, int level) {
  // Do nothing when first starting
  if (level < 0) {
    return;
  }

  // Stop the camera if in acquisition
  if (camera_->IsAcquire()) {
    // Stop the image thread if camera is running
    camera_->Stop();
    camera_->Disconnect();
  }
  // Reconnect to the camera
  camera_->Connect();
  // Reconfigure the camera
  camera_->Configure(config.color);
  // Restart the camera
  camera_->Start();
}

}  // namespace flir_gige
