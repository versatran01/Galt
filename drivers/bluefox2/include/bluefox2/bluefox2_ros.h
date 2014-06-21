#ifndef BLUEFOX2_CAMERA_ROS_H_
#define BLUEFOX2_CAMERA_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "bluefox2/bluefox2.h"

using camera_info_manager::CameraInfoManager;
using image_transport::CameraPublisher;
using namespace sensor_msgs;
typedef boost::shared_ptr<CameraInfoManager> CameraInfoManagerPtr;
typedef boost::shared_ptr<bluefox2::Camera> CameraPtr;

namespace bluefox2 {

class CameraRos {
 public:
  CameraRos(const ros::NodeHandle &nh);
  ~CameraRos();
  void init();
  void publish();

 private:
  ros::NodeHandle nh_;
  unsigned int seq_;
  string node_;
  string calibration_url_;
  string frame_id_;
  CameraInfoManagerPtr camera_info_manager_;
  CameraInfoPtr camera_info_;
  CameraPublisher camera_pub_;
  CameraPtr camera_;

  mv_params_t readParams();

};  // class CameraRos

}  // namespace bluefox2

#endif  // BLUEFOX2_CAMERA_ROS_H_
