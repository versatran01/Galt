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
using sensor_msgs::CameraInfoPtr;

typedef boost::shared_ptr<CameraInfoManager> CameraInfoManagerPtr;
typedef boost::shared_ptr<bluefox2::Camera> CameraPtr;

namespace bluefox2 {

class CameraRos {
 public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  CameraRos(const ros::NodeHandle &nh);

  /**
   * @brief Destructor
   */
  ~CameraRos();

  /**
   * @brief Initialize camera
   */
  void init();

  /**
   * @brief Continuously publish image
   */
  void publish();

 private:
  ros::NodeHandle nh_;
  unsigned int seq_;
  std::string node_;
  std::string calibration_url_;
  std::string frame_id_;
  CameraPtr camera_;
  CameraInfoPtr camera_info_;
  CameraPublisher camera_pub_;
  CameraInfoManagerPtr camera_info_manager_;

  mv_params_s readParams();

};  // class CameraRos

}  // namespace bluefox2

#endif  // BLUEFOX2_CAMERA_ROS_H_
