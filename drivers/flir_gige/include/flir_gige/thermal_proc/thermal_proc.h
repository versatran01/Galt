#ifndef FLIR_GIGE_THERMAL_PROC_H_
#define FLIR_GIGE_THERMAL_PROC_H_

// In case flir_gige/camera_info has no timestamp
#define NO_TIMESTAMP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace flir_gige {

class ThermalProc {
 public:
  ThermalProc(const ros::NodeHandle &nh);

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Publisher heat_pub_;
  std_msgs::Float32MultiArrayPtr heat_map_;
  ros::Publisher color_pub_;
  sensor_msgs::ImagePtr color_map_;

#ifdef NO_TIMESTAMP
  image_transport::Subscriber image_sub_;
  void ImageCallback(const sensor_msgs::ImageConstPtr &image);
#else
  image_transport::CameraSubscriber camera_sub_;
  void CameraCallback(const sensor_msgs::ImageConstPtr &image,
                      const sensor_msgs::CameraInfoConstPtr &cinfo);
#endif

};  // class ThermalProc

}  // namespace flir_gige

#endif  // FLIR_GIGE_THERMAL_PROC_H_
