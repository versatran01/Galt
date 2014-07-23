#ifndef FLIR_GIGE_CALIBRATOR_H_
#define FLIR_GIGE_CALIBRATOR_H_

#include <vector>
#include <utility>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

#include "opencv2/core/core.hpp"

#include "flir_gige/CalibConfig.h"

namespace flir_gige {

class Calibrator {
 public:
  using Point2 = cv::Point2d;
  using Point3 = cv::Point3d;

  Calibrator(const ros::NodeHandle &nh);

  void ImageCallback(const sensor_msgs::ImageConstPtr &image);
  void ReconfigureCallback(flir_gige::CalibConfig &config, int level);

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  dynamic_reconfigure::Server<flir_gige::CalibConfig> server_;

  int roi_width_;   // width of roi
  int roi_height_;  // height of roi
  double depth_;    // depth from camera to beacons
  double dist_;     // distance between two beacons
  int thresh_;      // pixel threshold for blob detection
  double sigma_;    // gaussian sigma
  double min_area_;
  double max_area_;
  double max_width_height_distortion_;
  double max_circular_distortion_;

  bool init_roi_{false};

  cv::Rect CalculateRoi(int width, int height);
  void GetImage(const sensor_msgs::ImageConstPtr &image, cv::Mat &image_gray,
                cv::Mat &image_color);
  void DrawRoi(cv::Mat &image_color, const cv::Rect &roi);
  double CalculateDistance(const Point2 &p1, const Point2 &p2);

};  // class Calibrator

}  // namespace flir_gige

#endif  // FLIR_GIGE_CALIBRATOR_CALIBRATOR_H_
