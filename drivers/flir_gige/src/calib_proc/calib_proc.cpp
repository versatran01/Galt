#include "flir_gige/calib_proc/calib_proc.h"

#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace flir_gige {

CalibProc::CalibProc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_{nh}, pnh_{pnh}, it_{nh} {
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  image_sub_ = it_.subscribe(ros::names::resolve("image_raw"), 1,
                             &CalibProc::ImageCallback, this, hints);
  image_pub_ = it_.advertise("image_calib", 1);
  cv::namedWindow("raw");
}

void CalibProc::ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
  cv::Mat image = cv_bridge::toCvCopy(
                      image_msg, sensor_msgs::image_encodings::MONO8)->image;
  cv::imshow("raw", image);
  cv::waitKey(1);
}

}  // namespace flir_gige
