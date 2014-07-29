#include "flir_gige/thermal_proc/thermal_proc.h"

#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <iterator>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

namespace flir_gige {

ThermalProc::ThermalProc(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
#ifdef NO_TIMESTAMP
  image_sub_ = it_.subscribe("image_raw", 1, &ThermalProc::ImageCallback, this);
#else
  camera_sub_ =
      it_.subscribeCamera("image_raw", 1, &ThermalProc::CameraCallback, this);

#endif
  heat_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("temperature", 1);
  heat_map_ = std_msgs::Float32MultiArrayPtr(new std_msgs::Float32MultiArray);
  heat_map_->layout.dim.push_back(std_msgs::MultiArrayDimension());
  heat_map_->layout.dim.push_back(std_msgs::MultiArrayDimension());

  color_pub_ = nh_.advertise<sensor_msgs::Image>("color_map", 1);
  color_map_ = sensor_msgs::ImagePtr(new sensor_msgs::Image);
}

#ifdef NO_TIMESTAMP
void ThermalProc::ImageCallback(const sensor_msgs::ImageConstPtr &image) {
  double B = 1428.0;
  double F = 1.0;
  double O = 118.126;
  double R = 377312.0;
//  double f = 329.3387;
//  int cx = 160, cy = 128;
#else
void ThermalProc::CameraCallback(const sensor_msgs::ImageConstPtr &image,
                                 const sensor_msgs::CameraInfoConstPtr &cinfo) {
  double B = cinfo->P[0];
  double F = cinfo->P[1];
  double O = cinfo->P[2];
  double R = cinfo->P[3];
#endif
  // Get image using cv_bridge
  cv_bridge::CvImagePtr raw_ptr;
  try {
    raw_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO16);
  }
  catch (const cv_bridge::Exception &e) {
    ROS_ERROR_STREAM("cv_bridge: " << e.what());
    return;
  }

  // Convert each pixle form 16-bit raw value to temperature
  const auto num_data = raw_ptr->image.total();
  const auto height = raw_ptr->image.rows;
  const auto width = raw_ptr->image.cols;
  if (heat_map_->data.size() != num_data) {
    heat_map_->data.resize(num_data);
  }
  cv::Mat heat(raw_ptr->image.size(), CV_32FC1);
  for (int i = 0; i < height; ++i) {
    auto *pheat = heat.ptr<float>(i);
    auto *praw = raw_ptr->image.ptr<uint16_t>(i);
    for (int j = 0; j < width; ++j) {
      pheat[j] = B / std::log(R / (praw[j] - O) + F) - 273.15;
    }
  }
  float *data = reinterpret_cast<float *>(heat.data);
  std::copy(data, data + num_data, heat_map_->data.begin());

  // Set dimension and publish
  heat_map_->layout.dim[0].size = height;
  heat_map_->layout.dim[0].label = "height";
  heat_map_->layout.dim[0].stride = height * width;
  heat_map_->layout.dim[1].size = width;
  heat_map_->layout.dim[1].label = "width";
  heat_map_->layout.dim[1].stride = width;
  heat_pub_.publish(heat_map_);

  // Convert to heat map if there's any subscriber
  if (color_pub_.getNumSubscribers() > 0) {
    cv::Mat image_jet;
  }

  // Display image for now
  cv::imshow("raw", raw_ptr->image);
  cv::waitKey(3);
}

}  // namespace flir_gige
