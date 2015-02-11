#ifndef GALT_SPECTRAL_METER_NODE_HPP
#define GALT_SPECTRAL_METER_NODE_HPP

#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

namespace galt {
namespace spectral_meter {

class Node {
 public:
  Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), it_(pnh_) {}

  /// Register all ROS callbacks.
  void configure();

  /// Display image and simple interface.
  void imageCallback(const sensor_msgs::ImageConstPtr& img);

 private:
  /// Call the ros service.
  void callDynamicReconfigure(int expose_us);

  /// Calculate update to exposure value
  void updateExposure(double measured_mean);

  /// Handle user mouse clicks
  void mouseCallback(int event, int x, int y, int flags, void*);

  /// Handle user mouse clicks (static)
  static void mouseCallbackStatic(int, int, int, int, void*);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  //  std::string camera_name_;
  double ui_scale_;
  int selection_size_;
  double target_reflectance_;
  double measured_reflectance_{0};
  double Kp_;
  int skip_frame_param_;

  cv::Point2i click_position_;
  bool position_updated_{false};

  std::string camera_topic_name_;
  std::string expose_rosparam_name_;
  int expose_us_;
  int skip_frames_{0};
};

std::pair<std::string, bool> generatePercentageString(double num, double den);
void drawPercentage(cv::Mat& image, const cv::Point& point, double num,
                    double den);
void drawExposeUs(cv::Mat& image, const cv::Point& point, int expose_us);

}  //  spectral_meter
}  //  galt

#endif  //   GALT_SPECTRAL_METER_NODE_HPP
