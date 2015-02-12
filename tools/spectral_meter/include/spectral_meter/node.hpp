#ifndef GALT_SPECTRAL_METER_NODE_HPP
#define GALT_SPECTRAL_METER_NODE_HPP

#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>

#include <spectral_meter/SpectralMeterDynConfig.h>
#include "spectral_meter/tracker.h"

namespace galt {
namespace spectral_meter {

class Node {
 public:
  using Config = ::spectral_meter::SpectralMeterDynConfig;

  Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  /// Register all ROS callbacks.
  void configure();

  /// Display image and simple interface.
  void imageCallback(const sensor_msgs::ImageConstPtr& img);

  /// Dynamic reconfigure callback
  void configCallback(Config& config, int level);

 private:
  /// Call the ros service.
  void callDynamicReconfigure(int expose_us);

  /// Calculate update to exposure value
  void updateExposure(double measured_mean);

  /// Handle user mouse clicks
  void mouseCallback(int event, int x, int y, int flags, void*);

  /// Handle user mouse clicks (static)
  static void mouseCallbackStatic(int, int, int, int, void*);

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  dynamic_reconfigure::Server<Config> cfg_server_;
  Config config_;
  Tracker tracker_;

  double ui_scale_;
  double measured_reflectance_{0};

  cv::Point click_position_;
  bool position_updated_{false};

  std::string camera_topic_name_;
  std::string expose_rosparam_name_;
  int expose_us_;
  int num_skip_frames_{0};
};

/// ================
/// Helper functions
/// ================
std::pair<std::string, bool> generatePercentageString(double num, double den);
void drawPercentage(cv::Mat& image, const cv::Point& point, double num,
                    double den);
void drawExposeUs(cv::Mat& image, const cv::Point& point, int expose_us);
void drawSelection(cv::Mat& image, const cv::Point2f& point, double box_size);
cv::Rect createRectAround(const cv::Point2i& center, double size,
                          const cv::Size& im_size);

}  //  spectral_meter
}  //  galt

#endif  //   GALT_SPECTRAL_METER_NODE_HPP
