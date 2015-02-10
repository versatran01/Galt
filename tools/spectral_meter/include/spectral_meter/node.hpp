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
  
  Node(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh_) {}
  
  /// Register all ROS callbacks.
  void configure();
  
  /// Display image and simple interface.
  void imageCallback(const sensor_msgs::ImageConstPtr& img);
  
private:
  
  /// Handle user mouse clicks
  void mouseCallback(int event, int x, int y, 
                     int flags, void*);
  
  /// Handle user mouse clicks (static)
  static void mouseCallbackStatic(int, int, int, int, void*);
  
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  double ui_scale_;
  
  cv::Point2i click_position_;
};

} //  spectral_meter
} //  galt

#endif  //   GALT_SPECTRAL_METER_NODE_HPP
