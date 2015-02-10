#ifndef GALT_SPECTRAL_METER_NODE_HPP
#define GALT_SPECTRAL_METER_NODE_HPP

#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace galt {
namespace spectral_meter {

class Node {
public:
  
  Node(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh_) {}
  
  /// Register all ROS callbacks.
  void configureTopics();
  
  /// Display image and simple interface.
  void imageCallback(const sensor_msgs::ImageConstPtr& img);
  
private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
};

} //  spectral_meter
} //  galt

#endif  //   GALT_SPECTRAL_METER_NODE_HPP
