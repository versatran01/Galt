#include <spectral_meter/node.hpp>

namespace galt {
namespace spectral_meter {

void Node::configureTopics() {
  sub_image_ = it_.subscribe("image", 1, &Node::imageCallback, this);
}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& img) {
  
  ROS_INFO("received image!");
  
}

} //  spectral_meter
} //  galt
