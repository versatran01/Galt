#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace led {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

 private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  image_sub_ = it_.subscribe("image", 1, &Node::imageCb, this);
}

void Node::imageCb(const sensor_msgs::ImageConstPtr& image_msg) {
  const auto image = cv_bridge::toCvCopy(image_msg)->image;
  cv::imshow("image", image);
  cv::waitKey(1);
}

}  // namespace led

int main(int argc, char** argv) {
  ros::init(argc, argv, "led_node");
  ros::NodeHandle pnh("~");
  led::Node node(pnh);
  ros::spin();
  return 0;
}
