#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace led {

class Node {
 public:
  explicit Node(const ros::NodeHandle& pnh);
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

 private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int max_threshold_;
  int min_threshold_;
};

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  image_sub_ = it_.subscribe("image", 1, &Node::imageCb, this);
  image_pub_ = it_.advertise("image_highlighted", 1);

  // get param for threshold
  pnh_.param("max_threshold", max_threshold_, 250);
  pnh_.param("min_threshold", min_threshold_, 5);
}

void Node::imageCb(const sensor_msgs::ImageConstPtr& image_msg) {
  const auto image = cv_bridge::toCvCopy(
                         image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // convert to rgb
  cv::Mat image_rgb;
  cv::cvtColor(image, image_rgb, CV_GRAY2BGR);

  // create max_mask and apply threshold
  cv::Mat mask_max;
  cv::threshold(image, mask_max, max_threshold_, 255, 0);
  image_rgb.setTo(cv::Scalar(51, 51, 255), mask_max);

  cv::Mat mask_min;
  // create min_mask and appply threshold
  cv::threshold(image, mask_min, min_threshold_, 255, 1);
  image_rgb.setTo(cv::Scalar(255, 51, 51), mask_min);

  // publish ros message
  sensor_msgs::ImagePtr image_out =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rgb).toImageMsg();
  image_pub_.publish(image_out);
}

}  // namespace led

int main(int argc, char** argv) {
  ros::init(argc, argv, "led_node");
  ros::NodeHandle pnh("~");
  led::Node node(pnh);
  ros::spin();
  return 0;
}
