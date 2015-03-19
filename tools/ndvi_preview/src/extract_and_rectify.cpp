#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_and_rectify");
  ros::NodeHandle pnh("~");

  std::string in_bag_path;
  if (!pnh.getParam("in_bag", in_bag_path)) {
    std::cout << in_bag_path << std::endl;
    ROS_ERROR("No input bag file specified.");
    return -1;
  }

  std::string out_bag_path;
  pnh.param<std::string>("out_bag", out_bag_path, "/tmp/test.bag");

  std::string red_camera_topic, nir_camera_topic;
  pnh.param<std::string>("red", red_camera_topic, "/spectral_670");
  pnh.param<std::string>("red", nir_camera_topic, "/spectral_800");

  std::vector<std::string> ndvi_camera_topics = {
      red_camera_topic + "/image_raw", red_camera_topic + "/camera_info",
      nir_camera_topic + "/image_raw", nir_camera_topic + "/camera_info"};

  for (const std::string& topic : ndvi_camera_topics) {
    ROS_INFO("Topic: %s", topic.c_str());
  }

  // Open rosbag
  rosbag::Bag in_bag(in_bag_path);
  rosbag::View view(in_bag, rosbag::TopicQuery(ndvi_camera_topics));
  ROS_INFO("Opened bag: %s", in_bag.getFileName().c_str());

  bool stop = false;
  for (auto it = view.begin(), it_end = view.end(); it != it_end; ++it) {
    const rosbag::MessageInstance& m = *it;
    sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      const auto image = cv_bridge::toCvShare(image_msg)->image;
    }
  }
}
