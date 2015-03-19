#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
 public:
  void newMessage(const boost::shared_ptr<M const>& msg) {
    this->signalMessage(msg);
  }
};

class KeyframeExtractor {
 public:
  KeyframeExtractor(const ros::NodeHandle& pnh) : pnh_(pnh) {
    pnh_.param("scale", scale_, 0.5);
  }

  void ImageCb(const sensor_msgs::Image::ConstPtr& image_msg) {
    const auto image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat image_scaled;
    cv::resize(image, image_scaled, cv::Size(0, 0), 0.5, 0.5);
    cv::imshow("image", image_scaled);
    cv::waitKey(1);
  }

 private:
  ros::NodeHandle pnh_;
  double scale_;
};

//  _                _
// | |__   __ _  ___| | __
// | '_ \ / _` |/ __| |/ /
// | | | | (_| | (__|   <
// |_| |_|\__,_|\___|_|\_\
//

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_and_rectify");
  ros::NodeHandle pnh("~");

  std::string in_bag_path;
  if (!pnh.getParam("in_bag", in_bag_path)) {
    std::cout << in_bag_path << std::endl;
    ROS_ERROR("No input bag file specified.");
    return -1;
  }

  int queue_size;
  pnh.param("queue_size", queue_size, 20);

  std::string out_bag_path;
  pnh.param<std::string>("out_bag", out_bag_path, "/tmp/test.bag");

  std::string camera_ns;
  pnh.param<std::string>("camera", camera_ns, "/spectral_670");
  const std::string image_topic = camera_ns + "/image_rect";

  // Open rosbag
  rosbag::Bag in_bag(in_bag_path);
  rosbag::View view(in_bag, rosbag::TopicQuery(image_topic));

  ROS_INFO("Opened bag: %s", in_bag.getFileName().c_str());

  KeyframeExtractor extractor(pnh);

  for (auto it = view.begin(), it_end = view.end(); it != it_end; ++it) {
    const rosbag::MessageInstance& m = *it;

    // Image topic
    if (m.getTopic() == image_topic || ("/" + m.getTopic() == image_topic)) {
      sensor_msgs::ImageConstPtr image_msg =
          m.instantiate<sensor_msgs::Image>();
      if (image_msg) extractor.ImageCb(image_msg);
    }
  }
}
