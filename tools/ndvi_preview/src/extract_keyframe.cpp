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

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
 public:
  void newMessage(const boost::shared_ptr<M const>& msg) {
    this->signalMessage(msg);
  }
};

void CameraCb(const sensor_msgs::Image::ConstPtr& image_msg,
              const sensor_msgs::CameraInfo::ConstPtr& cinfo_msg) {
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::imshow("image", image);
  cv::waitKey(1);
}

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
  const std::string cinfo_topic = camera_ns + "/camera_info";
  const std::vector<std::string> camera_topics = {image_topic, cinfo_topic};

  // Open rosbag
  rosbag::Bag in_bag(in_bag_path);
  rosbag::View view(in_bag, rosbag::TopicQuery(camera_topics));

  ROS_INFO("Opened bag: %s", in_bag.getFileName().c_str());

  BagSubscriber<sensor_msgs::Image> sub_image;
  BagSubscriber<sensor_msgs::CameraInfo> sub_cinfo;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>
      sync(sub_image, sub_cinfo, 25);
  sync.registerCallback(boost::bind(&CameraCb, _1, _2));

  for (auto it = view.begin(), it_end = view.end(); it != it_end; ++it) {
    const rosbag::MessageInstance& m = *it;

    // Image topic
    if (m.getTopic() == image_topic || ("/" + m.getTopic() == image_topic)) {
      sensor_msgs::ImageConstPtr image_msg =
          m.instantiate<sensor_msgs::Image>();
      if (image_msg) sub_image.newMessage(image_msg);
    }

    // CameraInfo topic
    if (m.getTopic() == cinfo_topic || ("/" + m.getTopic() == cinfo_topic)) {
      sensor_msgs::CameraInfoConstPtr cinfo_msg =
          m.instantiate<sensor_msgs::CameraInfo>();
      if (cinfo_msg) sub_cinfo.newMessage(cinfo_msg);
    }
  }
}
