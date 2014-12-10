#include <fruit_tracker/node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fruit_tracker/fruit_tracker.hpp>

namespace galt {
namespace fruit_tracker {

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh_) {
  
  //  subscribe to image & camera info
  sub_image_.subscribe(it_, "image", kROSQueueSize);
  sub_cam_info_.subscribe(pnh_, "camera_info", kROSQueueSize);
  
  sync_image_ = std::make_shared<Synchronizer>(TimeSyncPolicy(kROSQueueSize),
                                               sub_image_,sub_cam_info_);
  sync_image_->registerCallback(
        boost::bind(&Node::imageCallback, this, _1, _2));
}

Node::~Node() {}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
  
  cv_bridge::CvImageConstPtr bridgedImg = 
      cv_bridge::toCvShare(image_msg, "bgr8");
  if (!bridgedImg || bridgedImg->image.empty()) {
    ROS_ERROR("Failed to convert image to bgr8");
    return;
  }
  
  //  input image
  cv::Mat input = bridgedImg->image;
  
  std::vector<Fruit> fruits;
  Fruit::detect(input,fruits);  /// @todo: do stuff with the fruit...
  
  static ros::Publisher pub = pnh_.advertise<sensor_msgs::Image>("something",1);
  
  sensor_msgs::Image outputImage;
  outputImage.header.frame_id = "who_cares";
  outputImage.header.stamp = image_msg->header.stamp;
  outputImage.width = input.cols;
  outputImage.height = input.rows;
  outputImage.step = input.cols*3;
  outputImage.encoding = "bgr8";
  const size_t size = input.rows*input.cols*3;
  outputImage.data.resize(size);
  memcpy(&outputImage.data[0], input.ptr(), size);
  pub.publish(outputImage);
}

} //  fruit_tracker
} //  galt