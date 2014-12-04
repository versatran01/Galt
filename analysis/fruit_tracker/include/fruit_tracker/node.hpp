#ifndef FRUIT_TRACKER_NODE_HPP
#define FRUIT_TRACKER_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>

namespace galt {
namespace fruit_tracker {

class Node {
public:
  Node(const ros::NodeHandle& pnh);
  virtual ~Node();
private:
  static constexpr int kROSQueueSize = 10;
  
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cam_info_;
  
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, sensor_msgs::CameraInfo> TimeSyncPolicy;
  
  typedef message_filters::Synchronizer<TimeSyncPolicy> Synchronizer;
  
  std::shared_ptr<Synchronizer> sync_image_;
  
  void imageCallback(const sensor_msgs::ImageConstPtr&,
                     const sensor_msgs::CameraInfoConstPtr&);
};

} //  namespace fruit_tracker
} //  namespace galt

#endif
