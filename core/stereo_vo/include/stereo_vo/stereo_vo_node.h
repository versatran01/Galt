#ifndef GALT_STEREO_VO_NODE_H_
#define GALT_STEREO_VO_NODE_H_

#include <memory>
#include <cstdint>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

namespace galt {

namespace stereo_vo {

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class StereoVoNode {
 public:
  StereoVoNode(const ros::NodeHandle& nh);

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<CameraInfo> l_cinfo_sub_, r_cinfo_sub_;
  using ExactPolicy = ExactTime<Image, CameraInfo, Image, CameraInfo>;
  using ApproximatePolicy =
      ApproximateTime<Image, CameraInfo, Image, CameraInfo>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::unique_ptr<ExactSync> exact_sync_;
  std::unique_ptr<ApproximateSync> approximate_sync_;

  void SubscribeStereoTopics(const std::string& image_topic,
                             const image_transport::TransportHints& hints);
  void StereoCallback(const ImageConstPtr& l_image_msg,
                      const CameraInfoConstPtr& l_cinfo_msg,
                      const ImageConstPtr& r_image_msg,
                      const CameraInfoConstPtr& r_cinfo_msg);
};  // class StereoVoNode

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_NODE_H_
