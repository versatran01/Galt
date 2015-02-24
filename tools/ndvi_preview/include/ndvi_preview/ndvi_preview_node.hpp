#ifndef GALT_NDVI_PREVIEW_NODE_HPP_
#define GALT_NDVI_PREVIEW_NODE_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>

namespace ndvi_preview {

using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfoConstPtr;
using message_filters::sync_policies::ApproximateTime;
using ImageSubscriberFilter = image_transport::SubscriberFilter;
using CinfoSubscriberFilter =
    message_filters::Subscriber<sensor_msgs::CameraInfo>;
using ApproximatePolicy =
    ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                    sensor_msgs::Image, sensor_msgs::CameraInfo>;
using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;

class NdviPreviewNode {
 public:
  explicit NdviPreviewNode(const ros::NodeHandle& pnh);

  void ConnectCb();
  void SyncedCameraCb(const ImageConstPtr& nir_image_msg,
                      const CameraInfoConstPtr& nir_cinfo_msg,
                      const ImageConstPtr& red_image_msg,
                      const CameraInfoConstPtr& red_cinfo_msg);

 private:
  void SubscribeSyncedTopics();
  void SubscribeSingleCamera(const std::string& camera,
                             ImageSubscriberFilter& sub_image,
                             CinfoSubscriberFilter& sub_cinfo);

  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ImageSubscriberFilter sub_image_nir_, sub_image_red_;
  CinfoSubscriberFilter sub_cinfo_nir_, sub_cinfo_red_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  image_transport::Publisher pub_image_ndvi_;
  boost::mutex connect_mutex_;
  int queue_size_;
};

}  // namespace ndvi_preview

#endif  // GALT_NDVI_PREVIEW_NODE_HPP_
