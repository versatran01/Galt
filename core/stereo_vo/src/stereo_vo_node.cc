#include "stereo_vo/stereo_vo_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {

namespace stereo_vo {

StereoVoNode::StereoVoNode(const ros::NodeHandle& nh) : nh_{nh}, it_{nh} {
  // Queue size 1 should be OK;
  // the one that matters is the synchronizer queue size.
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  SubscribeStereoTopics("image_rect", hints);

  // Setup synchronize policy
  int queue_size;
  nh_.param<int>("queue_size", queue_size, 10);
  bool approx;
  nh_.param<bool>("approximate_sync", approx, false);
  if (approx) {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                l_image_sub_, l_cinfo_sub_,
                                                r_image_sub_, r_cinfo_sub_));
    approximate_sync_->registerCallback(
        boost::bind(&StereoVoNode::StereoCallback, this, _1, _2, _3, _4));

  } else {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), l_image_sub_,
                                    l_cinfo_sub_, r_image_sub_, r_cinfo_sub_));
    exact_sync_->registerCallback(
        boost::bind(&StereoVoNode::StereoCallback, this, _1, _2, _3, _4));
  }
}

void StereoVoNode::SubscribeStereoTopics(
    const std::string& image_topic,
    const image_transport::TransportHints& hints) {
  using namespace ros::names;
  l_image_sub_.subscribe(it_, resolve(append("left", image_topic)), 1, hints);
  l_cinfo_sub_.subscribe(nh_, resolve("left/camera_info"), 1);
  r_image_sub_.subscribe(it_, resolve(append("right", image_topic)), 1, hints);
  r_cinfo_sub_.subscribe(nh_, resolve("right/camera_info"), 1);
}

void StereoVoNode::StereoCallback(const ImageConstPtr& l_image_msg,
                                  const CameraInfoConstPtr& l_cinfo_msg,
                                  const ImageConstPtr& r_image_msg,
                                  const CameraInfoConstPtr& r_cinfo_msg) {
  cv::Mat l_image =
      cv_bridge::toCvCopy(l_image_msg, image_encodings::MONO8)->image;
  cv::Mat r_image =
      cv_bridge::toCvCopy(r_image_msg, image_encodings::MONO8)->image;

  cv::imshow("l_rect", l_image);
  cv::imshow("r_rect", r_image);
  cv::waitKey(1);
}

}  // namespace stereo_vo

}  // namespace galt
