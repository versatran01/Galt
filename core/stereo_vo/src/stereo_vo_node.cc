#include "stereo_vo/stereo_vo_node.h"

#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {

namespace stereo_vo {

StereoVoNode::StereoVoNode(const ros::NodeHandle& nh) : nh_{nh}, it_{nh} {
  // Queue size 1 should be OK;
  // the one that matters is the synchronizer queue size.
  image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
  SubscribeStereoTopics("image_rect", "camera_info", hints);

  // Setup synchronize policy
  int queue_size;
  nh_.param<int>("queue_size", queue_size, 5);
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

  // Read and update StereoVoConfig
  stereo_vo_.UpdateConfig(ReadConfig(nh_));
  cfg_server_.setCallback(
      boost::bind(&StereoVoNode::ReconfigureCallback, this, _1, _2));
}

void StereoVoNode::SubscribeStereoTopics(
    const std::string& image_topic, const std::string& cinfo_topic,
    const image_transport::TransportHints& hints) {
  using namespace ros::names;
  std::string left("left");
  std::string right("right");
  l_image_sub_.subscribe(it_, resolve(append(left, image_topic)), 1, hints);
  l_cinfo_sub_.subscribe(nh_, resolve(append(left, cinfo_topic)), 1);
  r_image_sub_.subscribe(it_, resolve(append(right, image_topic)), 1, hints);
  r_cinfo_sub_.subscribe(nh_, resolve(append(right, cinfo_topic)), 1);
}

void StereoVoNode::ReconfigureCallback(const StereoVoDynConfig &config, int level) {
  StereoVoConfig new_config;
  new_config.num_features = config.num_features;
  new_config.min_features = config.min_features;
  stereo_vo_.UpdateConfig(new_config);
}

void StereoVoNode::StereoCallback(const ImageConstPtr& l_image_msg,
                                  const CameraInfoConstPtr& l_cinfo_msg,
                                  const ImageConstPtr& r_image_msg,
                                  const CameraInfoConstPtr& r_cinfo_msg) {
  // Get stereo camera infos
  static image_geometry::StereoCameraModel model;
  if (!model.initialized() && (!l_cinfo_msg->K[0] || !r_cinfo_msg->K[0])) {
    ROS_WARN_THROTTLE(1, "Uncalibrated camera.");
    return;
  }
  model.fromCameraInfo(l_cinfo_msg, r_cinfo_msg);

  // Get stereo images
  cv::Mat l_image_rect =
      cv_bridge::toCvCopy(l_image_msg, image_encodings::MONO8)->image;
  cv::Mat r_image_rect =
      cv_bridge::toCvCopy(r_image_msg, image_encodings::MONO8)->image;

  // Initialize stereo visual odometry if not
  if (!stereo_vo_.init()) {
    // 1)   Do only once:
    // 1.1) Caputre one frame of stereo images
    // 1.2) Extract and match features between them
    // 1.3) Triangulate features from two stereo images
    stereo_vo_.Initialize(l_image_rect, r_image_rect, model);
    return;
  }

  stereo_vo_.Iterate(l_image_rect, r_image_rect);
}

const StereoVoConfig ReadConfig(const ros::NodeHandle &nh)  {
  StereoVoConfig config;
  nh.param<int>("num_features", config.num_features, 100);
  nh.param<int>("min_features", config.min_features, 50);
  return config;
}

}  // namespace stereo_vo

}  // namespace galt
