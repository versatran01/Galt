#include "stereo_vo/stereo_vo_node.h"

#include <sensor_msgs/PointCloud.h>
#include <image_geometry/stereo_camera_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <cstdint>

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

  points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("triangulated_points",1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose",1);
  
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

void StereoVoNode::ReconfigureCallback(const StereoVoDynConfig& config,
                                       int level) {
  stereo_vo_.UpdateConfig(config);
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
  CameraInfo linfo = *l_cinfo_msg;
  CameraInfo rinfo = *r_cinfo_msg;
  linfo.header.frame_id = "0";
  rinfo.header.frame_id = "0";
  model.fromCameraInfo(linfo, rinfo);

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
  
  //  current pose: world to camera
  auto pose = stereo_vo_.GetCurrentPose();
  
  //  publish point cloud for visualization
  const std::vector<Feature>& features = stereo_vo_.GetCurrentFeatures();
  
  sensor_msgs::PointCloud cloud;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "rgb";
  
  union {
    uint8_t rgb[4];
    float val;
  } color;
  
  for (const Feature& feat : features) {
    if (feat.triangulated) {
      geometry_msgs::Point32 p32;
      kr::vec3<scalar_t> p(feat.point.x,feat.point.y,feat.point.z);
      
      //  convert to world coordinates
      //p = pose.q.conjugate().matrix() * p + pose.p;
      
      p32.x = p[0];
      p32.y = p[1];
      p32.z = p[2];
      
      cloud.points.push_back( p32 );
      
      color.rgb[0] = 255;
      color.rgb[1] = 255;
      color.rgb[2] = 0;
      color.rgb[3] = 0;
      
      channel.values.push_back(color.val);
    }
  }
  
  cloud.channels.push_back(channel);
  cloud.header.stamp = l_image_msg->header.stamp;
  cloud.header.frame_id = "0";
  points_pub_.publish(cloud);
  
  //  publish current pose
  geometry_msgs::PoseStamped geoPose;
  geoPose.header.stamp = cloud.header.stamp;
  geoPose.header.frame_id = "0";
  tf::quaternionEigenToMsg(pose.q.cast<double>(),geoPose.pose.orientation);
  tf::pointEigenToMsg(pose.p.cast<double>(),geoPose.pose.position);
  pose_pub_.publish(geoPose);
}

const StereoVoDynConfig ReadConfig(const ros::NodeHandle& nh) {
  StereoVoDynConfig config;
  nh.param<int>("num_features", config.num_features, 100);
  nh.param<int>("min_features", config.min_features, 50);
  nh.param<int>("max_level", config.max_level, 50);
  nh.param<int>("win_size", config.win_size, 50);
  return config;
}

}  // namespace stereo_vo

}  // namespace galt
