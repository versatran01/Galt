#include "stereo_vo/stereo_vo_node.h"

#include <cstdint>

#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <image_geometry/stereo_camera_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen_conversions/eigen_msg.h>

namespace galt {

namespace stereo_vo {

StereoVoNode::StereoVoNode(const ros::NodeHandle& nh)
    : nh_{nh}, it_{nh}, stereo_vo_(ReadConfig(nh)) {
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

  // Setup dynamic reconfigure server
  cfg_server_.setCallback(
      boost::bind(&StereoVoNode::ReconfigureCallback, this, _1, _2));

  // Setup all publishers
  points_pub_ =
      nh_.advertise<sensor_msgs::PointCloud>("triangulated_points", 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  traj_pub_ = nh_.advertise<visualization_msgs::Marker>("traj", 1);
  // Initialize common fields of marker
  traj_.ns = "stereo_vo";
  traj_.id = 0;
  traj_.type = visualization_msgs::Marker::LINE_STRIP;
  traj_.action = visualization_msgs::Marker::ADD;
  traj_.color.r = 1.0;
  traj_.color.g = 0.1;
  traj_.color.b = 1.0;
  traj_.color.a = 1.0;
  traj_.scale.x = 0.1;
  traj_.lifetime = ros::Duration();
  traj_.pose.orientation.w = 1.0;
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
  static image_geometry::StereoCameraModel stereo_model;
  if (!stereo_model.initialized() &&
      (!l_cinfo_msg->K[0] || !r_cinfo_msg->K[0])) {
    ROS_WARN_THROTTLE(1, "Uncalibrated camera.");
    return;
  }
  CameraInfo linfo = *l_cinfo_msg;
  CameraInfo rinfo = *r_cinfo_msg;
  linfo.header.frame_id = "/stereo";
  rinfo.header.frame_id = "/stereo";
  stereo_model.fromCameraInfo(linfo, rinfo);

  // Get stereo images
  cv::Mat l_image_rect =
      cv_bridge::toCvCopy(l_image_msg, image_encodings::MONO8)->image;
  cv::Mat r_image_rect =
      cv_bridge::toCvCopy(r_image_msg, image_encodings::MONO8)->image;
  auto stereo_image = std::make_pair(l_image_rect, r_image_rect);

  // Initialize stereo visual odometry if not
  if (!stereo_vo_.init()) {
    stereo_vo_.Initialize(stereo_image, stereo_model);
    return;
  }

  stereo_vo_.Iterate(stereo_image);
  auto camera_pose = KrPoseToRosPose(stereo_vo_.pose_world());

  // Publish PointCloud from keyframe pose and features
//  PublishPointCloud(stereo_vo_.point3ds(), stereo_vo_.corners(),
//                    l_image_msg->header.stamp, "0");
  PublishPoseStamped(camera_pose, l_image_msg->header.stamp, "0");
  PublishTrajectory(camera_pose, l_image_msg->header.stamp, "0");
}

/*
void StereoVoNode::PublishPointCloud(
    const std::map<Feature::Id, Feature>& features,
    const std::vector<Corner>& corners, const ros::Time& time,
    const std::string& frame_id) const {

  sensor_msgs::PointCloud cloud;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "rgb";

  union {
    uint8_t rgb[4];
    float val;
  } color;

  color.rgb[0] = 255;
  color.rgb[1] = 255;
  color.rgb[2] = 0;
  color.rgb[3] = 0;

  for (const Corner& corner : corners) {

    const auto& id = corner.id();
    const auto& feat_ite = features.find(id);
    if (feat_ite != features.end()) {
      const Feature& feat = feat_ite->second;

      geometry_msgs::Point32 p32;
      kr::vec3<scalar_t> p(feat.p_world().x, feat.p_world().y,
                           feat.p_world().z);
      p32.x = p[0];
      p32.y = p[1];
      p32.z = p[2];

      cloud.points.push_back(p32);
      channel.values.push_back(color.val);
    }
  }

  cloud.channels.push_back(channel);
  cloud.header.stamp = time;
  cloud.header.frame_id = frame_id;
  points_pub_.publish(cloud);
}
*/

void StereoVoNode::PublishPoseStamped(const geometry_msgs::Pose& pose,
                                      const ros::Time& time,
                                      const std::string& frame_id) const {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = time;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = pose;
  pose_pub_.publish(pose_stamped);
}

void StereoVoNode::PublishTrajectory(const geometry_msgs::Pose& pose,
                                     const ros::Time& time,
                                     const std::string& frame_id) {
  traj_.header.stamp = time;
  traj_.header.frame_id = frame_id;
  geometry_msgs::Point p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  p.z = pose.position.z;
  traj_.points.push_back(p);
  traj_pub_.publish(traj_);
}

const StereoVoConfig ReadConfig(const ros::NodeHandle& nh) {
  StereoVoConfig config;
  nh.param<int>("cell_size", config.cell_size, 50);
  nh.param<int>("shi_max_corners", config.shi_max_corners, 200);
  nh.param<double>("shi_quality_level", config.shi_quality_level, 0.01);
  nh.param<double>("shi_min_distance", config.shi_min_distance, 12);

  nh.param<int>("klt_max_level", config.klt_max_level, 3);
  nh.param<int>("klt_win_size", config.klt_win_size, 13);

  nh.param<double>("pnp_ransac_inliers", config.pnp_ransac_inliers, 0.7);
  nh.param<double>("pnp_ransac_error", config.pnp_ransac_error, 4.0);

  nh.param<int>("kf_size", config.kf_size, 4);
  nh.param<double>("kf_dist_thresh", config.kf_dist_thresh, 1.5);
  nh.param<double>("kf_yaw_thresh", config.kf_yaw_thresh, 45);
  nh.param<double>("kf_min_filled", config.kf_min_filled, 0.7);

  nh.param<double>("tri_max_eigenratio", config.tri_max_eigenratio, 1.0e5);
  return config;
}

geometry_msgs::Pose KrPoseToRosPose(const Pose& kr_pose) {
  geometry_msgs::Pose ros_pose;
  ros_pose.orientation.w = kr_pose.q.w();
  ros_pose.orientation.x = -kr_pose.q.x();
  ros_pose.orientation.y = -kr_pose.q.y();
  ros_pose.orientation.z = -kr_pose.q.z();
  tf::pointEigenToMsg(kr_pose.p.cast<double>(), ros_pose.position);
  return ros_pose;
}

}  // namespace stereo_vo

}  // namespace galt
