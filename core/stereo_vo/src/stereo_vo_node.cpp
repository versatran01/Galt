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
#include <tf2/transform_datatypes.h>

namespace galt {

namespace stereo_vo {

StereoVoNode::StereoVoNode(const ros::NodeHandle& nh,
                           const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), it_(nh), tf_listener_(core_),
      traj_viz_(pnh_), cov_viz_(pnh_) {
  
  pnh_.param<std::string>("parent_frame_id", parent_frame_id_, "world");
  pnh_.param<std::string>("frame_id", frame_id_, "stereo_vo");
  SubscribeStereoTopics("image_rect", "camera_info", "raw");
  cfg_server_.setCallback(boost::bind(&StereoVoNode::ConfigCb, this, _1, _2));
  
  pub_point_ = pnh_.advertise<sensor_msgs::PointCloud>("points", 1);
  
  //  configure visualization tools
  traj_viz_.set_color(kr::rviz_helper::colors::GREEN);
  traj_viz_.set_alpha(1);
  traj_viz_.set_num_skip(12);
  cov_viz_.set_color(kr::rviz_helper::colors::GREEN);
  cov_viz_.set_alpha(0.5);
  tf_pub_.set_child_frame_id(frame_id_);
}

void StereoVoNode::SubscribeStereoTopics(const std::string& image_topic,
                                         const std::string& cinfo_topic,
                                         const std::string& transport) {
  image_transport::TransportHints hints(transport, ros::TransportHints(), nh_);
  exact_sync_.reset(new ExactSync(ExactPolicy(5), sub_l_image_, sub_l_cinfo_,
                                  sub_r_image_, sub_r_cinfo_));
  exact_sync_->registerCallback(
      boost::bind(&StereoVoNode::StereoCb, this, _1, _2, _3, _4));
  using namespace ros::names;
  std::string left("left");
  std::string right("right");
  sub_l_image_.subscribe(it_, append(left, image_topic), 1, hints);
  sub_l_cinfo_.subscribe(nh_, append(left, cinfo_topic), 1);
  sub_r_image_.subscribe(it_, append(right, image_topic), 1, hints);
  sub_r_cinfo_.subscribe(nh_, append(right, cinfo_topic), 1);
}

/*
void StereoVoNode::OdometryCb(const nav_msgs::OdometryConstPtr& odom_msg) {
  // Set frame_id only once
  if (frame_id_.empty()) {
    frame_id_ = odom_msg->header.frame_id;
    ROS_INFO("Set frame id to %s", frame_id_.c_str());
  }

  // Unsubscribe if stereo vo is already initialized
  if (stereo_vo_.init()) {
    odom_sub_.shutdown();
    ROS_INFO("StereoVo initialized, unsubscribe from gps_kf/odometry");
    return;
  }

  // Get the latest transform from stereo to world
  try {
    const geometry_msgs::TransformStamped transform = core_.lookupTransform(
        odom_msg->header.frame_id, "stereo", ros::Time(0));
    const geometry_msgs::Vector3& t = transform.transform.translation;
    const geometry_msgs::Quaternion& r = transform.transform.rotation;
    kr::vec3<scalar_t> p(t.x, t.y, t.z);
    kr::quat<scalar_t> q(r.w, r.x, r.y, r.z);
    stereo_vo_.set_pose(KrPose(q, p));
    stereo_vo_.set_init_pose(true);
  }
  catch (const tf2::TransformException& e) {
    ROS_WARN("%s", e.what());
  }

  // Set frame_id only once
  if (frame_id_.empty()) frame_id_ = odom_msg->header.frame_id;
  if (!stereo_vo_.init_pose()) stereo_vo_.set_init_pose(true);
}
*/

void StereoVoNode::ConfigCb(StereoVoDynConfig& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", nh_.getNamespace().c_str(),
             "Initializing dynamic reconfigure server");
  }
  if (!(config.klt_win_size % 2)) {
    config.klt_win_size++;
  }
  config_ = config;
  stereo_vo_.set_config(config);
}

void StereoVoNode::StereoCb(const ImageConstPtr& l_image_msg,
                            const CameraInfoConstPtr& l_cinfo_msg,
                            const ImageConstPtr& r_image_msg,
                            const CameraInfoConstPtr& r_cinfo_msg) {
  // Get stereo camera infos
  static image_geometry::StereoCameraModel stereo_model;
  stereo_model.fromCameraInfo(l_cinfo_msg, r_cinfo_msg);
  if (!stereo_model.initialized() &&
      (!l_cinfo_msg->K[0] || !r_cinfo_msg->K[0])) {
    ROS_ERROR_THROTTLE(1, "Uncalibrated camera.");
    return;
  }

  // Get stereo images
  CvStereoImage stereo_image = FromImage(l_image_msg, r_image_msg);

  if (!stereo_vo_.init()) {
    stereo_vo_.Initialize(stereo_image, stereo_model);
    return;
  }

  const bool inserted_kf = 
      stereo_vo_.Iterate(stereo_image, l_image_msg->header.stamp);

  //  publish any new points that were triangulated
  if (inserted_kf) {
    /// @todo: make frame_id a parameter...
    PublishPointCloud(l_image_msg->header.stamp, "stereo");
  }
  PublishPoseAndViz(l_image_msg->header.stamp);
  
  //  publish points and pose
  //  if (!frame_id_.empty()) {
  //    PublishPointCloud(l_image_msg->header.stamp);
  //    const geometry_msgs::Pose pose =
  //        static_cast<geometry_msgs::Pose>(stereo_vo_.pose());
  //    tf_pub_.PublishTransform(pose, frame_id_, l_image_msg->header.stamp);
  //    traj_viz_.PublishTrajectory(pose.position, frame_id_,
  //                                l_image_msg->header.stamp);
  //  }
}

void StereoVoNode::PublishPointCloud(const ros::Time& time,
                                     const std::string& frame_id) const {
  const std::deque<KeyFrame>& kfs = stereo_vo_.key_frames();
  const KeyFrame& kf = kfs.back();  //  latest keyframe
  
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = time;
  cloud.header.frame_id = frame_id;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "rgb";

  //  iterate over points and find the highest and lowest covariance
  double t_max = -std::numeric_limits<double>::infinity();
  double t_min = -t_max;
  for (const Feature& f : kf.features()) {
    double t = f.p_cov_cam.trace();
    t_max = std::max(t, t_max);
    t_min = std::min(t, t_min);
  }
  
  union {
    uint8_t rgb[4];
    float val;
  } color;

  for (const Feature& feature : kf.features()) {
    //  publish camera frame point
    const double scale = (feature.p_cov_cam.trace() - t_min) / (t_max - t_min);
    color.rgb[0] = 255 * scale;
    color.rgb[1] = 255 * (1 - scale);
    color.rgb[2] = 0;
    color.rgb[3] = 0;

    geometry_msgs::Point32 p32;
    p32.x = feature.p_cam.x;
    p32.y = feature.p_cam.y;
    p32.z = feature.p_cam.z;

    cloud.points.push_back(p32);
    channel.values.push_back(color.val);
  }

  cloud.channels.push_back(channel);
  pub_point_.publish(cloud);
}

void StereoVoNode::PublishPoseAndViz(const ros::Time& time) {
  
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.stamp = time;
  pose.header.frame_id = parent_frame_id_;
  pose.pose.pose = static_cast<geometry_msgs::Pose>(stereo_vo_.w_T_c());
  
  const kr::mat<scalar_t,6,6> cur_cov = stereo_vo_.pose_covariance();
  for (int i=0; i < 6; i++) {
    for (int j=0; j < 6; j++) {
      pose.pose.covariance[i*6 + j] = cur_cov(i,j);
    }
  }
  
  tf_pub_.PublishTransform(pose.pose.pose,parent_frame_id_,time);
  cov_viz_.PublishCovariance(pose);
  traj_viz_.PublishTrajectory(pose.pose.pose, pose.header);
}

CvStereoImage StereoVoNode::FromImage(const ImageConstPtr& l_image_msg,
                                      const ImageConstPtr& r_image_msg) {
  cv::Mat l_image_rect =
      cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  cv::Mat r_image_rect =
      cv_bridge::toCvCopy(r_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  // For now this will only correct right image's mean to match the left one
  if (config_.proc_image) {
    ProcessStereoImage(r_image_rect, l_image_rect);
  }
  return {l_image_rect, r_image_rect};
}

void ProcessStereoImage(const cv::Mat& l_image, cv::Mat& r_image) {
  const cv::Scalar l_mean = cv::mean(l_image);
  const cv::Scalar r_mean = cv::mean(r_image);
  r_image.convertTo(r_image, -1, 1, l_mean[0] - r_mean[0]);
}

}  // namespace stereo_vo
}  // namespace galt
