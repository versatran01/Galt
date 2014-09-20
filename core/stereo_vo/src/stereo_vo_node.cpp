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

StereoVoNode::StereoVoNode(const ros::NodeHandle& nh) : nh_(nh), it_(nh) {
  SubscribeStereoTopics("image_rect", "camera_info", "raw");
  cfg_server_.setCallback(boost::bind(&StereoVoNode::ConfigCb, this, _1, _2));
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
  sub_l_image_.subscribe(it_, resolve(append(left, image_topic)), 1, hints);
  sub_l_cinfo_.subscribe(nh_, resolve(append(left, cinfo_topic)), 1);
  sub_r_image_.subscribe(it_, resolve(append(right, image_topic)), 1, hints);
  sub_r_cinfo_.subscribe(nh_, resolve(append(right, cinfo_topic)), 1);
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

void StereoVoNode::ConfigCb(const StereoVoDynConfig& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", nh_.getNamespace().c_str(),
             "Initializing dynamic reconfigure server");
  }
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

  // Initialize stereo visual odometry if not
  //  if (!stereo_vo_.init()) {
  //    stereo_vo_.Initialize(stereo_image, stereo_model_);
  //    return;
  //  }
  //  stereo_vo_.I
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

/*
void StereoVoNode::PublishPointCloud(const ros::Time& time,
                                     const std::string& frame_id) const {
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = time;
  cloud.header.frame_id = frame_id;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "rgb";

  union {
    uint8_t rgb[4];
    float val;
  } color;

  const std::map<Id, Point3d> points = stereo_vo_.points();
  for (const Feature& feature : stereo_vo_.features()) {
    //  find corresponding point 3d
    std::map<Id, Point3d>::const_iterator point_ite = points.find(feature.id());
    assert(point_ite != points.end());

    const Point3d& point3d = point_ite->second;

    if (!point3d.is_inlier()) {
      color.rgb[0] = 255;
      color.rgb[1] = 255;
      color.rgb[2] = 0;
      color.rgb[3] = 0;
    } else {
      color.rgb[0] = 0;
      color.rgb[1] = 255;
      color.rgb[2] = 255;
      color.rgb[3] = 0;
    }

    geometry_msgs::Point32 p32;
    const vec3 p(point3d.p_world().x, point3d.p_world().y, point3d.p_world().z);
    p32.x = p[0];
    p32.y = p[1];
    p32.z = p[2];

    cloud.points.push_back(p32);
    channel.values.push_back(color.val);
  }

  cloud.channels.push_back(channel);
  point_pub_.publish(cloud);
}
*/

/*
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
*/

CvStereoImage FromImage(const ImageConstPtr& l_image_msg,
                        const ImageConstPtr& r_image_msg) {
  cv::Mat l_image_rect =
      cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  cv::Mat r_image_rect =
      cv_bridge::toCvCopy(r_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  return {l_image_rect, r_image_rect};
}

}  // namespace stereo_vo

}  // namespace galt
