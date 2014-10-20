#ifndef GALT_STEREO_VO_NODE_H_
#define GALT_STEREO_VO_NODE_H_

#include <utility>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <stereo_vo/StereoVoDynConfig.h>

#include <rviz_helper/marker_visualizer.hpp>
#include <rviz_helper/tf_publisher.hpp>

#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/common.h"

namespace galt {
namespace stereo_vo {

using sensor_msgs::Image;
using sensor_msgs::CameraInfo;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::CameraInfoConstPtr;
using message_filters::sync_policies::ExactTime;

class StereoVoNode {
 public:
  StereoVoNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

 private:
  using CinfoSubscriberFilter = message_filters::Subscriber<CameraInfo>;
  using ExactPolicy = ExactTime<Image, CameraInfo, Image, CameraInfo>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;

  void SubscribeStereoTopics(const std::string& image_topic,
                             const std::string& cinfo_topic,
                             const std::string& transport);

  //  void OdometryCb(const nav_msgs::OdometryConstPtr& odom_msg);

  void StereoCb(const ImageConstPtr& l_image_msg,
                const CameraInfoConstPtr& l_cinfo_msg,
                const ImageConstPtr& r_image_msg,
                const CameraInfoConstPtr& r_cinfo_msg);

  void ConfigCb(StereoVoDynConfig& config, int level);

  CvStereoImage FromImage(const ImageConstPtr& l_image_msg,
                          const ImageConstPtr& r_image_msg);
  ros::NodeHandle nh_, pnh_;
  std::string parent_frame_id_;
  std::string frame_id_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_l_image_;
  image_transport::SubscriberFilter sub_r_image_;
  CinfoSubscriberFilter sub_l_cinfo_;
  CinfoSubscriberFilter sub_r_cinfo_;
  boost::shared_ptr<ExactSync> exact_sync_;
  dynamic_reconfigure::Server<StereoVoDynConfig> cfg_server_;
  StereoVoDynConfig config_;
  stereo_vo::StereoVo stereo_vo_;
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher pub_point_;

  kr::viz::TfPublisher tf_pub_;
  kr::viz::TrajectoryVisualizer traj_viz_;
  kr::viz::CovarianceVisualizer cov_viz_;

  /// Publish a coloured point cloud of all current features.
  void PublishPointCloud(const ros::Time& time,
                         const std::string& frame_id) const;
  
  /// Publish pose w/ covariance and rviz markers.
  void PublishPoseAndViz(const ros::Time& time);
};

void ProcessStereoImage(const cv::Mat& l_image, cv::Mat& r_image);

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_NODE_H_
