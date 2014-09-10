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

#include <rviz_helper/rviz_helper.h>

#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/common.h"

namespace galt {
namespace stereo_vo {

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class StereoVoNode {
 public:
  StereoVoNode(const ros::NodeHandle& nh);

 private:
  using CinfoSubscriberFilter = message_filters::Subscriber<CameraInfo>;
  using ExactPolicy = ExactTime<Image, CameraInfo, Image, CameraInfo>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;

  void SubscribeStereoTopics(const std::string& image_topic,
                             const std::string& cinfo_topic,
                             const image_transport::TransportHints& hints);

  void OdometryCb(const nav_msgs::OdometryConstPtr& odom_msg);

  void StereoCb(const ImageConstPtr& l_image_msg,
                const CameraInfoConstPtr& l_cinfo_msg,
                const ImageConstPtr& r_image_msg,
                const CameraInfoConstPtr& r_cinfo_msg);

  void ReconfigureCb(const StereoVoDynConfig& config, int level);

  ros::NodeHandle nh_;                              ///< Private nodehandle
  image_transport::ImageTransport it_;              ///< Private image transport
  image_transport::SubscriberFilter l_image_sub_;   ///< Left image subscriber
  image_transport::SubscriberFilter r_image_sub_;   ///< Right image subscriber
  CinfoSubscriberFilter l_cinfo_sub_;               ///< Left cinfo subscriber
  CinfoSubscriberFilter r_cinfo_sub_;               ///< Right cinfo subscriber
  ros::Subscriber odom_sub_;                        ///< Kf odometry subscriber
  boost::shared_ptr<ExactSync> exact_sync_;         ///< Exact time sync policy
  visualization_msgs::Marker traj_;                 ///< Trajectory marker
  image_geometry::StereoCameraModel stereo_model_;  ///< Stereo camera model
  rviz_helper::TfPublisher tf_pub_;                 ///< Transfrom publisher
  rviz_helper::TrajectoryVisualizer traj_viz_;      ///< Trajectory visualizer
  std::string frame_id_;                            ///< Reference frame
  StereoVo stereo_vo_;                              ///< Stereo visual odometry
  tf2::BufferCore core_;
  tf2_ros::TransformListener tf_listener_;
  dynamic_reconfigure::Server<StereoVoDynConfig> cfg_server_;

  //  void PublishPoseStamped(const geometry_msgs::Pose& pose,
  //                          const ros::Time& time,
  //                          const std::string& frame_id) const;
  //  void PublishPointCloud(const std::map<Id, Point3d>& point3ds,
  //                         const std::deque<FramePtr>& key_frames,
  //                         const ros::Time& time,
  //                         const std::string& frame_id) const;
  //  void PublishTrajectory(const geometry_msgs::Pose& pose, const ros::Time&
  // time,
  //                         const std::string& frame_id);
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_NODE_H_
