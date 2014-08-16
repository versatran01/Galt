#ifndef GALT_STEREO_VO_NODE_H_
#define GALT_STEREO_VO_NODE_H_

#include <memory>
#include <utility>
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

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

#include <dynamic_reconfigure/server.h>
#include <stereo_vo/StereoVoDynConfig.h>

#include <visualization_msgs/Marker.h>

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
  ros::NodeHandle nh_;                             ///< Private nodehandle
  image_transport::ImageTransport it_;             ///< Private image transport
  image_transport::SubscriberFilter l_image_sub_;  ///< Left image subscriber
  image_transport::SubscriberFilter r_image_sub_;  ///< Right image subscriber
  using CinfoSubscriberFilter = message_filters::Subscriber<CameraInfo>;
  CinfoSubscriberFilter l_cinfo_sub_;  ///< Left cinfo subscriber
  CinfoSubscriberFilter r_cinfo_sub_;  ///< Right cinfo subscriber
  using ExactPolicy = ExactTime<Image, CameraInfo, Image, CameraInfo>;
  using ApproximatePolicy =
      ApproximateTime<Image, CameraInfo, Image, CameraInfo>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::unique_ptr<ExactSync> exact_sync_;  ///< Exact time sync policy
  std::unique_ptr<ApproximateSync>
      approximate_sync_;  ///< Approximate time sync policy

  ros::Publisher points_pub_;        ///< Point cloud publisher
  ros::Publisher pose_pub_;          ///< Camera pose publisher
  ros::Publisher traj_pub_;          ///< Camera trjectory publisher
  visualization_msgs::Marker traj_;  ///< Trajectory marker

  dynamic_reconfigure::Server<StereoVoDynConfig>
      cfg_server_;      ///< Dynamic reconfigure server
  StereoVo stereo_vo_;  ///< Stereo visual odometry

  /**
   * @brief SubscribeStereoTopics Subscribe to stereo image topics
   * @param image_topic String of image topic name
   * @param cinfo_topic String of camera info topic name
   * @param hints ImageTransport hings
   */
  void SubscribeStereoTopics(const std::string& image_topic,
                             const std::string& cinfo_topic,
                             const image_transport::TransportHints& hints);
  /**
   * @brief StereoCallback Callback of stereo camera
   * @param l_image_msg Left image
   * @param l_cinfo_msg Left camera info
   * @param r_image_msg Right image
   * @param r_cinfo_msg Right camera info
   */
  void StereoCallback(const ImageConstPtr& l_image_msg,
                      const CameraInfoConstPtr& l_cinfo_msg,
                      const ImageConstPtr& r_image_msg,
                      const CameraInfoConstPtr& r_cinfo_msg);
  /**
   * @brief ReconfigureCallback Dynamic reconfigure callback
   * @param config Stereo vo dynamic config
   * @param level Don't really know what this is
   */
  void ReconfigureCallback(const StereoVoDynConfig& config, int level);

  void PublishPoseStamped(const geometry_msgs::Pose& pose,
                          const ros::Time& time,
                          const std::string& frame_id) const;
  void PublishPointCloud(const std::map<Id, Point3d> &point3ds,
                         const std::vector<Feature> &features,
                         const ros::Time& time,
                         const std::string& frame_id) const;
  void PublishTrajectory(const geometry_msgs::Pose& pose, const ros::Time& time,
                         const std::string& frame_id);
};  // class StereoVoNode

/**
 * @brief ReadConfig Read config from ros parameter server
 * @param nh Private note handle
 * @return Stereo Vo config
 */
const StereoVoDynConfig ReadConfig(const ros::NodeHandle& nh);
/**
 * @brief KrPoseToRosPose Convert kr::Pose to geometry_msgs::Pose
 * @param kr_pose A kr::Pose
 * @return geometry_msgs::Pose
 */
geometry_msgs::Pose KrPoseToRosPose(const Pose& kr_pose);

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_NODE_H_
