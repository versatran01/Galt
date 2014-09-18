#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/point3d.h"
#include "stereo_vo/feature_detector.h"
#include "stereo_vo/keyframe.h"
#include "stereo_vo/tracker.h"

#include <vector>
#include <memory>

#include <image_geometry/stereo_camera_model.h>

#include "opencv2/core/core.hpp"

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

class StereoVo {
 public:
  StereoVo() : init_(false), init_pose_(false) {}

  void set_config(const StereoVoDynConfig &config) {
    config_ = config;
    detector_.set_cell_size(config_.cell_size);
  }

  bool init() const { return init_; }

  bool init_pose() const { return init_pose_; }
  void set_init_pose(bool init_pose) { init_pose_ = init_pose; }

  const KrPose &pose() const { return pose_; }
  void set_pose(const KrPose &pose) { pose_ = pose; }

  const std::map<Id, Point3d> &points() const { return points_; }

  const std::vector<Feature> &features() const {
    return temporal_tracker_.features();
  }

  /**
   * @brief Initialize
   * @param stereo_image
   * @param model
   */
  void Initialize(const CvStereoImage &stereo_image,
                  const StereoCameraModel &model);
  /**
   * @brief Iterate Do one iteration of stereo visual odometry
   * @param stereo_image Incoming stereo image
   */
  void Iterate(const CvStereoImage &stereo_image);

 private:
  /// Add a key frame.
  void AddKeyFrame(const CvStereoImage &stereo_image);

  /// Check if a key frame should be added.
  bool ShouldAddKeyFrame() const;

  /**
   * @brief Estimate pose with Ransac PnP.
   */
  void EstimatePose();

  bool init_;                 ///< True if stereo_vo is initialized
  bool init_pose_;            ///< True if initial pose is set
  KrPose pose_;               ///< Pose initialized from gps_kf
  StereoCameraModel model_;   ///< Stereo camera model
  StereoVoDynConfig config_;  ///< Dynamic reconfigure config of stereo_vo
  FeatureDetector detector_;  ///< Grid based feature detector
  std::vector<KeyFramePtr> key_frames_;  ///< A deque of key frames in window

  Tracker temporal_tracker_;  ///< Track features previous to current.
  Tracker spatial_tracker_;   ///< Track features left to right.
  cv::Mat prev_left_image_;   ///< Last left image.

  std::map<Id, Point3d> points_;  ///< 3D points, keyed by unique ID.
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_H_
