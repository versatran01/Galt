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

  const std::map<Id,Point3d>& points() const { return points_; }
  
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
  void AddKeyFrame(const CvStereoImage& stereo_image);
  
//  void TrackSpatial(const CvStereoImage &stereo_image,
//                    std::vector<Feature> &features,
//                    std::vector<CvPoint2> &r_corners);
  //  void Triangulate(const KrPose &pose, std::vector<Feature> &features,
  //                   std::vector<CvPoint2> &corners);
  //  bool ShouldAddKeyFrame(const FramePtr &frame) const;
  //  void TrackTemporal(const FramePtr &frame1, const FramePtr &frame2,
  //                     const FramePtr &key_frame);
  //  void TrackTemporal(const cv::Mat &image1, const cv::Mat &image2,
  //                     const std::vector<Feature> &features1,
  //                     std::vector<Feature> &features2,
  //                     const FramePtr &key_frame);
  /**
   * @brief EstimatePose Estimate pose of current frame based on its 2d features
   * @param frame Current frame
   * @param point3ds Triangulated 3d points in world frame
   */
  //  void EstimatePose(const FramePtr &frame, const std::map<Id, Point3d>
  // point3ds,
  //                    const KrPose &old_pose);
//  void OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
//                   const std::vector<CvPoint2> &points1,
//                   std::vector<CvPoint2> &points2, std::vector<uchar> &status);
//  void FindFundamentalMat(const std::vector<CvPoint2> &points1,
//                          const std::vector<CvPoint2> &points2,
//                          std::vector<uchar> &status);
  /**
   * @brief TriangulatePoint
   * @param left Left observation in pixels.
   * @param right Right observation in pixels.
   * @param point3d Resulting 3d point in camera frame.
   * @return False if the triangulation is poor and the feature should be
   * rejected.
   */
  //  bool TriangulatePoint(const CvPoint2 &left, const CvPoint2 &right,
  //                        kr::vec3<scalar_t> &p_cam);

  /**
   * @brief NukeOutliers
   */
  //  void NukeOutliers();

  //  FramePtr &prev_key_frame() { return key_frames_.back(); }
  //  const FrameConstPtr prev_key_frame() const { return key_frames_.back(); }

  bool init_;                 ///< True if stereo_vo is initialized
  bool init_pose_;            ///< True if initial pose is set
  KrPose pose_;               ///< Pose initialized from gps_kf
  StereoCameraModel model_;   ///< Stereo camera model
  StereoVoDynConfig config_;  ///< Dynamic reconfigure config of stereo_vo
  FeatureDetector detector_;  ///< Grid based feature detector
  std::vector<KeyFramePtr> key_frames_;  ///< A deque of key frames in window

  Tracker temporal_tracker_;  ///< Track features previous to current.
  Tracker spatial_tracker_;   ///< Track features left to right.
  cv::Mat prev_left_;         ///< Last left image.
  
  std::map<Id, Point3d> points_; ///< 3D points, keyed by unique ID.
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_H_
