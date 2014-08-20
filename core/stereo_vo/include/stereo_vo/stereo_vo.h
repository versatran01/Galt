#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
#include "stereo_vo/optimizer.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/point3d.h"
#include "stereo_vo/corner_detector.h"
#include "stereo_vo/frame.h"

#include <vector>
#include <memory>
#include <deque>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include "stereo_vo/pose.h"

#include "opencv2/core/core.hpp"

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

class StereoVo {
 public:
  /**
   * @brief StereoVo Constructor
   * @param config Dynamic reconfigure config of stereo_vo
   */
  StereoVo(const StereoVoConfig &config)
      : config_(config),
        detector_(config.cell_size, config.shi_max_corners,
                  config.shi_quality_level, config.shi_quality_level) {}

  const bool init() const { return init_; }
  const FramePtr &prev_frame() const { return prev_frame_; }
  const KrPose &pose_world() const { return prev_frame_->pose(); }
  const std::deque<FramePtr> &key_frames() const { return key_frames_; }
  const std::map<Id, Point3d> &point3ds() const { return point3ds_; }
  const std::vector<Feature> &features() const {
    return prev_frame_->features();
  }

  /**
   * @brief Initialize Initialize stereo visual odometry
   * @param stereo_image Incoming stereo image
   * @param model Stereo camera model
   */
  void Initialize(const CvStereoImage &stereo_image,
                  const StereoCameraModel &model);
  /**
   * @brief Iterate Do one iteration of stereo visual odometry
   * @param stereo_image Incoming stereo image
   */
  void Iterate(const CvStereoImage &stereo_image);
  /**
   * @brief UpdateConfig
   * @param config Dynamic reconfigure config of stereo_vo
   */
  void UpdateConfig(const StereoVoConfig &config) { config_ = config; }

 private:
  /**
   * @brief ShouldAddKeyFrame Determines whether to add a key frame or not
   * @param frame Current frame
   * @return True if we should add a key frame
   */
  bool ShouldAddKeyFrame(const FramePtr &frame) const;
  /**
   * @brief AddKeyFrame Add current frame to key frame
   * @param frame Current frame
   */
  void AddKeyFrame(const FramePtr &frame);
  /**
   * @brief TrackSpatial Track features across stereo images
   * @param stereo_image Left and right images
   * @param features Features currently being tracked on left image
   * @param r_corners Tracked corners on right image
   */
  void TrackSpatial(const CvStereoImage &stereo_image,
                    std::vector<Feature> &features,
                    std::vector<CvPoint2> &r_corners);
  void Triangulate(const KrPose &pose, std::vector<Feature> &features,
                   std::vector<CvPoint2> &corners);
  void TrackTemporal(const FramePtr &frame1, const FramePtr &frame2,
                     const FramePtr &key_frame);
  void TrackTemporal(const cv::Mat &image1, const cv::Mat &image2,
                     const std::vector<Feature> &features1,
                     std::vector<Feature> &features2,
                     const FramePtr &key_frame);
  /**
   * @brief EstimatePose Estimate pose of current frame based on its 2d features
   * @param frame Current frame
   * @param point3ds Triangulated 3d points in world frame
   */
  void EstimatePose(const FramePtr &frame,
                    const std::map<Id, Point3d> point3ds);
  void OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
                   const std::vector<CvPoint2> &points1,
                   std::vector<CvPoint2> &points2, std::vector<uchar> &status);
  void FindFundamentalMat(const std::vector<CvPoint2> &points1,
                          const std::vector<CvPoint2> &points2,
                          std::vector<uchar> &status);
  /**
   * @brief TriangulatePoint
   * @param left Left observation in pixels.
   * @param right Right observation in pixels.
   * @param point3d Resulting 3d point in camera frame.
   * @return False if the triangulation is poor and the feature should be
   * rejected.
   */
  bool TriangulatePoint(const CvPoint2 &left, const CvPoint2 &right,
                        kr::vec3<scalar_t> &p_cam);

  FramePtr &prev_key_frame() { return key_frames_.back(); }
  const FrameConstPtr prev_key_frame() const { return key_frames_.back(); }

  bool init_{false};              ///< True if stereo_vo is initialized
  StereoCameraModel model_;       ///< Stereo camera model
  StereoVoConfig config_;         ///< Dynamic reconfigure config of stereo_vo
  GoodFeatureDetector detector_;  ///< Corner detector
  WindowedOptimizer optimizer_;  ///< Windowed optimizer
  FramePtr prev_frame_;           ///< Previous frame
  std::deque<FramePtr> key_frames_;  ///< A deque of key frames in window
  std::map<Id, Point3d> point3ds_;   ///< Triangulated 3d points in world frame
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_H_
