#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
#include "stereo_vo/key_frame.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/corner_detector.h"

#include <vector>
#include <memory>
#include <deque>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <kr_math/pose.hpp>

#include "opencv2/core/core.hpp"

namespace galt {

namespace stereo_vo {

using image_geometry::StereoCameraModel;

class StereoVo {
 public:
  StereoVo(const StereoVoConfig &config)
      : config_(config),
        detector_(config.shi_max_corners, config.shi_quality_level,
                  config.shi_min_distance) {}

  const bool init() const { return init_; }
  const std::map<Feature::Id, Feature> features() const { return features_; }

  /**
   * @brief Pose with respect to the world frame.
   * @return kr::Pose
   */
  const Pose &absolute_pose() const { return absolute_pose_; }
  const std::vector<Corner> &corners() const { return corners_; }
  const std::deque<KeyFrame> &key_frames() const { return key_frames_; }

  void Initialize(const CvStereoImage &stereo_image,
                  const StereoCameraModel &model);
  void Iterate(const CvStereoImage &stereo_image);
  void UpdateConfig(const StereoVoConfig &config) { config_ = config; }

 private:
  bool ShouldAddKeyFrame(size_t num_corners) const;
  void AddKeyFrame(const Pose &pose, const CvStereoImage &stereo_image,
                   std::vector<Corner> &corners);
  void TrackSpatial(const CvStereoImage &stereo_image,
                    std::vector<Corner> &corners,
                    std::vector<CvPoint2> &r_points);
  void Triangulate(const Pose &pose, std::vector<Corner> &corners,
                   std::vector<CvPoint2> &points);
  void TrackTemporal(const cv::Mat &image_prev, const cv::Mat &image,
                     const std::vector<Corner> &corners_input,
                     std::vector<Corner> &corners_output, KeyFrame &key_frame);
  Pose EstimatePose();
  void OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
                   const std::vector<CvPoint2> &points1,
                   std::vector<CvPoint2> &points2, std::vector<uchar> &status);
  void FindFundamentalMat(const std::vector<CvPoint2> &points1,
                          const std::vector<CvPoint2> &points2,
                          std::vector<uchar>& status);

  /**
   * @brief TriangulatePoint
   * @param left Left observation in pixels.
   * @param right Right observation in pixels.
   * @param output Resulting 3D point.
   * @return False if the triangulation is poor and the feature should be
   * rejected.
   */
  bool TriangulatePoint(const Pose &pose, const CvPoint2 &left,
                        const CvPoint2 &right, CvPoint3 &output);

  KeyFrame &key_frame_prev() { return key_frames_.back(); }

  bool init_{false};
  StereoCameraModel model_;
  StereoVoConfig config_;

  Pose absolute_pose_;
  GlobalCornerDetector detector_;
  std::vector<Corner> corners_;
  std::deque<KeyFrame> key_frames_;
  std::map<Feature::Id, Feature> features_;
  CvStereoImage stereo_image_prev_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H_
