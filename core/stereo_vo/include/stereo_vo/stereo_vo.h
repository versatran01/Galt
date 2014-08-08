#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
#include "stereo_vo/key_frame.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/feature_detector.h"

#include <vector>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <kr_math/pose.hpp>

#include "opencv2/core/core.hpp"

namespace galt {

namespace stereo_vo {

using image_geometry::StereoCameraModel;

class StereoVo {
 public:
  StereoVo(const StereoVoConfig& config) : config_(config),
    detector_(config.cell_size, config.max_corners, 0.005) {}

  void Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                  const StereoCameraModel &model);
  void Iterate(const cv::Mat &l_image, const cv::Mat &r_image);
  void EstimatePose();

  void UpdateConfig(const StereoVoConfig &config) { config_ = config; }

  bool AddKeyFrame();

  const Pose &current_pose() const { return current_pose_; }
  const bool init() const { return init_; }

 private:
  bool init_{false};
  StereoCameraModel model_;
  StereoVoConfig config_;

  Pose current_pose_;
  Features features_;
  KeyFrames key_frames_;
  GoodFeatureDetector detector_;

  cv::Mat l_image_prev_;
  cv::Mat r_image_prev_;

  void TriangulateFeatures();
};


void TrackFeatures(const cv::Mat &image1, const cv::Mat &image2,
                   Features &features,
                   std::function<void(Feature *, const CvPoint2 &)> update_func,
                   const int win_size, const int max_level);

// struct Feature {
//  Feature() : triangulated(false) {}

//  CvPoint2 next;         /// New frame feature in pixle space
//  CvPoint2 left, right;  /// Key frame feature in pixel space
//  CvPoint2 left_coord,
//      right_coord;    /// Key frame feature in normalized image space
//  CvPoint3 point;     /// Key frame feature in world space
//  bool triangulated;  /// Key frame featreu has been triangulated
//};

// class KeyFrame {
//  friend class StereoVo;

// public:
//  void Update(const cv::Mat &l_image, const cv::Mat &r_image,
//              const StereoVoConfig &config, const StereoCameraModel &model,
//              const kr::Pose<scalar_t> &pose, bool init = false);
//  const int NumMatches() const { return features_.size(); }
//  const kr::Pose<scalar_t> &pose() const { return pose_; }

// private:
//  scalar_t Triangulate(const StereoCameraModel &model);
//  cv::Mat l_image_, r_image_;
//  cv::Mat prev_image_;
//  std::vector<Feature> features_;
//  int length;  /// Number of frames after this keyframe
//  kr::Pose<scalar_t> pose_;
//};
}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H
