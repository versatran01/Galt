#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
#include "stereo_vo/key_frame.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/feature_detector.h"

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
        detector_(config.cell_size, config.shi_max_corners,
                  config.shi_quality_level) {}

  const bool init() const { return init_; }
  const Pose &current_pose() const { return current_pose_; }
  const std::vector<Feature> &features() const { return features_; }

  void Initialize(const CvStereoImage &stereo_image,
                  const StereoCameraModel &model);
  void Iterate(const CvStereoImage &stereo_image);
  void UpdateConfig(const StereoVoConfig &config) { config_ = config; }

 private:
  Pose EstimatePose();
  void AddKeyFrame(const Pose &pose, const CvStereoImage &stereo_image,
                   std::vector<Feature> features);
  // void TrackTemporal(const cv::Mat& l_image_prev, const cv::Mat& l_image,
  //                   std::set<Feature::Id>& removable);

  bool init_{false};
  StereoCameraModel model_;
  StereoVoConfig config_;

  Pose current_pose_;
  Feature::Id feature_cnt_;
  GoodFeatureDetector detector_;
  std::vector<Feature> features_;
  std::deque<KeyFrame> key_frames_;
  CvStereoImage stereo_image_prev_;
};

void TrackFeatures(const cv::Mat &image1, const cv::Mat &image2,
                   std::vector<Feature> &features,
                   std::function<void(Feature *, const CvPoint2 &)> update_func,
                   const int win_size, const int max_level);
}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H_
