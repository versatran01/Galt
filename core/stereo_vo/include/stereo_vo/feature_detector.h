#ifndef GALT_STEREO_VO_FEATURE_DETECTOR_H_
#define GALT_STEREO_VO_FEATURE_DETECTOR_H_

#include "stereo_vo/common.h"

#include <map>
#include <utility>

namespace galt {

namespace stereo_vo {

class FeatureDetectorBase {
 public:
  FeatureDetectorBase(int cell_size) : cell_size_{cell_size} {}
  virtual ~FeatureDetectorBase() {}

  virtual void DetectFeatures(const cv::Mat& image,
                              std::vector<Feature>& features) const = 0;

 private:
  int cell_size_;
  std::map<std::pair<int, int>, bool> grid_;
};

class GoodFeatureDetector : public FeatureDetectorBase {
 public:
  GoodFeatureDetector(int cell_size, int max_corners, double quality_level,
                      double min_distance)
      : FeatureDetectorBase(cell_size),
        max_corners_{max_corners},
        quality_level(quality_level_),
        min_distance(min_distance_) {}
  void DetectFeatures(const cv::Mat& image,
                      std::vector<Feature>& features) const override;

 private:
  int max_corners_;
  double quality_level_;
  double min_distance_;

  void Detect(const cv::Mat& image, Corners2& corners) const;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_DETECTOR_H
