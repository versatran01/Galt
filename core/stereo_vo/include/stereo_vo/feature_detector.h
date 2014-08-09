#ifndef GALT_STEREO_VO_FEATURE_DETECTOR_H_
#define GALT_STEREO_VO_FEATURE_DETECTOR_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <set>
#include <utility>

namespace galt {

namespace stereo_vo {

using Grid = std::set<std::pair<int, int>>;

class FeatureDetectorBase {
 public:
  FeatureDetectorBase(int cell_size) : cell_size_{cell_size} {}
  virtual ~FeatureDetectorBase() {}

  virtual void DetectFeatures(const cv::Mat& image,
                              CvCorners2& corners) const = 0;
  void AddFeatures(const cv::Mat& image, Features& features) const;

 protected:
  const Grid CreateGrid(const cv::Mat& image, const Features& features) const;
  int cell_size_;
};

class GoodFeatureDetector : public FeatureDetectorBase {
 public:
  GoodFeatureDetector(int cell_size, int max_corners, double quality_level)
      : FeatureDetectorBase(cell_size),
        max_corners_{max_corners},
        quality_level_{quality_level},
        min_distance_{static_cast<double>(cell_size / max_corners)} {}

  void DetectFeatures(const cv::Mat& image, CvCorners2& corners) const override;

 private:
  int max_corners_;
  double quality_level_;
  double min_distance_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_DETECTOR_H
