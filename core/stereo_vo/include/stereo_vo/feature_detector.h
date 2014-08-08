#ifndef GALT_STEREO_VO_FEATURE_DETECTOR_H_
#define GALT_STEREO_VO_FEATURE_DETECTOR_H_

#include "stereo_vo/common.h"

#include <map>
#include <utility>

namespace galt {

namespace stereo_vo {

 using Grid = std::map<std::pair<int, int>, bool>;
 using Features = std::vector<Feature>;

class Feature;

class FeatureDetectorBase {
 public:

  FeatureDetectorBase(int cell_size) : cell_size_{cell_size} {}
  virtual ~FeatureDetectorBase() {}

  virtual void DetectFeatures(const cv::Mat& image,
                              Features& features) const = 0;

 protected:
  const Grid CreateGrid(const cv::Mat& image, const Features& features) const;

 private:
  int cell_size_;
};

class GoodFeatureDetector : public FeatureDetectorBase {
 public:
  GoodFeatureDetector(int cell_size, int max_corners, double quality_level,
                      double min_distance)
      : FeatureDetectorBase(cell_size),
        max_corners{max_corners},
        quality_level{quality_level_},
        min_distance{min_distance_} {}

  void DetectFeatures(const cv::Mat& image, Features& features) const override;

 private:
  void DetectGoodFeatures(const cv::Mat& image, Corners2& corners) const;

  int max_corners_;
  double quality_level_;
  double min_distance_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_DETECTOR_H
