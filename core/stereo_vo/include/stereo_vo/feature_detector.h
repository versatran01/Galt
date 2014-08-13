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
  virtual ~FeatureDetectorBase() {}
  virtual void AddFeatures(const cv::Mat& image,
                           std::vector<Corner>& corners) = 0;

 protected:
  Feature::Id cnt_{0};
};

class GridDetectorBase : public FeatureDetectorBase {
 public:
  GridDetectorBase(int cell_size) : cell_size_{cell_size} {}

  virtual void DetectCorners(const cv::Mat& image,
                             std::vector<CvPoint2>& corners) const = 0;
  void AddFeatures(const cv::Mat& image, std::vector<Corner>& corners);
  double GridFilled(const cv::Mat& image, const std::vector<Corner>& corners);

 protected:
  const Grid CreateGrid(const std::vector<Corner>& corners) const;
  int cell_size_;
  Grid grid_;
  int grid_rows_, grid_cols_;
};

class GoodFeatureDetector : public GridDetectorBase {
 public:
  GoodFeatureDetector(int cell_size, int max_corners, double quality_level)
      : GridDetectorBase(cell_size),
        max_corners_{max_corners},
        quality_level_{quality_level},
        min_distance_{static_cast<double>(cell_size / max_corners)} {}

  void DetectCorners(const cv::Mat& image,
                     std::vector<CvPoint2>& cv_points) const override;

 private:
  int max_corners_;
  double quality_level_;
  double min_distance_;
};

class GlobalFeatureDetector : public FeatureDetectorBase {
 public:
  GlobalFeatureDetector(int max_corners, double quality_level,
                        double min_distance)
      : max_corners_{max_corners},
        quality_level_{quality_level},
        min_distance_{min_distance} {}

  void AddFeatures(const cv::Mat& image, std::vector<Corner>& corners);

 private:
  cv::Mat CreateMask(const cv::Mat& image, const std::vector<Corner>& corners);
  int max_corners_;
  double quality_level_;
  double min_distance_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_DETECTOR_H_
