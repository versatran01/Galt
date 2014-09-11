#ifndef GALT_STEREO_VO_FEATURE_DETECTOR_H_
#define GALT_STEREO_VO_FEATURE_DETECTOR_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <set>
#include <utility>

namespace galt {
namespace stereo_vo {

using Grid = std::set<std::pair<int, int>>;

class FeatureDetector {
 public:
  FeatureDetector() : cell_size_(40) {}

  void set_cell_size(int cell_size) { cell_size_ = cell_size; }
  size_t AddFeatures(const cv::Mat& image,
                     std::vector<Feature>& features) const;

 private:
  Grid CreateGrid(const std::vector<Feature>& features) const;
  void DetectCorners(const cv::Mat& image, const Grid& grid,
                     std::vector<CvPoint2>& corners) const;

  const static int border_ = 25;
  int cell_size_;
};

void CornerSubPix(const cv::Mat& image, std::vector<CvPoint2>& corners);

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_DETECTOR_H_
