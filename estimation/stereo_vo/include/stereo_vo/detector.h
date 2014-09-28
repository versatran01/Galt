#ifndef GALT_STEREO_VO_DETECTOR_H_
#define GALT_STEREO_VO_DETECTOR_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <set>
#include <utility>

namespace galt {
namespace stereo_vo {

using Grid = std::set<std::pair<int, int>>;

class FeatureDetector {
 public:
  const static int border_ = 25;

  FeatureDetector() : cell_size_(50) {}

  void set_cell_size(int cell_size) { cell_size_ = cell_size; }
  std::vector<CvPoint2> AddFeatures(const cv::Mat& image,
                                    std::vector<Feature>& features) const;

 private:
  int Discretize(scalar_t value) {
    return static_cast<int>(value / cell_size_);
  }

  Grid CreateGrid(const std::vector<Feature>& features) const;
  void DetectCorners(const cv::Mat& image, const Grid& grid,
                     std::vector<CvPoint2>& corners) const;

  int cell_size_;
};

void CornerSubPix(const cv::Mat& image, std::vector<CvPoint2>& corners);
bool IsCloseToImageBorder(const CvPoint2& point, const cv::Mat& image,
                          int border);
}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_DETECTOR_H_
