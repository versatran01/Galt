#ifndef GALT_STEREO_VO_CORNER_DETECTOR_H_
#define GALT_STEREO_VO_CORNER_DETECTOR_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <set>
#include <utility>

namespace galt {

namespace stereo_vo {

using Grid = std::set<std::pair<int, int>>;

class CornerDetectorBase {
 public:
  virtual ~CornerDetectorBase() {}
  virtual void AddCorners(const cv::Mat& image,
                          std::vector<Corner>& corners) const = 0;

 protected:
  mutable Corner::Id cnt_{0};
};

class GridDetectorBase : public CornerDetectorBase {
 public:
  GridDetectorBase(int cell_size) : cell_size_{cell_size} {}

  void AddCorners(const cv::Mat& image, std::vector<Corner>& corners) const;
  double GridFilled(const cv::Mat& image,
                    const std::vector<Corner>& corners) const;
  virtual void DetectPoints(const cv::Mat& image, const cv::Mat& mask,
                            const int num_points,
                            std::vector<CvPoint2>& points) const = 0;

 protected:
  const cv::Mat CreateMask(const cv::Mat& image, const Grid& grid) const;
  const Grid CreateGrid(const std::vector<Corner>& corners) const;
  int cell_size_;
};

class GoodFeatureDetector : public GridDetectorBase {
 public:
  GoodFeatureDetector(int cell_size, int max_corners, double quality_level,
                      double min_distance)
      : GridDetectorBase(cell_size),
        max_corners_{max_corners},
        quality_level_{quality_level},
        min_distance_{min_distance} {}

 private:
  void DetectPoints(const cv::Mat& image, const cv::Mat& mask,
                    const int num_points,
                    std::vector<CvPoint2>& points) const override;
  int max_corners_;
  double quality_level_;
  double min_distance_;
};

class GlobalCornerDetector : public CornerDetectorBase {
 public:
  GlobalCornerDetector(int max_corners, double quality_level,
                       double min_distance)
      : max_corners_{max_corners},
        quality_level_{quality_level},
        min_distance_{min_distance} {}

  void AddCorners(const cv::Mat& image, std::vector<Corner>& corners);

 private:
  int max_corners_;
  double quality_level_;
  double min_distance_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_CORNER_DETECTOR_H_
