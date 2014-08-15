#ifndef GALT_STEREO_VO_CORNER_DETECTOR_H_
#define GALT_STEREO_VO_CORNER_DETECTOR_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <set>
#include <utility>

namespace galt {

namespace stereo_vo {

using Grid = std::set<std::pair<int, int>>;

static inline void MakeFeatureOld(std::vector<Feature>& features) {
  for (Feature& feature : features) feature.set_init(false);
}

class CornerDetectorBase {
 public:
  virtual ~CornerDetectorBase() {}
  /**
   * @brief AddFeatures Detect new corners and add them to features
   * @param image Input image
   * @param features A vector features
   * @return Number of new features added
   */
  virtual size_t AddFeatures(const cv::Mat& image,
                             std::vector<Feature>& features) const = 0;
  /**
   * @brief MakeFeatureOld Mark all features as old
   * @param features A vector of features
   */
};

class GridDetectorBase : public CornerDetectorBase {
 public:
  GridDetectorBase(int cell_size) : cell_size_{cell_size} {}

  size_t AddFeatures(const cv::Mat& image,
                     std::vector<Feature>& features) const;
  /**
   * @brief GridFilled Calculate percentage of grid being filled with corners
   * @param image Input image
   * @param features A vector of features
   * @return percentage of grid being filled
   */
  double GridFilled(const cv::Mat& image,
                    const std::vector<Feature>& features) const;
  /**
   * @brief DetectCorners Detect corners in image with mask
   * @param image Input image
   * @param mask ROI to detect corners
   * @param num_corners Number of corners to detect
   * @param corners Newly detected corners
   */
  virtual void DetectCorners(const cv::Mat& image, const cv::Mat& mask,
                             const int num_corners,
                             std::vector<CvPoint2>& corners) const = 0;

 protected:
  /**
   * @brief CreateMask Create a mask to detect corner in
   * @param image Input image
   * @param grid Current filled grid
   * @return An image mask
   */
  const cv::Mat CreateMask(const cv::Mat& image, const Grid& grid) const;
  /**
   * @brief CreateGrid Create grid based on feature distribution
   * @param features Features in current image
   * @return A grid
   */
  const Grid CreateGrid(const std::vector<Feature>& features) const;
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
  void DetectCorners(const cv::Mat& image, const cv::Mat& mask, int num_corners,
                     std::vector<CvPoint2>& corners) const override;
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

  size_t AddFeatures(const cv::Mat& image,
                     std::vector<Feature>& features) const override;

 private:
  int max_corners_;
  double quality_level_;
  double min_distance_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_CORNER_DETECTOR_H_
