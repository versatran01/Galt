#include "stereo_vo/feature_detector.h"
#include "stereo_vo/feature.h"

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

namespace galt {

namespace stereo_vo {

const Grid FeatureDetectorBase::CreateGrid(
    const std::vector<Feature> &features) const {
  Grid grid;
  // Marked filled grid
  for (const auto &feature : features) {
    const int x = static_cast<int>(feature.p_pixel_left().x / cell_size_);
    const int y = static_cast<int>(feature.p_pixel_left().y / cell_size_);
    grid.emplace(x, y);
  }
  return grid;
}

double FeatureDetectorBase::GridFilled(const cv::Mat &image,
                                       const std::vector<Feature> &features) {
  grid_ = CreateGrid(features);
  grid_cols_ = static_cast<int>(image.cols / cell_size_);
  grid_rows_ = static_cast<int>(image.rows / cell_size_);
  return static_cast<double>(grid_.size() / (grid_cols_ * grid_rows_));
}

void FeatureDetectorBase::AddFeatures(const cv::Mat &image,
                                      std::vector<Feature> &features) {
  if (grid_.empty()) grid_ = CreateGrid(features);
  // Mark all tracked features as not init
  for (auto &feature : features) feature.set_init(false);
  // Iterate through each dimension of the grid
  for (int y = 0; y < grid_rows_; ++y) {
    for (int x = 0; x < grid_cols_; ++x) {
      // Find unfilled cell
      if (grid_.find(std::make_pair(x, y)) == grid_.end()) {
        std::vector<CvPoint2> corners;
        // Extract image of that cell to detect features
        cv::Mat cell_image =
            image(cv::Range(y * cell_size_, (y + 1) * cell_size_),
                  cv::Range(x * cell_size_, (x + 1) * cell_size_));
        DetectCorners(cell_image, corners);
        // Add to existing features
        for (auto &corner : corners) {
          corner.x += x * cell_size_;
          corner.y += y * cell_size_;
          features.emplace_back(cnt_++, corner, true);
        }
      }
    }
  }

  // Track features in stereo image

  // call feature.triangulate() here for each feature
  grid_.clear();
}

void GoodFeatureDetector::DetectCorners(const cv::Mat &image,
                                        std::vector<CvPoint2> &corners) const {
  cv::goodFeaturesToTrack(image, corners, max_corners_, quality_level_,
                          min_distance_);
}

}  // namespace stereo_vo

}  // namespace galt
