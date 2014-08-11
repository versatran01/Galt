#include "stereo_vo/feature_detector.h"
#include "stereo_vo/feature.h"

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

namespace galt {

namespace stereo_vo {

const Grid FeatureDetectorBase::CreateGrid(
    const std::vector<Corner> &corners) const {
  Grid grid;
  // Marked filled grid
  for (const auto &corner : corners) {
    const int x = static_cast<int>(corner.p_pixel().x / cell_size_);
    const int y = static_cast<int>(corner.p_pixel().y / cell_size_);
    grid.emplace(x, y);
  }
  return grid;
}

double FeatureDetectorBase::GridFilled(const cv::Mat &image,
                                       const std::vector<Corner> &corners) {
  grid_ = CreateGrid(corners);
  grid_cols_ = static_cast<int>(image.cols / cell_size_);
  grid_rows_ = static_cast<int>(image.rows / cell_size_);
  return static_cast<double>(grid_.size()) / (grid_cols_ * grid_rows_);
}

void FeatureDetectorBase::AddFeatures(const cv::Mat &image,
                                      std::vector<Corner> &corners) {
  // Create a new grid if necessary
  if (grid_.empty()) grid_ = CreateGrid(corners);
  // Mark all tracked corners as old
  for (auto &corner : corners) corner.set_init(false);
  // Iterate through each dimension of the grid
  for (int y = 0; y < grid_rows_; ++y) {
    for (int x = 0; x < grid_cols_; ++x) {
      // Find unfilled cells
      if (grid_.find(std::make_pair(x, y)) == grid_.end()) {
        std::vector<CvPoint2> new_points;
        // Extract image of that cell to detect corners
        cv::Mat cell_image =
            image(cv::Range(y * cell_size_, (y + 1) * cell_size_),
                  cv::Range(x * cell_size_, (x + 1) * cell_size_));
        DetectCorners(cell_image, new_points);
        // Add new corners to existing corners
        for (auto &point : new_points) {
          point.x += x * cell_size_;
          point.y += y * cell_size_;
          corners.emplace_back(cnt_++, point, true);
        }
      }
    }
  }
  grid_.clear();
}

void GoodFeatureDetector::DetectCorners(
    const cv::Mat &image, std::vector<CvPoint2> &cv_points) const {
  cv::goodFeaturesToTrack(image, cv_points, max_corners_, quality_level_,
                          min_distance_);
}

}  // namespace stereo_vo

}  // namespace galt
