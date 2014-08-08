#include "stereo_vo/feature_detector.h"
#include "stereo_vo/feature.h"

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

namespace galt {

namespace stereo_vo {

const Grid FeatureDetectorBase::CreateGrid(const cv::Mat &image,
                                           const Features &features) const {
  Grid grid;
  // Marked filled grid
  for (const auto &feature : features) {
    const int x = static_cast<int>(feature.p_pixel_left().x / cell_size_);
    const int y = static_cast<int>(feature.p_pixel_left().y / cell_size_);
    grid[std::pair<int, int>(x, y)] = true;
  }

  return grid;
}

void GoodFeatureDetector::DetectFeatures(const cv::Mat &image,
                                         Features &features) const {
  Corners2 corners;
  // Create a grid
  Grid grid = CreateGrid(image, features);
  // Iterate through the grid
  for (const auto &cell : grid) {
    // Find unfilled cell
    if (!cell.second) {
      int x = cell.first.first;
      int y = cell.first.second;
      // Extract image of that cell to detect features
      cv::Mat cell_image =
          image(cv::Range(y * cell_size_, (y + 1) * cell_size_),
                cv::Range(x * cell_size_, (x + 1) * cell_size_));
      cv::goodFeaturesToTrack(cell_image, corners, max_corners_, quality_level_,
                              min_distance_);
      // Add to existing features
      if (!corners.empty()) {
        auto p = corners.front();
        p.x += x * cell_size_;
        p.y += y * cell_size_;

        Feature feature(p);
        features.push_back(feature);
      }
    }
  }
}

}  // namespace stereo_vo

}  // namespace galt
