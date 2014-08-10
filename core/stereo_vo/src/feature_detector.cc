#include "stereo_vo/feature_detector.h"
#include "stereo_vo/feature.h"

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>

namespace galt {

namespace stereo_vo {

const Grid FeatureDetectorBase::CreateGrid(
    const cv::Mat &image, const std::vector<Feature> &features) const {
  Grid grid;
  // Marked filled grid
  for (const auto &feature : features) {
    const int x = static_cast<int>(feature.p_pixel_left().x / cell_size_);
    const int y = static_cast<int>(feature.p_pixel_left().y / cell_size_);
    grid.insert(std::pair<int, int>(x, y));
  }

  return grid;
}

void FeatureDetectorBase::AddFeatures(const cv::Mat &image,
                                      std::vector<Feature> &features) const {
  std::vector<CvPoint2> corners;
  // Create a grid
  Grid grid = CreateGrid(image, features);
  const int grid_dim_x = static_cast<int>(image.cols / cell_size_);
  const int grid_dim_y = static_cast<int>(image.rows / cell_size_);
  // Don't add features if some percentage of the grid is filled
  double k = 0.7;
  if (grid.size() > (k * grid_dim_x * grid_dim_y)) return;
  // Iterate through each dimension of the grid
  for (int y = 0; y < grid_dim_y; ++y) {
    for (int x = 0; x < grid_dim_x; ++x) {
      // Find unfilled cell
      if (grid.find(std::pair<int, int>(x, y)) == grid.end()) {
        // Extract image of that cell to detect features
        cv::Mat cell_image =
            image(cv::Range(y * cell_size_, (y + 1) * cell_size_),
                  cv::Range(x * cell_size_, (x + 1) * cell_size_));
        DetectFeatures(cell_image, corners);
        // Add to existing features
        if (!corners.empty()) {
          for (auto corner : corners) {
            corner.x += x * cell_size_;
            corner.y += y * cell_size_;

            //  create a new feature here...
            // Feature feature(corner);
            // features.push_back(feature);
          }
        }
      }
    }
  }

  /// call feature.triangulate() here for each feature
}

void GoodFeatureDetector::DetectFeatures(const cv::Mat &image,
                                         std::vector<CvPoint2> &corners) const {
  cv::goodFeaturesToTrack(image, corners, max_corners_, quality_level_,
                          min_distance_);
}

}  // namespace stereo_vo

}  // namespace galt
