#include "stereo_vo/feature_detector.h"
#include "stereo_vo/feature.h"

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

void GoodFeatureDetector::DetectFeatures(const cv::Mat &image, Features &features) const {
  Corners2 corners;
  // Create a grid
  Grid grid = CreateGrid(image, features);
  // Iterate through the grid
  for (const auto &cell: grid) {
    // Find unfilled cell
    if (!cell.second()) {
      cv::Mat cell_image = image(cv::Range(), cv::Range());
      DetectGoodFeatures(cell_image, corners);
      if (!corners.empty()) {
        auto p = corners.front();
        int x = cell.first().first();
        int y = cell.first().second();
        p.x += x * cell_size_;
        p.y += y * cell_size_;

        Feature

      }
    }
  }

}
}  // namespace stereo_vo

}  // namespace galt
