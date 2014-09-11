#include "stereo_vo/feature_detector.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/utils.h"

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {
namespace stereo_vo {

Grid FeatureDetector::CreateGrid(const std::vector<Feature> &features) const {
  Grid grid;
  for (const Feature &feature : features) {
    const int x = static_cast<int>(feature.p_pixel().x / cell_size_);
    const int y = static_cast<int>(feature.p_pixel().y / cell_size_);
    grid.emplace(x, y);
  }
  return grid;
}

size_t FeatureDetector::AddFeatures(const cv::Mat &image,
                                    std::vector<Feature> &features) const {
  // Mark all tracked corners as old
  MakeFeaturesOld(features);
  // Create a new grid
  const Grid grid = CreateGrid(features);
  std::vector<CvPoint2> corners;
  DetectCorners(image, grid, corners);
  CornerSubPix(image, corners);

  for (const CvPoint2 &corner : corners) {
    features.emplace_back(corner);
  }
  return corners.size();
}

void FeatureDetector::DetectCorners(const cv::Mat &image, const Grid &grid,
                                    std::vector<CvPoint2> &corners) const {
  const auto grid_cols = static_cast<int>(image.cols / cell_size_);
  const auto grid_rows = static_cast<int>(image.rows / cell_size_);
  // Detect one corner in each empty grid
  for (int y = 0; y < grid_rows; ++y) {
    for (int x = 0; x < grid_cols; ++x) {
      if (grid.find(std::make_pair(x, y)) == grid.cend()) {
        cv::Mat sub_image =
            image(cv::Range(y * cell_size_, (y + 1) * cell_size_),
                  cv::Range(x * cell_size_, (x + 1) * cell_size_));
        std::vector<CvPoint2> new_corner;
        cv::goodFeaturesToTrack(sub_image, new_corner, 1, 0.001,
                                cell_size_ / 3);
        if (!new_corner.empty()) {
          CvPoint2 &corner = new_corner.front();
          corner.x += x * cell_size_;
          corner.y += y * cell_size_;
          // Corner should never be too close to image border
          if (corner.x > border_ && corner.y > border_ &&
              corner.x < (image.cols - border_) &&
              corner.y < (image.rows - border_)) {
            corners.emplace_back(corner.x, corner.y);
          }
        }
      }
    }
  }
}

void CornerSubPix(const cv::Mat &image, std::vector<CvPoint2> &corners) {
  cv::TermCriteria criteria =
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
  /// Calculate the refined corner locations
  cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
}

}  // namespace stereo_vo
}  // namespace galt
