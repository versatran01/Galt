#include "stereo_vo/corner_detector.h"
#include "stereo_vo/feature.h"

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {

namespace stereo_vo {

const Grid GridDetectorBase::CreateGrid(
    const std::vector<Corner> &corners) const {
  Grid grid;
  // Marked filled grid
  for (const Corner &corner : corners) {
    const int x = static_cast<int>(corner.p_pixel().x / cell_size_);
    const int y = static_cast<int>(corner.p_pixel().y / cell_size_);
    grid.emplace(x, y);
  }
  return grid;
}

double GridDetectorBase::GridFilled(const cv::Mat &image,
                                    const std::vector<Corner> &corners) const {
  const Grid grid = CreateGrid(corners);
  auto grid_cols = static_cast<int>(image.cols / cell_size_);
  auto grid_rows = static_cast<int>(image.rows / cell_size_);
  return static_cast<double>(grid.size()) / (grid_cols * grid_rows);
}

const cv::Mat GridDetectorBase::CreateMask(const cv::Mat &image,
                                           const Grid &grid) const {
  // Initialize the mask to empty
  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
  auto grid_cols = static_cast<int>(image.cols / cell_size_);
  auto grid_rows = static_cast<int>(image.rows / cell_size_);
  for (int y = 0; y < grid_rows; ++y) {
    for (int x = 0; x < grid_cols; ++x) {
      // If a cell does not have any corners, fill that part of the mask
      if (grid.find(std::make_pair(x, y)) == grid.end()) {
        mask(cv::Rect(x * cell_size_, y * cell_size_, cell_size_, cell_size_)) =
            255;
      }
    }
  }
  return mask;
}

void GridDetectorBase::AddCorners(const cv::Mat &image,
                                  std::vector<Corner> &corners) const {
  // Mark all tracked corners as old
  for (Corner &corner : corners) corner.set_init(false);
  // Create a new grid
  const Grid grid = CreateGrid(corners);
  // Initialize a mask
  const cv::Mat mask = CreateMask(image, grid);
  // Detect corners only in mask
  std::vector<CvPoint2> points;
  DetectPoints(image, mask, corners.size(), points);
  // Add newly detected points to corners with a unique id
  for (const CvPoint2 &point : points) {
    corners.emplace_back(cnt_++, point, true);
  }
}

void GoodFeatureDetector::DetectPoints(const cv::Mat &image,
                                       const cv::Mat &mask,
                                       const int num_corners,
                                       std::vector<CvPoint2> &points) const {
  if (max_corners_ < num_corners) return;
  cv::goodFeaturesToTrack(image, points, max_corners_ - num_corners,
                          quality_level_, min_distance_, mask);
}

void GlobalCornerDetector::AddCorners(const cv::Mat &image,
                                      std::vector<Corner> &corners) const {
  //  mark old features as old
  for (Corner &corner : corners) corner.set_init(false);

  std::vector<CvPoint2> points;
  auto num_corners = max_corners_ - corners.size();
  if (num_corners < 1) return;
  cv::goodFeaturesToTrack(image, points, num_corners, quality_level_,
                          min_distance_);

  for (const CvPoint2 &point : points) {
    corners.emplace_back(cnt_++, point, true);
  }
}

}  // namespace stereo_vo

}  // namespace galt
