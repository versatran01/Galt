#include "stereo_vo/corner_detector.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/utils.h"

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace galt {

namespace stereo_vo {

const Grid GridDetectorBase::CreateGrid(
    const std::vector<Feature> &features) const {
  Grid grid;
  // Marked filled grid
  for (const Feature &feature : features) {
    const int x = static_cast<int>(feature.p_pixel().x / cell_size_);
    const int y = static_cast<int>(feature.p_pixel().y / cell_size_);
    grid.emplace(x, y);
  }
  return grid;
}

double GridDetectorBase::GridFilled(
    const cv::Mat &image, const std::vector<Feature> &features) const {
  const Grid grid = CreateGrid(features);
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
  //  cv::imshow("mask", mask);
  //  cv::waitKey(1);
  return mask;
}

size_t GridDetectorBase::AddFeatures(const cv::Mat &image,
                                     std::vector<Feature> &features) const {
  // Mark all tracked corners as old
  MakeFeatureOld(features);
  // Create a new grid
  const Grid grid = CreateGrid(features);
  // Initialize a mask
  const cv::Mat mask = CreateMask(image, grid);
  // Detect corners only in mask
  std::vector<CvPoint2> corners;
  DetectCorners(image, mask, features.size(), corners);
  CornerSubPix(image, corners);

  // Add newly detected corners to features
  ROS_INFO("Adding %lu corners", corners.size());
  for (const CvPoint2 &corner : corners) features.emplace_back(corner);
  return corners.size();
}

void GoodFeatureDetector::DetectCorners(const cv::Mat &image,
                                        const cv::Mat &mask, int num_corners,
                                        std::vector<CvPoint2> &corners) const {
  // Return if we have enough
  if (max_corners_ <= num_corners) return;
  cv::goodFeaturesToTrack(image, corners, max_corners_ - num_corners,
                          quality_level_, min_distance_, mask);
}

}  // namespace stereo_vo

}  // namespace galt
