#include "stereo_vo/stereo_vo.h"

#include <sstream>

#include "opencv2/highgui/highgui.hpp"

namespace galt {

namespace stereo_vo {

StereoVo::StereoVo() {}

void StereoVo::Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                          const StereoCameraModel &model) {
  model_ = model;
  key_frame_.Update(l_image, r_image, config_.num_features);

  std::cout << "StereoVo initialized, baseline: " << model_.baseline()
            << std::endl;

  // Create a window for display
  cv::namedWindow("two_frame",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  Display(l_image, r_image);
  init_ = true;
  return;
}

void StereoVo::Iterate(const cv::Mat &l_image, const cv::Mat &r_image) {
  Display(l_image, r_image);
  // Save the new images
//  key_frame_.Update(l_image, r_image);
}

void StereoVo::Display(const cv::Mat &l_image, const cv::Mat &r_image) {
  int n_rows = l_image.rows;
  int n_cols = l_image.cols;
  static cv::Mat two_frame(2 * n_rows, 2 * n_cols, CV_8UC1);

  // Copy 2 frames 4 images on to one image
  key_frame_.l_image_.copyTo(two_frame(cv::Rect(0, 0, n_cols, n_rows)));
  key_frame_.r_image_.copyTo(two_frame(cv::Rect(n_cols, 0, n_cols, n_rows)));
  l_image.copyTo(two_frame(cv::Rect(0, n_rows, n_cols, n_rows)));
  r_image.copyTo(two_frame(cv::Rect(n_cols, n_rows, n_cols, n_rows)));

  // Convert to color
  cv::Mat two_frame_color;
  cv::cvtColor(two_frame, two_frame_color, CV_GRAY2BGR);

  // Draw features on key frame
  double radius = 5.0;
  for (const auto &p : key_frame_.features_) {
    cv::circle(two_frame_color, p, radius, cv::Scalar(0, 255, 0), 2);
  }

  // Add text annotation
  double offset_x = 10.0, offset_y = 30.0;
  auto font = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 1.0, thickness = 2.0;
  auto text_color = cv::Scalar(255, 0, 0);
  // Which frame?
  cv::putText(two_frame_color, "key frame", cv::Point2f(offset_x, offset_y),
              font, scale, text_color, thickness);
  cv::putText(two_frame_color, "current frame",
              cv::Point2f(offset_x, n_rows + offset_y), font, scale, text_color,
              thickness);
  // How many matching features?
  std::ostringstream ss;
  ss << config_.min_features;
  cv::putText(two_frame_color, ss.str(),
              cv::Point2f(offset_x, n_rows - offset_y / 2), font, scale, text_color,
              thickness);
  ss.str(std::string());
  ss << config_.num_features;
  cv::putText(two_frame_color, ss.str(),
              cv::Point2f(n_cols + offset_x, 2 * n_rows - offset_y / 2), font,
              scale, text_color, thickness);

  // Display image
  cv::imshow("two_frame", two_frame_color);
  cv::waitKey(1);
}

void KeyFrame::Update(const cv::Mat &l_image, const cv::Mat &r_image,
                      const int num_feautres) {
  l_image_ = l_image;
  r_image_ = r_image;
  cv::goodFeaturesToTrack(l_image, features_, num_feautres, 0.01, 10);
  std::cout << features_.size() << std::endl;
}

}  // namespace stereo_vo

}  // namespace galt
