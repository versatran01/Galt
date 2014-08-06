#include "stereo_vo/stereo_vo.h"

#include <sstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

namespace galt {

namespace stereo_vo {

StereoVo::StereoVo() {}

void StereoVo::Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                          const StereoCameraModel &model) {
  model_ = model;
  key_frame_.Update(l_image, r_image, config_);

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
  key_frame_.Update(l_image, r_image, config_);
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

  // Draw good features on key frame
  for (const auto &p : key_frame_.l_features_) {
    cv::circle(two_frame_color, p, 6, cv::Scalar(0, 0, 255), 2);
  }
  auto feature_color = cv::Scalar(0, 255, 0);
  auto line_color = cv::Scalar(255, 0, 0);
  // Draw matching features on key frame
  for (unsigned i = 0; i != key_frame_.status_.size(); ++i) {
    if (key_frame_.status_[i]) {
      auto l_p = key_frame_.l_features_[i];
      auto r_p = key_frame_.r_features_[i];
      r_p = r_p + cv::Point2f(n_cols, 0);
      cv::circle(two_frame_color, l_p, 3, feature_color, 2);
      cv::circle(two_frame_color, r_p, 3, feature_color, 2);
      cv::line(two_frame_color, l_p, r_p, line_color, 1);
    }
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
              cv::Point2f(offset_x, n_rows - offset_y / 2), font, scale,
              text_color, thickness);
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
                      const StereoVoDynConfig &config) {
  // Collect relevant options
  int max_iter = 25;
  double epsilon = 0.01;
  int win_size = config.win_size;
  int max_level = config.max_level;
  int num_features = config.num_features;

  l_image_ = l_image;
  r_image_ = r_image;
  cv::goodFeaturesToTrack(l_image, l_features_, num_features, 0.01, 10);
  // Some temporary settings
  cv::TermCriteria term_criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iter, epsilon);
  cv::calcOpticalFlowPyrLK(l_image_, r_image_, l_features_, r_features_,
                           status_, cv::noArray(), cv::Size(win_size, win_size),
                           max_level, term_criteria);
}

}  // namespace stereo_vo

}  // namespace galt
