#include "stereo_vo/utils.h"

#include <deque>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

namespace galt {
namespace stereo_vo {

void Display(const cv::Mat &image,
             const std::vector<Feature> &keyframe_features,
             const std::vector<Feature> &tracked_features) {
  cv::namedWindow("display",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  // Convert to color
  cv::Mat display;
  cv::cvtColor(image, display, CV_GRAY2BGR);

  // Draw tracked features
  DrawFeatures(display, tracked_features, cv_color::GREEN);
  DrawCorrespondence(display, keyframe_features, tracked_features,
                     cv_color::YELLOW);

  // Draw annotations
  AnnotateFeatureCounts(display, keyframe_features, cv_color::RED, 1);
  AnnotateFeatureCounts(display, tracked_features, cv_color::GREEN, 4);

  // Display
  cv::imshow("display", display);
  cv::waitKey(1);
}

void DrawFeatures(cv::Mat &image, const std::vector<Feature> &features,
                  const cv::Scalar &color) {
  std::for_each(features.cbegin(), features.cend(),
                [&](const Feature &feature) {
    cv::circle(image, feature.p_pixel(), 1, color, 2);
  });
}

void DrawCorrespondence(cv::Mat &image, const std::vector<Feature> &features1,
                        const std::vector<Feature> &features2,
                        const cv::Scalar &color) {
  for (const Feature &f1 : features1) {
    const auto it =
        std::find_if(features2.cbegin(), features2.cend(),
                     [&f1](const Feature &f2) { return f1.id() == f2.id(); });
    if (it != features2.end()) {
      cv::line(image, f1.p_pixel(), it->p_pixel(), color);
    }
  }
}

void AnnotateFeatureCounts(cv::Mat &image, const std::vector<Feature> &features,
                           const cv::Scalar &color, int quadrant) {

  scalar_t offset_x = 15;
  scalar_t offset_y = 30;
  scalar_t k = 5;
  quadrant = quadrant < 1 ? 1 : (quadrant > 4 ? 4 : quadrant);
  static std::vector<CvPoint2> positions = {
      {image.cols - k * offset_x, offset_y},
      {offset_x, offset_y},
      {offset_x, image.rows - offset_y},
      {image.cols - k * offset_x, image.rows - offset_y}};
  cv::putText(image, std::to_string(features.size()), positions[quadrant - 1],
              cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
}

/*
void Display(const CvStereoImage &stereo_image,
             const std::vector<Feature> &features, const FramePtr &key_frame)
{
  auto &l_image = stereo_image.first;
  auto &r_image = stereo_image.second;
  auto &l_image_prev = key_frame->l_image();
  auto &r_image_prev = key_frame->r_image();
  int n_rows = l_image.rows;
  int n_cols = l_image.cols;
  static cv::Mat display_gray(2 * n_rows, 2 * n_cols, CV_8UC1);

  // Copy 2 frames 4 images on to one display
  l_image_prev.copyTo(display_gray(cv::Rect(0, 0, n_cols, n_rows)));
  r_image_prev.copyTo(display_gray(cv::Rect(n_cols, 0, n_cols, n_rows)));
  l_image.copyTo(display_gray(cv::Rect(0, n_rows, n_cols, n_rows)));
  r_image.copyTo(display_gray(cv::Rect(n_cols, n_rows, n_cols, n_rows)));

  // Convert to color
  cv::Mat display;
  cv::cvtColor(display_gray, display, CV_GRAY2BGR);

  // Add text annotation
  double offset_x = 10.0, offset_y = 30.0;
  auto font = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 1.0, thickness = 2.0;
  auto text_color = cv_color::CYAN;
  // Which frame?
  cv::putText(display, "current frame", CvPoint2(offset_x, n_rows + offset_y),
              font, scale, text_color, thickness);
  // How many matching features?
  std::ostringstream ss;
  ss << "C/F: " << features.size() << "/" << key_frame->features().size();
  cv::putText(display, ss.str(), CvPoint2(offset_x, n_rows * 2 - offset_y /
2),
              font, scale, text_color, thickness);
  //  ss.str(std::string());
  //  ss << "Features: " << key_frame.features().size();
  //  cv::putText(display, ss.str(), CvPoint2(offset_x, n_rows - offset_y /
2),
  //              font, scale, cv_color::YELLOW, thickness);

  // Draw currently tracked features on current frame and key frame
  for (const Feature &feature : features) {
    auto p = feature.p_pixel() + CvPoint2(0, n_rows);
    auto color = cv_color::RED;
    if (feature.init()) color = cv_color::MAGENTA;
    cv::circle(display, p, 1, color, 2);
    cv::circle(display, feature.p_pixel(), 1, color, 2);
  }

  // Draw key frame features on key frame left
  const auto &kf_features = key_frame->features();
  for (const Feature &feature : kf_features) {
    auto color = cv_color::GREEN;
    if (feature.init()) color = cv_color::ORANGE;
    cv::circle(display, feature.p_pixel(), 1, color, 2);
  }

  // Draw lines between corresponding features
  for (const Feature &feature : features) {
    auto id = feature.id();
    const auto it =
        std::find_if(kf_features.cbegin(), kf_features.cend(),
                     [id](const Feature &f) { return id == f.id(); });
    if (it != kf_features.end()) {
      const CvPoint2 &p1 = it->p_pixel();
      const CvPoint2 &p2 = feature.p_pixel();
      cv::line(display, p1, p2, cv_color::YELLOW);
    }
  }

  // Display image
  cv::imshow("display", display);
  cv::waitKey(1);
}

}  // namespace stereo_vo
}  // namespace galt
