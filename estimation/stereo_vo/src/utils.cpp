#include "stereo_vo/utils.h"

#include <deque>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

namespace galt {
namespace stereo_vo {
void Display(const CvStereoImage &stereo_image,
             const std::vector<Feature> &tracked_features,
             const KeyFrame &key_frame) {
  cv::namedWindow("display",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  Display(key_frame.l_image(), key_frame.r_image(), stereo_image.first,
          stereo_image.second, key_frame.features(), key_frame.r_corners(),
          tracked_features);
}

void Display(const cv::Mat &l_image_prev, const cv::Mat &r_image_prev,
             const cv::Mat &l_image, const cv::Mat &r_image,
             const cv::vector<Feature> &keyframe_features,
             const cv::vector<CvPoint2> &r_corners,
             const cv::vector<Feature> &tracked_features) {
  int n_rows = l_image_prev.rows;
  int n_cols = l_image_prev.cols;
  cv::Mat display_gray(2 * n_rows, 2 * n_cols, CV_8UC1);

  // Copy 2 frames 4 iamges on to one display
  l_image_prev.copyTo((display_gray(cv::Rect(0, 0, n_cols, n_rows))));
  r_image_prev.copyTo(display_gray(cv::Rect(n_cols, 0, n_cols, n_rows)));
  l_image.copyTo(display_gray(cv::Rect(0, n_rows, n_cols, n_rows)));
  r_image.copyTo(display_gray(cv::Rect(n_cols, n_rows, n_cols, n_rows)));

  // Convert to color
  cv::Mat display;
  cv::cvtColor(display_gray, display, CV_GRAY2BGR);

  // Draw stuff on display
  DrawFeatures(display, keyframe_features, cv_color::GREEN);
  DrawFeatures(display, tracked_features, cv_color::GREEN, CvPoint2(0, n_rows));
  DrawCorners(display, r_corners, cv_color::ORANGE, CvPoint2(n_cols, 0));
  DrawStereoMatch(display, keyframe_features, r_corners, cv_color::YELLOW,
                  CvPoint2(n_cols, 0));

  DrawCorrespondence(display, keyframe_features, tracked_features,
                     cv_color::CYAN, CvPoint2(0, n_rows));

  // Show image
  cv::imshow("display", display);
  cv::waitKey(1);
}

void DrawStereoMatch(cv::Mat &image, const std::vector<Feature> &features,
                     const std::vector<CvPoint2> &corners,
                     const cv::Scalar &color, const CvPoint2 &offset) {
  auto fit = std::find_if(features.cbegin(), features.cend(),
                          [](const Feature &f) { return f.fresh; });
  for (const CvPoint2 &c : corners) {
    const Feature &f = *fit++;
    cv::line(image, f.px, c + offset, color);
  }
}

void DrawCorners(cv::Mat &image, const std::vector<CvPoint2> &corners,
                 const cv::Scalar &color, const CvPoint2 &offset) {
  std::for_each(corners.cbegin(), corners.cend(), [&](const CvPoint2 &c) {
    cv::circle(image, c + offset, 1, color, 2);
  });
}

void DrawFeatures(cv::Mat &image, const std::vector<Feature> &features,
                  cv::Scalar color, const CvPoint2 &offset) {
  std::for_each(features.cbegin(), features.cend(), [&](const Feature &f) {
    if (f.fresh) color = cv_color::RED;
    cv::circle(image, f.px + offset, 1, color, 2);
  });
}

void DrawCorrespondence(cv::Mat &image, const std::vector<Feature> &features1,
                        const std::vector<Feature> &features2,
                        const cv::Scalar &color, const CvPoint2 &offset) {
  for (const Feature &f1 : features1) {
    const auto it =
        std::find_if(features2.cbegin(), features2.cend(),
                     [&f1](const Feature &f2) { return f1.id == f2.id; });
    if (it != features2.end()) {
      const Feature &f2 = *it;
      cv::line(image, f1.px, f2.px + offset, color);
    }
  }
}

/*
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
*/

}  // namespace stereo_vo
}  // namespace galt
