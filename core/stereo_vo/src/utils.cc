#include "stereo_vo/utils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace galt {

namespace stereo_vo {

void Display(const cv::Mat &l_image_prev, const cv::Mat &l_image,
             const cv::Mat &r_image_prev, const cv::Mat &r_image,
             const Features &features) {
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

  // Draw features with different color code
  // newly added (pixel_next, not triangulated - red)
  // triangulated (pixel_left, pixel_right, pixel_next - orange)
  // optimized (pixel_left, pixel-right, pixel_next, - green)
  for (const auto &feature : features) {
    auto l_p = feature.p_pixel_left();
    auto r_p = feature.p_pixel_right() + CvPoint2(n_cols, 0);
    auto n_p = feature.p_pixel_next() + CvPoint2(0, n_rows);

    if (!feature.triangulated()) {  // newly added
      cv::circle(display, n_p, 3, cv_color::RED, 2);
    } else {
      cv::Scalar color;
      if (feature.ready()) {  // optimized
        color = cv_color::GREEN;
      } else {  // triangulated
        color = cv_color::ORANGE;
      }
      cv::circle(display, l_p, 3, color, 2);
      cv::circle(display, r_p, 3, color, 2);
      cv::circle(display, n_p, 3, color, 2);
      cv::line(display, l_p, r_p, color, 1);
      cv::line(display, l_p, n_p, color, 1);
    }
  }

  // Add text annotation
  double offset_x = 10.0, offset_y = 30.0;
  auto font = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 1.0, thickness = 2.0;
  // Which frame?
  cv::putText(display, "previous frame", CvPoint2(offset_x, offset_y), font,
              scale, cv_color::YELLOW, thickness);
  cv::putText(display, "current frame",
              CvPoint2(n_cols + offset_x, n_rows + offset_y), font, scale,
              cv_color::YELLOW, thickness);
  // How many matching features?
  std::ostringstream ss;
  ss << features.size();
  cv::putText(display, ss.str(), CvPoint2(offset_x, n_rows - offset_y / 2),
              font, scale, cv_color::YELLOW, thickness);

  // Display image
  cv::imshow("display", display);
  cv::waitKey(1);
}

}  // namespace stereo_vo

}  // namespace galt
