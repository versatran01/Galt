#include "stereo_vo/utils.h"

#include <deque>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

namespace galt {

namespace stereo_vo {

void Display(const CvStereoImage &stereo_image,
             const std::vector<Feature> &features,
             const FramePtr &key_frame) {
  /*
   auto &l_image = stereo_image.first;
   auto &r_image = stereo_image.second;
   auto &l_image_prev = key_frame.l_image();
   auto &r_image_prev = key_frame.r_image();
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
   cv::putText(display, "key frame", CvPoint2(offset_x, offset_y), font, scale,
               text_color, thickness);
   cv::putText(display, "current frame", CvPoint2(offset_x, n_rows + offset_y),
               font, scale, text_color, thickness);
   // How many matching features?
   std::ostringstream ss;
   ss << "C/F: " << tracked_corners.size() << "/" << key_frame.corners().size();
   cv::putText(display, ss.str(), CvPoint2(offset_x, n_rows * 2 - offset_y / 2),
               font, scale, text_color, thickness);
   //  ss.str(std::string());
   //  ss << "Features: " << key_frame.features().size();
   //  cv::putText(display, ss.str(), CvPoint2(offset_x, n_rows - offset_y / 2),
   //              font, scale, cv_color::YELLOW, thickness);

   // Draw currently tracked corners on current frame and key frame
   for (const Corner &corner : tracked_corners) {
     auto p = corner.p_pixel() + CvPoint2(0, n_rows);
     auto color = cv_color::RED;
     if (corner.init()) color = cv_color::MAGENTA;
     cv::circle(display, p, 2, color, 2);
     cv::circle(display, corner.p_pixel(), 1, color, 2);
   }

   // Draw key frame corners on key frame left
   const auto &kf_corners = key_frame.corners();
   for (const Corner &corner : kf_corners) {
     auto color = cv_color::GREEN;
     if (corner.init()) color = cv_color::ORANGE;
     cv::circle(display, corner.p_pixel(), 1, color, 2);
   }

   // Draw lines between corresponding corners
   for (const Corner &corner : tracked_corners) {
     auto id = corner.id();
     const auto it =
         std::find_if(kf_corners.cbegin(), kf_corners.cend(),
                      [id](const Corner &c) { return id == c.id(); });
     if (it != kf_corners.end()) {
       const CvPoint2 &p1 = it->p_pixel();
       const CvPoint2 &p2 = corner.p_pixel();
       cv::line(display, p1, p2, cv_color::YELLOW);
     }
   }

   // Display image
   cv::imshow("display", display);
   cv::waitKey(1);
   */
}

void Display(const FramePtr& frame, const FramePtr& key_frame) {}
}  // namespace stereo_vo

}  // namespace galt
