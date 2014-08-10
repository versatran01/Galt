#ifndef GALT_STEREO_VO_UTILS_H_
#define GALT_STEREO_VO_UTILS_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/key_frame.h"

#include <deque>

namespace galt {

namespace stereo_vo {

namespace cv_color {
const cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar RED = cv::Scalar(0, 0, 255);
const cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar ORANGE = cv::Scalar(0, 128, 255);
const cv::Scalar MAGENTA = cv::Scalar(255, 0, 255);
}

template <typename T, typename U>
void PruneByStatus(const std::vector<U> &status, std::vector<T> &objects) {
  ROS_ASSERT_MSG(status.size() == objects.size(),
                 "status and object size mismatch");
  auto it_obj = objects.begin();
  for (const auto &s : status) {
    if (s) {
      it_obj++;
    } else {
      it_obj = objects.erase(it_obj);
    }
  }
}

void Display(const cv::Mat &l_image_prev, const cv::Mat &l_image,
             const cv::Mat &r_image_prev, const cv::Mat &r_image,
             const std::vector<Feature> &features,
             const std::deque<KeyFrame> &key_frames);

static inline CvPoint2 PixelToCoord(double fx, double fy, double cx, double cy,
                                    const CvPoint2 &pixel) {
  return CvPoint2((pixel.x - cx) / fx, (pixel.y - cy) / fy);
}

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_UTILS_H_
