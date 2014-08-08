#ifndef GALT_STEREO_VO_UTILS_H_
#define GALT_STEREO_VO_UTILS_H_

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

struct CvColor {
  cv::Scalar blue = cv::Scalar(255, 0, 0);
  cv::Scalar green = cv::Scalar(0, 255, 0);
  cv::Scalar red = cv::Scalar(0, 0, 255);
  cv::Scalar yellow = cv::Scalar(0, 255, 255);
  cv::Scalar magenta = cv::Scalar(255, 0, 255);
} cv_color;

}  // namespace stereo_vo

}  // namespace galt

#endif // GALT_STEREO_VO_UTILS_H_
