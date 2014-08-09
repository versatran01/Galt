#ifndef GALT_STEREO_VO_COMMON_H_
#define GALT_STEREO_VO_COMMON_H_

#include <ros/ros.h>
#include "kr_math/pose.hpp"
#include <opencv2/core/core.hpp>
#include "stereo_vo/StereoVoDynConfig.h"

namespace galt {
namespace stereo_vo {

using scalar_t = float;
using Pose = kr::Pose<scalar_t>;
using CvPoint2 = cv::Point_<scalar_t>;
using CvPoint3 = cv::Point3_<scalar_t>;
using CvCorners2 = std::vector<CvPoint2>;
using StereoVoConfig = ::stereo_vo::StereoVoDynConfig;

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_COMMON_H_