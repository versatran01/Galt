#ifndef GALT_STEREO_VO_COMMON_H_
#define GALT_STEREO_VO_COMMON_H_

#include <ros/ros.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core/core.hpp>
#include "stereo_vo/StereoVoDynConfig.h"
#include <stereo_vo/StereoFeaturesStamped.h>
#include <kr_math/pose.hpp>

namespace galt {
namespace stereo_vo {

using scalar_t = float;
using Id = unsigned int;
using CvPoint2 = cv::Point_<scalar_t>;
using CvPoint3 = cv::Point3_<scalar_t>;
using KrPose = kr::Pose<scalar_t>;
using CvImagePyramid = std::vector<cv::Mat>;
using CvStereoImage = std::pair<cv::Mat, cv::Mat>;
using CvStereoPyramid = std::pair<CvImagePyramid, CvImagePyramid>;
using StereoVoDynConfig = ::stereo_vo::StereoVoDynConfig;
using FeatureMsg = ::stereo_vo::FeatureMsg;
using StereoFeaturesStamped = ::stereo_vo::StereoFeaturesStamped;

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_COMMON_H_
