#ifndef GALT_STEREO_VO_UTILS_H_
#define GALT_STEREO_VO_UTILS_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/frame.h"

#include <algorithm>
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
const cv::Scalar CYAN = cv::Scalar(255, 255, 0);
}

/// @note: these two could possibly be rewrote using erase and remove_if for
/// better performance and readability
/*
template <typename T, typename U>
void PruneByStatus(const std::vector<U> &status, std::vector<T> &objects,
                   std::set<T> &removed) {
  ROS_ASSERT_MSG(status.size() == objects.size(),
                 "status and object size mismatch");
  auto it_obj = objects.begin();
  for (const auto &s : status) {
    if (s) {
      it_obj++;
    } else {
      removed.insert(*it_obj);
      it_obj = objects.erase(it_obj);
    }
  }
}
*/

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

void Display(const CvStereoImage &image,
             const std::vector<Feature> &tracked_features,
             const KeyFrame &keyframe);
void Display(const cv::Mat &l_image_prev, const cv::Mat &r_image_prev,
             const cv::Mat &l_image, const cv::Mat &r_image,
             const cv::vector<Feature> &keyframe_features,
             const cv::vector<CvPoint2> &r_corners,
             const cv::vector<Feature> &tracked_features);
void DrawCorners(cv::Mat &image, const std::vector<CvPoint2> &corners,
                 const cv::Scalar &color,
                 const CvPoint2 &offset = CvPoint2(0, 0));
void DrawFeatures(cv::Mat &image, const std::vector<Feature> &features,
                  cv::Scalar color, const CvPoint2 &offset = CvPoint2(0, 0));
void DrawCorrespondence(cv::Mat &image, const std::vector<Feature> &features1,
                        const std::vector<Feature> &features2,
                        const cv::Scalar &color, const CvPoint2 &offset);
void DrawStereoMatch(cv::Mat &image, const std::vector<Feature> &features,
                     const std::vector<CvPoint2> &corners,
                     const cv::Scalar &color,
                     const CvPoint2 &offset = CvPoint2(0, 0));
// void AnnotateFeatureCounts(cv::Mat &image, const std::vector<Feature>
// &features,
//                           const cv::Scalar &color, int quadrant);
}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_UTILS_H_
