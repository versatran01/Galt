#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include <vector>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

#include "opencv2/core/core.hpp"

#include "stereo_vo/StereoVoDynConfig.h"

namespace galt {

namespace stereo_vo {

using namespace image_geometry;
using ::stereo_vo::StereoVoDynConfig;

class StereoVo;
class KeyFrame;

typedef float scalar_t;

struct Feature {
  cv::Point_<scalar_t> left, right;            /// Feature in pixel space
  cv::Point_<scalar_t> leftCoord, rightCoord;  /// Feature in normalized image space
  cv::Point3_<scalar_t> point;                 /// Feature in world space
};

class KeyFrame {
  friend class StereoVo;

 public:
  void Update(const cv::Mat &l_image, const cv::Mat &r_image,
              const StereoVoDynConfig &config, const StereoCameraModel& model);
  const int NumMatches() const { return features_.size(); }

 private:
  void Triangulate(const StereoCameraModel &model);
  cv::Mat l_image_, r_image_;
  std::vector<Feature> features_;
};

class StereoVo {
 public:
  StereoVo();
  const bool init() const { return init_; }
  void Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                  const StereoCameraModel &model);
  void Iterate(const cv::Mat &l_image, const cv::Mat &r_image);
  void UpdateConfig(const StereoVoDynConfig &config) { config_ = config; }

 private:
  bool init_{false};
  StereoCameraModel model_;
  KeyFrame key_frame_;
  StereoVoDynConfig config_;

  void Display(const cv::Mat &l_image, const cv::Mat &r_image,
               const std::vector<cv::Point2f> &new_features);
};

std::vector<cv::Point2f> ExtractByStatus(
    const std::vector<cv::Point2f> &features, const std::vector<uchar> &status);

template <typename T>
void PruneByStatus(const std::vector<uchar>& status, 
                   std::vector<T>& objects) {
  auto ite_p = objects.begin();
  for (auto ite_s = status.begin(); ite_s != status.end(); ite_s++) {
    if (*ite_s) {
      ite_p++;
    } else {
      ite_p = objects.erase(ite_p);
    }
  }
}

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H
