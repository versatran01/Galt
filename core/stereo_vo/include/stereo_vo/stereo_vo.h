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

struct Feature {
  cv::Point2f left, right;            /// Feature in pixel space
  cv::Point2f leftCoord, rightCoord;  /// Feature in normalized image space
  cv::Point3f point;                  /// Feature in world space
};

class KeyFrame {
  friend class StereoVo;

 public:
  typedef double scalar_t;
  
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
}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H
