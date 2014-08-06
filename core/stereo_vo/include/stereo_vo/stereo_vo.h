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

class KeyFrame {
  friend class StereoVo;

 public:
  void Update(const cv::Mat &l_image, const cv::Mat &r_image,
              const StereoVoDynConfig &config);
  const int NumMatches() const { return l_features_.size(); }

 private:
  void Triangulate();
  cv::Mat l_image_, r_image_;
  std::vector<cv::Point2f> l_features_, r_features_;
  std::vector<cv::Point2f> l_coords_, r_coords_;
  std::vector<cv::Point3f> points_;
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
