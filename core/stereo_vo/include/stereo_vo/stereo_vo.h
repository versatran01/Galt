#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>

#include "opencv2/core/core.hpp"

#include "stereo_vo/StereoVoDynConfig.h"

namespace galt {

namespace stereo_vo {

using namespace image_geometry;
class StereoVo;

struct StereoVoConfig {
  int num_features;
  int min_features;
};

class KeyFrame {
  friend class StereoVo;

 public:
  void Update(const cv::Mat &l_image, const cv::Mat &r_image) {
    l_image_ = l_image;
    r_image_ = r_image;
  }

 private:
  cv::Mat l_image_;
  cv::Mat r_image_;
};

class StereoVo {
 public:
  StereoVo();
  const bool init() const { return init_; }
  void Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                  const StereoCameraModel &model);
  void Iterate(const cv::Mat &l_image, const cv::Mat &r_image);
  void UpdateConfig(const StereoVoConfig &config) { config_ = config; }

 private:
  bool init_{false};
  StereoCameraModel model_;
  KeyFrame key_frame_;
  StereoVoConfig config_;

  void Display(const cv::Mat &l_image, const cv::Mat &r_image);
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H
