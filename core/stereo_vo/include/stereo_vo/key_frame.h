#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <map>

namespace galt {

namespace stereo_vo {

class KeyFrame {
 public:
  KeyFrame(const Pose &pose, const std::map<Feature::Id,Feature>& features,
           const cv::Mat &l_image, const cv::Mat &r_image)
      : pose_(pose), features_(features), l_image_(l_image), r_image_(r_image) {}
  const Pose &pose() const { return pose_; }
  const cv::Mat &l_image() const { return l_image_; }
  const cv::Mat &r_image() const { return r_image_; }
  const std::map<Feature::Id,Feature>& features() const { return features_; }
  
 private:
  Pose pose_;
  std::map<Feature::Id,Feature> features_;
  cv::Mat l_image_, r_image_;  
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
