#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <map>

namespace galt {

namespace stereo_vo {

class KeyFrame {
 public:
  KeyFrame(const Pose &pose, const std::map<Feature::Id, Feature> &features,
           const CvStereoImage &stereo_image)
      : pose_(pose), features_(features), stereo_image_(stereo_image) {}

  const Pose &pose() const { return pose_; }
  const CvStereoImage &stereo_image() const { return stereo_image_; }
  const cv::Mat &l_image() const { return stereo_image_.first; }
  const cv::Mat &r_image() const { return stereo_image_.second; }
  const std::map<Feature::Id, Feature> &features() const { return features_; }

 private:
  Pose pose_;
  std::map<Feature::Id, Feature> features_;
  CvStereoImage stereo_image_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
