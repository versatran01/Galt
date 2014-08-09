#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

class KeyFrame {
 public:
  KeyFrame(const Pose &pose, const cv::Mat &l_image, const cv::Mat &r_image)
      : pose_(pose), l_image_(l_image), r_image_(r_image) {}
  const Pose &pose() const { return pose_; }
  const cv::Mat &l_image() const { return l_image_; }
  const cv::Mat &r_image() const { return r_image_; }

 private:
  Pose pose_;
  cv::Mat l_image_, r_image_;
};

using KeyFrames = std::vector<KeyFrame>;

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
