#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

class KeyFrame {
 public:
  const Pose pose() const { return pose_; }

 private:
  cv::Mat l_image_, r_image_;
  Pose pose_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
