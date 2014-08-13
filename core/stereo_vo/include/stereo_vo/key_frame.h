#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <map>

namespace galt {

namespace stereo_vo {

class KeyFrame {
 public:
  KeyFrame() {}
  KeyFrame(const Pose &pose, const std::vector<Corner> &corners,
           const CvStereoImage &stereo_image)
      : pose_(pose), corners_(corners), stereo_image_(stereo_image) {}

  const Pose &pose() const { return pose_; }
  void set_pose(const Pose& pose) { pose_ = pose; }
  const cv::Mat &l_image() const { return stereo_image_.first; }
  const cv::Mat &r_image() const { return stereo_image_.second; }
  const CvStereoImage &stereo_image() const { return stereo_image_; }
  const std::vector<Corner> &corners() const { return corners_; }

  void RemoveById(const std::vector<Feature::Id> &ids_to_remove) {
    //  not terribly efficient, but ids_to_remove will typically be small
    for (const Feature::Id &id : ids_to_remove) {
      auto ite = std::find_if(corners_.begin(), corners_.end(),
                              [id](const Corner &c) { return c.id() == id; });
      if (ite != corners_.end()) {
        if (ite->init()) {
          corners_.erase(ite);
        }
      }
    }
  }

 private:
  Pose pose_;
  std::vector<Corner> corners_;
  CvStereoImage stereo_image_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
