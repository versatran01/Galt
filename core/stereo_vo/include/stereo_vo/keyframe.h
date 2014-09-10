#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <map>
#include <memory>

namespace galt {
namespace stereo_vo {

class KeyFrame {
 public:
  using Ptr = boost::shared_ptr<KeyFrame>;
  using ConstPtr = boost::shared_ptr<const KeyFrame>;

  static Id frame_counter;

  KeyFrame();
  KeyFrame(const CvStereoImage &stereo_image);

  const Id &id() const { return id_; }
  const KrPose &pose() const { return pose_; }
  void set_pose(const KrPose &pose) { pose_ = pose; }

  size_t num_features() const { return features_.size(); }
  std::vector<Feature> &features() { return features_; }
  const std::vector<Feature> &features() const { return features_; }
  //  void set_features(const std::vector<Feature> &features) {
  //    features_ = features;
  //  }

  const CvStereoImage &stereo_image() const { return stereo_image_; }
  const cv::Mat &l_image() const { return stereo_image_.first; }
  const cv::Mat &r_image() const { return stereo_image_.second; }

  //  size_t RemoveById(const std::set<Id> &ids_to_remove, bool force = false);

 private:
  Id id_;                          ///< id of this frame
  KrPose pose_;                    ///< pose of the frame in world frame
  CvStereoImage stereo_image_;     ///< stereo images of this frame
  std::vector<Feature> features_;  ///< all features observed in this frame
};

using KeyFramePtr = KeyFrame::Ptr;
using KeyFrameConstPtr = KeyFrame::ConstPtr;

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
