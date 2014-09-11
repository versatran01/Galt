#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <map>
#include <memory>

namespace galt {
namespace stereo_vo {

class Frame {
 public:
  using Ptr = boost::shared_ptr<Frame>;
  using ConstPtr = boost::shared_ptr<const Frame>;

  static Id frame_counter;

  Frame();
  Frame(const CvStereoImage &stereo_image);

  const Id &id() const { return id_; }
  const KrPose &pose() const { return pose_; }
  void set_pose(const KrPose &pose) { pose_ = pose; }
  const bool &is_keyframe() const { return is_keyframe_; }

  size_t num_features() const { return features_.size(); }
  std::vector<Feature> &features() { return features_; }
  const std::vector<Feature> &features() const { return features_; }
  //  void set_features(const std::vector<Feature> &features) {
  //    features_ = features;
  //  }

  const CvStereoImage &stereo_image() const { return stereo_image_; }
  const cv::Mat &l_image() const { return stereo_image_.first; }
  const cv::Mat &r_image() const { return stereo_image_.second; }

  void SetKeyFrame() { is_keyframe_ = true; }
  //  size_t RemoveById(const std::set<Id> &ids_to_remove, bool force = false);

 private:
  Id id_;                          ///< id of this frame
  KrPose pose_;                    ///< pose of the frame in world frame
  bool is_keyframe_;               ///< if this frame is a keyframe
  CvStereoImage stereo_image_;     ///< stereo images of this frame
  std::vector<Feature> features_;  ///< all features observed in this frame
};

typedef Frame::Ptr FramePtr;
using FrameConstPtr = Frame::ConstPtr;

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_
