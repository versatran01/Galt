#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/pose.h"

#include <map>
#include <memory>

namespace galt {
namespace stereo_vo {

/**
 * @brief The Frame class, respresents a camera frame
 * A frame represent two stereo images, its estimated pose and all features
 * observed in the left image
 */
class Frame {
 public:
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

  Frame() = default;
  Frame(const CvStereoImage &stereo_image);

  const Id &id() const { return id_; }
  const KrPose &pose() const { return pose_; }
  void set_pose(const KrPose &pose) { pose_ = pose; }
  const bool &is_keyframe() const { return is_keyframe_; }

  const size_t num_features() const { return features_.size(); }
  std::vector<Feature> &features() { return features_; }
  const std::vector<Feature> &features() const { return features_; }
  void set_features(const std::vector<Feature> &features) {
    features_ = features;
  }

  const CvStereoImage &stereo_image() const { return stereo_image_; }
  const cv::Mat &l_image() const { return stereo_image_.first; }
  const cv::Mat &r_image() const { return stereo_image_.second; }
  /**
   * @brief SetKeyFrame Mark this frame as key frame
   * @param right_corners
   */
  void SetKeyFrame(const std::vector<CvPoint2> &right_corners) {
    auto it_feat = features_.begin(), ite_feat = features_.end();
    auto it_right = right_corners.begin();
    for (; it_feat != ite_feat; ++it_feat, ++it_right) {
      it_feat->set_p_right(*it_right);
    }
    is_keyframe_ = true;
  }
  /**
   * @brief RemoveById Remove only new features which id is in ids_to_remove
   * @param ids_to_remove A set of ids to remove
   */
  size_t RemoveById(const std::set<Id> &ids_to_remove, bool force = false);

 private:
  Id id_;                          ///< id of this frame
  KrPose pose_;                    ///< pose of the frame in world frame
  bool is_keyframe_;               ///< if this frame is a keyframe
  CvStereoImage stereo_image_;     ///< stereo images of this frame
  std::vector<Feature> features_;  ///< all features observed in this frame
  /// @todo: this pyramid is here for later speeding up optical flow by reusing
  /// the previous one
  std::vector<cv::Mat> pyramid_;  ///< image pyramid of the left image
};

using FramePtr = Frame::Ptr;
using FrameConstPtr = Frame::ConstPtr;

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_KEY_FRAME_H_