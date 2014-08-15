#ifndef GALT_STEREO_VO_KEY_FRAME_H_
#define GALT_STEREO_VO_KEY_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

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
  /// @note: Instead of statically allocate a frame, I switched to shared_ptr
  /// Please check all usage of them since I'm not very familiar with that.
  /// Especially function input and output parameters, should I pass shared_ptr
  /// by reference or value? eg. const shared_ptr<const T>& ? or
  /// const shared_ptr<T>&? or simply pass by value since that's the intended
  /// usage of shared_ptr
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

  static Id frame_counter;

  Frame() = default;
  Frame(const CvStereoImage &stereo_image)
      : id_(frame_counter++),
        is_keyframe_(false),
        stereo_image_(stereo_image) {}

  const Id &id() const { return id_; }
  const Pose &pose() const { return pose_; }
  void set_pose(const Pose &pose) { pose_ = pose; }
  const bool &is_keyframe() const { return is_keyframe_; }

  const size_t num_feautres() const { return features_.size(); }
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
   */
  void SetKeyFrame() { is_keyframe_ = true; }
  /**
   * @brief RemoveById Remove only new features which id is in ids_to_remove
   * @param ids_to_remove A set of ids to remove
   * @note I rewrote this part with std::remove_if
   */
  void RemoveById(const std::set<Id> &ids_to_remove);

 private:
  Id id_;                          ///< id of this frame
  Pose pose_;                      ///< pose of the frame in world frame
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
