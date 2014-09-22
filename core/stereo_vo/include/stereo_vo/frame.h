#ifndef GALT_STEREO_VO_FRAME_H_
#define GALT_STEREO_VO_FRAME_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"

#include <sophus/se3.hpp>

namespace galt {
namespace stereo_vo {

class KeyFrame {
 public:
  static Id counter;

  KeyFrame();
  KeyFrame(Id id, const Sophus::SE3d &w_T_f, const CvStereoImage &stereo_image,
           const std::vector<Feature> &features,
           const std::vector<CvPoint2> &r_corners)
      : id_(id),
        w_T_f_(w_T_f),
        stereo_image_(stereo_image),
        features_(features),
        r_corners_(r_corners) {}
  KeyFrame(const Sophus::SE3d &w_T_f, const CvStereoImage &stereo_image,
           const std::vector<Feature> &features,
           const std::vector<CvPoint2> &r_corners)
      : KeyFrame(counter++, w_T_f, stereo_image, features, r_corners) {}

  const Id &id() const { return id_; }
  const Sophus::SE3d &w_T_f() const { return w_T_f_; }
  const std::vector<Feature> &features() const { return features_; }
  const std::vector<CvPoint2> &r_corners() const { return r_corners_; }
  const CvStereoImage &stereo_image() const { return stereo_image_; }
  const cv::Mat &l_image() const { return stereo_image_.first; }
  const cv::Mat &r_image() const { return stereo_image_.second; }

  //  size_t num_features() const { return features_.size(); }
  //  std::vector<Feature> &features() { return features_; }
  //  const std::vector<Feature> &features() const { return features_; }
  //  void set_features(const std::vector<Feature> &features) {
  //    features_ = features;
  //  }

  //  size_t RemoveById(const std::set<Id> &ids_to_remove, bool force = false);

 private:
  Id id_;
  Sophus::SE3d w_T_f_;
  CvStereoImage stereo_image_;
  std::vector<Feature> features_;
  std::vector<CvPoint2> r_corners_;
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_FRAME_H_
