#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include <vector>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <kr_math/pose.hpp>

#include "opencv2/core/core.hpp"

#include "stereo_vo/StereoVoDynConfig.h"

namespace galt {

namespace stereo_vo {

using namespace image_geometry;
using StereoVoConfig = ::stereo_vo::StereoVoDynConfig;

class StereoVo;
class KeyFrame;

using scalar_t = float;
using CvPoint2 = cv::Point_<scalar_t>;
using CvPoint3 = cv::Point3_<scalar_t>;

struct Feature {
  Feature() : triangulated(false) {}

  CvPoint2 next;         /// New frame feature in pixle space
  CvPoint2 left, right;  /// Key frame feature in pixel space
  CvPoint2 left_coord,
      right_coord;    /// Key frame feature in normalized image space
  CvPoint3 point;     /// Key frame feature in world space
  bool triangulated;  /// Key frame featreu has been triangulated
};

class KeyFrame {
  friend class StereoVo;

 public:
  void Update(const cv::Mat &l_image, const cv::Mat &r_image,
              const StereoVoConfig &config, const StereoCameraModel &model,
              const kr::Pose<scalar_t> &pose);
  const int NumMatches() const { return features_.size(); }
  const kr::Pose<scalar_t> &pose() const { return pose_; }

 private:
  void Triangulate(const StereoCameraModel &model);
  cv::Mat l_image_, r_image_;
  cv::Mat prev_image_;
  std::vector<Feature> features_;
  int length;  /// Number of frames after this keyframe
  kr::Pose<scalar_t> pose_;
};

class StereoVo {
 public:
  StereoVo();
  const bool init() const { return init_; }
  void Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                  const StereoCameraModel &model);
  void Iterate(const cv::Mat &l_image, const cv::Mat &r_image);
  void UpdateConfig(const StereoVoConfig &config) { config_ = config; }
  const kr::Pose<scalar_t> &GetKeyFramePose() const {
    return key_frame_.pose();
  }

  const std::vector<Feature> &GetCurrentFeatures() const {
    return key_frame_.features_;
  }

  const kr::Pose<scalar_t> &GetCurrentPose() const { return current_pose_; }

 private:
  bool init_{false};
  StereoCameraModel model_;
  KeyFrame key_frame_;
  kr::Pose<scalar_t> current_pose_;
  StereoVoConfig config_;

  void Display(const cv::Mat &l_image, const cv::Mat &r_image,
               const std::vector<CvPoint2> &new_features);
};

template <typename T>
void PruneByStatus(const std::vector<uchar> &status, std::vector<T> &objects) {
  auto ite_p = objects.begin();
  for (auto ite_s = status.begin(); ite_s != status.end(); ite_s++) {
    if (*ite_s) {
      ite_p++;
    } else {
      ite_p = objects.erase(ite_p);
    }
  }
}

void TrackFeatures(const cv::Mat &image1, const cv::Mat &image2,
                   std::vector<CvPoint2> &features1,
                   std::vector<CvPoint2> &features2, std::vector<uchar> &status,
                   const StereoVoConfig &config);

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H
