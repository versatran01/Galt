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
using ::stereo_vo::StereoVoDynConfig;

class StereoVo;
class KeyFrame;

typedef float scalar_t;
typedef cv::Point_<scalar_t> CvPoint2;
typedef cv::Point3_<scalar_t> CvPoint3;

struct Feature {
  Feature() : triangulated(false) {}
  
  CvPoint2 left, right;              /// Feature in pixel space
  CvPoint2 left_coord, right_coord;  /// Feature in normalized image space
  CvPoint3 point;                    /// Feature in world space
  bool triangulated;                 /// Feature has been triangulated
};

class KeyFrame {
  friend class StereoVo;

 public:
  void Update(const cv::Mat &l_image, const cv::Mat &r_image,
              const StereoVoDynConfig &config, const StereoCameraModel& model);
  const int NumMatches() const { return features_.size(); }

 private:
  void Triangulate(const StereoCameraModel &model);
  cv::Mat l_image_, r_image_;
  std::vector<Feature> features_;
  kr::Pose<scalar_t> pose_;
};

class StereoVo {
 public:
  StereoVo();
  const bool init() const { return init_; }
  void Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                  const StereoCameraModel &model);
  void Iterate(const cv::Mat &l_image, const cv::Mat &r_image);
  void UpdateConfig(const StereoVoDynConfig &config) { config_ = config; }

  const std::vector<Feature>& GetCurrentFeatures() const {
    return key_frame_.features_;
  }
  
  const kr::Pose<scalar_t>& GetCurrentPose() const {
    return current_pose_;
  }
  
 private:
  bool init_{false};
  StereoCameraModel model_;
  KeyFrame key_frame_;
  kr::Pose<scalar_t> current_pose_;
  StereoVoDynConfig config_;

  void Display(const cv::Mat &l_image, const cv::Mat &r_image,
               const std::vector<CvPoint2> &new_features);
};

template <typename T>
void PruneByStatus(const std::vector<uchar>& status, 
                   std::vector<T>& objects) {
  auto ite_p = objects.begin();
  for (auto ite_s = status.begin(); ite_s != status.end(); ite_s++) {
    if (*ite_s) {
      ite_p++;
    } else {
      ite_p = objects.erase(ite_p);
    }
  }
}

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_H
