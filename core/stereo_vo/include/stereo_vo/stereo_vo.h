#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
//#include "stereo_vo/feature.h"
//#include "stereo_vo/point3d.h"
//#include "stereo_vo/feature_detector.h"
//#include "stereo_vo/keyframe.h"
//#include "stereo_vo/tracker.h"

#include <vector>
#include <memory>

#include <image_geometry/stereo_camera_model.h>

#include "opencv2/core/core.hpp"

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

class StereoVo {
 public:
  StereoVo() : init_(false) {}

  void set_config(const StereoVoDynConfig &config) { config_ = config; }
  bool init() const { return init_; }

  void Initialize(const CvStereoImage &stereo_image,
                  const StereoCameraModel &model);
  //  void Iterate(const CvStereoImage &stereo_image);

 private:
  //  void AddKeyFrame(const CvStereoImage &stereo_image);
  //  bool ShouldAddKeyFrame() const;
  //  void EstimatePose();

  bool init_;
  StereoCameraModel model_;
  StereoVoDynConfig config_;
  //  std::vector<KeyFramePtr> key_frames_;
  //  FeatureDetector detector_;
  //  Tracker temporal_tracker_;
  //  Tracker spatial_tracker_;
  //  cv::Mat prev_left_image_;
  //  std::map<Id, Point3d> points_;
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_H_
