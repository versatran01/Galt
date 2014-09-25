#ifndef GALT_STEREO_VO_H_
#define GALT_STEREO_VO_H_

#include "stereo_vo/common.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/detector.h"
#include "stereo_vo/frame.h"

#include <vector>
#include <deque>

#include <image_geometry/stereo_camera_model.h>
#include <sophus/se3.hpp>
#include <kr_math/base_types.hpp>

#include "opencv2/core/core.hpp"

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

class StereoVo {
 public:
  StereoVo()
      : init_(false),
        pnh_("~"),
        pub_features_(pnh_.advertise<StereoFeaturesStamped>("features", 1)) {}

  void set_config(const StereoVoDynConfig &config) {
    config_ = config;
    detector_.set_cell_size(config_.cell_size);
  }
  bool init() const { return init_; }
  const KrPose &w_T_c() const { return w_T_c_; }
  
  const kr::mat<scalar_t,6,6>& pose_covariance() const { 
    return pose_covariance_; 
  }

  void Initialize(const CvStereoImage &stereo_image,
                  const StereoCameraModel &model);
  bool Iterate(const CvStereoImage &stereo_image, const ros::Time &time);

  const std::deque<KeyFrame>& key_frames() const { return key_frames_; }
  
 private:
  bool InitializePose();
  bool AddKeyFrame(const KrPose& pose, const kr::mat<scalar_t,6,6> &pose_cov, 
                   const CvStereoImage &stereo_image,
                   std::vector<Feature> &features);
  void TrackSpatial(const cv::Mat &image1, const cv::Mat &image2,
                    std::vector<CvPoint2> &l_corners,
                    std::vector<CvPoint2> &r_corners);
  void TrackTemporal(const cv::Mat &image1, const cv::Mat &image2,
                     std::vector<Feature> &features);
  void OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
                   const std::vector<CvPoint2> &points1,
                   std::vector<CvPoint2> &points2, std::vector<uchar> &status);
  void FindFundamentalMat(const std::vector<CvPoint2> &points1,
                          const std::vector<CvPoint2> &points2,
                          std::vector<uchar> &status);
  bool ShouldAddKeyFrame() const;
  void PublishStereoFeatures(const KeyFrame &keyframe,
                             const ros::Time &time) const;
 
  /// Triangulate left and right observations into 3d space.
  bool TriangulatePoint(const CvPoint2& lp, ///< Left point in pixels.
                        const CvPoint2& rp, ///< Right point in pixels.
                        kr::vec3<scalar_t>& p3d,    ///< Output point in camera.
                        kr::mat3<scalar_t>& sigma); ///< Covariance.

  bool init_;
  StereoCameraModel model_;
  StereoVoDynConfig config_;
  KrPose w_T_c_;
  kr::mat<scalar_t,6,6> pose_covariance_;
  FeatureDetector detector_;
  std::deque<KeyFrame> key_frames_;
  std::vector<Feature> tracked_features_;
  CvStereoImage prev_stereo_;

  ros::NodeHandle pnh_;
  ros::Publisher pub_features_;
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_H_
