#ifndef GALT_STEREO_VO_POINT_H_
#define GALT_STEREO_VO_POINT_H_

#include <stereo_vo/common.h>
#include <kr_vision/feature.hpp>
#include <image_geometry/stereo_camera_model.h>

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

class KeyFrame;
typedef std::shared_ptr<KeyFrame> KeyFramePtr;

class Point3d {
 public:
  class Observation {
   public:
    Observation(Id id, const CvPoint2& p_pixel, scalar_t depth,
                const KeyFramePtr& ptr)
        : id_(id), p_pixel_(p_pixel), depth_(depth), key_frame_(ptr) {}

    Observation(const Observation&) = default;

    const Id& id() const { return id_; }
    const CvPoint2& p_pixel() const { return p_pixel_; }
    const scalar_t& depth() const { return depth_; }
    const KeyFramePtr& key_frame() const { return key_frame_; }

   private:
    Id id_;
    CvPoint2 p_pixel_;
    scalar_t depth_;
    KeyFramePtr key_frame_;
  };

  Point3d(const Id id) : id_(id), is_inlier_(false) {}

  Point3d() : Point3d(0) {}
  Point3d(const Point3d&) = default;

  const CvPoint3& p_world() const { return p_world_; }
  void set_p_world(const CvPoint3& p_world) { p_world_ = p_world; }

  bool is_initialized() const { return !observations_.empty(); }

  void set_is_inlier(const bool& inlier) { is_inlier_ = inlier; }
  bool is_inlier() const { return is_inlier_; }

  bool AddObservation(const StereoVoDynConfig& config,
                      const StereoCameraModel& model,
                      const KeyFramePtr& key_frame, const CvPoint2& left,
                      const CvPoint2& right = CvPoint2(-1, -1));

 private:
  Id id_;             ///< unique id of this point
  CvPoint3 p_world_;  ///< position of this point in world frame

  std::vector<Observation> observations_;  ///< Collection of observations

  kr::DepthFilter<scalar_t> filter_;  ///< filter on feature depth
  bool is_inlier_;                    ///< Is the point an inlier?
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GATL_STEREO_VO_POINT_H_
