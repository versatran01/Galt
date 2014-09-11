#ifndef GALT_STEREO_VO_POINT_H_
#define GALT_STEREO_VO_POINT_H_

#include <stereo_vo/common.h>
#include <kr_math/feature.hpp>
#include <image_geometry/stereo_camera_model.h>

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

class KeyFrame;
typedef std::shared_ptr<KeyFrame> KeyFramePtr;

/**
 * @brief The Point class, represents a 3D point in world frame and some
 * associated information. `d` refers to dimension, and not specifically
 * to double.
 */
class Point3d {
 public:
  /**
   * @brief Observation of a Point3d.
   */
  class Observation {
  public:
    Observation(Id id, const CvPoint2& p_pixel, scalar_t depth,
                const KeyFramePtr& ptr) :
      id_(id), p_pixel_(p_pixel), depth_(depth), key_frame_(ptr) {}
    
    //  statically copyable
    Observation(const Observation&) = default;
    
    const Id& id() const { return id_; }
    const CvPoint2& p_pixel() const { return p_pixel_; }
    const scalar_t& depth() const { return depth_; }
    const KeyFramePtr& key_frame() const { return key_frame_; }
    
  private:
    Id id_;             /// Id of the associated feature.
    CvPoint2 p_pixel_;  /// Pixel space coordinate of feature. 
    scalar_t depth_;    /// Depth estimate generated from this observation.
    KeyFramePtr key_frame_; ///< Key frame of this observation.
  };
  
  /**
   * @brief Point3d
   * @param id Unique ID assigned to this point.
   */
  Point3d(const Id id) : id_(id) {}
  
  Point3d() : id_(0) {}
  Point3d(const Point3d&) = default;

  const CvPoint3& p_world() const { return p_world_; }
  void set_p_world(const CvPoint3& p_world) { p_world_ = p_world; }

  /**
   * @brief isInitialized
   * @return True if the point is triangulated and has its first observation.
   */
  bool IsInitialized() const { return !observations_.empty(); }
  
  /**
   * @brief AddObservation
   * @param config Configuration with options.
   * @param model Stereo camera model.
   * @param key_frame Key frame to which this observation belongs.
   * @param left Left camera point in pixels.
   * @param right Right camera point in pixels.
   * 
   * @note right is not required if the point3 is initialized already.
   * 
   * @return True if the observation triangulation was ok, false otherwise.
   */
  bool AddObservation(const StereoVoDynConfig& config,
                      const StereoCameraModel& model,
                      const KeyFramePtr& key_frame,
                      const CvPoint2& left,
                      const CvPoint2& right = CvPoint2(-1,-1));
  
 private:
  Id id_;             ///< unique id of this point
  CvPoint3 p_world_;  ///< position of this point in world frame
  
  std::vector<Observation> observations_; ///< Collection of observations
  
  kr::DepthFilter<scalar_t> filter_;  ///< filter on feature depth
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GATL_STEREO_VO_POINT_H_
