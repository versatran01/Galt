#ifndef GALT_STEREO_VO_POINT_H_
#define GALT_STEREO_VO_POINT_H_

#include "stereo_vo/common.h"

namespace galt {
namespace stereo_vo {

/**
 * @brief The Point class, represents a 3D point in world frame
 */
class Point3d {
 public:
  Point3d() = default;
  Point3d(const Id id, const CvPoint3& p_world) : id_(id), p_world_(p_world) {}

  const CvPoint3& p_world() const { return p_world_; }
  void set_p_world(const CvPoint3& p_world) { p_world_ = p_world; }

 private:
  Id id_;             ///< id of this point
  CvPoint3 p_world_;  ///< position of this point in world frame
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GATL_STEREO_VO_POINT_H_
