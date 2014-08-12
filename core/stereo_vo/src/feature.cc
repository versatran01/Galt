#include <stereo_vo/feature.h>
#include <kr_math/feature.hpp>

namespace galt {

namespace stereo_vo {

bool Feature::triangulate(const image_geometry::StereoCameraModel &model,
                          scalar_t eigen_threshold) {
  //  camera model
  const scalar_t lfx = model.left().fx(), lfy = model.left().fy();
  const scalar_t lcx = model.left().cx(), lcy = model.left().cy();
  const scalar_t rfx = model.right().fx(), rfy = model.right().fy();
  const scalar_t rcx = model.right().cx(), rcy = model.right().cy();

  Pose poseLeft;   //  identity
  Pose poseRight;  //  shifted right along x
  poseRight.p[0] = model.baseline();

  //  normalized coordinates
  kr::vec2<scalar_t> lPt((p_pixel_.x - lcx) / lfx, (p_pixel_.y - lcy) / lfy);

  kr::vec2<scalar_t> rPt((p_pixel_right_.x - rcx) / rfx,
                         (p_pixel_right_.y - rcy) / rfy);

  kr::vec3<scalar_t> p3D;
  scalar_t ratio;

  kr::triangulate(poseLeft, lPt, poseRight, rPt, p3D, ratio);

  if (ratio > eigen_threshold) {
    return false;
  }

  //  point is valid, refine it some more
  std::vector<Pose> poses({poseLeft, poseRight});
  std::vector<kr::vec2<scalar_t>> obvs({lPt, rPt});

  if (!kr::refinePoint(poses, obvs, p3D)) {
    return false;
  }

  p_cam_left_.x = p3D[0];
  p_cam_left_.y = p3D[1];
  p_cam_left_.z = p3D[2];
  return true;
}

}  // namespace stereo_vo

}  // namespace galt
