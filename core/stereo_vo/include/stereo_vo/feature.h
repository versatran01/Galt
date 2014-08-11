#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <memory>
#include <functional>
#include <cstdint>

#include <stereo_vo/common.h>
#include <image_geometry/stereo_camera_model.h>

namespace galt {

namespace stereo_vo {

class Corner {
 public:
  using Id = uint64_t;

  Corner() = default;
  Corner(const Id& id, const CvPoint2& p_pixel, const bool& init)
      : id_(id), p_pixel_(p_pixel), init_(init) {}

  const CvPoint2& p_pixel() const { return p_pixel_; }
  void set_p_pixel(const CvPoint2& p_pixel) { p_pixel_ = p_pixel; }

  const Id& id() const { return id_; }
  void set_id(const Id& id) { id_ = id; }

  const bool& init() const { return init_; }
  void set_init(const bool& init) { init_ = init; }

 protected:
  Id id_;
  CvPoint2 p_pixel_;
  bool init_;
};

class Feature : public Corner {
 public:
  //  Feature(const Corner::Id& id, const CvPoint2& p_pixel_left,
  //          const CvPoint2& p_pixel_right, const CvPoint3& p_cam_left,
  //          const bool& init)
  //      : Corner(id, p_pixel_left, init),
  //        p_pixel_right_(p_pixel_right),
  //        p_cam_left_(p_cam_left) {}

  Feature() = default;
  Feature(const Corner& corner, const CvPoint2& p_pixel_right)
      : Corner(corner), p_pixel_right_(p_pixel_right) {}

  const CvPoint3& p_cam_left() const { return p_cam_left_; }
  void set_p_cam_left(const CvPoint3& p_cam_left) { p_cam_left_ = p_cam_left; }

  const CvPoint2& p_pixel_right() const { return p_pixel_right_; }
  void set_p_pixel_right(const CvPoint2& p_pixel) { p_pixel_right_ = p_pixel; }

  const CvPoint2& p_pixel_left() const { return p_pixel(); }
  void set_p_pixel_left(const CvPoint2& p_pixel) { set_p_pixel(p_pixel); }

  /**
   * @brief Triangulate a feature in the current camera frame.
   *
   * @param model ROS Stereo Camera model
   * @param eigen_threshold Maximum ratio of largest to smallest eigenvalues.
   * Those features exceeding this ratio are discarded.
   *
   * @return True on success, false if the feature is rejected as an outlier.
   * @note Those features which are rejected should be removed from the
   * feature list.
   */
  bool triangulate(const image_geometry::StereoCameraModel& model,
                   scalar_t eigen_threshold);

 private:
  CvPoint2 p_pixel_right_;
  CvPoint3 p_cam_left_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
