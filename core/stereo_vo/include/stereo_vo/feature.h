#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <memory>
#include <functional>

#include <stereo_vo/common.h>
#include <image_geometry/stereo_camera_model.h>

namespace galt {

namespace stereo_vo {

class Feature {
 public:
  typedef uint64_t Id;

  Feature(Id id, const CvPoint2& p_pixel_left, bool init) : 
    id_(id), p_pixel_left_(p_pixel_left), init_(init) {}
  
  Feature(Id id, const CvPoint3& p_cam_left, const CvPoint2& p_pixel_left,
          const CvPoint2& p_pixel_right, bool init)
      : id_(id),
        p_cam_left_(p_cam_left),
        p_pixel_left_(p_pixel_left),
        p_pixel_right_(p_pixel_right),
        init_(init) {}

  const CvPoint3& p_cam_left() const { return p_cam_left_; }
  void set_p_cam_left(const CvPoint3& p_cam_left) { p_cam_left_ = p_cam_left; }

  const CvPoint2& p_pixel_left() const { return p_pixel_left_; }
  void set_p_pixel_left(const CvPoint2& p_pixel) { p_pixel_left_ = p_pixel; }

  const CvPoint2& p_pixel_right() const { return p_pixel_right_; }
  void set_p_pixel_right(const CvPoint2& p_pixel) { p_pixel_right_ = p_pixel; }

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
  Id id_;
  CvPoint3 p_cam_left_;
  CvPoint2 p_pixel_left_;
  CvPoint2 p_pixel_right_;
  bool init_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
