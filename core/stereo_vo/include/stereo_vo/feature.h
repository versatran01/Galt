#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <memory>
#include <functional>

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

class Feature {
 public:
  typedef uint64_t Id;
    
  Feature(Id id, const CvPoint3& p_cam_left, const CvPoint2& p_pixel_left,
          const CvPoint2& p_pixel_right, bool init) : id_(id), p_cam_left_(p_cam_left),
          p_pixel_left_(p_pixel_left), p_pixel_right_(p_pixel_right), init_(init) {}

  const CvPoint3& p_cam_left() const { return p_cam_left_; }
  void set_p_cam_left(const CvPoint3& p_cam_left) { p_cam_left_ = p_cam_left; }

  const CvPoint2& p_pixel_left() const { return p_pixel_left_; }
  void set_p_pixel_left(const CvPoint2& p_pixel) { p_pixel_left_ = p_pixel; }

  const CvPoint2& p_pixel_right() const { return p_pixel_right_; }
  void set_p_pixel_right(const CvPoint2& p_pixel) { p_pixel_right_ = p_pixel; }

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
