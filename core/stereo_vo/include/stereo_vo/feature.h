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

class Feature {
 public:
  using Id = uint64_t;

  Feature() = default;
  Feature(Feature::Id id, const CvPoint3& p_cam)
      : id_(id), p_cam_(p_cam) {}

  const CvPoint3& p_cam() const { return p_cam_; }
  void set_p_cam(const CvPoint3& p_cam) { p_cam_ = p_cam; }

 private:
  Id id_;
  CvPoint3 p_cam_;
};

//class Observation {
// public:
//  Observation() = default;
//  Observation(Feature::Id id, const CvPoint2& p_pixel)
//      : id_(id), p_pixel_(p_pixel) {}

//  const CvPoint2& p_pixel() const { return p_pixel_; }
//  void set_p_pixel(const CvPoint2& p_pixel) { p_pixel_ = p_pixel; }

// private:
//  Feature::Id id_;
//  CvPoint2 p_pixel_;
//};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
