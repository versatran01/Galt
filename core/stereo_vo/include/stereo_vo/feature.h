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
  Feature(Feature::Id id, const CvPoint3& p_world)
      : id_(id), p_world_(p_world) {}

  const CvPoint3& p_world() const { return p_world_; }
  void set_p_world(const CvPoint3& p_world) { p_world_ = p_world; }

 private:
  Id id_;
  CvPoint3 p_world_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
