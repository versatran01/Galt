#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <memory>
#include <functional>

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

class KeyFrame;

void UpdateFeatures();

struct Point {
  CvPoint2 p_pixel;
  CvPoint2 p_coord;
  std::shared_ptr<KeyFrame> key_frame;
};

using Points = std::vector<Point>;

class Feature {
 public:
  friend void UpdateFeatures();

  Feature(const CvPoint2& pixel) : p_pixel_left_{pixel} {}
  const bool ready() const { return ready_; }

  const bool triangulated() const { return triangulated_; }
  void set_triangulated(bool triangulated) { triangulated_ = triangulated; }

  const Points& points() { return points_; }

  const CvPoint3& p_world() const { return p_world_; }
  void set_p_world(const CvPoint3& p_world) { p_world_ = p_world; }

  const CvPoint2& p_pixel_left() const { return p_pixel_left_; }
  void set_p_pixel_left(const CvPoint2& p_pixel_left) {
    p_pixel_left_ = p_pixel_left;
  }

  const CvPoint2& p_pixel_right() const { return p_pixel_right_; }
  void set_p_pixel_right(const CvPoint2& p_pixel_right) {
    p_pixel_right_ = p_pixel_right;
  }

  const CvPoint2& p_pixel_prev() const { return p_pixel_prev_; }
  void set_p_pixel_prev(const CvPoint2& p_pixel_prev) {
    p_pixel_prev_ = p_pixel_prev;
  }

 private:
  bool ready_{false};
  bool triangulated_{false};
  CvPoint3 p_world_;
  CvPoint2 p_pixel_left_;
  CvPoint2 p_pixel_prev_;
  CvPoint2 p_pixel_right_;
  Points points_;
};

using Features = std::vector<Feature>;

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
