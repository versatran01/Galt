#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <memory>

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

class KeyFrame;

struct Point {
  CvPoint2 p_pixel;
  CvPoint2 p_coord;
  std::shared_ptr<KeyFrame> key_frame;
};

class Feature {
 public:
  const bool ready() const { return ready_; }
  
  const bool triangulated() const { return triangulated_; }
  void set_triangulated(bool triangulated) { triangulated_ = triangulated; }
  
  const std::vector<Point>& points() { return points_; }

  const CvPoint3& p_world() const { return p_world_; }
  void set_p_world(const CvPoint3& p_world) { p_world_ = p_world; }
  
  const CvPoint2& p_pixel_left() const { return p_pixel_left_; }
  const CvPoint2& p_pixel_right() const { return p_pixel_right_; }
  
 private:
  bool ready_{false};
  bool triangulated_{false};
  CvPoint3 p_world_;
  CvPoint2 p_pixel_left_;
  CvPoint2 p_pixel_right_;
  std::vector<Point> points_;
};
}
}

#endif  // GALT_STEREO_VO_FEATURE_H_
