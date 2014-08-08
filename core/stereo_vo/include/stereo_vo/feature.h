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
  const std::vector<Point>& points() { return points_; }

  const CvPoint3& p_world() const { return p_world_; }
  
  const CvPoint2& p_pixel_left() const { return p_pixel_left_; }
  
 private:
  bool ready_{false};
  CvPoint3 p_world_;
  CvPoint2 p_pixel_left_;
  std::vector<Point> points_;
};
}
}

#endif  // GALT_STEREO_VO_FEATURE_H_
