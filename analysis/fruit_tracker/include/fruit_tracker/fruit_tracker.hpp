#ifndef FRUIT_TRACKER_HPP
#define FRUIT_TRACKER_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace galt {
namespace fruit_tracker {

class Fruit {
public:
  Fruit() {}  /// @todo: add other constructors, etc...
  
  static void detect(const cv::Mat& input, std::vector<Fruit>& output);
  
  const cv::Point2d& imagePosition() const { return image_position_; }
  void setImagePosition(const cv::Point2d& vec) { image_position_ = vec; }
  
  double imageArea() const { return image_area_; }
  void setImageArea(double area) { image_area_ = area; }
    
  double imageRadius() const { return image_radius_; }
  void setImageRadius(double image_radius) { image_radius_ = image_radius; }
  
private:
  cv::Point2d image_position_;
  double image_area_;
  double image_radius_;
};

/// @todo: build an actual tracker, etc...
class Tracker {
public:
  Tracker() {}
  virtual ~Tracker() {}
  
private:
  
};

} //  namespace fruit_tracker
} //  namespace galt

#endif // FRUIT_TRACKER_HPP
