#ifndef FRUIT_TRACKER_HPP
#define FRUIT_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace galt {
namespace fruit_tracker {

class Fruit {
public:
  Fruit() {}  /// @todo: add other constructors, etc...
  
  static void detect(cv::Mat input, std::vector<Fruit>& output);
  
  const cv::Point2d& imagePosition() const { return image_position_; }
  void setImagePosition(const cv::Point2d& vec) { image_position_ = vec; }
  
  double imageArea() const { return image_area_; }
  void setImageArea(double area) { image_area_ = area; }
    
  double imageRadius() const { return image_radius_; }
  void setImageRadius(double image_radius) { image_radius_ = image_radius; }
  
  /// Calculate fruit matching score?
  double matchingScore(const Fruit& fruit) const;
  
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

  void track(const cv::Mat& input);
  
private:
  std::vector<int> match(const std::vector<Fruit>& group1,
                         const std::vector<Fruit>& group2);
  
  std::vector<Fruit> tracked_fruits_;
  cv::Mat previous_mat_;
};

} //  namespace fruit_tracker
} //  namespace galt

#endif // FRUIT_TRACKER_HPP
