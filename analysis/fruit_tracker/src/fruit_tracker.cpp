#include <fruit_tracker/fruit_tracker.hpp>
#include <Eigen/Dense>

#define BUILD_DEBUG

namespace galt {
namespace fruit_tracker {

void Fruit::detect(const cv::Mat& input, std::vector<Fruit>& output) {
  
  cv::Mat hsv_input;
  cv::cvtColor(input,hsv_input,cv::COLOR_BGR2HSV);
  //  extract separate channels from HSV
  std::vector<cv::Mat> channels(3);
  cv::split(hsv_input,channels);
  //cv::imshow("h",channels[0]);
  //cv::imshow("s",channels[1]);
  //cv::imshow("v",channels[2]);
  
  //  threshold the h-channel (blacks only)
  cv::threshold(channels[0],channels[0],0.1*255,255,cv::THRESH_BINARY_INV);
  //  threshold the v-channel (whites only)
  cv::threshold(channels[2],channels[2],0.95*255,255,cv::THRESH_BINARY_INV);
  
  //  make the detection mask by adding them together...
  cv::Mat detect(hsv_input.rows,hsv_input.cols,CV_8UC1);
  for (int i=0; i < detect.rows; i++) {
    for (int j=0; j < detect.cols; j++) {
      detect.at<uchar>(i,j) = (channels[0].at<uchar>(i,j) && 
          channels[2].at<uchar>(i,j)) * 255;
    }
  }
  //  box blur it
  cv::blur(detect,detect,cv::Size(3,3));
  cv::Mat contour_image;
  detect.copyTo(contour_image);
   
#ifdef BUILD_DEBUG
  cv::Mat output_image;
  input.copyTo(output_image);
#endif
  output.clear();
  
  //  do connected components
  std::vector <std::vector<cv::Point>> contours;
  cv::findContours(contour_image,contours,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);
  //  throw away using area threshold
  for (const std::vector<cv::Point>& contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    if (area > 100) {
      //  find the moments
      cv::Moments moment = cv::moments(contour);
      double inv_m00 = 1 / moment.m00;
      cv::Point2d center = cv::Point2d(moment.m10 * inv_m00, 
                                       moment.m01 * inv_m00);
      const double radius = std::sqrt(area / M_PI);
      
      Fruit fruit;
      fruit.setImageArea(area);
      fruit.setImagePosition(center);
      fruit.setImageRadius(radius);
      output.push_back(fruit);
      
      //  draw onto input image
#ifdef BUILD_DEBUG
      cv::circle(output_image,center,static_cast<int>(radius*1.2),cv::Scalar(0,0,255),2);
#endif
    }
  }
  
#ifdef BUILD_DEBUG
  cv::imshow("bgr", output_image);
  cv::imshow("hsv", hsv_input);
  cv::imshow("detect", detect);
  cv::waitKey(1);
#endif
}

void Tracker::track(const cv::Mat& input) {
  
  //  detect fruits in new image
  std::vector <Fruit> detections;
  Fruit::detect(input, detections);
  if (detections.empty()) {
    return;
  }
  
#ifdef BUILD_DEBUG
  //  draw the old and new image
  cv::Mat track_image(input.rows,input.cols*2,CV_8UC3);
  if (!previous_mat_.empty()) {
    previous_mat_.copyTo(track_image(cv::Range(0,input.rows),cv::Range(0,input.cols)));
  }
  input.copyTo(track_image(cv::Range(0,input.rows),cv::Range(input.cols,input.cols*2)));
#endif
  
  //  match fruit
  if (!tracked_fruits_.empty()) {
    std::vector<int> matches = match(tracked_fruits_, detections);
    assert(matches.size() == tracked_fruits_.size());
    
    std::vector<Fruit>::iterator ite = tracked_fruits_.begin();
    for (const int& match : matches) {
      if (match < 0) {  //  lost...
        
#ifdef BUILD_DEBUG
        //  draw the lost one in red
        cv::circle(track_image,ite->imagePosition(),ite->imageRadius(),
                   cv::Scalar(0,0,255),2);
#endif  
        ite = tracked_fruits_.erase(ite);            
      } else {
#ifdef BUILD_DEBUG
        //  draw old + new in blue, with line between them
        cv::circle(track_image,ite->imagePosition(),ite->imageRadius(),
                   cv::Scalar(255,0,0),2);
        
        const Fruit& matched_fruit = 
        cv::circle();
#endif
        
        //  found a match, update positions...
        ite++;
      }
    }
    
    //  now add new fruits...
    std::sort(matches.begin(),matches.end());
    
    for (size_t i=0; i < detections.size(); i++) {
      if (!std::binary_search(matches.begin(),
                              matches.end(),static_cast<int>(i))) {
        //  not already matched, must be new...
        tracked_fruits_.push_back(detections[i]);
        
        //  draw new one in green
        cv::Point2d center = detections[i].imagePosition();
        center.x += input.cols;
        cv::circle(track_image,center,detections[i].imageRadius(),
                   cv::Scalar(0,255,0),2);
      }
    }
    
  } else {
    tracked_fruits_ = detections;
  }
  
  previous_mat_ = input;
}

double Fruit::matchingScore(const Fruit& fruit) const {
  
  const cv::Point2d& p0 = this->image_position_;
  const cv::Point2d& p1 = fruit.image_position_;
  double distance = std::sqrt((p0.x-p1.x)*(p0.x-p1.x) + 
                              (p0.y-p1.y)*(p0.y-p1.y));
  
  double rad_change = std::abs(fruit.imageRadius() - imageRadius());
  return distance*1 + rad_change*0;
}

std::vector<int> Tracker::match(const std::vector<Fruit>& group1,
                                const std::vector<Fruit>& group2) {
  
  const size_t N = group1.size();
  const size_t M = group2.size();
  std::vector <int> results;
  if (!N || !M) {
    return results;
  }
  
  //  bruit force matching, cost is O(NM)
  //  check everything against everything
  //  both N & M are small, < 100
  /// @todo: replace with something better eventually...
  
  struct Match {
    size_t i; //  from
    size_t j; //  to
    double score;
    
    bool operator < (const Match& rhs) const {
      return score < rhs.score;
    }
  };
  std::vector<std::vector<Match>> matches;
  std::vector<Match> best_matches;
  matches.resize(M);
  
  for (size_t i=0; i < N; i++) {
    std::vector<Match>& matches_i = matches[i];
    for (size_t j=0; j < M; j++) {
      const double score = group1[i].matchingScore(group2[j]);
      Match match;
      match.i = i;
      match.j = j;
      match.score = score;
      matches_i.push_back(match);
    }
    //  order by lowest to highest
    std::sort(matches_i.begin(),matches_i.end());
    const Match& best = matches_i.front();
    best_matches.push_back(best);
  }
  //  sort the best....
  std::sort(best_matches.begin(),best_matches.end());
  
  //  now greedily assign, using threshold as secondary check...
  assert(best_matches.size() == N);
  results.resize(N);
  size_t idx=0;
  for (const Match& match : best_matches) {
    if (match.score > 100) {
      //  reject, mark as lost
      results[match.i] = -1;
    } else {
      results[match.i] = static_cast<int>(match.j);
    }
    idx++;
  }
}

} //  fruit_tracker
} //  galt
