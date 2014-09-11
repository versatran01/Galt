#include <stereo_vo/tracker.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace galt {
namespace stereo_vo {

Tracker::Tracker() : window_size_(21,21), max_levels_(3), ransac_thresh_(1) {}

void Tracker::AddFeatures(const std::vector<Feature>& nfeats) {
  features_.reserve(features_.size() + nfeats.size());
  features_.insert(features_.end(), nfeats.begin(), nfeats.end());
}

void Tracker::Track(const cv::Mat& from, const cv::Mat& to) {
  assert(!from.empty() && !to.empty());
  
  if (!features_.empty()) {
    std::vector<CvPoint2> source;
    std::vector<CvPoint2> dest;
    std::vector<uchar> status;
    std::vector<float> errors;
    
    for (const Feature& f : features_) {
      source.push_back(f.p_pixel());
    }
    //  perform optical flow calculation
    cv::calcOpticalFlowPyrLK(from,to,source,dest,
                             status,errors,window_size(),max_levels());
    
    //  remove outliers
    EraseByStatus(source,status);
    EraseByStatus(dest,status);
    EraseByStatus(features_,status);  /// @todo: record ids somewhere?
    status.clear();
    
    if (!source.empty()) {
      cv::findFundamentalMat(source,dest,status,
                             cv::FM_RANSAC,ransac_thresh(),0.99);
      //  update features with new positions
      EraseByStatus(dest,status);
      EraseByStatus(features_,status);
      for (size_t i=0; i < dest.size(); i++) {
        features_[i].set_p_pixel(dest[i]);
      }
    }
  }
}

}
}
