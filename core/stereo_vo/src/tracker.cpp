#include <stereo_vo/tracker.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace galt {
namespace stereo_vo {

Tracker::Tracker() : window_size_(21, 21), max_levels_(3), ransac_thresh_(1) {}

void Tracker::AddFeatures(const std::vector<Feature>& nfeats) {
  features_.insert(features_.end(), nfeats.begin(), nfeats.end());
}

std::set<Id> Tracker::Track(const cv::Mat& from, const cv::Mat& to,
                            std::vector<Feature>& originals) {
  assert(!from.empty() && !to.empty());
  std::set<Id> erased;
  if (!features_.empty()) {
    originals.clear();
    std::vector<CvPoint2> source;
    std::vector<CvPoint2> dest;
    std::vector<uchar> status;
    std::vector<float> errors;

    for (const Feature& f : features_) {
      source.push_back(f.p_pixel());
    }
    //  perform optical flow calculation
    cv::calcOpticalFlowPyrLK(from, to, source, dest, status, errors,
                             window_size(), max_levels());

    //  check output values
    for (size_t i = 0; i < dest.size(); i++) {
      const CvPoint2& p = dest[i];
      if (p.x < 4 || p.y < 4 || p.x > to.cols - 4 || p.y > to.rows - 4) {
        status[i] = 0;  //  reject
      }
    }

    //  remove outliers
    EraseByStatus(source, status);
    EraseByStatus(dest, status);
    EraseByStatus(features_, status, erased);
    status.clear();

    if (!source.empty()) {
      cv::findFundamentalMat(source, dest, status, cv::FM_RANSAC,
                             ransac_thresh(), 0.999);
      //  update features with new positions
      EraseByStatus(dest, status);
      EraseByStatus(features_, status, erased);
      for (size_t i = 0; i < dest.size(); i++) {
        originals.push_back(features_[i]);
        features_[i].set_p_pixel(dest[i]);
      }
    }
  }
  return erased;
}

std::set<Id> Tracker::RetainInliers(const std::vector<int>& inliers) {
  std::set<Id> outliers;
  std::vector<uchar> status(features_.size(), 0);
  for (const int& idx : inliers) {
    assert(static_cast<unsigned>(idx) < status.size());
    status[idx] = 1;  //  retain this feature
  }
  EraseByStatus(features_, status, outliers);
  return outliers;
}

void Tracker::EraseByStatus(std::vector<Feature>& vec,
                            const std::vector<uchar>& status,
                            std::set<Id>& erased) {
  assert(vec.size() == status.size());
  auto ite_vec = vec.begin();
  for (const uchar& flag : status) {
    if (!flag) {
      //  save id of all erased features
      erased.insert(ite_vec->id());
      ite_vec = vec.erase(ite_vec);
    } else {
      ite_vec++;
    }
  }
}
}
}
