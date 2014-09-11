#ifndef GALT_STEREO_VO_TRACKER_H_
#define GALT_STEREO_VO_TRACKER_H_

#include <stereo_vo/common.h>
#include <stereo_vo/feature.h>

namespace galt {
namespace stereo_vo {

class Tracker {
 public:
  Tracker();
  Tracker(const Tracker&) = delete;
  
  /**
   * @brief AddFeatures
   * @param nfeats New features to add.
   */
  void AddFeatures(const std::vector<Feature>& nfeats);
  
  /**
   * @brief Track
   * @param from
   * @param to
   */
  void Track(const cv::Mat& from, const cv::Mat& to);

  const cv::Size& window_size() const { return window_size_; }
  void set_window_size(const cv::Size& sz) { window_size_ = sz; }
  
  const size_t& max_levels() const { return max_levels_; }
  void set_max_levels(const size_t& lev) { max_levels_ = lev; }
  
  const double& ransac_thresh() const { return ransac_thresh_; }
  void set_ransac_thresh(const double& thresh) { ransac_thresh_ = thresh; }
  
  const std::vector<Feature>& features() const { return features_; }
  
  bool empty() const { return features_.empty(); }
  
 private:
  std::vector<Feature> features_;
  cv::Size window_size_;  /// Default to (21,21)
  size_t max_levels_;     /// Default to 3
  double ransac_thresh_;  /// Default to 1
  
  /// Erase vector based on status flags.
  template <typename T>
  static void EraseByStatus(std::vector<T>& vec, 
                       const std::vector<uchar>& status) {
    assert(vec.size() == status.size());
    auto ite_vec = vec.begin();
    for (const uchar& flag : status) {
      if (!flag) {
        ite_vec = vec.erase(ite_vec);
      } else {
        ite_vec++;
      }
    }
  }
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_TRACKER_H_
