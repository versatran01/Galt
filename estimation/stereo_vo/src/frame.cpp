#include "stereo_vo/frame.h"

#include <opencv2/video/video.hpp>

namespace galt {
namespace stereo_vo {

Id KeyFrame::counter = 0;

/*
size_t Frame::RemoveById(const std::set<Id> &ids_to_remove, bool force) {
  //  features_.erase(
  //      std::remove_if(features_.begin(), features_.end(), [&](const Feature
  // &f) {
  //        return (f.init() &&
  //                (ids_to_remove.find(f.id()) != ids_to_remove.end()));
  //      }),
  //      features_.end());

  size_t erased = 0;
  for (auto ite = features_.begin(); ite != features_.end();) {
    bool erase = false;
    if (ite->init() || force) {
      if (ids_to_remove.find(ite->id()) != ids_to_remove.end()) {
        erase = true;
      }
    }
    if (erase) {
      ite = features_.erase(ite);
      erased++;
    } else {
      ite++;
    }
  }

  if (features_.empty()) {
    ROS_WARN("Frame %lu is empty", id());
  }

  return erased;
}
*/

}  // namespace stereo_vo
}  // namespace galt
