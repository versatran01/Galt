#include "stereo_vo/frame.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/utils.h"

namespace galt {

namespace stereo_vo {

Frame::Frame(const CvStereoImage &stereo_image)
    : is_keyframe_(false),
      stereo_image_(stereo_image) {
  static Id frame_counter = 0;
  id_ = frame_counter++;
}

void Frame::RemoveById(const std::set<Id> &ids_to_remove) {
  std::remove_if(features_.begin(), features_.end(), [&](const Feature &f) {
    return (f.init() && (ids_to_remove.find(f.id()) != ids_to_remove.end()));
  });
}

}  // namespace stereo_vo

}  // namespace galt
