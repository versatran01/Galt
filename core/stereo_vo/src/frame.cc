#include "stereo_vo/frame.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/utils.h"

namespace galt {

namespace stereo_vo {

Id Frame::frame_counter = 0;

void Frame::RemoveById(const std::set<Id> &ids_to_remove) {
  std::remove_if(features_.begin(), features_.end(), [&](const Feature &f) {
    return (f.init() && (ids_to_remove.find(f.id()) != ids_to_remove.end()));
  });
}

}  // namespace stereo_vo

}  // namespace galt
