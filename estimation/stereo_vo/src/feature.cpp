#include "stereo_vo/feature.h"

namespace galt {
namespace stereo_vo {

Id Feature::counter = 0;

void ExtractCorners(const std::vector<Feature>& features,
                    std::vector<CvPoint2>& corners) {
  std::for_each(features.cbegin(), features.cend(), [&](const Feature& f) {
    corners.emplace_back(f.px.x, f.px.y);
  });
}

void ExtractIds(const std::vector<Feature>& features, std::vector<Id>& ids) {
  std::for_each(features.cbegin(), features.cend(),
                [&](const Feature& f) { ids.emplace_back(f.id); });
}

void MakeFeaturesOld(std::vector<Feature>& features) {
  std::for_each(features.begin(), features.end(),
                [](Feature& f) { f.fresh = false; });
}

}  // namespace stereo_vo
}  // namespace galt
