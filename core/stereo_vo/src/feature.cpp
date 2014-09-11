#include "stereo_vo/feature.h"

namespace galt {
namespace stereo_vo {

Id Feature::feature_count = 0;

std::vector<CvPoint2> ExtractCorners(const std::vector<Feature>& features) {
  std::vector<CvPoint2> corners;
  for (const Feature& feature : features) corners.push_back(feature.p_pixel());
  return corners;
}

std::vector<Id> ExtractIds(const std::vector<Feature>& features) {
  std::vector<Id> ids;
  for (const Feature& feature : features) ids.push_back(feature.id());
  return ids;
}

void MakeFeaturesOld(std::vector<Feature>& features) {
  std::for_each(features.begin(), features.end(),
                [](Feature& feature) { feature.set_init(false); });
}

}  // namespace stereo_vo
}  // namespace galt
