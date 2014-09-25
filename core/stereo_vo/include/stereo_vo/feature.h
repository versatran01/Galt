#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <stereo_vo/common.h>

namespace galt {
namespace stereo_vo {

struct Feature {
 public:
  static Id counter;

  Feature() = default;
  Feature(const CvPoint2& px) : Feature(counter++, px, true) {}
  Feature(const Id& id, const CvPoint2& px, const bool& fresh = true)
      : id(id), px(px), fresh(fresh) {}

  Id id;
  CvPoint2 px;
  CvPoint3 p_world; /// < Position world frame.
  CvPoint3 p_cam;   /// < Position in camera frame.
  kr::mat3<scalar_t> p_cov_cam; ///< Covariance in camera frame.
  kr::mat3<scalar_t> p_cov_world; ///< Covariance in world frame.
  bool fresh;
  int level;
};

void ExtractCorners(const std::vector<Feature>& features,
                    std::vector<CvPoint2>& corners);
void ExtractIds(const std::vector<Feature>& features, std::vector<Id>* ids);
void MakeFeaturesOld(std::vector<Feature>& features);

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
