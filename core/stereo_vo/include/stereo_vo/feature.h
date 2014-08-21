#ifndef GALT_STEREO_VO_FEATURE_H_
#define GALT_STEREO_VO_FEATURE_H_

#include <memory>
#include <functional>
#include <cstdint>

#include <stereo_vo/common.h>
#include <image_geometry/stereo_camera_model.h>

namespace galt {
namespace stereo_vo {

/**
 * @brief The Feature class
 * A feature is just a pixel on the image picked out by corner detector. Each
 * feature will have its unique id.
 */
class Feature {
 public:
  static Id feature_count;

  /**
   * @brief Feature Default constructor
   */
  Feature() = default;
  /**
   * @brief Feature Constructor Create a feature
   * @param id Unique id of this feature
   * @param p_pixel Pixel position of this feature
   * @param init If this feature is newly detected
   */
  Feature(const Id& id, const CvPoint2& p_pixel, const bool& init = true)
      : id_(id), p_pixel_(p_pixel), init_(init) {}
  /**
   * @brief Feature Constructor Create a newly detected feautre
   * @param p_pixel Pixel position of this feature
   */
  Feature(const CvPoint2& p_pixel) : Feature(feature_count++, p_pixel, true) {}

  const Id& id() const { return id_; }
  void set_id(const Id& id) { id_ = id; }

  const CvPoint2& p_pixel() const { return p_pixel_; }
  void set_p_pixel(const CvPoint2& p_pixel) { p_pixel_ = p_pixel; }

  const CvPoint2& p_right() const { return p_right_; }
  void set_p_right(const CvPoint2& p_right) { p_right_ = p_right; }

  const bool& init() const { return init_; }
  void set_init(const bool& init) { init_ = init; }

 private:
  Id id_;             ///< id of this feature
  CvPoint2 p_pixel_;  ///< pixel position
  CvPoint2 p_right_;  ///< pixel position in right image
  bool init_;         ///< true if this feature was newly added
};

/**
 * @brief ExtractCorners Extract corners from a vector of features
 * @param features A vector of features
 * @return A vector of corners
 */
std::vector<CvPoint2> ExtractCorners(const std::vector<Feature>& features);

/**
 * @brief ExtractIds Extract ids from a vector of features
 * @param features A vector of features
 * @return A vector of ids
 */
std::vector<Id> ExtractIds(const std::vector<Feature>& features);

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_FEATURE_H_
