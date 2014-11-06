/*
 * SpectrometerPose.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 16/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_SPECTROMETERPOSE_HPP_
#define GALT_SPECTRAL_SPECTROMETERPOSE_HPP_

#include <kr_math/base_types.hpp>
#include <kr_math/yaml.hpp>

// TODO: finish documenting
namespace galt {

/**
 * @brief Describes the pose of a spectrometer effector in a camera frame.
 */
class SpectrometerPose {
 public:
  SpectrometerPose();

  SpectrometerPose(const kr::Vec3d &position, const kr::Vec3d &direction,
                   double fov, double squaredError);

  const kr::Vec3d &getPosition() const;

  const kr::Vec3d &getDirection() const;

  const double &getFov() const;

  const double &getSquaredError() const;

  /**
   * @brief distanceToPlane Distance to plane defined by [o,n].
   * @param o A point in the plane, in camera frame.
   * @param n Normal of the plane, in camera frame.
   * @return Distance along the spectrometer direction to the plane, or
   *  std::numeric_limits::infinity() if the plane is perp. to the sensor.
   */
  double distanceToPlane(const kr::Vec3d &o, const kr::Vec3d &n) const;

 private:
  kr::Vec3d position_;
  kr::Vec3d direction_;
  double fov_;
  double squaredError_;
};

}  // namespace galt

namespace YAML {

/**
 * @brief Encodes SpectrometerPose to and from yaml.
 */
template <>
struct convert<galt::SpectrometerPose> {
  static Node encode(const galt::SpectrometerPose &rhs);
  static bool decode(const Node &node, galt::SpectrometerPose &rhs);
};
}

/**
 * @brief Emit YAML for a spectrometer pose.
 * @param out Emitter to encode to.
 * @param pose Pose to encode.
 * @return Emitter.
 */
YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::SpectrometerPose &pose);

#endif  // GALT_SPECTRAL_SPECTROMETERPOSE_HPP_
