/*
 * SensorPoseConfig.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 16/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_SENSORPOSECONFIG_HPP_
#define GALT_SPECTRAL_SENSORPOSECONFIG_HPP_

#include <kr_math/base_types.hpp>
#include <kr_math/yaml.hpp>

namespace galt {

/**
 * @brief Describes the pose of a spectrometer effector in a camera frame.
 */
class SpectrometerPose {
public:
  SpectrometerPose();

  SpectrometerPose(const kr::vec3d &position, const kr::vec3d &direction,
                   double fov);

  kr::vec3d getPosition() const;

  kr::vec3d getDirection() const;

  double getFov() const;

private:
  kr::vec3d position_;
  kr::vec3d direction_;
  double fov_;
};

} // namespace galt

namespace YAML {

/**
 * @brief Encodes SpectrometerPose to and from yaml.
 */
template <> struct convert<galt::SpectrometerPose> {
  static Node encode(const galt::SpectrometerPose &rhs);
  static bool decode(const Node &node, galt::SpectrometerPose &rhs);
};
}

#endif // GALT_SPECTRAL_SENSORPOSECONFIG_HPP_
