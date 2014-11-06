/*
 * CameraCalibration.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 21/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_CAMERACALIBRATION_HPP_
#define GALT_SPECTRAL_CAMERACALIBRATION_HPP_

#include <spectral/SpectrometerPose.hpp>
#include <spectral/FilterProfile.hpp>
#include <yaml-cpp/yaml.h>

namespace galt {

/**
 * @brief Result of a multispectral camera calibration session.
 */
struct CameraCalibration {

  /**
   * @brief CameraCalibration constructor. Initializes defaults.
   */
  CameraCalibration();

  std::string cameraSerial;     /// Camera serial number or name.
  int cameraExposure;           /// Camera exposure in microseconds.
  std::string calibrationDate;  /// Date of calibration in ISO8601 format.

  double slope;         /// Slope of the calibration.
  double intercept;     /// Intercept of the calibration
  double squaredError;  /// Mean square error in the calibration

  galt::SpectrometerPose spectrometerPose;  /// Pose at time of calibration.
  galt::FilterProfile filterProfile;  /// Profile of the filter on the camera.
};
}

namespace YAML {
/**
 * @brief Encodes CameraCalibration to and from yaml.
 */
template <>
struct convert<galt::CameraCalibration> {
  static Node encode(const galt::CameraCalibration &rhs);
  static bool decode(const Node &node, galt::CameraCalibration &rhs);
};
}

/**
 * @brief Emit YAML for a camera calibration.
 * @param out Emitter to encode to.
 * @param pose Filter profile to encode.
 * @return Emitter.
 */
YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::CameraCalibration &calib);

#endif  // GALT_SPECTRAL_CAMERACALIBRATION_HPP_
