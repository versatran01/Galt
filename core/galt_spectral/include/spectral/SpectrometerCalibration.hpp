/*
 * SpectrometerCalibration.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 22/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_SPECTROMETERCALIBRATION_HPP
#define GALT_SPECTRAL_SPECTROMETERCALIBRATION_HPP

#include <spectral/Spectrum.hpp>
#include <vector>

namespace galt {

/**
 * @brief Data collected during calibration.
 * @note The calibration itself is not included here, but all relevant data
 * are stored for post-processing.
 */
struct SpectrometerCalibration {

  /**
   * @brief A sample collected during the calibration.
   */
  struct Sample {
    double reflectance;          /// Known sample reflectance.
    galt::Spectrum measurement;  /// Measured spectra.
  };

  std::string calibrationDate;  /// Date of calibration.

  galt::Spectrum sourceSpectrum;  /// Spectrum of source used in calibration.

  std::vector<Sample> sampleSpectra;  /// Samples collected during calibration.
};
}

namespace YAML {

/**
 * @brief Encodes SpectrometerCalibration to and from yaml.
 */
template <>
struct convert<galt::SpectrometerCalibration> {
  static Node encode(const galt::SpectrometerCalibration &rhs);
  static bool decode(const Node &node, galt::SpectrometerCalibration &rhs);
};
}

/**
 * @brief Emit YAML for a spectrometer calibration.
 * @param out Emitter to encode to.
 * @param pose Filter profile to encode.
 * @return Emitter.
 */
YAML::Emitter &operator<<(YAML::Emitter &out,
                          const galt::SpectrometerCalibration &calib);

#endif  // GALT_SPECTRAL_SPECTROMETERCALIBRATION_HPP
