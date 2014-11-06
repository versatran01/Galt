/*
 * Spectrum.hpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#ifndef GALT_SPECTRAL_SPECTRUM_HPP_
#define GALT_SPECTRAL_SPECTRUM_HPP_

#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>

namespace galt {

/**
 * @brief Describes measured intensity of the spectrum at certain wavelengths.
 */
class Spectrum {
 public:
  /**
   * @brief Spectrum initializes empty.
   */
  Spectrum();

  /**
   * @brief Spectrum Construct a new spectrum.
   * @param wavelengths Wavelengths in nm, must be in sorted order.
   * @param intensities Relative measure of intensity for each wavelength.
   *
   * @throws invalid_argument if vectors do not match in size.
   * invalid_argument if wavelengths are not in ascending order.
   */
  Spectrum(const std::vector<double> &wavelengths,
           const std::vector<double> &intensities);

  /**
   * @brief Spectrum Construct a spectrum from a map.
   * @param data Mapping of wavelength to intensity.
   */
  Spectrum(const std::map<double, double> &data);

  /**
   * @brief Get the wavelengths in this spectrum.
   * @return
   */
  const std::vector<double> &getWavelengths() const;

  /**
   * @brief Get the intensities in this spectrum.
   * @return
   */
  const std::vector<double> &getIntensities() const;

  /**
   * @brief Number of points in the spectrum.
   * @return std::size_t
   */
  std::size_t size() const;

  /**
   * @brief Check if this spectrum has an exact set of wavelengths.
   * @param wavelengths Wavelengths to compare with.
   * @return True if all the wavelengths are the same.
   *
   * @note This should be the case if both spectra come from the same sensor.
   */
  bool hasWavelengths(const std::vector<double> &wavelengths) const;

  /**
   * @brief Sample the receiver at specific wavelengths to create a new
   * spectrum.
   * @param wavelengths Wavelengths at which to sample.
   * @throws invalid_argument if wavelengths is empty.
   *
   * @note Linear interpolation is used to sample between wavelengths.
   * Wavelengths falling outside the range of the receiver receive an intensity
   * of zero.
   */
  void resample(const std::vector<double> &wavelengths);

  /**
   * @brief Multiply two spectra together, storing the result in place.
   * @param s Spectrum to multiply against the receiver.
   *
   * @note If s uses different wavelengths than the receiver, it will be
   * resampled to match the receiver. In general, there is no guarantee that
   * A*B == B*A.
   */
  void multiply(const Spectrum &s);

  /**
   * @brief Take 1/x where x are the intensity values.
   * @note Does not check for 0 before division.
   */
  void invert();

  /**
   * @brief Scale the intensities by a scalar value.
   * @param s Value by which to scale.
   */
  void scale(double s);

  /**
   * @brief Integrate numerically to obtain the area under the spectrum.
   * @note Uses the trapezoid technique. For spectra with only one sample, this
   * method will return zero.
   * @return Total area under the curve.
   */
  double integrate() const;

 private:
  std::vector<double> wavelengths_;
  std::vector<double> intensities_;
};
}

namespace YAML {

/**
 * @brief Encodes Spectrum to and from yaml.
 */
template <>
struct convert<galt::Spectrum> {
  static Node encode(const galt::Spectrum &rhs);
  static bool decode(const Node &node, galt::Spectrum &rhs);
};
}

/**
 * @brief Emit YAML for a Spectrum pose.
 * @param out Emitter to encode to.
 * @param pose Spectrum to encode.
 * @return Emitter.
 */
YAML::Emitter &operator<<(YAML::Emitter &out, const galt::Spectrum &pose);

#endif  // GALT_SPECTRAL_SPECTRUM_HPP_
