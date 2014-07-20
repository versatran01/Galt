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
   * @brief Spectrum
   * @param spec
   */
  // Spectrum(const Spectrum& spec);

  /**
   * @brief Spectrum
   * @param spec
   */
  // explicit Spectrum(Spectrum&& spec);

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
   * @note Is s uses different wavelengths than the receiver, it will be
   * resampled to match the receiver. In general, there is no guarantee that 
   * A*B == B*A.
   */
  void multiply(const Spectrum &s);

private:
  std::vector<double> wavelengths_;
  std::vector<double> intensities_;
};
}

#endif // GALT_SPECTRAL_SPECTRUM_HPP_
