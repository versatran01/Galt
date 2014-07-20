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
   *  invalid_argument if wavelengths are not in ascending order.
   */
  Spectrum(const std::vector<double>& wavelengths, 
           const std::vector<double>& intensities);
  
  /**
   * @brief getWavelengths
   * @return 
   */
  const std::vector<double>& getWavelengths() const;
  
  /**
   * @brief getIntensities
   * @return 
   */
  const std::vector<double>& getIntensities() const;
  
  /**
   * @brief size
   * @return 
   */
  std::size_t size() const;
  
  /**
   * @brief 
   * @param out
   * @param wavelengths
   * @return 
   */
  void resample(Spectrum& out, const std::vector<double>& wavelengths);
  
  /**
   * @brief Multiply two spectra together.
   * @note In general A*B =/= B*A due to potential resampling.
   * @param s Spectrum to multiply against the receiver.
   * @return Product of the two spectra.
   */
  Spectrum multiply(const Spectrum& s) const;
  
private:
  std::map<double,double> spectrum_;
};

}

#endif // GALT_SPECTRAL_SPECTRUM_HPP_
