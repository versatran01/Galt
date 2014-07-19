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
  
  Spectrum();
  
  Spectrum(const std::vector<double>& wavelengths, 
           const std::vector<double>& intensities);
  
private:
  std::vector<double> wavelengths_;
  std::vector<double> intensities_;
};

}

#endif // GALT_SPECTRAL_SPECTRUM_HPP_
