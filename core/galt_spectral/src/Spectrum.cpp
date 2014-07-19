/*
 * Spectrum.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#include <spectral/Spectrum.hpp>

namespace galt {

Spectrum::Spectrum() : wavelengths_(), intensities_() {}

Spectrum::Spectrum(const std::vector<double> &wavelengths,
                   const std::vector<double> &intensities)
    : wavelengths_(wavelengths), intensities_(intensities) {}
}
