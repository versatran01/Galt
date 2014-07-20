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

#include <stdexcept>
#include <algorithm>
#include <utility>
#include <spectral/Spectrum.hpp>

namespace galt {

Spectrum::Spectrum() : wavelengths_(), intensities_() {}

Spectrum::Spectrum(const std::vector<double> &wavelengths,
                   const std::vector<double> &intensities)
    : wavelengths_(wavelengths), intensities_(intensities) {
  
  if (wavelengths.size() != intensities.size()) {
    throw std::invalid_argument("Wavelength and intensity sizes must match");
  }

  for (size_t i=0; i < wavelengths.size(); i++) {
    
  }
  
  if ( !std::is_sorted(wavelengths.begin(), wavelengths.end()) ) {
    throw std::invalid_argument("Wavelengths must be sorted");
  }
}

const std::vector<double> Spectrum::getWavelengths() const {
  return wavelengths_;
}

const std::vector<double>& Spectrum::getIntensities() const {
  return intensities_;
}

std::size_t Spectrum::size() const { return wavelengths_.size(); }

void Spectrum::resample(Spectrum& out, const std::vector<double>& wavelengths) {
  if (wavelengths.empty()) {
    throw std::invalid_argument("Sample wavelengths cannot be empty");
  }
  
  out.wavelengths_.resize(wavelengths.size());
  out.intensities_.resize(wavelengths.size());
  
  auto I = wavelengths_.begin();
  for (const double lambda : wavelengths) {
    double mul = 0.0;
    
    //  first element greater than lambda
    const auto greater = std::upper_bound(I, wavelengths_.end(), lambda);
    if (greater == wavelengths_.end()) {
      //  edge case: equal to last value
      if (wavelengths_.back() == lambda) {
        mul = 1.0;
      } else {
        //  no values greater than lambda (0 multiplier)
      }
    } else {
      auto leq = greater;
      
      while (leq > wavelengths_.begin() && *leq > lambda) {
        leq--;
      }
      
      if (leq == wavelengths_.begin()) {
        //  edge case: equal to first value
        if (wavelengths_.front() == leq) {
          
        } else {
          //  no values less than lambda
          
        }
      }
      
    }
    
    I = greater;
  }
  
  return out;
}

Spectrum Spectrum::multiply(const Spectrum& s) const {
  Spectrum result;
  result.wavelengths_.resize( size() );
  result.intensities_.resize( size() );
  
  //  re-sample the right hand side spectrum
  size_t lIndex;
  ssize_t rIndex = 0;
  for (lIndex=0; lIndex < size(); lIndex++) {
    
    const double lhs = wavelengths_[lIndex];
    while (lhs > s.wavelengths_[rIndex]) {
       
    }
  }
  
  return result;
}

}
