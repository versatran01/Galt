//  Misc things for pose and spec calib.
//  TODO: Refactor everything here when the application itself is broken up.

#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <vector>
#include <opencv2/opencv.hpp>

template <typename Scalar>
cv::Point_<Scalar> distortPoint(const std::vector<Scalar>& coeffs, const cv::Point_<Scalar>& src)
{
  Scalar k[8] = {0,0,0,0,0,0,0,0};
  for (size_t i=0; i < coeffs.size(); i++) {
    k[i] = coeffs[i];
  }
  
  const Scalar xx = src.x*src.x;
  const Scalar yy = src.y*src.y;
  const Scalar xy = src.x*src.y;
  
  const Scalar r2 = xx + yy;
  const Scalar rad = (1 + r2*(k[0] + r2*(k[1] + k[4]*r2))) / (1 + r2*(k[5] + r2*(k[6] + k[7]*r2)));
  
  //  distort
  const Scalar x = src.x*rad + 2*k[2]*xy + k[3]*(r2 + 2*xx);
  const Scalar y = src.y*rad + k[2]*(r2 + 2*yy) + 2*k[3]*xy;
  
  cv::Point_<Scalar> dst;
  dst.x = x;
  dst.y = y;
  return dst;
}

#endif // UTILITIES_HPP
