#ifndef GALT_STEREO_VO_COST_FUNCTOR_H_
#define GALT_STEREO_VO_COST_FUNCTOR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <image_geometry/pinhole_camera_model.h>

#include "stereo_vo/common.h"

namespace galt {

namespace stereo_vo {

using image_geometry::PinholeCameraModel;

struct ReprojectionError {
  ReprojectionError(double x, double y, const PinholeCameraModel& model)
      : x(x), y(y), model(model) {}

  template <typename T>
  bool operator()(const T* const camera, const T* const point3,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    /*T p[3];
    // Rotates point from world frame to camera frame
    ceres::AngleAxisRotatePoint(camera, point3, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Compute final projected point position.
    T predicted_x = T(model.fx()) * xp + T(model.cx());
    T predicted_y = T(model.fy()) * yp + T(model.cy());

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(x);
    residuals[1] = predicted_y - T(y);*/
    residuals[0] = T(0);
    residuals[1] = T(0);
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double x, const double y,
                                     const PinholeCameraModel& model) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
        new ReprojectionError(x, y, model)));
  }

  double x;
  double y;
  PinholeCameraModel model;
};

struct FixedReprojectionError {
  FixedReprojectionError(double x, double y, const PinholeCameraModel& model,
                         const double* point3)
      : x(x), y(y), model(model), point3(point3) {}

  template <typename T>
  bool operator()(const T* const camera, T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    /*T p[3];
    T p_world[3];
    p_world[0] = T(point3[0]);
    p_world[1] = T(point3[1]);
    p_world[2] = T(point3[2]);
    // Rotates point from world frame to camera frame
    ceres::AngleAxisRotatePoint(camera, p_world, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Compute final projected point position.
    T predicted_x = T(model.fx()) * xp + T(model.cx());
    T predicted_y = T(model.fy()) * yp + T(model.cy());

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(x);
    residuals[1] = predicted_y - T(y);*/
    residuals[0] = T(0);
    residuals[1] = T(0);
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double x, const double y,
                                     const PinholeCameraModel& model,
                                     const double* point3) {
    return (new ceres::AutoDiffCostFunction<FixedReprojectionError, 2, 6>(
        new FixedReprojectionError(x, y, model, point3)));
  }

  double x;
  double y;
  PinholeCameraModel model;
  const double* point3;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_COST_FUNCTOR_H_
