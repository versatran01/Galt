#ifndef GALT_STEREO_VO_OPTIMIZER_H_
#define GALT_STEREO_VO_OPTIMIZER_H_

#error This file has been disabled deliberately. Check CMakeLists.txt before including it.

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>

#include <image_geometry/stereo_camera_model.h>
#include "stereo_vo/common.h"
//#include "stereo_vo/keyframe.h"
#include "stereo_vo/point3d.h"

namespace galt {
namespace stereo_vo {
/// @note: There are two things to try here.
/// 1 is following StereoVOExample.cpp. Do a windowed optimization with
/// LevenbergMarquardt algorithm. This version will create a nonlinear factor
/// graph only based on the key frames and add GenericStereoFactorGraph to the
/// graph. And then adding our solvePnP results as initials. After the
/// optimization, results will be used to update the keyframe poses. The 3d
/// points calculated will be only used for getting an initial pose estimate.
/// This will be implemented in the WindowedOptimizer class.
/// 2 is following VisualISAM2Example.cpp. Which will maintain one global graph
/// and use iSAM2 to do incremental smoothinga and mapping
using image_geometry::StereoCameraModel;
/**
 * @brief The OptimizerBase class
 */
class OptimizerBase {
 public:
  virtual ~OptimizerBase() {}

  /**
   * @brief Initialize Initialize gtsam
   * This will be called when the first key frame is added
   * @param frame First key frame
   */
  virtual void Initialize(const StereoCameraModel &model) = 0;
  /**
   * @brief Optimize Optimize currently stored key frames
   * @param key_frames
   */
  virtual void Optimize(std::deque<FramePtr> &key_frames,
                        std::map<Id, Point3d>& point3s) = 0;

 protected:
  gtsam::NonlinearFactorGraph graph_;
  gtsam::noiseModel::Isotropic::shared_ptr noise_model_;
};

/**
 * @brief The WindowedOptimizer class
 */
class WindowedOptimizer : public OptimizerBase {
 public:
  using StereoFactor = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;
  virtual void Initialize(const StereoCameraModel &model) override;
  virtual void Optimize(std::deque<FramePtr> &key_frames,
                        std::map<Id, Point3d>& point3s) override;

 private:
  gtsam::Cal3_S2Stereo::shared_ptr stereo_model_;
};

class IncrementalOptimizer : public OptimizerBase {
 public:
  virtual void Initialize(const StereoCameraModel &model) override;
  
  virtual void Optimize(std::deque<FramePtr> &key_frames,
                        std::map<Id, Point3d>& point3s) override;
 private:
  gtsam::Cal3_S2::shared_ptr camera_model_;
  gtsam::Cal3_S2Stereo::shared_ptr stereo_model_;
  
  gtsam::Values::shared_ptr estimate_;
  std::shared_ptr<gtsam::NonlinearISAM> isam_;
  
  std::set<Id> prevIds_;
  
  typedef gtsam::GenericProjectionFactor<gtsam::Pose3, 
    gtsam::Point3, gtsam::Cal3_S2> ProjectionFactor;
  
  typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
    StereoProjectionFactor;
};

class G2OOptimizer : public OptimizerBase {
public:
  virtual void Initialize(const StereoCameraModel &model) override;

  virtual void Optimize(std::deque<FramePtr> &key_frames,
                        std::map<Id, Point3d>& point3s) override;
private:
  image_geometry::PinholeCameraModel left_camera_;
};

}  // namespace stereo_vo
}  // namespace galt

#endif  // GALT_STEREO_VO_OPTIMIZER_H_
