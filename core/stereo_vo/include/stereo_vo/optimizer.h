#ifndef GALT_STEREO_VO_OPTIMIZER_H_
#define GALT_STEREO_VO_OPTIMIZER_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/slam/StereoFactor.h>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

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
/**
 * @brief The OptimizerBase class
 */
class OptimizerBase {
 public:
  OptimizerBase() = default;
  virtual ~OptimizerBase() {}

 protected:
  gtsam::NonlinearFactorGraph graph_;
};

/**
 * @brief The WindowedOptimizer class
 */
class WindowedOptimizer : public OptimizerBase {};

class IncrementalOptimizer : public OptimizerBase {};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_OPTIMIZER_H_
