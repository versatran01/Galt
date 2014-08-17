#ifndef GALT_STEREO_VO_OPTIMIZER_H_
#define GALT_STEREO_VO_OPTIMIZER_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace galt {

namespace stereo_vo {

class OptimizerBase {
 public:
  OptimizerBase() = default;
  virtual ~OptimizerBase() {}

 protected:
  gtsam::NonlinearFactorGraph graph_;
};

class WindowedOptimizer : public OptimizerBase {};

class IncrementalOptimizer : public OptimizerBase {};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_OPTIMIZER_H_
