#ifndef GALT_SAM_ESTIMATOR_HPP_
#define GALT_SAM_ESTIMATOR_HPP_

#include <sam_estimator/gtsam.hpp>

namespace galt {
namespace sam_estimator {

class SamEstimator {
public:
  
  SamEstimator();
  
  void addImu();
  
  void addOdometry();
  
  void addGps();
  
  void addLidar();
  
private:
};

}
}

#endif
