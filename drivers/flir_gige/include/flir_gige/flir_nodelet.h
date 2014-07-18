#ifndef FLIR_GIGE_FLIR_NODELET_H_
#define FLIR_GIGE_FLIG_NODELET_H_

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "flir_gige/flir_gige.h"

namespace flir_gige {

class FlirNodelet : public nodelet::Nodelet {
 public:
  FlirNodelet();
  ~FlirNodelet();

  FlirNodelet(const FlirNodelet&) = delete;
  FlirNodelet& operator=(const FlirNodelet&) = delete;

  virtual void onInit();

 private:
  std::unique_ptr<flir_gige::FlirGige> flir_gige_;

};  // class FlirNodelet

}  // namespace flir_gige

#endif  // FLIR_GIGE_FLIR_NODELET_H_
