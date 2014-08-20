#include "stereo_vo/optimizer.h"
#include "stereo_vo/frame.h"

#include <ros/ros.h>

namespace galt {

namespace stereo_vo {

using namespace gtsam;

void WindowedOptimizer::Initialize(const Frame &frame,
                                   const StereoCameraModel &model) {
  noise_model_ = noiseModel::Isotropic::Sigma(3, 1);
  const image_geometry::PinholeCameraModel &left = model.left();
  stereo_model_.reset(new Cal3_S2Stereo(left.fx(), left.fy(), 0, left.cx(),
                                        left.cy(), model.baseline()));

  ROS_INFO("Initializing windowed optimizer");
}

void WindowedOptimizer::Optimize(std::deque<FramePtr> &key_frames) {
  for (const FramePtr &frame_ptr : key_frames) {
    const std::vector<Feature> &features = frame_ptr->features();
    for (const Feature &feature : features) {

    }
  }
  ROS_INFO("Optimize %i key frames", (int)key_frames.size());
}


//  --------------------
//  IncrementalOptimizer
//  --------------------

void IncrementalOptimizer::Initialize(const Frame &frame,
                                      const StereoCameraModel &model) {
  // Define the camera observation noise model, 1 pixel in u and v
  noise_model_ = noiseModel::Isotropic::Sigma(2, 1.0);
  const image_geometry::PinholeCameraModel &left = model.left();
  camera_model_.reset(
      new Cal3_S2(left.fx(), left.fy(), 0, left.cx(), left.cy()));

  ROS_INFO("Initializing incremental optimizer");
}

void IncrementalOptimizer::Optimize(std::deque<FramePtr> &key_frames) {
  
  bool first_run = false;
  if (!isam_) {
    //  initialize a new ISAM optimzer
    /// @todo: Make the interval (3) a parameter
    isam_ = std::make_shared<gtsam::NonlinearISAM>(3);
    first_run = true;
  }
  
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values estimate;
  
  //  take the most recent key frame
  const FramePtr& frame_ptr = key_frames.back();
  const Frame& frame = *frame_ptr;
  
  //  pose estimate and symbol for this frame
  gtsam::Pose3 pose3(frame.pose().matrix().cast<double>());
  gtsam::Symbol frame_sym('x', frame.id());
  
  //  consider all observations associated with this frame
  for (const Feature& feature : frame.features()) {
    //  point where the feature was observed
    gtsam::Point2 p2(feature.p_pixel().x, feature.p_pixel().y);
    
    //  create a projection residual
    gtsam::Symbol point_sym('p', feature.id());
    ProjectionFactor factor(p2,*noise_model_,frame_sym,point_sym,camera_model_);
    graph.push_back(factor);
    
    if (feature.init()) {
      //  feature is new, so provide iSAM with an initial guess
      gtsam::Point3 guess();
      estimate.insert(point_sym,)
    }
  }
  
  //  insert initial estimate for pose
  estimate.insert(frame_sym, pose3);
    
      
  if (first_run) {
    
  } else {
    //  update iSAM
    isam_->update(graph,estimate);
  }
}

}  // namespace stereo_vo

}  // namespace galt
