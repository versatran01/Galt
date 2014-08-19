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

void IncrementalOptimizer::Initialize(const Frame &frame,
                                      const StereoCameraModel &model) {
  // Define the camera observation noise model, 1 pixel in u and v
  noise_model_ = noiseModel::Isotropic::Sigma(2, 1.0);
  const image_geometry::PinholeCameraModel &left = model.left();
  camera_model_.reset(
      new Cal3_S2(left.fx(), left.fy(), 0, left.cx(), left.cy()));

  ROS_INFO("Initializing windowed optimizer");
}

}  // namespace stereo_vo

}  // namespace galt
