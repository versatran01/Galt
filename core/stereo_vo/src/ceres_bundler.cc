#include "stereo_vo/ceres_bundler.h"
#include "stereo_vo/cost_functor.h"

namespace galt {
namespace stereo_vo {

void CeresBundler::Optimize(std::deque<KeyFrame> &key_frames, int win_size) {
  winSize_ = win_size;
  NukeEverything();
  SplitFeatureIds(key_frames);
  CreateGraph(key_frames);
  SolveProblem();
  UpdateKeyFrames(key_frames);
}

void CeresBundler::CreateGraph(const std::deque<KeyFrame> &key_frames) {
  
}

void CeresBundler::NukeEverything(bool from_orbit) {
  storage_.clear();
  fixed_.clear(); 
  edges_.clear();
  mutables_.clear();
  immutables_.clear();
  cameras_.clear();
  point3s_.clear();
}

void CeresBundler::SplitFeatureIds(const std::deque<KeyFrame> &key_frames) {
  
}

void CeresBundler::AddResidualBlock(const Edge &edge) {
  
}

void CeresBundler::SolveProblem() {
  
}

void CeresBundler::UpdateKeyFrames(std::deque<KeyFrame> &key_frames) {
  
}

}
}
