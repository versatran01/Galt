#include "stereo_vo/ceres_bundler.h"
#include "stereo_vo/cost_functor.h"

namespace galt {
namespace stereo_vo {

void CeresBundler::Optimize(std::deque<KeyFrame> &key_frames,
                            const image_geometry::StereoCameraModel& cameraModel,
                            int win_size) {
  winSize_ = win_size;
  model_ = cameraModel;
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
  //  iterate over key frames and sort all features into one of two groups
  size_t kf_count=0;
  for (auto kfi = key_frames.rbegin(); kfi != key_frames.rend(); kfi++) {
   
    //  is this key frame in the current window?
    const bool in_window = (kf_count < winSize_);
    const KeyFrame& kf = *kfi;
    
    bool features_found = false;
    for (const std::pair<Feature::Id,Feature>& feat : kf.features()) {
      const Feature::Id id = feat.second.id();
      
      if (in_window) {
        //  assume this is a mutable point, since it is in the window
        mutables_.insert(id);
      } else {
        //  this point is also in past keyframes, move it to immutables
        const auto ite = mutables_.find(id);
        if (ite != mutables_.end()) {
          mutables_.erase(ite);
          immutables_.insert(id);
          features_found = true;
        } else {
          //  nothing to be done, this point is irrelevant to us
        }
      }
    }
    
    if (!features_found) {
      //  no more features found in this keyframe, we can bail out early
      break;
    }
    
    kf_count++;
  }
}

void CeresBundler::AddResidualBlock(const Edge &edge, image_geometry::PinholeCameraModel &model) {
  
  //  create residuals for this edge
  auto cam_ite = cameras_.find(edge.cam_id());
  auto point_ite = point3s_.find(edge.pt3_id());
  
  ROS_ASSERT_MSG((cam_ite != cameras_.end()) && 
                 (point_ite != point3s_.end()), "Graph is malformed");
  
  const CameraNode& cam = cam_ite->second;
  const Point3Node& point = point_ite->second;
  
  ceres::CostFunction * func=0;
  if (point.locked()) {
    //  this point appears in a previous frame, use fixed residual
    //  2 x 6 cost function
    func = FixedReprojectionError::Create(edge.x(), edge.y(), 
                                          model, point.ptr());
    
    problem_.AddResidualBlock(func, NULL, cam.ptr());
  } else {
    //  optimize the point also
    //  2 x (6+3) cost function
    func = ReprojectionError::Create(edge.x(), edge.y(), model);
    problem_.AddResidualBlock(func, NULL, cam.ptr(), point.ptr());
  }
}

void CeresBundler::SolveProblem() {
  
  //  configure ceres options, hardcode most of these for now
  options_ = ceres::Solver::Options();
  options_.max_num_iterations = 5;
  options_.num_threads = 1;
  options_.max_solver_time_in_seconds = 10;
  options_.gradient_tolerance = 1e-16;
  options_.gradient_tolerance = 1e-12;
  options_.use_inner_iterations = true;
  options_.linear_solver_type = ceres::SPARSE_SCHUR;
  options_.preconditioner_type = ceres::JACOBI;
  options_.visibility_clustering_type = ceres::CANONICAL_VIEWS;
  options_.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options_,&problem_,&summary);
  
  ROS_INFO_STREAM("Solving summary: " << summary.FullReport() << "\n");
}

void CeresBundler::UpdateKeyFrames(std::deque<KeyFrame> &key_frames) {
  
  //  update 
  
}

}
}
