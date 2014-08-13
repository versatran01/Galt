#include "stereo_vo/ceres_bundler.h"
#include "stereo_vo/cost_functor.h"

#include <Eigen/Geometry>
#include <Eigen/Core>

namespace galt {

namespace stereo_vo {

void CeresBundler::Optimize(
    std::deque<KeyFrame> &key_frames,
    const image_geometry::StereoCameraModel &cameraModel, int win_size) {
  win_size_ = win_size;
  model_ = cameraModel;
  NukeEverything();
  SplitFeatureIds(key_frames);
  CreateGraph(key_frames);
  SolveProblem();
  UpdateKeyFrames(key_frames);
}

void CeresBundler::CreateGraph(const std::deque<KeyFrame> &key_frames) {
  // Iterate through keyframe the first time to save camera poses
  NodeBase::Id id = 0;
  // Iterate through key frame backwards and then through features to save 3d
  // points
  auto itb_kf = key_frames.rbegin(), ite_kf = key_frames.rend();
  for (auto it_kf = itb_kf; it != ite_kf; ++it) {
    bool in_window = (it_kf - itb_kf) > win_size_;
    const KeyFrame &key_frame = *it_kf;
    if (in_window) {
      // Add key frame pose to camera pose
      Eigen::AngleAxis<scalar_t> aa(key_frame.pose().q);
      const auto r_vec = aa.axis() * aa.angle();
      const auto t_vec = it->pose().p;
      CameraNode c_node(id, &*storage_.rbegin());
      storage_.push_back(r_vec(0));
      storage_.push_back(r_vec(1));
      storage_.push_back(r_vec(2));
      storage_.push_back(t_vec(0));
      storage_.push_back(t_vec(1));
      storage_.push_back(t_vec(2));
      cameras_.emplace(id, c_node);
      id++;
      // Because we are in window, all the features will be either mutable or
      // immutable. So we check if feature is in mutable, if not, then it must
      // be immutable
      const auto &features = key_frame.features();
      for (const std::pair<Featuer::Id, Feature> &feature_pair : features) {
        const Feature &feature = feature_pair.second;
        kr::vec3<scalar_t> p_cam(feature.p_cam_left().x, feature.p_cam_left().y,
                                 feature.p_cam_left().z);
        auto p_world = key_frame.pose().q.matrix() * p_cam + key_frame.pose().p;
        const auto it_mut = mutables_.find(feature.id());
        if (it_mut != mutables_.end()) {
          // We found this feature in mutable, add
          Point3Node p_node(id, &*storage_.rbegin(), false);
          storage_.push_back(p_world(0));
          storage_.push_back(p_world(1));
          storage_.push_back(p_world(2));
        } else {
          // Didn't find it in mutable, it must be an immutable feature
        }
      }
    }
  }

  {
    // This key frame is in the optimization window, add it to cameras
    if (in_window) {
    }
    // Iterate through all features in this key frame
    const auto &features = key_frame.features();
    for (const std::pair<Feature::Id, Feature> &feature_pair : features) {
      // Check if this feature is in mutables or immutables
      const Feature &feature = feature_pair.second;
      const auto it_mut = mutables_.find(feature.id());
      if (it_mut != mutables_.end()) {
        // This feature is in mutables, add it to storage
        // Rotate this feature into world frame
        Point3Node p_node(id, &*storage_.rbegin(), false);
        storage_.push_back(p_world(0));
        storage_.push_back(p_world(1));
        storage_.push_back(p_world(2));
        // Add to map of points
        point3s_.emplace(id, p_node);
        edges_.emplace_back(c_node.id(), p_node.id(), feature.p_pixel().x,
                            feature.p_pixel().y);
        id++;
      } else {
        // This feature is not in mutables, check whether if it's in immutables
        const auto it_immut = immutables_.find(feature.id());
        if (it_immut != immutables_.end()) {
          // Add it to fixed
          kr::vec3<scalar_t> p_cam(feature.p_cam_left().x,
                                   feature.p_cam_left().y,
                                   feature.p_cam_left().z);
          auto p_world =
              key_frame.pose().q.matrix() * p_cam + key_frame.pose().p;
          Point3Node p_node(id, &*fixed_.rbegin(), true);
          fixed_.push_back(p_world(0));
          fixed_.push_back(p_world(1));
          fixed_.push_back(p_world(2));
          point3s_.emplace(id, p_node);
          id++;
        }
      }
    }
  }
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
  size_t kf_count = 0;
  for (auto kfi = key_frames.rbegin(); kfi != key_frames.rend(); kfi++) {

    //  is this key frame in the current window?
    const bool in_window = (kf_count < win_size_);
    const KeyFrame &kf = *kfi;

    bool features_found = false;
    for (const std::pair<Feature::Id, Feature> &feat : kf.features()) {
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

void CeresBundler::AddResidualBlock(const Edge &edge,
                                    image_geometry::PinholeCameraModel &model) {

  //  create residuals for this edge
  auto cam_ite = cameras_.find(edge.cam_id());
  auto point_ite = point3s_.find(edge.pt3_id());

  ROS_ASSERT_MSG((cam_ite != cameras_.end()) && (point_ite != point3s_.end()),
                 "Graph is malformed");

  const CameraNode &cam = cam_ite->second;
  const Point3Node &point = point_ite->second;

  ceres::CostFunction *func = 0;
  if (point.locked()) {
    //  this point appears in a previous frame, use fixed residual
    //  2 x 6 cost function
    func =
        FixedReprojectionError::Create(edge.x(), edge.y(), model, point.ptr());

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
  
  ROS_INFO_STREAM("Summary " << summary.BriefReport());
}

void CeresBundler::UpdateKeyFrames(std::deque<KeyFrame> &key_frames) {
  
  auto 
  
}

}  // namespace stereo_vo

}  // galt
