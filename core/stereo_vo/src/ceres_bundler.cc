#include "stereo_vo/ceres_bundler.h"
#include "stereo_vo/cost_functor.h"

#include <Eigen/Geometry>
#include <Eigen/Core>

namespace galt {

namespace stereo_vo {

void CeresBundler::Optimize(
    std::deque<KeyFrame> &key_frames, std::map<Feature::Id, Feature> &features,
    const image_geometry::StereoCameraModel &cameraModel, int win_size) {
  win_size_ = win_size;
  model_ = cameraModel;
  NukeEverything();
  SplitFeatureIds(key_frames, features);
  CreateGraph(key_frames, features);
  SolveProblem();
  UpdateMap(key_frames, features);
}

void CeresBundler::CreateGraph(const std::deque<KeyFrame> &key_frames,
                               const std::map<Feature::Id, Feature> &features) {
  // Iterate through keyframe the first time to save camera poses
  NodeBase::Id cam_id = 0;
  // Iterate through key frame backwards to save camera pose and then through
  // features to save 3d points
  auto itb_kf = key_frames.cend() - win_size_, ite_kf = key_frames.cend();
  for (auto it_kf = itb_kf; it_kf != ite_kf; ++it_kf) {
    const KeyFrame &key_frame = *it_kf;
    // Add key frame pose to camera pose
    Eigen::AngleAxis<scalar_t> aa(key_frame.pose().q);
    const auto r_vec = aa.axis() * aa.angle();
    const auto t_vec = key_frame.pose().p;
    CameraNode c_node(cam_id, &*storage_.end());
    storage_.push_back(r_vec(0));
    storage_.push_back(r_vec(1));
    storage_.push_back(r_vec(2));
    storage_.push_back(t_vec(0));
    storage_.push_back(t_vec(1));
    storage_.push_back(t_vec(2));
    cameras_.emplace(cam_id, c_node);
    cam_id++;
    // Now we iterate every observation in each key frame and add it to edges
    for (const Corner &corner : key_frame.corners()) {
      const Feature::Id feat_id = corner.id();
      edges_.emplace_back(cam_id, feat_id, corner.p_pixel().x, corner.p_pixel().y);
    }
  }

  for (const Feature::Id mut_id : mutables_) {
    const Feature& feature = features[mut_id];
    Point3Node p_node(mut_id, &*storage_.end(), false);
    storage_.push_back(feature.p_world().x);
    storage_.push_back(feature.p_world().y);
    storage_.push_back(feature.p_world().z);
    point3s_.emplace(mut_id, p_node);
  }

  for (const Feature::Id immut_id : immutables_) {
    const Feature& feature = features[mut_id];
    Point3Node p_node(mut_id, &*storage_.end(), true);
    fixed_.push_back(feature.p_world().x);
    fixed_.push_back(feature.p_world().y);
    fixed_.push_back(feature.p_world().z);
    point3s_.emplace(immut_id, p_node);
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

void CeresBundler::SplitFeatureIds(
    const std::deque<KeyFrame> &key_frames,
    const std::map<Feature::Id, Feature> &features) {

  //  iterate over key frames and sort all features into one of two groups
  size_t kf_count = 0;
  for (auto kfi = key_frames.rbegin(); kfi != key_frames.rend(); kfi++) {

    //  is this key frame in the current window?
    const bool in_window = (kf_count < win_size_);
    const KeyFrame &kf = *kfi;

    bool features_found = false;
    for (const Corner &corner : kf.corners()) {
      if (in_window) {
        //  assume this is a mutable point, since it is in the window
        mutables_.insert(corner.id());
      } else {
        //  this point is also in past keyframes, move it to immutables
        const auto ite = mutables_.find(corner.id());
        if (ite != mutables_.end()) {
          mutables_.erase(ite);
          immutables_.insert(corner.id());
          features_found = true;
        } else {
          //  do nothing, this corner is not useful to us
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
  options_.function_tolerance = 1e-12;
  options_.use_inner_iterations = true;
  options_.linear_solver_type = ceres::SPARSE_SCHUR;
  options_.preconditioner_type = ceres::JACOBI;
  options_.visibility_clustering_type = ceres::CANONICAL_VIEWS;
  options_.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;

  ceres::Solver::Summary summary;
  ceres::Solve(options_, &problem_, &summary);

  ROS_INFO_STREAM("Summary " << summary.BriefReport());
}

void CeresBundler::UpdateMap(std::deque<KeyFrame> &key_frames,
                             std::map<Feature::Id, Feature> &features) {

  //  iterate forwards over keyframes, consider only those in window
  size_t kf_count = 0;
  for (auto kf_ite = key_frames.begin(); kf_ite != key_frames.end();
       kf_ite++, kf_count++) {

    if (key_frames.size() - kf_count > win_size_) {
      continue;
    }

    /// @todo: fetch updated keyframe pose here and write back to map
    Pose kf_pose;

    for (const Corner &corner : kf_ite->corners()) {
      //  pull corrected point from results
      auto p3_ite = point3s_.find(corner.id());
      ROS_ASSERT_MSG(p3_ite != point3s_.end(), "Point in window that was not optimized!");
      
      const Point3Node& p3 = p3_ite->second;
      Feature& feat = features[corner.id()];
      feat.set_p_world(CvPoint3(p3.ptr()[0],p3.ptr()[1],p3.ptr()[2]));
    }
  }
}

}  // namespace stereo_vo

}  // galt
