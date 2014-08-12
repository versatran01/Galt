#ifndef GALT_STEREO_VO_CERES_BUNDLER_H_
#define GALT_STEREO_VO_CERES_BUNDLER_H_

#include "stereo_vo/common.h"
#include "stereo_vo/key_frame.h"
#include "stereo_vo/feature.h"

#include "image_geometry/stereo_camera_model.h"

#include <deque>
#include <map>

#include <ceres/ceres.h>

namespace galt {

namespace stereo_vo {

class NodeBase {
 public:
  using Id = uint64_t;
  NodeBase(const Id id, double *ptr) : id_(id), ptr_(ptr) {}

  double *ptr() const { return ptr_; }
  const Id id() const { return id_; }

 protected:
  Id id_;
  double *ptr_;
};

class CameraNode : public NodeBase {
 public:
  CameraNode(const Id id, double *ptr) : NodeBase(id, ptr) {}
  static const int kSize = 6;
};

class Point3Node : public NodeBase {
 public:
  Point3Node(const Id id, double *ptr, const bool locked)
      : NodeBase(id, ptr), locked_(locked) {}
  static const int kSize = 3;
  const bool locked() const { return locked_; }

 private:
  bool locked_;
};

class Edge {
 public:
  Edge(NodeBase::Id cam_id, NodeBase::Id pt3_id, scalar_t x, scalar_t y)
      : cam_id_(cam_id), pt3_id_(pt3_id), x_(x), y_(y) {}

  const NodeBase::Id cam_id() const { return cam_id_; }
  const NodeBase::Id pt3_id() const { return pt3_id_; }
  const scalar_t x() const { return x_; }
  const scalar_t y() const { return y_; }

 private:
  NodeBase::Id cam_id_;
  NodeBase::Id pt3_id_;
  scalar_t x_;
  scalar_t y_;
};

class CeresBundler {
 public:
  CeresBundler() { options_.linear_solver_type = ceres::SPARSE_SCHUR; }
  void Optimize(std::deque<KeyFrame> &key_frames,
                const image_geometry::StereoCameraModel &cameraModel,
                int win_size);

 private:
  // Iterate through key frames, then iterate through features, store features
  // in windowed key frames into storage and features outside window but still
  // tracked in window into fixed. And for each featrue, create an edge
  void CreateGraph(const std::deque<KeyFrame> &key_frames);
  // Temporary function that resets relevant member variables
  void NukeEverything(bool from_orbit = true);
  // Iterate through all features and classify them into mutables or immutables
  void SplitFeatureIds(const std::deque<KeyFrame> &key_frames);
  void AddResidualBlock(const Edge &edge,
                        image_geometry::PinholeCameraModel &model);
  // Set ceres-related options and solve problem
  void SolveProblem();
  // Update key frames will results from ceres-solver
  void UpdateKeyFrames(std::deque<KeyFrame> &key_frames);

  ceres::Problem problem_;
  ceres::Solver::Options options_;

  image_geometry::StereoCameraModel model_;

  size_t win_size_;  //  size of the window in # of keyframes

  std::vector<double> storage_;  // a contiguous block of memory that stores
                                 // mutable camera poses and 3d points that will
                                 // be optimized, we store points first and then
                                 // camera poses
  std::vector<double> fixed_;    // a contiguous block of memeory that stores
  // immutable 3d points that will not be optimized
  std::vector<Edge> edges_;

  std::set<Feature::Id>
      mutables_;  // id of features that are only observed in window
  std::set<Feature::Id> immutables_;  // id of features that are observed both
                                      // in and out of window
  std::map<NodeBase::Id, CameraNode> cameras_;
  std::map<NodeBase::Id, Point3Node> point3s_;
};

}  // namespace stereo_vo

}  // namespace galt

#endif  // GALT_STEREO_VO_CERES_BUNDLER_H_
