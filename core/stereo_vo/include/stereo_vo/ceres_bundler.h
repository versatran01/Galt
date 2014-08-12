#ifndef GALT_STEREO_VO_CERES_BUNDLER_H_
#define GALT_STEREO_VO_CERES_BUNDLER_H_

#include "stereo_vo/common.h"
#include "stereo_vo/key_frame.h"

#include <deque>
#include <map>

#include <ceres/ceres.h>

namespace galt {

namespace stereo_vo {

class NodeBase {
 public:
  using Id = uint64_t;

 protected:
  Id id_;
  double *ptr_;
};

class CameraNode : public NodeBase {
 public:
  const int kSize = 6;

 private:
};

class Point3Node : public NodeBase {
 public:
  const int kSize = 3;

 private:
  bool locked_;
};

class Edge {
 public:
  Edge(NodeBase::Id cam_id, NodeBase::Id pt3_id, scalar_t x, scalar_t y)
      : cam_id_(cam_id), pt3_id_(pt3_id), x_(x), y_(y) {}

 private:
  NodeBase::Id cam_id_;
  NodeBase::Id pt3_id_;
  scalar_t x_;
  scalar_t y_;
};

class CeresBundler {
 public:
  void Optimize(std::deque<KeyFrame> &key_frames, int win_size);

 private:
  void CreateGraph();
  void AddResidualBlock(const Edge& edge);
  void SolveProblem();

  ceres::Problem problem_;
  std::vector<double> storage_;
  std::vector<double> fixed_;
  std::vector<Edge> edges_;
  std::map<NodeBase::Id, CameraNode> cameras_;
  std::map<NodeBase::Id, Point3Node> point3s_;
};
}
}

#endif  // GALT_STEREO_VO_CERES_BUNDLER_H_
