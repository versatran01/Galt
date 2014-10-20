#include "stereo_vo/optimizer.h"
#include "stereo_vo/frame.h"

#include <ros/ros.h>

namespace galt {
namespace stereo_vo {

using namespace gtsam;

void WindowedOptimizer::Initialize(const StereoCameraModel& model) {
  noise_model_ = noiseModel::Isotropic::Sigma(3, 0.1);
  const image_geometry::PinholeCameraModel& left = model.left();
  stereo_model_.reset(new Cal3_S2Stereo(left.fx(), left.fy(), 0, left.cx(),
                                        left.cy(), model.baseline()));
  ROS_INFO("Initializing windowed optimizer");
}

void WindowedOptimizer::Optimize(std::deque<FramePtr>& key_frames,
                                 std::map<Id, Point3d>& point3ds) {
  NonlinearFactorGraph graph;
  Values initial_estimates;
  size_t x_init = key_frames.front()->id();
  std::set<size_t> l_set;
  for (const FramePtr& frame_ptr : key_frames) {
    // Add camera initial pose estimate
    const KeyFrame& frame = *frame_ptr;
    const size_t x = frame.id();
#warning This usage of matrix() is probably invalid now
    const Matrix3 mat3 = frame.pose().q.conjugate().matrix().cast<double>();
    const Rot3 rot3(mat3);
    const Point3 trans(frame.pose().p[0], frame.pose().p[1], frame.pose().p[2]);
    Pose3 pose3(rot3, trans);
    initial_estimates.insert(Symbol('x', x), pose3);
    const std::vector<Feature>& features = frame_ptr->features();
    for (const Feature& feature : features) {
      // Add stereo factors to graph
      const double u_l = feature.p_pixel().x;
      const double u_r = feature.p_right().x;
      const double v = feature.p_pixel().y;
      const size_t l = feature.id();
      graph.push_back(StereoFactor(StereoPoint2(u_l, u_r, v), noise_model_,
                                   Symbol('x', x), Symbol('l', l),
                                   stereo_model_));
      // if the landmark variable included in this factor has not yet been added
      // to the initial variable value estimate, add it
      if (!initial_estimates.exists(Symbol('l', l))) {
        assert(point3ds.find(l) != point3ds.end());
        const Point3d p3 = point3ds[l];
        initial_estimates.insert(
            Symbol('l', l),
            Point3(p3.p_world().x, p3.p_world().y, p3.p_world().z));
        l_set.insert(l);
      }
    }
  }
  Pose3 first_pose = initial_estimates.at<Pose3>(Symbol('x', x_init));
  // constrain the first pose such that it cannot change from its original value
  // during optimization
  // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
  // QR is much slower than Cholesky, but numerically more stable
  // graph.push_back(NonlinearEquality<Pose3>(Symbol('x', x_init), first_pose));

  gtsam::Vector6 pose_noise_std;
  for (int i = 0; i < 3; i++) {
    pose_noise_std[i] = 0.3;
    pose_noise_std[i + 3] = 0.2;
  }
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise;
  pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_noise_std);
  gtsam::PriorFactor<gtsam::Pose3> pose_prior(Symbol('x', x_init), first_pose,
                                              pose_noise);
  graph.push_back(pose_prior);

  LevenbergMarquardtOptimizer optimizer =
      LevenbergMarquardtOptimizer(graph, initial_estimates);
  Values result = optimizer.optimize();

  // Put resulting poses back to key frames
  for (const FramePtr& frame_ptr : key_frames) {
    KeyFrame& frame = *frame_ptr;
    const size_t x = frame.id();
    Pose3 pose = result.at<Pose3>(Symbol('x', x));
    //    pose.translation().print();
    //    std::cout << frame.pose().p << std::endl;
    kr::quat<scalar_t> quat(
        pose.rotation().quaternion()(0), pose.rotation().quaternion()(1),
        pose.rotation().quaternion()(2), pose.rotation().quaternion()(3));
    kr::vec3<scalar_t> translation(
        pose.translation().x(), pose.translation().y(), pose.translation().z());
    KrPose kr_pose(quat.conjugate(), translation);
    //    std::cout << kr_pose.q.matrix() << std::endl;
    //    std::cout << frame.pose().q.matrix() << std::endl;
    //    std::cout << "-----" << std::endl;
    frame.set_pose(kr_pose);
  }
  // Put resulting points back to point3ds
  for (const size_t l : l_set) {
    const Point3 point3 = result.at<Point3>(Symbol('l', l));
    point3ds[l].set_p_world(CvPoint3(point3.x(), point3.y(), point3.z()));
  }
  //  std::cout << "initial: " << key_frames.front()->pose().p << std::endl;
  //  first_pose.translation().print();
  //  graph_.resize(0);
  ROS_INFO("Optimize %i key frames", (int)key_frames.size());
}

//  --------------------
//  IncrementalOptimizer
//  --------------------

void IncrementalOptimizer::Initialize(const StereoCameraModel& model) {
  // Define the camera observation noise model, 1 pixel in u and v
  noise_model_ = noiseModel::Isotropic::Sigma(2, 1.0);
  const image_geometry::PinholeCameraModel& left = model.left();
  camera_model_.reset(
      new Cal3_S2(left.fx(), left.fy(), 0, left.cx(), left.cy()));
  stereo_model_.reset(new Cal3_S2Stereo(left.fx(), left.fy(), 0, left.cx(),
                                        left.cy(), model.baseline()));

  ROS_INFO("Initializing incremental optimizer");
}

void IncrementalOptimizer::Optimize(std::deque<FramePtr>& key_frames,
                                    std::map<Id, Point3d>& point3s) {
  bool first_run = false;
  if (!isam_) {
    isam_ =
        std::make_shared<gtsam::NonlinearISAM>(3);  //  TODO: make this a param
    estimate_.reset(new gtsam::Values());
    first_run = true;
  }

  //  take the most recent key frame
  const FramePtr& frame_ptr = key_frames.back();
  KeyFrame& frame = *frame_ptr;

  //  pose estimate and symbol for this frame
#warning This usage of matrix() is probably invalid now
  const gtsam::Matrix3 mat3 =
      frame.pose().q.conjugate().matrix().cast<double>();
  const gtsam::Rot3 rot3(mat3);
  const gtsam::Point3 trans(frame.pose().p[0], frame.pose().p[1],
                            frame.pose().p[2]);
  gtsam::Pose3 pose3(rot3, trans);
  gtsam::Symbol frame_sym('x', frame.id());

  int count = 0;

  //  consider all observations associated with this frame
  for (const Feature& feature : frame.features()) {
    //  point where the feature was observed
    gtsam::Point2 p2(feature.p_pixel().x, feature.p_pixel().y);

    //  create a projection residual
    gtsam::Symbol point_sym('p', feature.id());
    ProjectionFactor factor(p2, noise_model_, frame_sym, point_sym,
                            camera_model_);
    graph_.push_back(factor);

    //    const double u_l = feature.p_pixel().x;
    //    const double u_r = feature.p_right().x;
    //    const double v = feature.p_pixel().y;
    //    graph_.push_back(StereoProjectionFactor(StereoPoint2(u_l, u_r, v),
    //                                            noise_model_,
    //                                  frame_sym, point_sym,
    //                                  stereo_model_));

    if (prevIds_.find(feature.id()) != prevIds_.end()) {
      count++;
    }

    if (feature.init()) {
      //  feature is new, so add initial guess
      auto p3_ite = point3s.find(feature.id());
      assert(p3_ite != point3s.end());  //  graph is malformed
      const Point3d& p3d = p3_ite->second;

      gtsam::Point3 guess(p3d.p_world().x, p3d.p_world().y, p3d.p_world().z);
      assert(estimate_->find(point_sym) == estimate_->end());  //  malformed
      estimate_->insert(point_sym, guess);

      // if (first_run) {
      gtsam::noiseModel::Isotropic::shared_ptr pointNoise =
          gtsam::noiseModel::Isotropic::Sigma(3, 0.5);
      graph_.push_back(
          gtsam::PriorFactor<gtsam::Point3>(point_sym, guess, pointNoise));
      //}
    } else {
      // ROS_INFO("Not init!");
    }
  }
  ROS_INFO("%i features in common with previous key frame", count);
  prevIds_.clear();

  //  insert initial estimate for pose
  assert(estimate_->find(frame_sym) ==
         estimate_->end());  //  must be a new pose
  estimate_->insert(frame_sym, pose3);

  if (first_run) {
    //  ROS_INFO("Incremental optimizer first run, skipping update");
    //  need to insert prior for pose on the first pass
    //  0.3m std on position, 0.1rad on rotation as per example
    gtsam::Vector6 pose_noise_std;
    for (int i = 0; i < 3; i++) {
      pose_noise_std[i] = 0.3;
      pose_noise_std[i + 3] = 0.1;
    }
    gtsam::noiseModel::Diagonal::shared_ptr pose_noise;
    pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_noise_std);
    gtsam::PriorFactor<gtsam::Pose3> pose_prior(frame_sym, pose3, pose_noise);
    graph_.push_back(pose_prior);
  }

  if (!first_run) {
    ROS_INFO("Updating iSAM");
    const auto tstart = ros::Time::now();
    //  update iSAM
    isam_->update(graph_, *estimate_);
    ROS_INFO("Time required: %f (s)", (ros::Time::now() - tstart).toSec());

    Values updatedEstimate = isam_->estimate();

    //  update the pose
    const gtsam::Pose3 updatedPose =
        updatedEstimate.at<gtsam::Pose3>(frame_sym);

    KrPose converted;
    converted.p = kr::vec3<scalar_t>(updatedPose.translation().x(),
                                     updatedPose.translation().y(),
                                     updatedPose.translation().z());
#warning This usage of matrix() is probably invalid now
    converted.q =
        kr::quat<scalar_t>(updatedPose.rotation().matrix().cast<scalar_t>())
            .conjugate();

    frame.set_pose(converted);
    // std::cout << "Pos: " << converted.p << std::endl;

    for (const Feature& feature : frame.features()) {
      gtsam::Symbol point_sym('p', feature.id());
      assert(updatedEstimate.find(point_sym) != updatedEstimate.end());

      const gtsam::Point3 p3 = updatedEstimate.at<gtsam::Point3>(point_sym);

      //  update the points
      Point3d p3d = point3s[feature.id()];
      ROS_INFO("Before: %f, %f, %f", p3d.p_world().x, p3d.p_world().y,
               p3d.p_world().z);
      ROS_INFO("After: %f, %f, %f", p3.x(), p3.y(), p3.z());

      point3s[feature.id()] =
          Point3d(feature.id(), CvPoint3(p3.x(), p3.y(), p3.z()));

      prevIds_.insert(feature.id());
    }

    //  cleanup
    graph_.resize(0);
    estimate_->clear();
  }
}

void G2OOptimizer::Initialize(const StereoCameraModel& model) {
  left_camera_ = model.left();
}

void G2OOptimizer::Optimize(std::deque<FramePtr>& key_frames,
                            std::map<Id, Point3d>& point3s) {

  //  set up a solver
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);

  g2o::BlockSolverX::LinearSolverType* linearSolver =
      new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX* solver = new g2o::BlockSolverX(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* algo =
      new g2o::OptimizationAlgorithmLevenberg(solver);

  algo->setMaxTrialsAfterFailure(5);  //  copied from SVO
  optimizer.setAlgorithm(
      algo);  /// @todo: figure out how memory is managed here

  int g2o_ids = 0;

  //  Eigen::Vector2d center(left_camera_.cx(), left_camera_.cy());
  //  g2o::CameraParameters * cam_params =
  //      new g2o::CameraParameters(left_camera_.fx(), center, 0);
  Eigen::Vector2d center(0, 0);
  g2o::CameraParameters* cam_params = new g2o::CameraParameters(1, center, 0);
  cam_params->setId(0);  /// @todo: what is this id for?

  if (!optimizer.addParameter(cam_params)) {
    throw std::runtime_error("failed to add camera params");
  }

  const int window_size = 3;
  std::map<Id, g2o::VertexSBAPointXYZ*> added_features;
  std::map<Id, g2o::VertexSE3Expmap*> added_frames;

  //  identify any singleton features
  /// @todo: move this elsewhere
  std::map<Id, bool> point_singleton;
  for (const FramePtr& ptr : key_frames) {
    const KeyFrame& frame = *ptr;
    for (const Feature& feat : frame.features()) {
      auto ite = point_singleton.find(feat.id());
      if (ite == point_singleton.end()) {
        //  first observation, set to true
        point_singleton[feat.id()] = true;
      } else {
        //  seen again, not a singleton
        ite->second = false;
      }
    }
  }
  const size_t cnt =
      std::count_if(point_singleton.begin(), point_singleton.end(),
                    [](const std::pair<Id, bool>& p) { return p.second; });
  ROS_INFO("Will ignore %lu singleton features", cnt);

  //  iterate over all key frames and create g2o graph
  size_t kf_count = 0;
  for (auto kf_ite = key_frames.rbegin(); kf_ite != key_frames.rend();
       kf_ite++, kf_count++) {

    const KeyFrame& frame = *(*kf_ite);
    const KrPose& pose = frame.pose();
    const bool in_window = (kf_count < window_size);

    //  automatically make a frame for this pose
    g2o::VertexSE3Expmap* frame_vert = new g2o::VertexSE3Expmap();
    frame_vert->setId(++g2o_ids);  //  temp solution
    frame_vert->setFixed(!in_window);

    ROS_INFO("Added frame %lu, fixed: %s", frame.id(),
             (in_window) ? "false" : "true");

    g2o::SE3Quat se3quat(pose.q.cast<double>(),
                         pose.translation().cast<double>());
    frame_vert->setEstimate(se3quat);

    ROS_INFO("Adding frame vertex %lu", frame.id());
    if (!optimizer.addVertex(frame_vert)) {
      throw std::runtime_error("failed to add frame vertex");
    }
    added_frames[frame.id()] = frame_vert;

    //  consider all observations in key frame
    for (const Feature& feat : frame.features()) {
      auto ite = added_features.find(feat.id());
      const bool added = (ite != added_features.end());

      if (point_singleton[feat.id()]) {
        continue;  //  ignore all singletons
      }

      g2o::VertexSBAPointXYZ* gpoint = 0;
      if (!added && in_window) {
        //  we have not created a point yet, add one
        const Point3d& p3 = point3s[feat.id()];

        gpoint = new g2o::VertexSBAPointXYZ();
        gpoint->setId(++g2o_ids);
        gpoint->setFixed(false);  //  get from map of all features
        gpoint->setEstimate(
            kr::vec3d(p3.p_world().x, p3.p_world().y, p3.p_world().z));
        if (!optimizer.addVertex(gpoint)) {
          throw std::runtime_error("failed to add vertex");
        }
        added_features[feat.id()] = gpoint;
      } else if (added) {
        gpoint = ite->second;
        if (!in_window) {
          gpoint->setFixed(true);
        }
      }

      if (gpoint != 0) {
        //  convert from pixel to image coordinates
        kr::vec2d p_image(feat.p_pixel().x, feat.p_pixel().y);
        p_image[0] = (p_image[0] - left_camera_.cx()) / left_camera_.fx();
        p_image[1] = (p_image[1] - left_camera_.cy()) / left_camera_.fy();

        //  this point is part of the optimization, add an edge
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0,
                        dynamic_cast<g2o::OptimizableGraph::Vertex*>(gpoint));
        edge->setVertex(
            1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_vert));
        edge->setMeasurement(p_image);
        edge->information() = kr::mat2d::Identity(2, 2);
        g2o::RobustKernelHuber* rk =
            new g2o::RobustKernelHuber();  // TODO: memory leak
        rk->setDelta(2);
        edge->setRobustKernel(rk);
        edge->setParameterId(0, 0);

        if (!optimizer.addEdge(edge)) {
          throw std::runtime_error("failed to add edge");
        }
      }
    }
  }

  //  do structure only bundle adjustment (todo : why?)
  //  copied from svo
  g2o::StructureOnlySolver<3> structure_only_ba;
  g2o::OptimizableGraph::VertexContainer points;
  for (g2o::OptimizableGraph::VertexIDMap::const_iterator it =
           optimizer.vertices().begin();
       it != optimizer.vertices().end(); ++it) {
    g2o::OptimizableGraph::Vertex* v =
        static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
    if (v->dimension() == 3 && v->edges().size() >= 2) points.push_back(v);
  }
  structure_only_ba.calc(points, 10);

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  ROS_INFO("Error before: %f", optimizer.activeChi2());
  optimizer.optimize(10);
  ROS_INFO("Error after: %f", optimizer.activeChi2());

  //  update keyframes
  for (const std::pair<Id, g2o::VertexSE3Expmap*>& pair : added_frames) {
    g2o::SE3Quat estimate = pair.second->estimate();

    //  find appropriate key-frame
    for (FramePtr ptr : key_frames) {
      if (ptr->id() == pair.first) {
        ptr->set_pose(
            KrPose(estimate.to_homogeneous_matrix().cast<scalar_t>()));
        break;
      }
    }
  }

  //  update points
  for (const std::pair<Id, g2o::VertexSBAPointXYZ*>& pair : added_features) {
    kr::vec3d estimate = pair.second->estimate();
    point3s[pair.first] =
        Point3d(pair.first, CvPoint3(estimate[0], estimate[1], estimate[2]));
  }
}

}  // namespace stereo_vo
}  // namespace galt
