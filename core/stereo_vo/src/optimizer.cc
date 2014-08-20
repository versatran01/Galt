#include "stereo_vo/optimizer.h"
#include "stereo_vo/frame.h"

#include <ros/ros.h>

namespace galt {

namespace stereo_vo {

using namespace gtsam;

void WindowedOptimizer::Initialize(const StereoCameraModel &model) {
  noise_model_ = noiseModel::Isotropic::Sigma(3, 0.1);
  const image_geometry::PinholeCameraModel &left = model.left();
  stereo_model_.reset(new Cal3_S2Stereo(left.fx(), left.fy(), 0, left.cx(),
                                        left.cy(), model.baseline()));
  ROS_INFO("Initializing windowed optimizer");
}

void WindowedOptimizer::Optimize(std::deque<FramePtr> &key_frames,
                                 std::map<Id, Point3d> &point3ds) {
  NonlinearFactorGraph graph;
  Values initial_estimates;
  size_t x_init = key_frames.front()->id();
  std::set<size_t> l_set;
  for (const FramePtr &frame_ptr : key_frames) {
    // Add camera initial pose estimate
    const Frame &frame = *frame_ptr;
    const size_t x = frame.id();
    KrPose pose(frame.pose().q, frame.pose().p);
//    KrPose pose(frame.pose().q.conjugate(), frame.pose().p);
    initial_estimates.insert(Symbol('x', x),
                             Pose3(pose.matrix().cast<double>()));
    const std::vector<Feature> &features = frame_ptr->features();
    for (const Feature &feature : features) {
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
  graph.push_back(NonlinearEquality<Pose3>(Symbol('x', x_init), first_pose));
  LevenbergMarquardtOptimizer optimizer =
      LevenbergMarquardtOptimizer(graph, initial_estimates);
  Values result = optimizer.optimize();

  // Put resulting poses back to key frames
  for (const FramePtr &frame_ptr : key_frames) {
    Frame &frame = *frame_ptr;
    const size_t x = frame.id();
    Pose3 pose = result.at<Pose3>(Symbol('x', x));
//    pose.translation().print();
//    std::cout << frame.pose().p << std::endl;
    kr::quat<scalar_t> quat(
        pose.rotation().quaternion()(0), pose.rotation().quaternion()(1),
        pose.rotation().quaternion()(2), pose.rotation().quaternion()(3));
    kr::vec3<scalar_t> translation(
        pose.translation().x(), pose.translation().y(), pose.translation().z());
//    KrPose kr_pose(quat.conjugate(), translation);
    KrPose kr_pose(quat, translation);
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

void IncrementalOptimizer::Initialize(const StereoCameraModel &model) {
  // Define the camera observation noise model, 1 pixel in u and v
  noise_model_ = noiseModel::Isotropic::Sigma(2, 1.0);
  const image_geometry::PinholeCameraModel &left = model.left();
  camera_model_.reset(
      new Cal3_S2(left.fx(), left.fy(), 0, left.cx(), left.cy()));

  ROS_INFO("Initializing incremental optimizer");
}

void IncrementalOptimizer::Optimize(std::deque<FramePtr> &key_frames,
                                    std::map<Id, Point3d> &point3s) {

  /*
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
    const FramePtr &frame_ptr = key_frames.back();
    const Frame &frame = *frame_ptr;

    //  pose estimate and symbol for this frame
    gtsam::Pose3 pose3(frame.pose().matrix().cast<double>());
    gtsam::Symbol frame_sym('x', frame.id());

    //  consider all observations associated with this frame
    for (const Feature &feature : frame.features()) {
      //  point where the feature was observed
      gtsam::Point2 p2(feature.p_pixel().x, feature.p_pixel().y);

      //  create a projection residual
      gtsam::Symbol point_sym('p', feature.id());
      ProjectionFactor factor(p2, *noise_model_, frame_sym, point_sym,
                              camera_model_);
      graph.push_back(factor);

      if (feature.init()) {
        //  feature is new, so provide iSAM with an initial guess
        gtsam::Point3 guess();
        estimate.insert(point_sym, )
      }
    }

    //  insert initial estimate for pose
    estimate.insert(frame_sym, pose3);

    if (first_run) {

    } else {
      //  update iSAM
      isam_->update(graph, estimate);
    }
  */
}
}  // namespace stereo_vo

}  // namespace galt
