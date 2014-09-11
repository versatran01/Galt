#include <stereo_vo/point3d.h>
#include <stereo_vo/keyframe.h>

namespace galt {
namespace stereo_vo {

/// Undo the camera matrix
CvPoint2 convertToImage(const image_geometry::PinholeCameraModel& model,
                        const CvPoint2& p_in) {
  const scalar_t fx = model.fx();
  const scalar_t fy = model.fy();
  const scalar_t cx = model.cx();
  const scalar_t cy = model.cy();
  return CvPoint2((p_in.x - cx) / fx, (p_in.y - cy) / fy);
}

bool Point3d::AddObservation(const StereoVoDynConfig &config, 
                             const image_geometry::StereoCameraModel& model,
                             const KeyFramePtr& key_frame,
                             const CvPoint2& left,
                             const CvPoint2& right) {
  const CvPoint2 p_left = convertToImage(model.left(), left);
  const CvPoint2 p_right = convertToImage(model.right(), right);
  scalar_t Xn;  //  depth estimate for this observation
  vec3 tri_position;
  scalar_t eigenratio;
  
  if (!IsInitialized()) {
    //  first observation, we need both left and right
    assert(right.x >= 0 && right.y >= 0);
   
    const KrPose left_pose; //  identity
    const KrPose right_pose(left_pose.q(), 
                            left_pose.p()+vec3(model.baseline(),0,0));
    
    Xn = kr::triangulate(left_pose,vec2(p_left.x,p_left.y),
                         right_pose,vec2(p_right.x,p_right.y),
                         tri_position,eigenratio);
    if (eigenratio > config.tri_max_eigenratio || Xn < 0) {
      //  bad triangulation, reject
      return false;
    }
    
    //  place into world frame
    tri_position = key_frame->pose().transformFromBody(tri_position);
    p_world_ = CvPoint3(tri_position[0],tri_position[1],tri_position[2]);
    
    //  initialize depth filter
    filter_.initialize(config.filter_min_depth,
                       config.filter_max_depth,
                       config.filter_meas_std);
    filter_.mu = Xn;
  } else {
    //  later observation, use only left in conjunction with the reference pose
    const KeyFramePtr& ref_frame = observations_.front().key_frame();
    const CvPoint2 ref_point = convertToImage(model.left(),
                                              observations_.front().p_pixel());
    const KrPose& ref_pose = ref_frame->pose();
    const KrPose& cur_pose = key_frame->pose();
    
    Xn = kr::triangulate(ref_pose,vec2(ref_point.x,ref_point.y),
                         cur_pose,vec2(p_left.x,p_left.y),
                         tri_position,eigenratio);
    if (eigenratio > config.tri_max_eigenratio || Xn < 0) {
      return false;
    }
    //  recalculate using depth filter
    filter_.addMeasurement(Xn);
    const vec3 p_cam = vec3(ref_point.x,ref_point.y,1) * filter_.depth();
    const vec3 p_world = ref_pose.transformFromBody(p_cam);
    
    p_world_ = CvPoint3(p_world[0],p_world[1],p_world[2]);
  }
  observations_.emplace_back(id_,left,Xn,key_frame);
  return true;
}

}
}
