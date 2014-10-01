#ifndef PCL_EDITOR_BASE_HPP_
#define PCL_EDITOR_BASE_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

namespace pcl_editor {

typedef pcl::PointWithViewpoint MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<MyPoint>
    MyColorHandler;
using pcl::visualization::PCLVisualizer;

enum ConfigLevel { INIT = -1, EDIT = 0, SAVE = 1 };

template <typename ConfigType>
class PclEditorBase {
 public:
  PclEditorBase(const ros::NodeHandle& nh, const std::string& name, double freq)
      : nh_(nh),
        viewer_(new PCLVisualizer(name)),
        rate_(freq),
        cfg_server_(nh) {}

  virtual ~PclEditorBase() {}

  void Setup() {
    cfg_server_.setCallback(
        boost::bind(&PclEditorBase::ConfigCb, this, _1, _2));
  }

  void Start() {
    while (ros::ok() && ViewerOk()) {
      ros::spinOnce();
      SpinOnce();
      rate_.sleep();
    }
  }

 protected:
  virtual void InitializeViewer() {
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem();
    viewer_->initCameraParameters();
  }
  virtual void EditPointCloud() {}
  virtual void SavePointCloud() {}

  ros::NodeHandle nh_;
  PCLVisualizer::Ptr viewer_;
  ConfigType config_;

 private:
  bool ViewerOk() const { return !viewer_->wasStopped(); }
  void SpinOnce(int time_ms = 100) const { viewer_->spinOnce(time_ms); }
  void ConfigCb(ConfigType& config, int level) {
    config_ = config;
    // Initialize viewer
    if (level == ConfigLevel::INIT) {
      ROS_INFO("Initialize reconfigure server for %s",
               nh_.getNamespace().c_str());
      InitializeViewer();
      config.save = false;
    }

    if (level <= ConfigLevel::EDIT) {
      EditPointCloud();
    }

    if (level == ConfigLevel::SAVE) {
      if (config.save) {
        SavePointCloud();
      }
      config.save = false;
    }
  }

  ros::Rate rate_;
  dynamic_reconfigure::Server<ConfigType> cfg_server_;
};

template <typename PointType>
bool LoadPcdFile(const std::string& pcd, pcl::PointCloud<PointType>& cloud) {
  if (pcl::io::loadPCDFile<PointType>(pcd, cloud) < 0) {
    PCL_ERROR("Couldn't read file %s \n", pcd.c_str());
    return false;
  }
  PCL_INFO("Loaded %zu points from %s", cloud.size(), pcd.c_str());
  return true;
}

}  // namespace pcl_editor

#endif  // PCL_EDITOR_BASE_HPP_
