#ifndef PCL_MERGER_NODE_HPP_
#define PCL_MERGER_NODE_HPP_

#include "pcl_editor/pcl_editor_base.hpp"

#include <pcl_editor/MergerDynConfig.h>

namespace pcl_editor {

class PclMergerNode : public PclEditorBase<MergerDynConfig> {
 public:
  typedef PclEditorBase<MergerDynConfig> super;

  PclMergerNode(const ros::NodeHandle& nh)
      : PclEditorBase(nh, "merger", 10),
        cloud1_(new MyPointCloud),
        cloud2_(new MyPointCloud),
        id1_("cloud1"),
        id2_("cloud2") {
    // Load two pcd files to merge
    std::string pcd1;
    std::string pcd2;
    nh.param<std::string>("pcd1", pcd1, std::string());
    nh.param<std::string>("pcd2", pcd2, std::string());
    if (!(LoadPcdFile(pcd1, *cloud1_) && LoadPcdFile(pcd2, *cloud2_))) {
      throw std::runtime_error("Failed to load pcd files.");
    }
  }

  virtual void InitializeViewer();
  virtual void EditPointCloud();
  virtual void SavePointCloud();

 private:
  MyPointCloud::Ptr cloud1_, cloud2_, cloud_transformed_;
  std::string id1_, id2_;
};

}  // namespace pcl_editor

#endif  // PCL_MERGER_NODE_HPP_
