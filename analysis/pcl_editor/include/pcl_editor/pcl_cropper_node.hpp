#ifndef PCL_CROPPER_NODE_HPP_
#define PCL_CROPPER_NODE_HPP_

#include "pcl_editor/pcl_editor_base.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl_editor/CropperDynConfig.h>

namespace pcl_editor {

class PclCropperNode : public PclEditorBase<CropperDynConfig> {
 public:
  typedef PclEditorBase<CropperDynConfig> super;

  PclCropperNode(const ros::NodeHandle& nh)
      : PclEditorBase(nh, "cropper", 10),
        cloud_(new MyPointCloud),
        cloud_cropped_(new MyPointCloud),
        id_("cloud"),
        id_cropped_("cloud_cropped") {
    // Load pcd file to crop
    std::string pcd;
    nh.param<std::string>("pcd", pcd, std::string());
    if (!LoadPcdFile(pcd, *cloud_)) {
      throw std::runtime_error("Failed to load pcd file.");
    }
    // These look like they are hacky
    pass_x_.setFilterFieldName("x");
    pass_y_.setFilterFieldName("y");
    pass_z_.setFilterFieldName("z");
  }

  virtual void InitializeViewer();
  virtual void EditPointCloud();
  virtual void SavePointCloud();

 private:
  void FilterByXYZ(MyPointCloud& cloud_filtered);

  MyPointCloud::Ptr cloud_, cloud_cropped_;
  std::string id_, id_cropped_;
  pcl::PassThrough<MyPoint> pass_x_;
  pcl::PassThrough<MyPoint> pass_y_;
  pcl::PassThrough<MyPoint> pass_z_;
};

void InitializeConfig(const MyPointCloud& cloud, CropperDynConfig& config);

}  // namespace pcl_editor

#endif  // PCL_CROPPER_NODE_HPP_
