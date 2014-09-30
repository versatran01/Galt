#include "pcl_editor/pcl_editor.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>

namespace pcl_editor {

using namespace pcl;

bool LoadPcdFile(const std::string& pcd, const MyPointCloud::Ptr& cloud) {
  if (io::loadPCDFile<pcl::PointWithViewpoint>(pcd, *cloud) == -1) {
    PCL_ERROR("Couldn't read file %s \n", pcd.c_str());
    return false;
  }
  PCL_INFO("Loaded %zu points from %s", cloud->width * cloud->height,
           pcd.c_str());
  return true;
}

}  // namespace pcl_editor
