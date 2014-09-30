#ifndef PCL_EDITOR_HPP_
#define PCL_EDITOR_HPP_

#include <pcl/common/common.h>

namespace pcl_editor {

typedef pcl::PointWithViewpoint MyPoint;
typedef pcl::PointCloud<MyPoint> MyPointCloud;

bool LoadPcdFile(const std::string& pcd, const MyPointCloud::Ptr& cloud);

}  // namespace pcl_editor

#endif  // PCL_EDITOR_HPP_
