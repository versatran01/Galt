#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

namespace yield_map {

bool IsFolderValid(const std::string& dir) {
  using namespace boost;
  filesystem::path path(dir);
  if (!filesystem::exists(path)) return false;
  if (!filesystem::is_directory(path)) return false;
  return true;
}

std::vector<std::string> ReadFilesInDirWithExt(const std::string& dir,
                                               const std::string& ext) {
  namespace bfs = boost::filesystem;
  if (!IsFolderValid(dir))
    throw std::runtime_error(std::string("Invalid folder: ") + dir);

  bfs::path p(dir);

  std::vector<std::string> files;
  // Such nonsense that qt only autocompletes when specify full namespace
  for (bfs::directory_iterator it = bfs::directory_iterator(p);
       it != bfs::directory_iterator(); ++it) {
    const bfs::directory_entry& entry = *it;
    if (ext.compare(entry.path().extension().string()) == 0) {
      files.push_back(entry.path().string());
    }
  }

  std::sort(files.begin(), files.end());

  return files;
}

}  // namespace yield_map

int main(int argc, char** argv) {
  using namespace yield_map;

  ros::init(argc, argv, "generate_yield_map");
  ros::NodeHandle pnh("~");

  double leaf_size;
  pnh.param("leaf_size", leaf_size, 0.5);
  std::string pcd_dir;
  pnh.param<std::string>("pcd_dir", pcd_dir, "/home/chao/Workspace/bag/pcd");

  if (IsFolderValid(pcd_dir)) {
    ROS_INFO("Reading pcd files from %s", pcd_dir.c_str());
  } else {
    ROS_ERROR("Invalid directory: %s", pcd_dir.c_str());
    return -1;
  }

  // Get all pcd files
  std::vector<std::string> pcd_files = ReadFilesInDirWithExt(pcd_dir, ".pcd");

  // Load and concatenate all point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc_all =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc_filtered =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
  for (size_t i = 0; i < pcd_files.size(); ++i) {
    ROS_INFO("%d: %s", (int)i, pcd_files[i].c_str());
    pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_files[i], pcl_pc) == -1) {
      PCL_ERROR("Couldn't read file %s\n", pcd_files[i].c_str());
    }
    *pcl_pc_all += pcl_pc;
  }

  // Voxel filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(pcl_pc_all);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*pcl_pc_filtered);
  pcl_pc_filtered->header.frame_id = "world";

  ROS_INFO("Total point size: %d -> %d.", (int)pcl_pc_all->size(),
           (int)pcl_pc_filtered->size());

  // Set up publisher
  ros::Publisher pcl_pub;
  pcl_pub = pnh.advertise<sensor_msgs::PointCloud2>("all_cloud2", 1, true);
  pcl_pub.publish(*pcl_pc_filtered);
  ros::spin();
}
