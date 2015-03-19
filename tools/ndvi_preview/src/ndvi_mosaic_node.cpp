#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <boost/filesystem.hpp>

bool IsFolderValid(const std::string& dir) {
  namespace bfs = boost::filesystem;
  bfs::path path(dir);
  if (!bfs::exists(path) || !bfs::is_directory(path)) return false;
  return true;
}

std::vector<std::string> GetAllFilesWithExt(const std::string& dir,
                                            const std::string& ext) {
  namespace bfs = boost::filesystem;
  if (!IsFolderValid(dir))
    throw std::runtime_error(std::string("Invalid folder: ") + dir);

  bfs::path path(dir);
  std::vector<std::string> files;

  // Such nonsense that qt only autocompletes when specify full namespace
  std::for_each(bfs::directory_iterator(path), bfs::directory_iterator(),
                [&](const boost::filesystem::directory_entry& entry) {
    if (ext.compare(entry.path().extension().string()) == 0) {
      files.push_back(entry.path().string());
    }
  });

  std::sort(std::begin(files), std::end(files));
  return files;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ndvi_mosaic");
  ros::NodeHandle pnh("~");

  // Parameters
  double scale;
  pnh.param("scale", scale, 0.5);

  std::string image_dir;
  pnh.param<std::string>("image_dir", image_dir,
                         "/home/chao/Workspace/bag/aerial");

  int num_images;
  pnh.param("num_images", num_images, 10);
  if (num_images < 2) {
    ROS_WARN("Number of images is less than 2");
    return -1;
  }

  // Load all images
  std::vector<std::string> files = GetAllFilesWithExt(image_dir, ".png");
  std::vector<cv::Mat> images;
  int i = 0;
  for (const std::string& f : files) {
    auto image = cv::imread(f, CV_LOAD_IMAGE_COLOR);
    cv::Mat image_scaled;
    cv::resize(image, image_scaled, cv::Size(0, 0), scale, scale);
    images.push_back(image_scaled);
    ROS_INFO("Image: %s", f.c_str());
    if (++i == num_images) break;
  }

  // Stitching
  cv::Mat result;
  auto stitcher = cv::Stitcher::createDefault();
  auto status = stitcher.stitch(images, result);

  if (status != cv::Stitcher::OK) {
    ROS_WARN("Can't stitch images, error code = %d", (int)status);
    return -1;
  }
  cv::imshow("result", result);
  cv::waitKey(-1);
}
