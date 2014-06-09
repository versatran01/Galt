#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void image_callback(const sensor_msgs::ImageConstPtr &image,
                    const sensor_msgs::CameraInfoConstPtr &camera_info)
{
  cv_bridge::CvImagePtr cv_ptr;

  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);

  ROS_INFO("Height: %d, Width: %d", camera_info->height, camera_info->width);

  cv::imshow("image", cv_ptr->image);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");
  ROS_INFO("Starting node: receiver");

  cv::namedWindow("image", 1);
  ROS_INFO("Creating image window");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber camera_sub =
      it.subscribeCamera("image", 1, image_callback);

  ros::spin();

  return 0;
 }
