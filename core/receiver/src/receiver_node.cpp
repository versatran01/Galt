#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>

void cam_callback( const sensor_msgs::ImageConstPtr &img) {
  //Convert ros msg to cv Mat
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  cv::Mat image_gray;

  cv::imshow("camera", cv_ptr->image);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver_node");
  ROS_INFO("Starting node: receiver_node");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  cv::namedWindow("camera", 1);
  //Subscribe to camera message
  image_transport::Subscriber camera_sub = it.subscribe("camera", 1, cam_callback);
  ros::spin();

  return 0;
}
