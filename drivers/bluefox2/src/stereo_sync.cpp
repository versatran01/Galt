#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #define GUI
#ifdef GUI
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

typedef image_transport::SubscriberFilter ImageSubscriber;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CinfoSubscriber;
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
    sensor_msgs::CameraInfo> StereoSyncPolicy;

image_transport::CameraPublisher left_pub;
image_transport::CameraPublisher right_pub;

void sync_callback(const sensor_msgs::ImageConstPtr &image1,
                   const sensor_msgs::ImageConstPtr &image2,
                   const sensor_msgs::CameraInfoConstPtr &cinfo1,
                   const sensor_msgs::CameraInfoConstPtr &cinfo2) {
  // Synchronizer time stamp
  sensor_msgs::ImagePtr image2_sync(new sensor_msgs::Image(*image2));
  sensor_msgs::CameraInfoPtr cinfo2_sync(new sensor_msgs::CameraInfo(*cinfo2));
  image2_sync->header.stamp = image1->header.stamp;
  cinfo2_sync->header.stamp = cinfo1->header.stamp;

  // Publish synchronized topics
  left_pub.publish(image1, cinfo1);
  right_pub.publish(image2_sync, cinfo2_sync);

  ROS_INFO("camera1 time: %u.%u", image1->header.stamp.sec,
           image1->header.stamp.nsec);
  ROS_INFO("camera2 time: %u.%u", image2_sync->header.stamp.sec,
           image2_sync->header.stamp.nsec);
  ROS_INFO("height: %d,%d, width: %d,%d", cinfo1->height, cinfo2->height,
           cinfo1->width, cinfo2->width);

#ifdef GUI
  cv_bridge::CvImagePtr cv_ptr1;
  cv_bridge::CvImagePtr cv_ptr2;

  cv_ptr1 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::MONO8);
  cv_ptr2 = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::MONO8);
  cv::imshow("image1", cv_ptr1->image);
  cv::imshow("image2", cv_ptr2->image);
  cv::waitKey(1);
#endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_sync");
  ROS_INFO("Starting node: stereo_sync");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  // Subscriber
  ImageSubscriber image_sub1(it, "image1", 1);
  ImageSubscriber image_sub2(it, "image2", 1);
  CinfoSubscriber cinfo_sub1(nh, "cinfo1", 1);
  CinfoSubscriber cinfo_sub2(nh, "cinfo2", 1);

  // Publisher
  left_pub = it.advertiseCamera("left/image", 1);
  right_pub = it.advertiseCamera("right/image", 1);

#ifdef GUI
  cv::namedWindow("image1", 1);
  cv::namedWindow("image2", 1);
  ROS_INFO("Creating windows");
#endif

  // Use stereo sync policy
  message_filters::Synchronizer<StereoSyncPolicy> sync(
      StereoSyncPolicy(5), image_sub1, image_sub2, cinfo_sub1, cinfo_sub2);
  sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
