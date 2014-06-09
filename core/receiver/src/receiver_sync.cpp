#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void sync_callback(const sensor_msgs::ImageConstPtr &image1,
                   const sensor_msgs::ImageConstPtr &image2,
                   const sensor_msgs::CameraInfoConstPtr &cinfo1,
                   const sensor_msgs::CameraInfoConstPtr &cinfo2)
{
  cv_bridge::CvImagePtr cv_ptr1;
  cv_bridge::CvImagePtr cv_ptr2;

  cv_ptr1 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::MONO8);
  cv_ptr2 = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::MONO8);

  ROS_INFO("camera1 time: %u.%u", image1->header.stamp.sec,
           image1->header.stamp.nsec);
  ROS_INFO("camera2 time: %u.%u", image2->header.stamp.sec,
           image2->header.stamp.nsec);
  ROS_INFO("camera1 height: %d, width: %d", cinfo1->height,
           cinfo1->width);
  ROS_INFO("camera2 height: %d, width: %d", cinfo2->height,
           cinfo2->width);

  cv::imshow("image1", cv_ptr1->image);
  cv::imshow("image2", cv_ptr2->image);
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver_sync");
  ROS_INFO("Starting node: receiver_sync");

  cv::namedWindow("image1", 1);
  cv::namedWindow("image2", 1);
  ROS_INFO("Creating windows");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter image_sub1(it, "image1", 1);
  image_transport::SubscriberFilter image_sub2(it, "image2", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub1(nh, "cinfo1",
                                                                  1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub2(nh, "cinfo2",
                                                                  1);

  // This somehow doesn't work
  // message_filters::TimeSynchronizer<
  //     sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
  //     sensor_msgs::CameraInfo> sync(image_sub1, image_sub2, cinfo_sub1,
  //                                   cinfo_sub2, 5);

  // This might work
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo> StereoSyncPolicy;
  message_filters::Synchronizer<StereoSyncPolicy> sync(
      StereoSyncPolicy(5), image_sub1, image_sub2, cinfo_sub1, cinfo_sub2);

  sync.registerCallback(boost::bind(&sync_callback, _1, _2, _3, _4));

  ros::spin();

  // StereoSyncer ss(ros::NodeHandle("~"));
  // while(ros::ok()){
  //     ros::spin();
  // }

  return 0;
}
