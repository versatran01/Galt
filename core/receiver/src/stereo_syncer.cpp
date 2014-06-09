#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoSyncer {
public:
 MyClass(ros::NodeHandle nh)
     : it_(nh), image_sub1_(it_, "image1", 1), image_sub2_(it_, "image2", 1),
       cinfo_sub1_(it_, "cinfo1", 1), cinfo_sub2_(it_, "cinfo2", 1),
       sync(StereoSyncPolicy(5), image_sub1_, image_sub2_, cinfo_sub1_,
            cinfo_sub2_)
  {
    sync.registerCallback(boost::bind(&StereoSyncer::sync_callback, this, _1, _2, _3, _4));
  }

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
    ROS_INFO("camera1 height: %d, width: %d", cinfo1->height, cinfo1->width);
    ROS_INFO("camera2 height: %d, width: %d", cinfo2->height, cinfo2->width);

    cv::imshow("image1", cv_ptr1->image);
    cv::imshow("image2", cv_ptr2->image);
    cv::waitKey(1);
  }

private:
  image_transport::ImageTransport it_;

  image_transport::SubscriberFilter image_sub1_;
  image_transport::SubscriberFilter image_sub2_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub1_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub2_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo> StereoSyncPolicy;

  message_filters::Synchronizer<StereoSyncPolicy> sync;
};
