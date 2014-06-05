#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void image_callback(const sensor_msgs::ImageConstPtr &img1,
                    const sensor_msgs::ImageConstPtr &img2)
{
  cv_bridge::CvImagePtr cv_ptr1;
  cv_bridge::CvImagePtr cv_ptr2;

  cv_ptr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::MONO8);
  cv_ptr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::MONO8);

  ROS_INFO("cam1 sync: %u.%u", img1->header.stamp.sec, img1->header.stamp.nsec);
  ROS_INFO("cam2 sync: %u.%u", img2->header.stamp.sec, img2->header.stamp.nsec);

  cv::imshow("image1", cv_ptr1->image);
  cv::imshow("image2", cv_ptr2->image);
  cv::waitKey(5);
}

class MyClass {
public:
  MyClass(ros::NodeHandle nh)
    : it_(nh),
      image1_sub_(it_, "image1", 1),
      image2_sub_(it_, "image2", 1),
      sync(MySyncPolicy(5), image1_sub_, image2_sub_)
  {
    sync.registerCallback(boost::bind(&MyClass::image_callback, this, _1, _2));
  }

  void image_callback(const sensor_msgs::ImageConstPtr &img1,
                      const sensor_msgs::ImageConstPtr &img2)
  {
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;

    cv_ptr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::MONO8);
    cv_ptr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::MONO8);

    cv::imshow("image1", cv_ptr1->image);
    cv::imshow("image2", cv_ptr2->image);
    cv::waitKey(1);
  }

private:
  image_transport::ImageTransport it_;

  image_transport::SubscriberFilter image1_sub_;
  image_transport::SubscriberFilter image2_sub_;

  typedef message_filters::sync_policies::ApproximateTime <
  sensor_msgs::Image, sensor_msgs::Image > MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver_node");
  ROS_INFO("Starting node: receiver_node");

  cv::namedWindow("image1", 1);
  cv::namedWindow("image2", 1);
  ROS_INFO("Creating windows");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter image1_sub(it, "image1", 1);
  image_transport::SubscriberFilter image2_sub(it, "image2", 1);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image1_sub,
                                                   image2_sub);
  sync.registerCallback(boost::bind(&image_callback, _1, _2));

  ros::spin();
  // MyClass mc(ros::NodeHandle("~"));
  // while(ros::ok()){
  //     ros::spin();
  // }

  return 0;
}
