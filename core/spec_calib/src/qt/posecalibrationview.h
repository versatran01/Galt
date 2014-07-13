#ifndef POSECALIBRATIONVIEW_H
#define POSECALIBRATIONVIEW_H

#include <QWidget>
#include <memory>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <circle_tracker/Circles.h>
#include <monocular_pose_estimator/PixelArray.h>

namespace Ui {
class PoseCalibrationView;
}

class PoseCalibrationView : public QWidget
{
  Q_OBJECT
  
public:
  explicit PoseCalibrationView(QWidget *parent, const ros::NodeHandlePtr& nhp);
  ~PoseCalibrationView();
  
  //  ROS callback
  void syncCallback(const sensor_msgs::ImageConstPtr&,
                    const sensor_msgs::CameraInfoConstPtr&,
                    const circle_tracker::CirclesConstPtr&,
                    const geometry_msgs::PoseWithCovarianceStampedConstPtr&,
                    const monocular_pose_estimator::PixelArrayConstPtr&);
  
private:
  Ui::PoseCalibrationView *ui;
  ros::NodeHandlePtr nodeHandle_;
  
  std::shared_ptr<image_transport::ImageTransport> imgTransport_;
  
  //  ROS subscribers
  static constexpr uint32_t kROSQueueSize = 10;
  
  image_transport::SubscriberFilter subImage_;
  message_filters::Subscriber <sensor_msgs::CameraInfo> subCamInfo_;
  message_filters::Subscriber <circle_tracker::Circles> subCircles_;
  message_filters::Subscriber <geometry_msgs::PoseWithCovarianceStamped> subPose_;
  message_filters::Subscriber <monocular_pose_estimator::PixelArray> subPixels_;
  
  //  time sync
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo, circle_tracker::Circles,
      geometry_msgs::PoseWithCovarianceStamped, monocular_pose_estimator::PixelArray> TimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
};

#endif // POSECALIBRATIONVIEW_H
