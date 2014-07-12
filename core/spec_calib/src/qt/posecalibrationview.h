#ifndef POSECALIBRATIONVIEW_H
#define POSECALIBRATIONVIEW_H

#include <QWidget>
#include <memory>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <circle_tracker/Circles.h>

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
                    const circle_tracker::CirclesConstPtr&);
  
private:
  Ui::PoseCalibrationView *ui;
  ros::NodeHandlePtr nodeHandle_;
  
  std::shared_ptr<image_transport::ImageTransport> imgTransport_;
  
  //  ROS subscribers
  static constexpr uint32_t kROSQueueSize = 10;
  
  image_transport::SubscriberFilter subImage_;
  message_filters::Subscriber <sensor_msgs::CameraInfo> subCamInfo_;
  message_filters::Subscriber <circle_tracker::Circles> subCircles_;
  
  //  time sync
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo, circle_tracker::Circles> TimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
};

#endif // POSECALIBRATIONVIEW_H
