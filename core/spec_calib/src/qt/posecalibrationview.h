#ifndef POSECALIBRATIONVIEW_H
#define POSECALIBRATIONVIEW_H

#include <QWidget>
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

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
                    const sensor_msgs::CameraInfoConstPtr&);
  
private:
  Ui::PoseCalibrationView *ui;
  ros::NodeHandlePtr nodeHandle_;
  
  std::shared_ptr<image_transport::ImageTransport> imgTransport_;
  
  //  ROS subscribers
  image_transport::SubscriberFilter subImage_;
  message_filters::Subscriber <sensor_msgs::CameraInfo> subCamInfo_;
  
  //  time sync
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo> TimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
};

#endif // POSECALIBRATIONVIEW_H
