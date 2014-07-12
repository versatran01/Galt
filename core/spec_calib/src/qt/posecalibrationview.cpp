#include "posecalibrationview.h"
#include "ui_posecalibrationview.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>

PoseCalibrationView::PoseCalibrationView(QWidget *parent, const ros::NodeHandlePtr &nhp) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView), nodeHandle_(nhp)
{
  ui->setupUi(this);
  
  if (!nhp) {
    throw std::invalid_argument("Node handle pointer cannot be null");
  }
  
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(*nhp);
  subImage_.subscribe(*imgTransport_, "image", 20);
  subCamInfo_.subscribe(*nhp, "camera_info", 20);
  
  //  subscribe to synchronized topics
  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(TimeSyncPolicy(10), subImage_, subCamInfo_);
  sync_->registerCallback( boost::bind(&PoseCalibrationView::syncCallback, this, _1, _2) );  
  
  ROS_INFO("Subscribing to ~image and ~camera_info");
}

PoseCalibrationView::~PoseCalibrationView()
{
  delete ui;
}

void PoseCalibrationView::syncCallback(const sensor_msgs::ImageConstPtr& img,
                  const sensor_msgs::CameraInfoConstPtr &info)
{
  //  convert image to OpenCV format
  cv_bridge::CvImageConstPtr bridgedImagePtr = cv_bridge::toCvCopy(img,"rgb8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image to rgb8 with cv_bridge");
    return;
  }
  const cv_bridge::CvImage& bridgedImage = *bridgedImagePtr;
  
  //  update ui
  ui->imageWidget->setImage(bridgedImage.image);
}
