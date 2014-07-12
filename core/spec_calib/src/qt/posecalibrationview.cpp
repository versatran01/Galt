#include "posecalibrationview.h"
#include "ui_posecalibrationview.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>

//  TODO: copied from CameraIntx, refactor this into utility class later
template <typename Scalar>
cv::Point_<Scalar> distortPoint(const std::vector<Scalar>& coeffs, const cv::Point_<Scalar>& src)
{
  union {
      struct {
          Scalar k1,k2,p1,p2,k3,k4,k5,k6;
      } C;
      Scalar k[8];
  } __attribute__((packed));
  
  for (size_t i=0; i < 8; i++) {
    k[i] = 0;
  }
  for (size_t i=0; i < coeffs.size(); i++) {
    k[i] = coeffs[i];
  }
  
  const Scalar xx = src.x*src.x;
  const Scalar yy = src.y*src.y;
  const Scalar xy = src.x*src.y;
  
  const Scalar r2 = xx + yy;
  const Scalar rad = (1 + r2*(C.k1 + r2*(C.k2 + C.k3*r2))) / (1 + r2*(C.k4 + r2*(C.k5 + C.k6*r2)));
  
  //  distort
  const Scalar x = src.x*rad + 2*C.p1*xy + C.p2*(r2 + 2*xx);
  const Scalar y = src.y*rad + C.p1*(r2 + 2*yy) + 2*C.p2*xy;
  
  cv::Point_<Scalar> dst;
  dst.x = x;
  dst.y = y;
  return dst;
}

PoseCalibrationView::PoseCalibrationView(QWidget *parent, const ros::NodeHandlePtr &nhp) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView), nodeHandle_(nhp)
{
  ui->setupUi(this);
  
  if (!nhp) {
    throw std::invalid_argument("Node handle pointer cannot be null");
  }
  
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(*nhp);
  subImage_.subscribe(*imgTransport_, "image", kROSQueueSize);
  subCamInfo_.subscribe(*nhp, "camera_info", kROSQueueSize);
  subCircles_.subscribe(*nhp, "circles", kROSQueueSize);
  
  //  subscribe to synchronized topics
  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(TimeSyncPolicy(kROSQueueSize), subImage_, subCamInfo_, subCircles_);
  sync_->registerCallback( boost::bind(&PoseCalibrationView::syncCallback, this, _1, _2, _3) );  
  
  ROS_INFO("Subscribing to ~image, ~camera_info, ~circles");
}

PoseCalibrationView::~PoseCalibrationView()
{
  delete ui;
}

void PoseCalibrationView::syncCallback(const sensor_msgs::ImageConstPtr& img,
                  const sensor_msgs::CameraInfoConstPtr &info, const circle_tracker::CirclesConstPtr &circles)
{
  //  convert image to OpenCV format
  cv_bridge::CvImageConstPtr bridgedImagePtr = cv_bridge::toCvCopy(img,"rgb8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image to rgb8 with cv_bridge");
    return;
  }  
  cv::Mat image = bridgedImagePtr->image;
  
  //  camera intrinsics
  double fx = info->K[0];
  double cx = info->K[2];
  double fy = info->K[4];
  double cy = info->K[5];
  double f = std::sqrt(fx*fy);
  std::vector<double> distCoeffs = info->D;
  
  //  draw some circles
  for (const circle_tracker::Circle& circ : circles->circles) {
    cv::Point2d p(circ.centerX,circ.centerY);
    p = distortPoint(distCoeffs,p);
    
    double x = fx*p.x + cx;
    double y = fy*p.y + cy;
    double rad = circ.radius * f;
    
    cv::circle(image, cv::Point2d(x,y),rad,cv::Scalar(255,0,0),3);
  }
  
  //  update ui
  ui->imageWidget->setImage(image);
}
