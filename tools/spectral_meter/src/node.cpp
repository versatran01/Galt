#include <spectral_meter/node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace galt {
namespace spectral_meter {

const static std::string kWindowName = "spectral_meter (9000)";

void Node::configure() {
  sub_image_ = it_.subscribe("image", 1, &Node::imageCallback, this);
  pnh_.param("ui_scale", ui_scale_, 0.5);
  cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(kWindowName, 
                       &Node::mouseCallbackStatic,
                       reinterpret_cast<void*>(this));
}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& img) {
  cv_bridge::CvImageConstPtr bridged = cv_bridge::toCvShare(img, "mono8");
  if (!bridged || bridged->image.empty()) {
    ROS_ERROR_ONCE_NAMED("convert_error",
                         "Failed to convert image to mono8 w/ cv_bridge");
    return;
  }
  cv::Mat input_image = bridged->image;
  const cv::Size size(input_image.cols, input_image.rows);
  const cv::Size ui_size(size.width * ui_scale_, size.height * ui_scale_);
  
  //  create image for user interface (scale and conert to color)
  cv::Mat ui_image;
  cv::resize(input_image, ui_image, ui_size);
  cv::cvtColor(ui_image, ui_image, CV_GRAY2RGB);
  
  //  click position
  const int selx = click_position_.x;
  const int sely = click_position_.y;
  
  //  draw target position
  {
    //  purple
    const cv::Scalar line_color(138, 43, 226);
    
    const int minretx = std::max(0, selx - 10);
    const int maxretx = std::min(ui_size.width - 1, selx + 10);
    const int minrety = std::max(0, sely - 10);
    const int maxrety = std::min(ui_size.height - 1, sely + 10);
    
    //  vertical lines
    cv::line(ui_image,cv::Point2f(selx,0),
             cv::Point2f(selx,minrety),line_color,2);
    cv::line(ui_image,cv::Point2f(selx,maxrety),
             cv::Point2f(selx,ui_size.height - 1),line_color,2);
    
    //  horizontal lines
    cv::line(ui_image,cv::Point2f(0,sely),
             cv::Point2f(minretx,sely),line_color,2);
    cv::line(ui_image,cv::Point2f(maxretx,sely),
             cv::Point2f(ui_size.width - 1,sely),line_color,2);
    
    //  central box
    cv::Rect rect(minretx,minrety,maxretx - minretx,maxrety - minrety);
    cv::rectangle(ui_image,rect,line_color,2);
  }
  
  //  draw output
  cv::imshow(kWindowName, ui_image);
  
  //  refresh UI
  cv::waitKey(1);
}

void Node::mouseCallback(int event, int x, int y, int flags, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    click_position_ = cv::Point2i(x, y);
  }
  else if (event == cv::EVENT_RBUTTONDOWN) {
  }
  else if  (event == cv::EVENT_MBUTTONDOWN) {
  }
  else if (event == cv::EVENT_MOUSEMOVE) {
  }
}

void Node::mouseCallbackStatic(int event, int x, int y, 
                               int flags, void* userdata) {
  //  pass to instance
  Node* self = reinterpret_cast<Node*>(userdata);
  self->mouseCallback(event,x,y,flags,NULL);
}

} //  spectral_meter
} //  galt
