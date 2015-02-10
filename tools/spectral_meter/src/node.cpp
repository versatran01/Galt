#include <spectral_meter/node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace galt {
namespace spectral_meter {

cv::Rect createRectAround(const cv::Point2i& center, double size,
                          const cv::Size& im_size) {
  const int minx = std::floor(std::max(0.0, center.x - size / 2));
  const int maxx = std::ceil(std::min(im_size.width - 1., center.x + size / 2));
  const int miny = std::floor(std::max(0.0, center.y - size / 2));
  const int maxy =
      std::ceil(std::min(im_size.height - 1., center.y + size / 2));
  return cv::Rect(minx, miny, maxx - minx, maxy - miny);
}

const static std::string kWindowName = "spectral_meter (9000)";

void Node::configure() {
  image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  const auto resolved_image = pnh_.resolveName("image");
  ROS_WARN("Subscribing to: %s, transport: %s", resolved_image.c_str(),
           hints.getTransport().c_str());
  // We need a fully resolved name here to subscribe to transport correctly
  sub_image_ =
      it_.subscribe(resolved_image, 1, &Node::imageCallback, this, hints);
  ROS_WARN("Subscribed to: %s, transport: %s", sub_image_.getTopic().c_str(),
           sub_image_.getTransport().c_str());
  pnh_.param("ui_scale", ui_scale_, 0.5);
  //  pnh_.param<std::string>("camera", camera_name_, "color");
  if (!pnh_.hasParam("reflectance")) {
    throw std::invalid_argument("Argument reflectance is required!");
  }
  pnh_.getParam("reflectance", target_reflectance_);
  if (target_reflectance_ < 0 || target_reflectance_ > 1) {
    throw std::invalid_argument("Reflectance must be in the range [0,1]");
  }
  pnh_.param("selection_size", selection_size_, 40);  //  in pixels
  if (selection_size_ < 1) {
    throw std::invalid_argument("Selection size must be >= 1");
  }
  pnh_.param("kp", Kp_, 6000.0);
  if (Kp_ <= 0) {
    throw std::invalid_argument("Kp must be > 0");
  }
  pnh_.param("skip_frames", skip_frame_param_, 3);
  if (skip_frame_param_ < 0) {
    throw std::invalid_argument("skip_frames must be >= 0");
  }

  //  retrieve current camera exposure
  camera_topic_name_ = nh_.getNamespace();
  expose_rosparam_name_ = nh_.resolveName("expose_us");
  if (!nh_.getParamCached(expose_rosparam_name_, expose_us_)) {
    std::string err = "Failed to retrieve param " + expose_rosparam_name_;
    throw std::runtime_error(err);
  }

  cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(kWindowName, &Node::mouseCallbackStatic,
                       reinterpret_cast<void*>(this));

  ROS_INFO("Spectral Meter 9000 - The Ultimate in Exposure Calibration");
  ROS_INFO("\tConfiguring camera: %s", expose_rosparam_name_.c_str());
  ROS_INFO("\tReflectance: %f", target_reflectance_);
  ROS_INFO("\tSelection size: %i", selection_size_);
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

    //  clamp to at least one pixel
    const double box_size = std::max(selection_size_ * ui_scale_, 1.0);
    const cv::Rect draw_rect =
        createRectAround(click_position_, box_size, ui_size);
    const int minretx = draw_rect.x;
    const int maxretx = draw_rect.x + draw_rect.width;
    const int minrety = draw_rect.y;
    const int maxrety = draw_rect.y + draw_rect.height;

    //  vertical lines
    cv::line(ui_image, cv::Point2f(selx, 0), cv::Point2f(selx, minrety),
             line_color, 2);
    cv::line(ui_image, cv::Point2f(selx, maxrety),
             cv::Point2f(selx, ui_size.height - 1), line_color, 2);

    //  horizontal lines
    cv::line(ui_image, cv::Point2f(0, sely), cv::Point2f(minretx, sely),
             line_color, 2);
    cv::line(ui_image, cv::Point2f(maxretx, sely),
             cv::Point2f(ui_size.width - 1, sely), line_color, 2);

    //  central box
    cv::Rect rect(minretx, minrety, maxretx - minretx, maxrety - minrety);
    cv::rectangle(ui_image, rect, line_color, 2);
  }

  //  adjust camera
  if (position_updated_ && (skip_frames_--) <= 0) {
    //  read the position from the image
    const cv::Size scaled_pos(click_position_.x / ui_scale_,
                              click_position_.y / ui_scale_);
    const cv::Rect sel_rect =
        createRectAround(scaled_pos, selection_size_, size);

    if (sel_rect.area() > 0) {
      //  crop out region selected by user
      const cv::Mat region = input_image(sel_rect);
      //  calculate min/max and average
      double min, max;
      cv::minMaxLoc(region, &min, &max);
      const cv::Scalar scal_mean = cv::mean(region);
      const double mean = scal_mean[0];

      ROS_INFO("Selection min/max/mean: %f/%f/%f", min, max, mean);
      //  scale units into 'reflectance'
      updateExposure(mean / 255.0);
    }
    // position_updated_ = false;
  }

  //  draw output
  cv::imshow(kWindowName, ui_image);

  //  refresh UI
  cv::waitKey(1);
}

void Node::callDynamicReconfigure(int expose_us) {
  //  send to dynamic reconfigure
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse resp;
  dynamic_reconfigure::IntParameter int_param;
  dynamic_reconfigure::Config config;

  int_param.name = "expose_us";
  int_param.value = expose_us;
  config.ints.push_back(int_param);
  req.config = config;
  if (!ros::service::call(camera_topic_name_ + "/set_parameters", req, resp)) {
    ROS_WARN("ros::service::call failed while updating exposure");
  }
}

void Node::updateExposure(double measured_mean) {
  nh_.getParamCached(expose_rosparam_name_, expose_us_);

  const double err = target_reflectance_ - measured_mean;
  int inc = Kp_ * err;
  // ROS_INFO("Increment: %i", inc);
  if (std::abs(inc) < 100) {
    //  stop adjusting
    inc = 0;
  }
  //  calculate commanded exposure
  const int expose_us = expose_us_ + inc;

  ROS_INFO("Desired: %f, Measured: %f", target_reflectance_, measured_mean);
  ROS_INFO("Current: %i, Commanding: %i", expose_us_, expose_us);
  if (expose_us_ != expose_us) {
    callDynamicReconfigure(expose_us);
  }
  skip_frames_ = skip_frame_param_;
}

void Node::mouseCallback(int event, int x, int y, int flags, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    click_position_ = cv::Point2i(x, y);
    position_updated_ = true;
  } else if (event == cv::EVENT_RBUTTONDOWN) {
  } else if (event == cv::EVENT_MBUTTONDOWN) {
  } else if (event == cv::EVENT_MOUSEMOVE) {
  }
}

void Node::mouseCallbackStatic(int event, int x, int y, int flags,
                               void* userdata) {
  //  pass to instance
  Node* self = reinterpret_cast<Node*>(userdata);
  self->mouseCallback(event, x, y, flags, NULL);
}

}  //  spectral_meter
}  //  galt
