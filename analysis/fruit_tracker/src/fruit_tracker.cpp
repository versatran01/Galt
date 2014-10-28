#include <fruit_tracker/fruit_tracker.hpp>

#define BUILD_DEBUG

namespace galt {
namespace fruit_tracker {

void Fruit::detect(const cv::Mat& input, std::vector<Fruit>& output) {
  
  cv::Mat hsv_input;
  cv::cvtColor(input,hsv_input,cv::COLOR_BGR2HSV);
  //  extract separate channels from HSV
  std::vector<cv::Mat> channels(3);
  cv::split(hsv_input,channels);
  //cv::imshow("h",channels[0]);
  //cv::imshow("s",channels[1]);
  //cv::imshow("v",channels[2]);
  
  //  threshold the h-channel (blacks only)
  cv::threshold(channels[0],channels[0],0.1*255,255,cv::THRESH_BINARY_INV);
  //  threshold the v-channel (whites only)
  cv::threshold(channels[2],channels[2],0.95*255,255,cv::THRESH_BINARY_INV);
  
  //  make the detection mask by adding them together...
  cv::Mat detect(hsv_input.rows,hsv_input.cols,CV_8UC1);
  for (int i=0; i < detect.rows; i++) {
    for (int j=0; j < detect.cols; j++) {
      detect.at<uchar>(i,j) = (channels[0].at<uchar>(i,j) && 
          channels[2].at<uchar>(i,j)) * 255;
    }
  }
  //  box blur it
  cv::blur(detect,detect,cv::Size(3,3));
  cv::Mat contour_image;
  detect.copyTo(contour_image);
   
#ifdef BUILD_DEBUG
  cv::Mat output_image;
  input.copyTo(output_image);
#endif
  output.clear();
  
  //  do connected components
  std::vector <std::vector<cv::Point>> contours;
  cv::findContours(contour_image,contours,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);
  //  throw away using area threshold
  for (const std::vector<cv::Point>& contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    if (area > 100) {
      //  find the moments
      cv::Moments moment = cv::moments(contour);
      double inv_m00 = 1 / moment.m00;
      cv::Point2d center = cv::Point2d(moment.m10 * inv_m00, 
                                       moment.m01 * inv_m00);
      const double radius = std::sqrt(area / M_PI);
      
      Fruit fruit;
      fruit.setImageArea(area);
      fruit.setImagePosition(center);
      fruit.setImageRadius(radius);
      output.push_back(fruit);
      
      //  draw onto input image
#ifdef BUILD_DEBUG
      cv::circle(output_image,center,static_cast<int>(radius*1.2),cv::Scalar(0,0,255),2);
#endif
    }
  }
  
#ifdef BUILD_DEBUG
  cv::imshow("bgr", output_image);
  cv::imshow("hsv", hsv_input);
  cv::imshow("detect", detect);
  cv::waitKey(1);
#endif
}

} //  fruit_tracker
} //  galt
