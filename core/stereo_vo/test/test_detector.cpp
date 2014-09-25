#include <iostream>

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>

void printParams(cv::Algorithm* algorithm);

int main(int argc, char** argv) {
  std::string filename(
      "/home/chao/Workspace/repo/mrsl/Galt/core/stereo_vo/test/im0.jpg");
  cv::Mat image;
  image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

  if (!image.data) {
    ROS_WARN("Could ont open or find image: %s", filename.c_str());
    return -1;
  }

  std::vector<cv::KeyPoint> key_points;
  cv::Ptr<cv::FeatureDetector> detector =
      cv::FeatureDetector::create("GridBRISK");
  //  detector->set("nfeatures", 200);
  //  detector->set("minDistance", 20.0);
  //  detector->set("useHarrisDetector", true);
  detector->set("gridCols", 4);
  detector->set("gridRows", 4);
  detector->set("maxTotalKeypoints", 200);
  detector->detect(image, key_points);
  printParams(detector);

  ROS_INFO("number of corners: %d", (int)key_points.size());

  cv::Mat disp;
  cv::drawKeypoints(image, key_points, disp, cv::Scalar(255, 0, 0));

  cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
  cv::imshow("disp", disp);
  cv::waitKey(0);
}

void printParams(cv::Algorithm* algorithm) {
  std::vector<std::string> parameters;
  algorithm->getParams(parameters);

  for (int i = 0; i < (int)parameters.size(); i++) {
    std::string param = parameters[i];
    int type = algorithm->paramType(param);
    std::string helpText = algorithm->paramHelp(param);
    std::string typeText;

    switch (type) {
      case cv::Param::BOOLEAN:
        typeText = "bool";
        break;
      case cv::Param::INT:
        typeText = "int";
        break;
      case cv::Param::REAL:
        typeText = "real (double)";
        break;
      case cv::Param::STRING:
        typeText = "string";
        break;
      case cv::Param::MAT:
        typeText = "Mat";
        break;
      case cv::Param::ALGORITHM:
        typeText = "Algorithm";
        break;
      case cv::Param::MAT_VECTOR:
        typeText = "Mat vector";
        break;
    }
    std::cout << "Parameter '" << param << "' type=" << typeText
              << " help=" << helpText << std::endl;
  }
}
