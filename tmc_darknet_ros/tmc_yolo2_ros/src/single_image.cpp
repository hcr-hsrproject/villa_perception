/// @brief ROS node for yolo2
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <algorithm>
#include <string>
#include <utility>
#include "cv.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <tmc_yolo2_ros/Detections.h>
#include "tmc_yolo2_ros/yolo2_node.hpp"

uint8_t channel_r;
uint8_t channel_g;
uint8_t channel_b;

uint32_t image_w;
uint32_t image_h;
uint32_t net_w;
uint32_t net_h;

/// 画像から必要な情報を抽出し設定する
void setImageType(const cv::Mat& cv_mat,
                              const std::string encoding) {
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    channel_r = 0;
    channel_g = 1;
    channel_b = 2;
  } else if (encoding == sensor_msgs::image_encodings::BGR8) {
    channel_r = 2;
    channel_g = 1;
    channel_b = 0;
  } else {
    exit(EXIT_FAILURE);
  }
  image_w = cv_mat.cols;
  image_h = cv_mat.rows;
}

/// cv::Mat型の画像をdarket型の画像に変換する
void convertImage(const cv::Mat& cv_mat,
                              image* dst_darknet_image) {
  cv::Mat resize_mat = cv::Mat::ones(net_h, net_w, CV_8UC3);
  cv::resize(cv_mat, resize_mat, resize_mat.size(), cv::INTER_CUBIC);
  uint32_t i = 0;
  for (uint32_t row = 0; row < net_h; ++row) {
    for (uint32_t col = 0; col < net_w; ++col, ++i) {
      dst_darknet_image->data[i + net_w * net_h * 0] = resize_mat.at<cv::Vec3b>(row, col)[channel_r] / 255.;
      dst_darknet_image->data[i + net_w * net_h * 1] = resize_mat.at<cv::Vec3b>(row, col)[channel_g] / 255.;
      dst_darknet_image->data[i + net_w * net_h * 2] = resize_mat.at<cv::Vec3b>(row, col)[channel_b] / 255.;
    }
  }
}
std::string get_working_path()
{
   char temp[2048];
   return ( getcwd(temp, 2048) ? std::string( temp ) : std::string("") );
}

int main(int argc, char** argv) {
    if (argc < 5) {
        exit(1);
    }
    boost::shared_ptr<tmc_yolo2_ros::Detector> detector(new tmc_yolo2_ros::Detector);
    bool success = detector->Initialize(argv[1],argv[2],argv[3], 0.1, 0.3, net_w, net_h);
    if (!success) {
        exit(1);
    }
    
    std::string cwd = get_working_path();
    for (int i = 4; i < argc; i++) {
        std::cout << "Processing " << argv[i] << "..." << std::endl;
        std::string filepath = argv[i];
        size_t lastdot = filepath.find_last_of(".");
        size_t lastslash = filepath.find_last_of("/");
        std::string rawname = filepath.substr(lastslash, lastdot);
        std::string outpath = cwd;
        outpath.append(rawname);
        outpath.append("_detections.png");
        cv::Mat cvImage = cv::imread(argv[i], CV_LOAD_IMAGE_COLOR);
        if(! cvImage.data ) {
            std::cout <<  "Could not open or find the image" << std::endl ;
            return -1;
        }

        setImageType(cvImage, sensor_msgs::image_encodings::BGR8);
        image* darknetImage = new image;
        *darknetImage = make_image(net_w, net_h, 3);
        convertImage(cvImage,darknetImage);
        std::vector<tmc_yolo2_ros::Detection> detections = detector->Execute(darknetImage,image_w, image_h);
        std::ofstream detections_file;
        std::string text_outpath = cwd;
        text_outpath.append(rawname);
        text_outpath.append("_detections.txt");
        detections_file.open(text_outpath.c_str());

        for (std::vector<tmc_yolo2_ros::Detection>::iterator it = detections.begin() ; it != detections.end(); ++it) {

            tmc_yolo2_ros::Detection detection = *it;
            detections_file << detection.class_id << " " <<  detection.class_name << " " << detection.confidence << " ";
            detections_file << detection.x << " " << detection.y << " " << detection.width << " " << detection.height;
            detections_file << std::endl;
            if (detection.confidence < 0.35) {
                continue;
            }
            cv::rectangle(cvImage, cv::Point(detection.x - detection.width / 2.0,
                          detection.y - detection.height / 2.0),
                          cv::Point(detection.x + detection.width / 2.0,
                          detection.y + detection.height / 2.0), cv::Scalar(255,0,0),2);
            cv::rectangle(cvImage, cv::Point(detection.x, detection.y - 20),
                          cv::Point(detection.x + 15 * detection.class_name.length() + 5, detection.y),
                          cv::Scalar(255,255,255), -1);
            cv::putText(cvImage, detection.class_name, cv::Point(detection.x, detection.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255,0,0),2);

        }
        detections_file.close();
        cv::imwrite(outpath,cvImage);
    }
    
    return EXIT_SUCCESS;
}
