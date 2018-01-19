/// @brief ROS node for yolo2
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <algorithm>
#include <string>
#include <utility>
#include <cv_bridge/cv_bridge.h>
#include <tmc_yolo2_ros/Detections.h>
#include "tmc_yolo2_ros/yolo2_node.hpp"

namespace {
/// 認識結果として採用する信頼度の閾値 [0.0, 1.0]
static const double kDefaultConfidenceThreshold = 0.3;
/// Non-Maximum suppression処理を行う際の重複領域判定(IoU)の閾値 [0.0, 1.0]
static const double kDefaultNmsThreshold = 0.4;
}  // anonymous namespace

namespace tmc_yolo2_ros {

Yolo2Node::Yolo2Node(const boost::shared_ptr<Detector>& detector,
                     ros::NodeHandle& nh) :
  is_set_image_(false),
  is_set_image_type_(false),
  detector_(detector),
  read_stamp_(new ros::Time(0.0)),
  write_stamp_(new ros::Time(0.0)),
  read_image_(new image),
  write_image_(new image) {
  double confidence_threshold, nms_threshold;
  std::string config_path, weights_path, class_names_path;
  image_transport::ImageTransport it(nh);
  if (!nh.getParam("cfg_path", config_path)) {
    ROS_FATAL("Cannot get cfg_path parameter.");
    exit(EXIT_FAILURE);
  }
  if (!nh.getParam("weights_path", weights_path)) {
    ROS_FATAL("Cannot get weights_path parameter.");
    exit(EXIT_FAILURE);
  }
  if (!nh.getParam("class_names_path", class_names_path)) {
    ROS_FATAL("Cannot get class_names_path parameter.");
    exit(EXIT_FAILURE);
  }
  nh.param<double>("confidence_threshold", confidence_threshold, kDefaultConfidenceThreshold);
  nh.param<double>("nms_threshold", nms_threshold, kDefaultNmsThreshold);
  if (!detector_->Initialize(config_path, weights_path, class_names_path,
                             confidence_threshold, nms_threshold, net_w_, net_h_)) {
    ROS_FATAL("Cannot initialize detector.");
    exit(EXIT_FAILURE);
  }
  *read_image_ = make_image(net_w_, net_h_, 3);
  *write_image_ = make_image(net_w_, net_h_, 3);

  image_sub_ = it.subscribe("image", 1,
                            &Yolo2Node::ImageCallback_, this);
  detection_pub_ = nh.advertise<Detections>("detections", 5);
}

Yolo2Node::~Yolo2Node() {
  if (thread_ && thread_->joinable()) {
    thread_->interrupt();
    thread_->join();
  }
  free_image(*write_image_);
  free_image(*read_image_);
}

void Yolo2Node::RunThread() {
  thread_.reset(new boost::thread(boost::bind(&Yolo2Node::Run_, this)));
}

/// 画像から必要な情報を抽出し設定する
void Yolo2Node::SetImageType_(const cv::Mat& cv_mat,
                              const std::string encoding) {
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    channel_r_ = 0;
    channel_g_ = 1;
    channel_b_ = 2;
  } else if (encoding == sensor_msgs::image_encodings::BGR8) {
    channel_r_ = 2;
    channel_g_ = 1;
    channel_b_ = 0;
  } else {
    ROS_FATAL("Unsupported encoding");
    exit(EXIT_FAILURE);
  }
  image_w_ = cv_mat.cols;
  image_h_ = cv_mat.rows;
}

/// cv::Mat型の画像をdarket型の画像に変換する
void Yolo2Node::ConvertImage_(const cv::Mat& cv_mat,
                              image* dst_darknet_image) {
  cv::Mat resize_mat = cv::Mat::ones(net_h_, net_w_, CV_8UC3);
  cv::resize(cv_mat, resize_mat, resize_mat.size(), cv::INTER_CUBIC);
  uint32_t i = 0;
  for (uint32_t row = 0; row < net_h_; ++row) {
    for (uint32_t col = 0; col < net_w_; ++col, ++i) {
      dst_darknet_image->data[i + net_w_ * net_h_ * 0] = resize_mat.at<cv::Vec3b>(row, col)[channel_r_] / 255.;
      dst_darknet_image->data[i + net_w_ * net_h_ * 1] = resize_mat.at<cv::Vec3b>(row, col)[channel_g_] / 255.;
      dst_darknet_image->data[i + net_w_ * net_h_ * 2] = resize_mat.at<cv::Vec3b>(row, col)[channel_b_] / 255.;
    }
  }
}

/// 認識処理を実行する
void Yolo2Node::Run_() {
  boost::shared_ptr<Detections> detections(new Detections);
  while (!is_set_image_type_) {
    ros::Duration(0.01).sleep();
  }
  while (ros::ok()) {
    if (!is_set_image_) {
      ros::Duration(0.01).sleep();
      continue;
    }
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      std::swap(read_stamp_, write_stamp_);
      std::swap(read_image_, write_image_);
    }
    is_set_image_ = false;
    detections->header.stamp = *read_stamp_;
    detections->detections = detector_->Execute(read_image_, image_w_, image_h_);
    detection_pub_.publish(detections);
  }
}

/// 画像コールバック関数
void Yolo2Node::ImageCallback_(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cv_mat = cv_ptr->image;
  if (!is_set_image_type_) {
    SetImageType_(cv_mat, msg->encoding);
    is_set_image_type_ = true;
  }
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    *write_stamp_ = msg->header.stamp;
    ConvertImage_(cv_mat, write_image_);
  }
  is_set_image_ = true;
}

}  // namespace tmc_yolo2_ros
