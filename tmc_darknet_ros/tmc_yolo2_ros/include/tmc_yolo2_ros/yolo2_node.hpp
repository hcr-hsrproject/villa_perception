/// @brief ROS node for yolo2
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#ifndef TMC_YOLO2_ROS_YOLO2_NODE_HPP
#define TMC_YOLO2_ROS_YOLO2_NODE_HPP

#include <string>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "tmc_yolo2_ros/detector.hpp"

namespace tmc_yolo2_ros {
class Yolo2Node {
 public:
  explicit Yolo2Node(const boost::shared_ptr<Detector>& detector,
                     ros::NodeHandle& nh);
  ~Yolo2Node();
  void RunThread();

 protected:
  /// 新しいImageがセットされたか
  bool is_set_image_;
  /// SetImageType関数が呼ばれたか
  bool is_set_image_type_;
  /// RGB画像内R値アクセス用オフセット
  uint8_t channel_r_;
  /// RGB画像内B値アクセス用オフセット
  uint8_t channel_b_;
  /// RGB画像内G値アクセス用オフセット
  uint8_t channel_g_;
  /// 入力画像の(ネットワークに合わせてリサイズする前の)幅
  uint32_t image_w_;
  /// 入力画像の(ネットワークに合わせてリサイズする前の)高さ
  uint32_t image_h_;
  /// ネットワークにおける入力画像の幅
  uint32_t net_w_;
  /// ネットワークにおける入力画像の高さ
  uint32_t net_h_;
  /// 認識器
  boost::shared_ptr<Detector> detector_;
  /// 入力画像の時間情報(書き込み用)
  ros::Time* write_stamp_;
  /// 入力画像の時間情報(読み込み用)
  ros::Time* read_stamp_;
  /// 入力画像(書き込み用)
  image* write_image_;
  /// 入力画像(読み込み用)
  image* read_image_;

  ros::Publisher detection_pub_;
  image_transport::Subscriber image_sub_;

  boost::shared_ptr<boost::thread> thread_;
  boost::mutex mutex_;

  /// 画像から必要な情報を抽出し設定する
  /// param[in] cv_mat cv::Mat型の画像
  /// param[in] encoding 画像のエンコーディング情報
  void SetImageType_(const cv::Mat& cv_mat,
                     const std::string encoding);

  /// cv::Mat型の画像をdarket型の画像に変換する
  /// param[in] cv_mat cv::Mat型の画像
  /// param[out] dst_darknet_image darknet型の画像
  void ConvertImage_(const cv::Mat& cv_mat,
                     image* dst_darknet_image);

  /// 認識処理を実行する
  void Run_();

  /// 画像コールバック関数
  /// param[in] msg sensor_msgs/Image型のメッセージ
  void ImageCallback_(const sensor_msgs::ImageConstPtr& msg);
};
}  // namespace tmc_yolo2_ros

#endif  // TMC_YOLO2_ROS_YOLO2_NODE_HPP
