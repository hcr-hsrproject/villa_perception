/// @brief ROS node for yolo2 test
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <tmc_yolo2_ros/Detections.h>
#include "tmc_yolo2_ros/yolo2_node.hpp"

namespace {

const double kRate = 10.0;

}  // unnamed namespace

namespace tmc_yolo2_ros {

namespace enc = sensor_msgs::image_encodings;


class Yolo2NodeTest : public ::testing::Test {
 protected:
  Yolo2NodeTest() {
    image_pub_ =
      nh_.advertise<sensor_msgs::Image>("/dummy_image", 1, false);
    detections_sub_ =
      nh_.subscribe<Detections>("/yolo2_node_stub/detections", 1,
                                &Yolo2NodeTest::callback, this);
    ros::Rate rate(kRate);
    while (image_pub_.getNumSubscribers() == 0 || detections_sub_.getNumPublishers() == 0) {
      rate.sleep();
    }
  }

  virtual void SetUp() {
    detections_msg_count_ = 0;
  }

  sensor_msgs::ImageConstPtr CreateDummyImage() {
    cv::Mat mat = cv::Mat::zeros(1, 1,
                                 cv_bridge::getCvType(enc::RGB8));
    std_msgs::Header header;
    cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, enc::RGB8, mat);
    return cv_image.toImageMsg();
  }

  void PublishImage(uint8_t pub_num) {
    sensor_msgs::ImageConstPtr dummy_image = CreateDummyImage();

    ros::Rate loop_rate(kRate);
    ros::Time init = ros::Time::now();
    ros::Time now = init;
    uint8_t pub_count = 0;
    while ((now - init).toSec() < 1.0) {
      if (pub_count < pub_num) {
        image_pub_.publish(dummy_image);
        pub_count++;
      }
      ros::spinOnce();
      now = ros::Time::now();
      loop_rate.sleep();
    }
  }

  void callback(const DetectionsConstPtr& msg) {
    detections_msg_count_++;
  }

  int detections_msg_count_;
  ros::NodeHandle nh_;
  ros::Publisher image_pub_;
  ros::Subscriber detections_sub_;
};

// 画像を1度も発行しなかったら認識結果を取得しない
TEST_F(Yolo2NodeTest, ImageNotPublished) {
  PublishImage(0);
  EXPECT_EQ(0, detections_msg_count_);
}

// 画像を1度発行したら認識結果を1度取得
TEST_F(Yolo2NodeTest, SingleImagePublished) {
  PublishImage(1);
  EXPECT_EQ(1, detections_msg_count_);
}

// 画像を10度発行したら認識結果を10度取得
TEST_F(Yolo2NodeTest, MultipleImagePublished) {
  PublishImage(10);
  EXPECT_EQ(10, detections_msg_count_);
}

}  // namespace tmc_yolo2_ros


int main(int argc, char** argv) {
  ros::init(argc, argv, "tmc_yolo2_ros_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
