/// @brief Yolo2 Node Stub
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "tmc_yolo2_ros/yolo2_node.hpp"

namespace {
static const int kWidth = 160;
static const int kHeight = 120;
}  // anonymous namespace

namespace tmc_yolo2_ros {

class StubDetector : public Detector {
 public:
  StubDetector() {}

  bool Initialize(const std::string& config_path,
                  const std::string& weights_path,
                  const std::string& class_names_path,
                  double confidence_threshold,
                  double nms_threshold,
                  uint32_t& dst_width,
                  uint32_t& dst_height) {
    dst_width = kWidth;
    dst_height = kHeight;
    return true;
  }
  std::vector<tmc_yolo2_ros::Detection> Execute(image* darknet_image,
                                                uint32_t image_w,
                                                uint32_t image_h) {
    std::vector<tmc_yolo2_ros::Detection> detections;
    return detections;
  }
};

}  // namespace tmc_yolo2_ros



int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo2_node_stub");
  ros::NodeHandle nh("~");
  boost::shared_ptr<tmc_yolo2_ros::StubDetector> detector(new tmc_yolo2_ros::StubDetector);
  tmc_yolo2_ros::Yolo2Node yolo2_node_stub(detector, nh);
  yolo2_node_stub.RunThread();
  ros::spin();
  return EXIT_SUCCESS;
}
