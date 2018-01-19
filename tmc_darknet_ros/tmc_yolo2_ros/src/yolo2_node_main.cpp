/// @brief ROS node main function for yolo2
/// @brief Copyright (C) 2017 Toyota Motor Corporation
#include "tmc_yolo2_ros/yolo2_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo2_node");
  ros::NodeHandle nh("~");
  boost::shared_ptr<tmc_yolo2_ros::Detector> detector(new tmc_yolo2_ros::Detector);
  tmc_yolo2_ros::Yolo2Node yolo2_node(detector, nh);
  yolo2_node.RunThread();
  ros::spin();
  return EXIT_SUCCESS;
}
