cmake_minimum_required(VERSION 2.8.3)
project(villa_rude_carnie_ros)

find_package(catkin REQUIRED
    rospy
)

catkin_package(
  CATKIN_DEPENDS tmc_yolo2_ros
)

catkin_python_setup()
