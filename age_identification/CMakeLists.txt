cmake_minimum_required(VERSION 2.8.3)
project(age_identification)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  tmc_yolo2_ros
  villa_rude_carnie_ros
)


catkin_install_python(PROGRAMS scripts/age_publisher
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})