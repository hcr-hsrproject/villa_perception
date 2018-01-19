cmake_minimum_required(VERSION 2.8.3)
project(villa_calibration)

#SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
)

include_directories(
    include
    ${Ceres_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_executable(villa_calibration
    #src/CalibrationModel.cc
    src/CameraIntrinsics.cc
    src/CameraOptimizationParameters.cc
    src/CVUtil.cc
    src/DistortionModel.cc
    src/HomCartShortcuts.cc
    src/HomographyShortcuts.cc
    src/LinearAlgebraShortcuts.cc
    src/ModelChessboard.cc
    src/OptRTDistCostFunctor.cc
    src/RigidTrans.cc
    src/SingleCameraOptimizer.cc
    src/TransformShortcuts.cc
    src/Util.cc
    src/ZhangLinearEstimate.cc
    src/villa_calibration.cc)

target_link_libraries(villa_calibration
    ceres
    ${Ceres_LIBS}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

foreach(dir domain launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(TARGETS
        villa_calibration
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
