cmake_minimum_required(VERSION 2.8.3)
project(tmc_yolo2_ros)

if(DEFINED ENV{GPU_ENABLED})
  set(USE_GPU true)
else()
  set(USE_GPU false)
endif()

if(DEFINED ENV{CUDNN_ENABLED})
  set(USE_CUDNN false)
else()
  set(USE_CUDNN false)
endif()


if(USE_GPU)
  add_definitions(-DGPU)
  if(USE_CUDNN)
    add_definitions(-DCUDNN)
  endif()
  find_package(CUDA REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tmc_darknet
)

catkin_python_setup()

add_message_files(
  FILES
  Detection.msg
  Detections.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  message_runtime
  sensor_msgs
  std_msgs
  tmc_darknet
)

include_directories(
  SYSTEM ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

if(USE_GPU)
  include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
endif()

add_executable(yolo2_node
  src/detector.cpp
  src/yolo2_node.cpp
  src/yolo2_node_main.cpp
  )
add_dependencies(yolo2_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(yolo2_node ${catkin_LIBRARIES})
install(TARGETS yolo2_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
# Single Image
add_executable(single_image
        src/single_image.cpp
        src/detector.cpp
        )
add_dependencies(single_image ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(single_image ${catkin_LIBRARIES})
install(TARGETS single_image
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Training
add_executable(training src/training.c)
target_link_libraries(training
  ${catkin_LIBRARIES}
  pthread
)
install(TARGETS training
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
  scripts/yolo2_image_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(yolo2_node_stub
    src/detector.cpp
    src/yolo2_node.cpp
    test/yolo2_node_stub.cpp
    )
  add_dependencies(yolo2_node_stub
    ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(yolo2_node_stub ${catkin_LIBRARIES})

  add_rostest_gtest(yolo2_node_test
    test/yolo2_node-test.test
    test/yolo2_node-test.cpp
    )
  target_link_libraries(yolo2_node_test ${catkin_LIBRARIES})
endif()
