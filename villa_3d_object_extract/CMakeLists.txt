cmake_minimum_required(VERSION 2.8.3)
project(villa_3d_object_extract)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  people_msgs
  visualization_msgs  
  message_generation
  roslib
  std_msgs
  tmc_yolo2_ros
  tf
  villa_perception_msgs  
)

add_service_files(
   FILES
   #ShelfDetect.srv
   srv_picture_to_indices.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES villa_3d_object_extract
  CATKIN_DEPENDS 
  	roscpp 
  	message_runtime
    tmc_yolo2_ros
    villa_perception_msgs
  rospy
	sensor_msgs
	std_msgs
  DEPENDS
  	roscpp
	system_lib
  tf
)
#add_definitions(${PCL_DEFINITIONS})
#link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(STD_SRCS
    src/BoundingBoxDObject.cpp src/BoundingBoxPersonDesc.cpp src/Cluster3DBoundingBox.cpp
)

set(STD_SRCS_GRABCUT
    src/BoundingBoxDObject_nicolas.cpp src/BoundingBoxPersonDesc.cpp src/Cluster3DBoundingBox.cpp
)

#add_executable(tmc_yolo_3D_extractor_kinect1 ${STD_SRCS} src/tmc_yolo_3D_extractor_kinect1.cpp)
#target_link_libraries(tmc_yolo_3D_extractor_kinect1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(tmc_yolo_3D_extractor ${STD_SRCS} src/tmc_yolo_3D_extractor.cpp)
#target_link_libraries(tmc_yolo_3D_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(tmc_yolo_3D_extractor_roscallserv ${STD_SRCS_GRABCUT} src/tmc_yolo_3D_extractor_roscallserv.cpp)
#add_dependencies (tmc_yolo_3D_extractor_roscallserv villa_3d_object_extract_generate_messages_cpp )
#target_link_libraries(tmc_yolo_3D_extractor_roscallserv ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#add_executable(tmc_cluster_based_yolo_3D_extractor src/tmc_cluster_based_yolo_3D_extractor.cpp)
#target_link_libraries(tmc_cluster_based_yolo_3D_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(human_tmc_cluster_based_yolo_3D_extractor src/human_tmc_cluster_based_yolo_3D_extractor.cpp)
target_link_libraries(human_tmc_cluster_based_yolo_3D_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(chair_tmc_cluster_based_yolo_3D_extractor src/chair_tmc_cluster_based_yolo_3D_extractor.cpp)
target_link_libraries(chair_tmc_cluster_based_yolo_3D_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(tracking src/Tracking.cpp src/TrackingBoxDObject.cpp src/TrackingBoxPersonDesc.cpp src/Cluster3DBoundingBox.cpp src/Mean3DTrackingBox)
#target_link_libraries(tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES})
