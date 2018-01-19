cmake_minimum_required(VERSION 2.8.3)
project(face_recognition_ros)

find_package(catkin
    REQUIRED
    COMPONENTS
    rospy
    std_msgs
    message_runtime
    message_generation
    keyboard
    control_msgs
    actionlib_msgs
    )

 add_service_files(
   FILES
   face_recognition_srv.srv
   take_photo.srv
 )

add_action_files(
  DIRECTORY action
  FILES FaceRecog.action
)

  generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )

 catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

