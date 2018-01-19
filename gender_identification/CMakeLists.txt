cmake_minimum_required(VERSION 2.8.3)
project(gender_identification)

find_package(catkin
    REQUIRED
    rospy
    std_msgs
    sensor_msgs
    message_runtime
    message_generation
    )

add_service_files(
    FILES
    GenderIdentification.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)
