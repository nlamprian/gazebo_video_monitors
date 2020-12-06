cmake_minimum_required(VERSION 3.0.2)
project(gazebo_video_monitor_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    Pose.msg
    Strings.msg
)

add_service_files(
  FILES
    SetCamera.srv
    StartGmcmRecording.srv
    StartGvmRecording.srv
    StopRecording.srv
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_srvs
)
