cmake_minimum_required(VERSION 2.8.3)
project(gazebo_video_monitor_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    gazebo_ros
)

find_package(gazebo REQUIRED)

find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    Pose.msg
)

add_service_files(
  FILES
    SetCamera.srv
    StartRecording.srv
    StopRecording.srv
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_monitor_plugin
  CATKIN_DEPENDS
    roscpp
    message_runtime
    gazebo_ros
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gvm_multicamera_bootstrapper_plugin
  src/sensors/gvm_multicamera_sensor.cpp
  src/gvm_multicamera_bootstrapper_plugin.cpp
)
add_dependencies(gvm_multicamera_bootstrapper_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gvm_multicamera_bootstrapper_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

add_library(gazebo_video_monitor_plugin
  src/gazebo_video_monitor_plugin.cpp
)
add_dependencies(gazebo_video_monitor_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_video_monitor_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS 
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_monitor_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install(DIRECTORY test
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
