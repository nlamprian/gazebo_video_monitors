cmake_minimum_required(VERSION 2.8.3)
project(gazebo_video_monitor_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    gazebo_ros
    roscpp
    std_srvs
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
  INCLUDE_DIRS
    include
  LIBRARIES
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_recorder
    gazebo_video_monitor_plugin
    gazebo_multi_video_monitor_plugin
  CATKIN_DEPENDS
    gazebo_ros
    message_runtime
    roscpp
    std_srvs
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

add_library(gazebo_video_recorder
  src/utils/gazebo_video_recorder.cpp
)
target_link_libraries(gazebo_video_recorder
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_library(gazebo_video_monitor_plugin
  src/gazebo_monitor_base_plugin.cpp
  src/gazebo_video_monitor_plugin.cpp
)
add_dependencies(gazebo_video_monitor_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_video_monitor_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES}
  gazebo_video_recorder
)

add_library(gazebo_multi_video_monitor_plugin
  src/gazebo_monitor_base_plugin.cpp
  src/gazebo_multi_video_monitor_plugin.cpp
)
add_dependencies(gazebo_multi_video_monitor_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_multi_video_monitor_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES}
  gazebo_video_recorder
)

#############
## Install ##
#############

install(
  TARGETS 
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_recorder
    gazebo_video_monitor_plugin
    gazebo_multi_video_monitor_plugin
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
