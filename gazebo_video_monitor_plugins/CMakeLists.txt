cmake_minimum_required(VERSION 3.1.0)
project(gazebo_video_monitor_plugins)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
    gazebo_video_monitor_msgs
    roscpp
    std_srvs
)

find_package(gazebo REQUIRED)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

list(APPEND VIDEO_PLUGINS
  gazebo_video_monitor_plugin
  gazebo_multi_video_monitor_plugin
  gazebo_multi_camera_monitor_plugin
  gazebo_multi_view_monitor_plugin
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_recorder
    ${VIDEO_PLUGINS}
    camera_contains_plugin
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_video_monitor_msgs
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

## Bootstrapper plugin ################
add_library(gvm_multicamera_bootstrapper_plugin
  src/sensors/gvm_multicamera_sensor.cpp
  src/gvm_multicamera_bootstrapper_plugin.cpp
)
target_link_libraries(gvm_multicamera_bootstrapper_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

## Video recorder #####################
add_library(gazebo_video_recorder
  src/utils/gazebo_video_recorder.cpp
)
target_link_libraries(gazebo_video_recorder
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES}
)

## Video plugins ######################
foreach(PLUGIN ${VIDEO_PLUGINS})
  add_library(${PLUGIN}
    src/gazebo_monitor_base_plugin.cpp
    src/${PLUGIN}.cpp
  )
  target_link_libraries(${PLUGIN}
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES}
    gazebo_video_recorder
  )
endforeach()

## Utility plugins ####################
add_library(camera_contains_plugin
  src/utils/box_marker_visualizer.cpp
  src/camera_contains_plugin.cpp
)
target_link_libraries(camera_contains_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

install(
  TARGETS 
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_recorder
    ${VIDEO_PLUGINS}
    camera_contains_plugin
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
