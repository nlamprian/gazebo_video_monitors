cmake_minimum_required(VERSION 3.5)
project(gazebo_video_monitor_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_video_monitor_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(
  dependencies
    gazebo_dev
    gazebo_ros
    gazebo_video_monitor_interfaces
    rclcpp
    std_srvs
    yaml_cpp_vendor
)

link_directories(${gazebo_dev_LIBRARY_DIRS})

list(APPEND VIDEO_PLUGINS
  gazebo_video_monitor_plugin
  gazebo_multi_video_monitor_plugin
  gazebo_multi_camera_monitor_plugin
  gazebo_multi_view_monitor_plugin
)

## Bootstrapper plugin ################
add_library(gvm_multicamera_bootstrapper_plugin SHARED
  src/sensors/gvm_multicamera_sensor.cpp
  src/gvm_multicamera_bootstrapper_plugin.cpp
)
target_include_directories(gvm_multicamera_bootstrapper_plugin PUBLIC
  include ${ignition-math6_INCLUDE_DIRS}
)
target_link_libraries(gvm_multicamera_bootstrapper_plugin ${ignition-math6_LIBRARIES})
ament_target_dependencies(gvm_multicamera_bootstrapper_plugin ${dependencies})
ament_export_libraries(gvm_multicamera_bootstrapper_plugin)

## Video recorder #####################
add_library(gazebo_video_recorder SHARED
  src/utils/gazebo_video_recorder.cpp
)
target_include_directories(gazebo_video_recorder PUBLIC include)
target_link_libraries(gazebo_video_recorder ${OpenCV_LIBRARIES})
ament_target_dependencies(gazebo_video_recorder ${dependencies})
ament_export_libraries(gazebo_video_recorder)

## Video plugins ######################
foreach(PLUGIN ${VIDEO_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cpp)
  target_include_directories(${PLUGIN} PUBLIC
    include ${ignition-msgs8_INCLUDE_DIRS}
  )
  target_link_libraries(${PLUGIN}
    gazebo_video_recorder
    ${ignition-msgs8_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
  ament_target_dependencies(${PLUGIN} ${dependencies})
  ament_export_libraries(${PLUGIN})
endforeach()

## Utility plugins ####################
add_library(camera_contains_plugin SHARED
  src/utils/box_marker_visualizer.cpp
  src/camera_contains_plugin.cpp
)
target_include_directories(camera_contains_plugin PUBLIC
  include
  ${ignition-math6_INCLUDE_DIRS}
  ${ignition-msgs8_INCLUDE_DIRS}
  ${ignition-transport11_INCLUDE_DIRS}
)
target_link_libraries(camera_contains_plugin
  ${ignition-math6_LIBRARIES}
  ${ignition-msgs8_LIBRARIES}
  ${ignition-transport11_LIBRARIES}
)
ament_target_dependencies(camera_contains_plugin ${dependencies})
ament_export_libraries(camera_contains_plugin)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS 
    gvm_multicamera_bootstrapper_plugin
    gazebo_video_recorder
    ${VIDEO_PLUGINS}
    camera_contains_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(
#   DIRECTORY test
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(include)

ament_export_dependencies(${dependencies} OpenCV)

ament_package()
