cmake_minimum_required(VERSION 3.5)
project(gazebo_video_monitor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

set(
  msg_files
    msg/Pose.msg
    msg/Strings.msg
)

set(
  srv_files
    srv/SetCamera.srv
    srv/StartGmcmRecording.srv
    srv/StartGvmRecording.srv
    srv/StopRecording.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    std_srvs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
