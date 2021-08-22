cmake_minimum_required(VERSION 3.1.0)
project(gazebo_video_monitor_utils)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    scripts/wait_for_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
