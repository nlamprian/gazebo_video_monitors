#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_TYPES_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_TYPES_H

#include <memory>
#include <string>

#include <ignition/math/Pose3.hh>

struct ReferenceModelConfig {
  ReferenceModelConfig() : has_pose(false) {}

  void setPose(double x, double y, double z, double roll, double pitch,
               double yaw) {
    pose = ignition::math::Pose3d(x, y, z, roll, pitch, yaw);
    has_pose = true;
  }

  std::string model_name;
  std::string link_name;
  ignition::math::Pose3d pose;
  bool has_pose;
};

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_TYPES_H
