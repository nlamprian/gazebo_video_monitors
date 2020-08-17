/**
 * Copyright (C) 2020  Nikolaos Lamprianidis
 *
 * Gazebo Video Monitor Plugins is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as  published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Gazebo Video Monitor Plugins is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_TYPES_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_TYPES_H

#include <memory>
#include <string>

#include <ignition/math/Pose3.hh>

struct RefModelConfig {
  RefModelConfig() : link_name("link"), has_pose(false) {}

  void setPose(double x, double y, double z, double roll, double pitch,
               double yaw) {
    pose = ignition::math::Pose3d(x, y, z, roll, pitch, yaw);
    has_pose = true;
  }

  std::string camera_name;
  std::string model_name;
  std::string link_name;
  ignition::math::Pose3d pose;
  bool has_pose;
};

using RefModelConfigPtr = std::shared_ptr<RefModelConfig>;
using RefModelConfigConstPtr = std::shared_ptr<const RefModelConfig>;

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_TYPES_H
