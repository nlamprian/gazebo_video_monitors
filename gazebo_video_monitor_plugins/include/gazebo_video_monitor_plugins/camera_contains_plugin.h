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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_CAMERA_CONTAINS_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_CAMERA_CONTAINS_PLUGIN_H

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_ros/node.hpp>

#include <gazebo_video_monitor_plugins/utils/box_marker_visualizer.h>
#include <gazebo_video_monitor_interfaces/msg/strings.hpp>

namespace gazebo {

/**
 * @brief Publishes a camera select message when a model enters a certain space.
 * @details Tracks n models, and when one of them intersects the space of a box
 * container, it publishes a list of camera names.
 * @note Expects the following configuration:
 *   - updateRate: rate at which the plugin should run
 *   - visualize (optional, defaults to false): flag to enable or disable the
 *     visualization of the container
 *   - trackedModels: list of tracked model names
 *   - cameras: list of camera names
 *   - topic: name of the topic to which to publish the camera names
 *   - pose (6-tuple): pose of the container
 *   - size (3-tuple): size of the container
 */
class CameraContainsPlugin : public WorldPlugin {
 public:
  CameraContainsPlugin();
  virtual ~CameraContainsPlugin() override;
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

 private:
  bool contains(const std::string &name) const;
  void onUpdate(const common::UpdateInfo &info);

  std::string logger_prefix_;

  physics::WorldPtr world_;

  std::vector<std::string> tracked_models_;
  std::vector<std::string> cameras_;
  ignition::math::OrientedBoxd container_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<gazebo_video_monitor_interfaces::msg::Strings>::SharedPtr
      publisher_;
  gazebo_video_monitor_interfaces::msg::Strings msg_;
  bool contains_model_;

  BoxMarkerVisualizerPtr container_visualizer_;

  double update_period_;
  common::Time last_update_time_;
  event::ConnectionPtr update_connection_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_CAMERA_CONTAINS_PLUGIN_H
