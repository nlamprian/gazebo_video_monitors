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

#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <gazebo_video_monitor_plugins/camera_contains_plugin.h>
#include <gazebo_video_monitor_plugins/internal/utils.h>

namespace gazebo {

CameraContainsPlugin::CameraContainsPlugin()
    : logger_prefix_(getClassName<CameraContainsPlugin>()) {}

CameraContainsPlugin::~CameraContainsPlugin() { ros_node_.reset(); }

void CameraContainsPlugin::Load(physics::WorldPtr _world,
                                sdf::ElementPtr _sdf) {
  logger_prefix_ += _sdf->Get<std::string>("name") + ": ";
  world_ = _world;

  // Get the list of track models
  if (not _sdf->HasElement("trackedModels"))
    gzthrow(logger_prefix_ + "Failed to get trackedModels");
  YAML::Node node = YAML::Load(_sdf->Get<std::string>("trackedModels"));
  for (const auto &model : node)
    tracked_models_.push_back(model.as<std::string>());

  // Get the list of cameras
  if (not _sdf->HasElement("cameras"))
    gzthrow(logger_prefix_ + "Failed to get cameras");
  node = YAML::Load(_sdf->Get<std::string>("cameras"));
  for (const auto &camera : node) cameras_.push_back(camera.as<std::string>());

  // Get container pose
  if (not _sdf->HasElement("pose"))
    gzthrow(logger_prefix_ + "Failed to get pose");
  auto pose = _sdf->Get<ignition::math::Pose3d>("pose");

  // Get container size
  if (not _sdf->HasElement("size"))
    gzthrow(logger_prefix_ + "Failed to get size");
  auto size = _sdf->Get<ignition::math::Vector3d>("size");

  // Initialize container
  container_ = ignition::math::OrientedBoxd(size, pose);

  ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Initialize publisher
  if (not _sdf->HasElement("topic"))
    gzthrow(logger_prefix_ + "Failed to get topic");
  publisher_ =
      ros_node_
          ->create_publisher<gazebo_video_monitor_interfaces::msg::Strings>(
              _sdf->Get<std::string>("topic"), 10);
  msg_.names = cameras_;

  // Initialize container visualizer
  bool visualize = false;
  if (_sdf->HasElement("visualize")) visualize = _sdf->Get<bool>("visualize");
  if (visualize) {
    container_visualizer_ =
        std::make_shared<BoxMarkerVisualizer>(_sdf->Get<std::string>("name"));
    container_visualizer_->spawnMarker(4, size, pose);
  }

  // Get update rate
  if (not _sdf->HasElement("updateRate"))
    gzthrow(logger_prefix_ + "Failed to get updateRate");
  double update_rate = _sdf->Get<double>("updateRate");
  update_period_ = update_rate > 0 ? 1 / update_rate : 0;

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CameraContainsPlugin::onUpdate, this, std::placeholders::_1));
}

bool CameraContainsPlugin::contains(const std::string &name) const {
  auto model = world_->ModelByName(name);
  if (not model) return false;
  auto position = model->WorldPose().Pos();
  return container_.Contains(position);
}

void CameraContainsPlugin::onUpdate(const common::UpdateInfo &info) {
  if (info.simTime - last_update_time_ < update_period_) return;
  last_update_time_ = info.simTime;

  bool contains_model = std::any_of(
      tracked_models_.begin(), tracked_models_.end(),
      [&](const auto &name) -> bool { return this->contains(name); });

  if (contains_model) {
    if (not contains_model_) {
      publisher_->publish(msg_);
      contains_model_ = true;
    }
  } else {
    contains_model_ = false;
  }
}

GZ_REGISTER_WORLD_PLUGIN(CameraContainsPlugin)

}  // namespace gazebo
