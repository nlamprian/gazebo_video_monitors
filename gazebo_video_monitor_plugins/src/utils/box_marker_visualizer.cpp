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

#include <gazebo_video_monitor_plugins/utils/box_marker_visualizer.h>

using namespace ignition;

namespace gazebo {

BoxMarkerVisualizer::BoxMarkerVisualizer(const std::string &ns,
                                         const math::Color &color) {
  msg_.set_action(msgs::Marker::ADD_MODIFY);
  msg_.set_ns(ns);
  msg_.set_type(msgs::Marker::BOX);
  auto material = msg_.mutable_material();
  msgs::Set(material->mutable_diffuse(), color);
  msgs::Set(material->mutable_emissive(), color);
  msgs::Set(material->mutable_specular(), color);
  msgs::Set(material->mutable_ambient(), color);
}

BoxMarkerVisualizer::~BoxMarkerVisualizer() {
  msg_.set_action(msgs::Marker::DELETE_ALL);

  node_.Request("/marker", msg_);
}

void BoxMarkerVisualizer::spawnMarker(uint64_t id, const math::Vector3d &size,
                                      const math::Pose3d &pose) {
  msg_.set_id(id);
  // msg_.set_parent(parent_scoped_name);
  msgs::Set(msg_.mutable_pose(), pose);
  msgs::Set(msg_.mutable_scale(), size);

  node_.Request("/marker", msg_);
}

}  // namespace gazebo
