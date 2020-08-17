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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_BOX_MARKER_VISUALIZER_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_BOX_MARKER_VISUALIZER_H

#include <memory>
#include <string>

#include <ignition/msgs/marker.pb.h>
#include <ignition/math.hh>
#include <ignition/transport.hh>

namespace gazebo {

/**
 * @brief Offers a simple interface for spawning box markers.
 * @note Can spawn multiple markers which will be deleted upon destruction.
 */
class BoxMarkerVisualizer {
 public:
  BoxMarkerVisualizer(const std::string &ns,
                      const ignition::math::Color &color =
                          ignition::math::Color(0.0f, 0.4f, 0.8f, 0.2f));
  ~BoxMarkerVisualizer();
  void spawnMarker(uint64_t id, const ignition::math::Vector3d &size,
                   const ignition::math::Pose3d &pose);

 private:
  ignition::transport::Node node_;
  ignition::msgs::Marker msg_;
};

using BoxMarkerVisualizerPtr = std::shared_ptr<BoxMarkerVisualizer>;

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_BOX_MARKER_VISUALIZER_H
