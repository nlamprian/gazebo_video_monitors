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
