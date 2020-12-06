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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GVM_MULTICAMERA_BOOTSTRAPPER_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GVM_MULTICAMERA_BOOTSTRAPPER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_video_monitor_plugins/internal/utils.h>
#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>

namespace gazebo {

/**
 * @brief Registers the GvmMulticameraSensor class in the SensorFactory, and
 * adds a gvm_multicamera sensor to a given model.
 * @note Expects the following configuration:
 *   - sensor: configuration of a gvm_multicamera sensor
 *   - sensorReference: reference model configuration for the multicamera
 *     sensor. It contains the name of the model (normally ground_plane) with
 *     which to associate the sensor, and the name of the link to which to
 *     attach the sensor (see \ref parseRefModelConfig)
 */
class GvmMulticameraBootstrapperPlugin : public WorldPlugin {
 public:
  GvmMulticameraBootstrapperPlugin();
  virtual ~GvmMulticameraBootstrapperPlugin() override;
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
  virtual void Init() override;

 private:
  std::string logger_prefix_;
  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GVM_MULTICAMERA_BOOTSTRAPPER_PLUGIN_H
