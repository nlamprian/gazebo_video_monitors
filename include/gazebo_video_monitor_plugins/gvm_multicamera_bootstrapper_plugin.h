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
