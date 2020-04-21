#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GVM_MULTICAMERA_BOOTSTRAPPER_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GVM_MULTICAMERA_BOOTSTRAPPER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>
#include <gazebo_video_monitor_plugins/internal/utils.h>

namespace gazebo {

/**
 * @brief Registers the GvmMulticameraSensor class in the SensorFactory, and
 * adds a gvm_multicamera sensor to a given model.
 * @note Expects the following configuration:
 * sensor: configuration of a sensor of type gvm_multicamera
 * worldReference/model: name of the model with which to associate the
 * multicamera sensor (normally ground_plane)
 * worldReference/link: name of the link of the world model to which to add
 * the multicamera sensor
 */
class GvmMulticameraBootstrapperPlugin : public WorldPlugin {
 public:
  GvmMulticameraBootstrapperPlugin();
  virtual ~GvmMulticameraBootstrapperPlugin() override;
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
  virtual void Init() override;

 private:
  physics::WorldPtr world_;
  sdf::ElementPtr sdf_;
  ReferenceModelConfig world_model_config_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GVM_MULTICAMERA_BOOTSTRAPPER_PLUGIN_H
