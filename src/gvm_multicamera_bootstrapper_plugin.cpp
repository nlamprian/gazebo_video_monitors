#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorFactory.hh>

#include <gazebo_video_monitor_plugins/gvm_multicamera_bootstrapper_plugin.h>

namespace gazebo {

GvmMulticameraBootstrapperPlugin::GvmMulticameraBootstrapperPlugin() {
  sensors::SensorFactory::RegisterSensor(
      "gvm_multicamera", reinterpret_cast<sensors::SensorFactoryFn>(
                             &sensors::GvmMulticameraSensor::newSensor));
}

GvmMulticameraBootstrapperPlugin::~GvmMulticameraBootstrapperPlugin() {
  std::string scoped_sensor_name =
      world_->Name() + "::" + world_model_config_.model_name +
      "::" + world_model_config_.link_name +
      "::" + sdf_->GetElement("sensor")->Get<std::string>("name");
  event::Events::removeSensor(scoped_sensor_name);
}

void GvmMulticameraBootstrapperPlugin::Load(physics::WorldPtr _world,
                                            sdf::ElementPtr _sdf) {
  world_ = _world;
  sdf_ = _sdf;

  // Confirm sensor configuration
  if (not _sdf->HasElement("sensor") or
      _sdf->GetElement("sensor")->Get<std::string>("type") != "gvm_multicamera")
    gzthrow(
        "GvmMulticameraBootstrapperPlugin: Failed to find gvm_multicamera "
        "sensor configuration");

  // Load world model configuration
  if (not _sdf->HasElement("worldReference"))
    gzthrow("GvmMulticameraBootstrapperPlugin: Failed to get worldReference");
  world_model_config_ =
      parseReferenceModelConfig(_sdf->GetElement("worldReference"));
}

void GvmMulticameraBootstrapperPlugin::Init() {
  // Get world model
  auto model = world_->ModelByName(world_model_config_.model_name);
  if (not model)
    gzthrow("GvmMulticameraBootstrapperPlugin: Failed to get model " +
            world_model_config_.model_name);

  // Get link
  auto link = model->GetLink(world_model_config_.link_name);
  if (not link)
    gzthrow("GvmMulticameraBootstrapperPlugin: Failed to get link " +
            world_model_config_.link_name + " in model " +
            world_model_config_.model_name);

  // Add a multicamera sensor to the link
  event::Events::createSensor(sdf_->GetElement("sensor"), world_->Name(),
                              link->GetScopedName(), link->GetId());
}

GZ_REGISTER_WORLD_PLUGIN(GvmMulticameraBootstrapperPlugin)

}  // namespace gazebo
