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

#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorFactory.hh>

#include <gazebo_video_monitor_plugins/gvm_multicamera_bootstrapper_plugin.h>

namespace gazebo {

GvmMulticameraBootstrapperPlugin::GvmMulticameraBootstrapperPlugin()
    : logger_prefix_(getClassName<GvmMulticameraBootstrapperPlugin>() + ": "),
      spinner_(1, &callback_queue_),
      inited_(false) {
  sensors::SensorFactory::RegisterSensor(
      "gvm_multicamera", reinterpret_cast<sensors::SensorFactoryFn>(
                             &sensors::GvmMulticameraSensor::newSensor));
}

GvmMulticameraBootstrapperPlugin::~GvmMulticameraBootstrapperPlugin() {
  callback_queue_.clear();
  callback_queue_.disable();
  nh_->shutdown();

  if (not link_) return;
  std::string scoped_sensor_name =
      world_->Name() + "::" + link_->GetScopedName() +
      "::" + sdf_->GetElement("sensor")->Get<std::string>("name");
  event::Events::removeSensor(scoped_sensor_name);
}

void GvmMulticameraBootstrapperPlugin::Load(physics::WorldPtr _world,
                                            sdf::ElementPtr _sdf) {
  sdf_ = _sdf;
  world_ = _world;

  // Confirm sensor configuration
  if (not _sdf->HasElement("sensor") or
      _sdf->GetElement("sensor")->Get<std::string>("type") != "gvm_multicamera")
    gzthrow(logger_prefix_ +
            "Failed to find gvm_multicamera sensor configuration");

  // Load sensor model configuration
  if (not _sdf->HasElement("sensorReference"))
    gzthrow(logger_prefix_ + "Failed to get sensorReference");
  auto model_config = parseRefModelConfig(_sdf->GetElement("sensorReference"));

  // Get sensor model
  auto model = world_->ModelByName(model_config->model_name);
  if (not model)
    gzthrow(logger_prefix_ + "Failed to get model " + model_config->model_name);

  // Get sensor link
  link_ = model->GetLink(model_config->link_name);
  if (not link_)
    gzthrow(logger_prefix_ + "Failed to get link " + model_config->link_name +
            " in model " + model_config->model_name);

  nh_ = boost::make_shared<ros::NodeHandle>();
  nh_->setCallbackQueue(&callback_queue_);
  spinner_.start();

  if (_sdf->HasElement("initService"))
    init_service_server_ = nh_->advertiseService(
        _sdf->Get<std::string>("initService"),
        &GvmMulticameraBootstrapperPlugin::initServiceCallback, this);
}

void GvmMulticameraBootstrapperPlugin::Init() {
  // If the subscriber is enabled, it will be responsible for initialization
  if (not init_service_server_.getService().empty()) return;
  event::Events::createSensor(sdf_->GetElement("sensor"), world_->Name(),
                              link_->GetScopedName(), link_->GetId());
}

bool GvmMulticameraBootstrapperPlugin::initServiceCallback(
    std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res) {
  if (not inited_) {
    event::Events::createSensor(sdf_->GetElement("sensor"), world_->Name(),
                                link_->GetScopedName(), link_->GetId());
    inited_ = true;
  }
  return true;
}

GZ_REGISTER_WORLD_PLUGIN(GvmMulticameraBootstrapperPlugin)

}  // namespace gazebo
