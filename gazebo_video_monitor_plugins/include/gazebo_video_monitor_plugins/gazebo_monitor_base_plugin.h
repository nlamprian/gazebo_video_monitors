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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MONITOR_BASE_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MONITOR_BASE_PLUGIN_H

#include <algorithm>
#include <atomic>
#include <thread>
#include <unordered_map>

#include <boost/filesystem/operations.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_ros/node.hpp>

#include <gazebo_video_monitor_plugins/internal/utils.h>
#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>

namespace gazebo {

/**
 * @brief Abstract base monitor class.
 * @details Hosts common monitor members, and partially initializes the monitor.
 * @note Expects the following configuration:
 *   - setCameraService: name of the service for configuring a camera
 *   - savePath: path to which to save recordings
 *   - numberOfInitialAttachRetries (optional, defaults to 0): number of times
 *     to retry attaching the cameras during initialization
 */
template <typename StartServiceT, typename StopServiceT>
class GazeboMonitorBasePlugin : public SensorPlugin {
 public:
  using ImageDataPtrVector =
      std::vector<sensors::GvmMulticameraSensor::ImageDataPtr>;

  GazeboMonitorBasePlugin(const std::string &name)
      : logger_prefix_(name + ": "), terminating_(false) {}

  virtual ~GazeboMonitorBasePlugin() override {
    terminating_ = true;
    if (deferred_init_thread_.joinable()) deferred_init_thread_.join();
    ros_node_.reset();
  }

  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
    sdf_ = _sdf;
    world_ = physics::get_world(_sensor->WorldName());

    // Get multicamera sensor
    sensor_ = std::static_pointer_cast<sensors::GvmMulticameraSensor>(_sensor);
    if (sensor_ == nullptr)
      gzthrow(logger_prefix_ + "Failed to get gvm_multicamera sensor");

    // Get save path
    if (not sdf_->HasElement("savePath"))
      gzthrow(logger_prefix_ + "Failed to get savePath");
    std::string save_path = sdf_->Get<std::string>("savePath");
    save_path_ = boost::filesystem::path(save_path);
    if (not createDirectory(save_path_, ros_node_))
      gzthrow(logger_prefix_ + "Failed to create directory " + save_path);

    ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get camera reference configs
    if (not _sdf->HasElement("cameraReference")) return;
    auto names = sensor_->getCameraNames();
    auto sdf_cam_ref = _sdf->GetElement("cameraReference");
    while (sdf_cam_ref) {
      std::string name = sdf_cam_ref->Get<std::string>("name");
      if (std::find(names.begin(), names.end(), name) == names.end())
        gzthrow(logger_prefix_ +
                "Invalid camera reference; there is no camera with name " +
                name);
      cam_ref_configs_[name] = parseRefModelConfig(sdf_cam_ref, ros_node_);
      sdf_cam_ref = sdf_cam_ref->GetNextElement("cameraReference");
    }
  }

  virtual void Init() override {
    new_images_connection_ = sensor_->connectNewImages(std::bind(
        &GazeboMonitorBasePlugin::onNewImages, this, std::placeholders::_1));
    deferred_init_thread_ =
        std::thread(&GazeboMonitorBasePlugin::initialize, this);
  }

 protected:
  virtual void initRos() {
    // Initialize set camera service
    if (not sdf_->HasElement("setCameraService"))
      gzthrow(logger_prefix_ + "Failed to get setCameraService");
    sensor_->initRos(ros_node_, sdf_->Get<std::string>("setCameraService"));
  }

  void initialize() {
    // Wait for models
    gzdbg << logger_prefix_ << "Waiting for models before attaching cameras\n";
    common::Time timeout_time = world_->SimTime() + 120;
    while (not terminating_ and world_->SimTime() <= timeout_time) {
      if (std::all_of(cam_ref_configs_.begin(), cam_ref_configs_.end(),
                      [&](const auto &config) {
                        return world_->ModelByName(config.second->model_name) !=
                               nullptr;
                      })) {
        gzdbg << logger_prefix_ << "Models are loaded\n";
        break;
      }
      common::Time::Sleep(1);
    }
    if (world_->SimTime() > timeout_time) {
      if (not terminating_)
        gzerr << logger_prefix_ << "Models were not loaded in time\n";
      return;
    }

    // Attach cameras
    for (const auto &config : cam_ref_configs_)
      sensor_->attachToLink(config.first, *config.second, true);

    // HACK The cameras are not always moved correctly, even though their world
    // pose is reported just fine. It only gets fixed after a few tries
    // TODO Investigate this and handle properly
    int num_retries = 0;
    if (cam_ref_configs_.size() > 0 and
        sdf_->HasElement("numberOfInitialAttachRetries"))
      num_retries = sdf_->Get<int>("numberOfInitialAttachRetries");
    for (int i = 0; i < num_retries; ++i) {
      common::Time::Sleep(common::Time(1.0));
      for (const auto &config : cam_ref_configs_)
        sensor_->attachToLink(config.first, *config.second);
    }

    initRos();

    gzdbg << logger_prefix_ << "Initialized\n";
  }

  RefModelConfigConstPtr getCameraRefConfig(const std::string &name) const {
    return cam_ref_configs_.count(name) ? cam_ref_configs_.at(name) : nullptr;
  }

  virtual void onNewImages(const ImageDataPtrVector &images) = 0;

  const std::string logger_prefix_;

  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;
  sensors::GvmMulticameraSensorPtr sensor_;

  gazebo_ros::Node::SharedPtr ros_node_;

  typename rclcpp::Service<StartServiceT>::SharedPtr start_recording_service_;
  typename rclcpp::Service<StopServiceT>::SharedPtr stop_recording_service_;

  boost::filesystem::path save_path_;

 private:
  std::unordered_map<std::string, RefModelConfigConstPtr> cam_ref_configs_;

  std::thread deferred_init_thread_;
  std::atomic_bool terminating_;

  event::ConnectionPtr new_images_connection_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MONITOR_BASE_PLUGIN_H
