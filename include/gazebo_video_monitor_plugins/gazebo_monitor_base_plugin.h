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

#include <atomic>
#include <thread>
#include <unordered_map>

#include <boost/filesystem/operations.hpp>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

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
class GazeboMonitorBasePlugin : public SensorPlugin {
 public:
  using ImageDataPtrVector =
      std::vector<sensors::GvmMulticameraSensor::ImageDataPtr>;
  GazeboMonitorBasePlugin(const std::string &name);
  virtual ~GazeboMonitorBasePlugin() override;
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;
  virtual void Init() override;

 protected:
  virtual void initRos();
  void initialize();
  RefModelConfigConstPtr getCameraRefConfig(const std::string &name) const;
  virtual void onNewImages(const ImageDataPtrVector &images) = 0;

  const std::string logger_prefix_;

  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;
  sensors::GvmMulticameraSensorPtr sensor_;

  ros::NodeHandlePtr nh_;

  ros::ServiceServer start_recording_service_;
  ros::ServiceServer stop_recording_service_;

  boost::filesystem::path save_path_;

 private:
  std::unordered_map<std::string, RefModelConfigConstPtr> cam_ref_configs_;

  std::thread deferred_init_thread_;
  std::atomic_bool terminating_;

  event::ConnectionPtr new_images_connection_;

  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MONITOR_BASE_PLUGIN_H
