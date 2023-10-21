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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H

#include <mutex>
#include <vector>

#include <gazebo/common/Plugin.hh>

#include <gazebo_video_monitor_plugins/gazebo_monitor_base_plugin.h>
#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>
#include <gazebo_video_monitor_interfaces/srv/start_gvm_recording.hpp>
#include <gazebo_video_monitor_interfaces/srv/stop_recording.hpp>

namespace gazebo {

/**
 * @brief Provides a ROS interface for creating videos that are useful for
 * capturing the operation of a robot.
 * @details Creates videos that present two views of the gazebo world: one
 * stationary (world) view, and one view that is attached to a (robot) model.
 * Additional metadata are shown in the video, like real time, sim time, and
 * elapsed real time since the start of the recording.
 * @note The parent sensor should hold two cameras with the names world_camera
 * and robot_camera. By default, world_camera is attached to the parent link of
 * the sensor. robot_camera is user-configurable.<br>
 * Expects the following configuration
 * (extends the configuration of \ref GazeboMonitorBasePlugin):
 *   - startRecordingService: name of the service for starting a recording
 *   - stopRecordingService: name of the service for stopping and saving a
 *     recording
 *   - recorder: configuration of the video recorder
 *     (see \ref GazeboVideoRecorder)
 *   - cameraReference: reference model configuration with a name attribute
 *     pointing to the robot camera (see \ref parseRefModelConfig)
 */
class GazeboVideoMonitorPlugin
    : public GazeboMonitorBasePlugin<
          gazebo_video_monitor_interfaces::srv::StartGvmRecording,
          gazebo_video_monitor_interfaces::srv::StopRecording> {
 public:
  GazeboVideoMonitorPlugin();
  virtual ~GazeboVideoMonitorPlugin() override;
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;
  virtual void Reset() override;

 private:
  virtual void initRos() override;
  virtual void onNewImages(const ImageDataPtrVector &images) override;
  std::string stopRecording(bool discard, std::string filename = "");
  bool startRecordingServiceCallback(
      const gazebo_video_monitor_interfaces::srv::StartGvmRecording::Request::
          SharedPtr req,
      gazebo_video_monitor_interfaces::srv::StartGvmRecording::Response::
          SharedPtr res);
  bool stopRecordingServiceCallback(
      const gazebo_video_monitor_interfaces::srv::StopRecording::Request::
          SharedPtr req,
      gazebo_video_monitor_interfaces::srv::StopRecording::Response::SharedPtr
          res);

  const std::vector<std::string> camera_names_;

  GazeboVideoRecorderPtr recorder_;
  std::mutex recorder_mutex_;
  bool disable_window_;
  bool world_as_main_view_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H
