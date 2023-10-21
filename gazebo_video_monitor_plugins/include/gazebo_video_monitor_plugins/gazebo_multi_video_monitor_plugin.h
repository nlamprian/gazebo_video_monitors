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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIDEO_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIDEO_MONITOR_PLUGIN_H

#include <mutex>
#include <unordered_map>

#include <boost/filesystem/operations.hpp>

#include <std_srvs/srv/empty.hpp>

#include <gazebo/common/Plugin.hh>

#include <gazebo_video_monitor_plugins/gazebo_monitor_base_plugin.h>
#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>
#include <gazebo_video_monitor_interfaces/srv/stop_recording.hpp>

namespace gazebo {

/**
 * @brief Provides a ROS interface for creating multiple videos from different
 * cameras, all recording in the same time period.
 * @details Reads the n cameras managed by the parent sensor, records n videos,
 * and saves the videos under a group directory. Metadata can be shown in the
 * videos, like real time, sim time, and elapsed real time since the start of
 * the recording.
 * @note The parent sensor can hold an arbitrary number of cameras. The pose of
 * the cameras in their configuration can be wrt the sensor parent, or any other
 * model in the world. In the latter case, these models need to be specified in
 * the plugin configuration.<br> Expects the following configuration (extends
 * the configuration of \ref GazeboMonitorBasePlugin):
 *   - startRecordingService: name of the service for starting a recording
 *   - stopRecordingService: name of the service for stopping and saving a
 *     recording
 *   - recorder: configuration of the video recorder
 *     (see \ref GazeboVideoRecorder)
 *   - cameraReference ([1-n] times): reference model configuration with a name
 *     attribute pointing to one of the sensor cameras
 *     (see \ref parseRefModelConfig)
 */
class GazeboMultiVideoMonitorPlugin
    : public GazeboMonitorBasePlugin<
          std_srvs::srv::Empty,
          gazebo_video_monitor_interfaces::srv::StopRecording> {
 public:
  GazeboMultiVideoMonitorPlugin();
  virtual ~GazeboMultiVideoMonitorPlugin() override;
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;
  virtual void Reset() override;

 private:
  virtual void initRos() override;
  virtual void onNewImages(const ImageDataPtrVector &images) override;
  bool stopRecording(bool discard,
                     boost::filesystem::path group_directory = "");
  bool startRecordingServiceCallback(
      const std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res);
  bool stopRecordingServiceCallback(
      const gazebo_video_monitor_interfaces::srv::StopRecording::Request::
          SharedPtr req,
      gazebo_video_monitor_interfaces::srv::StopRecording::Response::SharedPtr
          res);

  std::unordered_map<std::string, GazeboVideoRecorderPtr> recorders_;
  std::mutex recorders_mutex_;

  bool add_timestamp_in_filename_;
  std::string file_timestamp_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIDEO_MONITOR_PLUGIN_H
