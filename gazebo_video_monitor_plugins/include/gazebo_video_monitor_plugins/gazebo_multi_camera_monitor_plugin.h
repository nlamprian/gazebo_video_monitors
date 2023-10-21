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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_CAMERA_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_CAMERA_MONITOR_PLUGIN_H

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ignition/msgs/stringmsg_v.pb.h>
#include <gazebo/common/Plugin.hh>

#include <gazebo_video_monitor_plugins/gazebo_monitor_base_plugin.h>
#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>
#include <gazebo_video_monitor_interfaces/msg/strings.hpp>
#include <gazebo_video_monitor_interfaces/srv/start_gmcm_recording.hpp>
#include <gazebo_video_monitor_interfaces/srv/stop_recording.hpp>

namespace gazebo {

/**
 * @brief Provides a ROS interface for creating multi-camera videos.
 * @details Records videos in which the stream can be configured from a pool of
 * n cameras. The source camera can be changed dynamically during the recording.
 * A second camera can be selected to be shown in picture-in-picture mode.
 * Metadata can be shown in the videos, like real time, sim time, and elapsed
 * real time since the start of the recording.
 * @note The parent sensor can hold an arbitrary number of cameras. The pose of
 * the cameras in their configuration can be wrt the sensor parent, or any other
 * model in the world. In the latter case, these models need to be specified in
 * the plugin configuration.<br> Expects the following configuration (extends
 * the configuration of \ref GazeboMonitorBasePlugin):
 *   - startRecordingService: name of the service for starting a recording
 *   - stopRecordingService: name of the service for stopping and saving a
 *     recording
 *   - cameraSelectTopic: name of the topic for selecting the cameras. One or
 *     two camera names are allowed in the messages.
 *   - recorder: configuration of the video recorder
 *     (see \ref GazeboVideoRecorder)
 *   - cameraReference ([1-n] times): reference model configuration with a name
 *     attribute pointing to one of the sensor cameras
 *     (see \ref parseRefModelConfig)
 */
class GazeboMultiCameraMonitorPlugin
    : public GazeboMonitorBasePlugin<
          gazebo_video_monitor_interfaces::srv::StartGmcmRecording,
          gazebo_video_monitor_interfaces::srv::StopRecording> {
 public:
  GazeboMultiCameraMonitorPlugin();
  virtual ~GazeboMultiCameraMonitorPlugin() override;
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;
  virtual void Reset() override;

 private:
  virtual void initRos() override;
  virtual void onNewImages(const ImageDataPtrVector &images) override;
  void cameraSelect(const std::vector<std::string> &names);
  void cameraSelectCallback(
      const boost::shared_ptr<const ignition::msgs::StringMsg_V> &msg);
  void cameraSelectRosCallback(
      const gazebo_video_monitor_interfaces::msg::Strings::SharedPtr msg);
  std::string stopRecording(bool discard, std::string filename = "");
  bool startRecordingServiceCallback(
      const gazebo_video_monitor_interfaces::srv::StartGmcmRecording::Request::
          SharedPtr req,
      gazebo_video_monitor_interfaces::srv::StartGmcmRecording::Response::
          SharedPtr res);
  bool stopRecordingServiceCallback(
      const gazebo_video_monitor_interfaces::srv::StopRecording::Request::
          SharedPtr req,
      gazebo_video_monitor_interfaces::srv::StopRecording::Response::SharedPtr
          res);

  transport::NodePtr node_;
  transport::SubscriberPtr camera_select_subscriber_;
  rclcpp::Subscription<gazebo_video_monitor_interfaces::msg::Strings>::SharedPtr
      camera_select_ros_subscriber_;

  std::unordered_map<std::string, size_t> camera_name_to_index_map_;
  std::vector<size_t> camera_indices_;

  GazeboVideoRecorderPtr recorder_;
  std::mutex recorder_mutex_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_CAMERA_MONITOR_PLUGIN_H
