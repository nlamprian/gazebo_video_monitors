#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H

#include <mutex>
#include <vector>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>

#include <gazebo_video_monitor_plugins/StartGvmRecording.h>
#include <gazebo_video_monitor_plugins/StopRecording.h>
#include <gazebo_video_monitor_plugins/gazebo_monitor_base_plugin.h>
#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>

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
class GazeboVideoMonitorPlugin : public GazeboMonitorBasePlugin {
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
      gazebo_video_monitor_plugins::StartGvmRecordingRequest &req,
      gazebo_video_monitor_plugins::StartGvmRecordingResponse &res);
  bool stopRecordingServiceCallback(
      gazebo_video_monitor_plugins::StopRecordingRequest &req,
      gazebo_video_monitor_plugins::StopRecordingResponse &res);

  const std::vector<std::string> camera_names_;

  GazeboVideoRecorderPtr recorder_;
  std::mutex recorder_mutex_;
  bool disable_window_;
  bool world_as_main_view_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H
