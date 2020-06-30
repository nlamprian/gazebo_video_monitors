#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIDEO_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIDEO_MONITOR_PLUGIN_H

#include <mutex>
#include <unordered_map>

#include <boost/filesystem/operations.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <gazebo/common/Plugin.hh>

#include <gazebo_video_monitor_plugins/StopRecording.h>
#include <gazebo_video_monitor_plugins/gazebo_monitor_base_plugin.h>
#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>

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
class GazeboMultiVideoMonitorPlugin : public GazeboMonitorBasePlugin {
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
  bool startRecordingServiceCallback(std_srvs::EmptyRequest &req,
                                     std_srvs::EmptyResponse &res);
  bool stopRecordingServiceCallback(
      gazebo_video_monitor_plugins::StopRecordingRequest &req,
      gazebo_video_monitor_plugins::StopRecordingResponse &res);

  std::unordered_map<std::string, GazeboVideoRecorderPtr> recorders_;
  std::mutex recorders_mutex_;

  bool add_timestamp_in_filename_;
  std::string file_timestamp_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIDEO_MONITOR_PLUGIN_H
