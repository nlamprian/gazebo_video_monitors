#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIEW_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIEW_MONITOR_PLUGIN_H

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <ignition/msgs/stringmsg_v.pb.h>
#include <gazebo/common/Plugin.hh>

#include <gazebo_video_monitor_plugins/StartGmcmRecording.h>
#include <gazebo_video_monitor_plugins/StopRecording.h>
#include <gazebo_video_monitor_plugins/Strings.h>
#include <gazebo_video_monitor_plugins/gazebo_monitor_base_plugin.h>
#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>

namespace gazebo {

/**
 * @brief Provides a ROS interface for creating multi-camera videos.
 * @details Records videos with up to 4 camera streams shown in parallel in the 
 * 4 quadrants of the videos. The source streams can be configured from a pool 
 * of n cameras and can be changed dynamically during the recording. Metadata 
 * can be shown in the videos, like real time, sim time, and elapsed real time 
 * since the start of the recording.
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
class GazeboMultiViewMonitorPlugin : public GazeboMonitorBasePlugin {
 public:
  GazeboMultiViewMonitorPlugin();
  virtual ~GazeboMultiViewMonitorPlugin() override;
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;
  virtual void Reset() override;

 private:
  virtual void initRos() override;
  virtual void onNewImages(const ImageDataPtrVector &images) override;
  const ImageDataPtrVector::value_type &getImage(
      const ImageDataPtrVector &images, size_t i) const;
  void cameraSelect(const std::vector<std::string> &names);
  void cameraSelectCallback(
      const boost::shared_ptr<const ignition::msgs::StringMsg_V> &msg);
  void cameraSelectRosCallback(
      const gazebo_video_monitor_plugins::StringsConstPtr &msg);
  std::string stopRecording(bool discard, std::string filename = "");
  bool startRecordingServiceCallback(
      gazebo_video_monitor_plugins::StartGmcmRecordingRequest &req,
      gazebo_video_monitor_plugins::StartGmcmRecordingResponse &res);
  bool stopRecordingServiceCallback(
      gazebo_video_monitor_plugins::StopRecordingRequest &req,
      gazebo_video_monitor_plugins::StopRecordingResponse &res);

  transport::NodePtr node_;
  transport::SubscriberPtr camera_select_subscriber_;
  ros::Subscriber camera_select_ros_subscriber_;

  std::unordered_map<std::string, size_t> camera_name_to_index_map_;
  std::vector<size_t> camera_indices_;
  ImageDataPtrVector::value_type image_null_;

  GazeboVideoRecorderPtr recorder_;
  std::mutex recorder_mutex_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_MULTI_VIEW_MONITOR_PLUGIN_H
