#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H

#include <mutex>
#include <thread>

#include <boost/filesystem/operations.hpp>

#include <opencv2/opencv.hpp>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/VideoEncoder.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_video_monitor_plugins/StartRecording.h>
#include <gazebo_video_monitor_plugins/StopRecording.h>
#include <gazebo_video_monitor_plugins/internal/utils.h>
#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>

namespace gazebo {

/**
 * @brief Provides a ROS interface for creating videos.
 * @details Creates videos that present two views from the gazebo world: one
 * stationary (world) view, and one (robot) view that is attached to a model.
 * Additional metadata are shown in the video, like real time, sim time, and
 * elapsed real time since the start of the recording.
 * @note Expects the following configuration:
 * setCameraService: name of the service for configuring a camera
 * startRecordingService: name of the service for starting a recording
 * stopRecordingService: name of the service for stopping and saving a recording
 * savePath: path to which to save recordings
 * addTimestampInFilename: flag to indicate whether to append the start
 * timestamp in the filename of a recording
 * logWallTime: flag to indicate whether to log wall or real time
 * numberOfInitialAttachRetries(optional): number of times to try attaching the
 * cameras during initialization. Defaults to 0
 * robotReference/modelParam(optional): name of the parameter on the parameter
 * server that holds the name of the robot model with which to associate the
 * robot camera
 * robotReference/model: name of the robot model with which to associate the
 * robot camera. If the model name is set by modelParam, this name is ignored
 * robotReference/link: name of the link of the robot model to which to attach
 * the robot camera
 */
class GazeboVideoMonitorPlugin : public SensorPlugin {
 public:
  GazeboVideoMonitorPlugin();
  virtual ~GazeboVideoMonitorPlugin() override;
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;
  virtual void Init() override;
  virtual void Reset() override;

 private:
  void initialize();
  void addEntityEventCallback(const std::string &name);
  std::string getRecordingPath(std::string filename, bool add_timestamp = true);
  cv::Mat toCvMat(
      const sensors::GvmMulticameraSensor::ImageDataPtr &data) const;
  void writeWindow(cv::Mat &image_main, cv::Mat &image_window);
  void writeMetadata(cv::Mat &image);
  void onNewImages(
      const std::vector<sensors::GvmMulticameraSensor::ImageDataPtr> &images);
  std::string stopRecording(bool discard, std::string filename = "");
  bool startRecordingServiceCallback(
      gazebo_video_monitor_plugins::StartRecordingRequest &req,
      gazebo_video_monitor_plugins::StartRecordingResponse &res);
  bool stopRecordingServiceCallback(
      gazebo_video_monitor_plugins::StopRecordingRequest &req,
      gazebo_video_monitor_plugins::StopRecordingResponse &res);

  sdf::ElementPtr sdf_;
  physics::WorldPtr world_;

  event::ConnectionPtr add_entity_connection_;
  std::thread deferred_init_thread_;

  const std::string camera_name_world_ = "world_camera";
  const std::string camera_name_robot_ = "robot_camera";
  ReferenceModelConfig robot_model_config_;
  int number_of_initial_attach_retries_;

  sensors::GvmMulticameraSensorPtr sensor_;
  event::ConnectionPtr new_images_connection_;

  ros::NodeHandlePtr nh_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;

  ros::ServiceServer start_recording_service_;
  ros::ServiceServer stop_recording_service_;

  bool log_wall_time_;
  bool world_as_main_view_;
  common::Time start_recording_time_;

  common::VideoEncoder video_encoder_;
  std::mutex video_encoder_mutex_;
  boost::filesystem::path save_path_;
  bool add_timestamp_in_filename_;
  std::string start_encoder_timestamp_;
};

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_GAZEBO_VIDEO_MONITOR_PLUGIN_H
