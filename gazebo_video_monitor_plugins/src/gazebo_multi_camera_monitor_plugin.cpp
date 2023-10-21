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

#include <algorithm>
#include <ctime>

#include <gazebo_video_monitor_plugins/gazebo_multi_camera_monitor_plugin.h>
#include <gazebo_video_monitor_plugins/internal/utils.h>

namespace gazebo {

GazeboMultiCameraMonitorPlugin::GazeboMultiCameraMonitorPlugin()
    : GazeboMonitorBasePlugin(getClassName<GazeboMultiCameraMonitorPlugin>()) {
  node_ = boost::make_shared<transport::Node>();
  node_->Init();
}

GazeboMultiCameraMonitorPlugin::~GazeboMultiCameraMonitorPlugin() {
  recorder_->reset();
}

void GazeboMultiCameraMonitorPlugin::Load(sensors::SensorPtr _sensor,
                                          sdf::ElementPtr _sdf) {
  GazeboMonitorBasePlugin::Load(_sensor, _sdf);

  // Get recorder config
  if (not sdf_->HasElement("recorder"))
    gzthrow(logger_prefix_ + "Failed to get recorder");
  auto sdf_recorder = sdf_->GetElement("recorder");

  // Confirm presence of cameras
  auto names = sensor_->getCameraNames();
  if (names.empty())
    gzthrow(logger_prefix_ << "There are no cameras in the sensor");

  // Initialize cameras configuration
  for (size_t i = 0; i < names.size(); ++i)
    camera_name_to_index_map_[names[i]] = i;
  camera_indices_.reserve(2);
  camera_indices_.push_back(0);

  // Initialize camera select gazebo subscriber
  camera_select_subscriber_ = node_->Subscribe(
      "~/" + sdf_->Get<std::string>("name") + "/camera_select",
      &GazeboMultiCameraMonitorPlugin::cameraSelectCallback, this);

  // Initialize recorder
  recorder_ = std::make_shared<GazeboVideoRecorder>(
      ros_node_, static_cast<unsigned int>(sensor_->UpdateRate()),
      getClassName<GazeboMultiCameraMonitorPlugin>());
  if (not sdf_->HasElement("recorder"))
    gzthrow(logger_prefix_ + "Failed to get recorder");
  recorder_->load(world_, sdf_->GetElement("recorder"));
}

void GazeboMultiCameraMonitorPlugin::Reset() {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  if (sensor_->isRecording()) stopRecording(true);
}

void GazeboMultiCameraMonitorPlugin::initRos() {
  GazeboMonitorBasePlugin::initRos();

  // Get start recording service name
  if (not sdf_->HasElement("startRecordingService"))
    gzthrow(logger_prefix_ + "Failed to get startRecordingService");
  auto start_service_name = sdf_->Get<std::string>("startRecordingService");

  // Get stop recording service name
  if (not sdf_->HasElement("stopRecordingService"))
    gzthrow(logger_prefix_ + "Failed to get stopRecordingService");
  auto stop_service_name = sdf_->Get<std::string>("stopRecordingService");

  // Get camera select topic name
  if (not sdf_->HasElement("cameraSelectTopic"))
    gzthrow(logger_prefix_ + "Failed to get cameraSelectTopic");
  auto camera_select_topic_name = sdf_->Get<std::string>("cameraSelectTopic");

  // Initialize recording services
  start_recording_service_ = ros_node_->create_service<
      gazebo_video_monitor_interfaces::srv::StartGmcmRecording>(
      start_service_name,
      std::bind(&GazeboMultiCameraMonitorPlugin::startRecordingServiceCallback,
                this, std::placeholders::_1, std::placeholders::_2));
  stop_recording_service_ =
      ros_node_
          ->create_service<gazebo_video_monitor_interfaces::srv::StopRecording>(
              stop_service_name,
              std::bind(
                  &GazeboMultiCameraMonitorPlugin::stopRecordingServiceCallback,
                  this, std::placeholders::_1, std::placeholders::_2));

  // Initialize camera select subscriber
  camera_select_ros_subscriber_ =
      ros_node_
          ->create_subscription<gazebo_video_monitor_interfaces::msg::Strings>(
              camera_select_topic_name, 10,
              std::bind(
                  &GazeboMultiCameraMonitorPlugin::cameraSelectRosCallback,
                  this, std::placeholders::_1));
}

void GazeboMultiCameraMonitorPlugin::onNewImages(
    const ImageDataPtrVector &images) {
  std::unique_lock<std::mutex> lock(recorder_mutex_, std::try_to_lock);
  if (not sensor_->isRecording() or not lock.owns_lock()) return;

  if (camera_indices_.size() == 1)
    recorder_->addFrame(images[camera_indices_[0]]);
  else
    recorder_->addFrame(images[camera_indices_[0]], images[camera_indices_[1]]);
}

void GazeboMultiCameraMonitorPlugin::cameraSelect(
    const std::vector<std::string> &names) {
  if (std::any_of(names.begin(), names.end(), [&](const auto &name) {
        return camera_name_to_index_map_.count(name) == 0;
      })) {
    RCLCPP_WARN_STREAM(ros_node_->get_logger(),
                       logger_prefix_
                           << "Received message with invalid camera names; "
                              "ignoring message");
    return;
  }

  size_t num_names = names.size();
  if (num_names > 2) {
    RCLCPP_WARN_STREAM(ros_node_->get_logger(),
                       logger_prefix_
                           << "Received message with more than 2 camera names; "
                              "ignoring the extra cameras");
    num_names = 2;
  } else if (num_names == 0) {
    RCLCPP_WARN_STREAM(ros_node_->get_logger(),
                       logger_prefix_
                           << "Received message with 0 camera names; "
                           << "ignoring message");
    return;
  }

  camera_indices_.clear();
  for (size_t i = 0; i < num_names; ++i)
    camera_indices_.push_back(camera_name_to_index_map_[names[i]]);
}

void GazeboMultiCameraMonitorPlugin::cameraSelectCallback(
    const boost::shared_ptr<const ignition::msgs::StringMsg_V> &msg) {
  std::vector<std::string> names;
  for (int i = 0; i < msg->data_size(); ++i) names.push_back(msg->data(i));
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  cameraSelect(names);
}

void GazeboMultiCameraMonitorPlugin::cameraSelectRosCallback(
    const gazebo_video_monitor_interfaces::msg::Strings::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  cameraSelect(msg->names);
}

std::string GazeboMultiCameraMonitorPlugin::stopRecording(
    bool discard, std::string filename) {
  sensor_->setRecording(false);
  return recorder_->stop(discard, filename);
}

bool GazeboMultiCameraMonitorPlugin::startRecordingServiceCallback(
    const gazebo_video_monitor_interfaces::srv::StartGmcmRecording::Request::
        SharedPtr req,
    gazebo_video_monitor_interfaces::srv::StartGmcmRecording::Response::
        SharedPtr /*res*/) {
  std::lock_guard<std::mutex> lock(recorder_mutex_);

  // Stop active recording
  if (sensor_->isRecording()) {
    RCLCPP_WARN_STREAM(
        ros_node_->get_logger(),
        logger_prefix_ << "There is already an active recording; resetting");
    stopRecording(true);
  }

  // Select cameras
  if (req->cameras.names.size() > 0) cameraSelect(req->cameras.names);

  // Start recording
  recorder_->start(save_path_, getStringTimestamp(std::time(nullptr)),
                   world_->RealTime());

  // Set state
  sensor_->setRecording(true);

  return true;
}

bool GazeboMultiCameraMonitorPlugin::stopRecordingServiceCallback(
    const gazebo_video_monitor_interfaces::srv::StopRecording::Request::
        SharedPtr req,
    gazebo_video_monitor_interfaces::srv::StopRecording::Response::SharedPtr
        res) {
  if (not sensor_->isRecording()) {
    RCLCPP_WARN_STREAM(
        ros_node_->get_logger(),
        logger_prefix_ << "No active recording; ignoring request");
    res->success = false;
    return true;
  }

  std::lock_guard<std::mutex> lock(recorder_mutex_);
  res->path = stopRecording(req->discard, req->filename);
  res->success = not res->path.empty() or req->discard;
  return true;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboMultiCameraMonitorPlugin)

}  // namespace gazebo
