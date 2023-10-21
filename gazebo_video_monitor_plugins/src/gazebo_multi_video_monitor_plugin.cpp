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

#include <ctime>

#include <gazebo_video_monitor_plugins/gazebo_multi_video_monitor_plugin.h>
#include <gazebo_video_monitor_plugins/internal/utils.h>

namespace gazebo {

GazeboMultiVideoMonitorPlugin::GazeboMultiVideoMonitorPlugin()
    : GazeboMonitorBasePlugin(getClassName<GazeboMultiVideoMonitorPlugin>()),
      add_timestamp_in_filename_(true) {}

GazeboMultiVideoMonitorPlugin::~GazeboMultiVideoMonitorPlugin() {
  for (auto &recorder : recorders_) recorder.second->reset();
}

void GazeboMultiVideoMonitorPlugin::Load(sensors::SensorPtr _sensor,
                                         sdf::ElementPtr _sdf) {
  GazeboMonitorBasePlugin::Load(_sensor, _sdf);

  // Get recorder config
  if (not sdf_->HasElement("recorder"))
    gzthrow(logger_prefix_ + "Failed to get recorder");
  auto sdf_recorder = sdf_->GetElement("recorder");

  // Get add timestamp flag
  if (sdf_recorder->HasElement("addTimestampInFilename"))
    add_timestamp_in_filename_ =
        sdf_recorder->Get<bool>("addTimestampInFilename");

  // Confirm presence of cameras
  auto names = sensor_->getCameraNames();
  if (names.empty())
    RCLCPP_WARN_STREAM(ros_node_->get_logger(),
                       logger_prefix_ << "There are no cameras in the sensor");

  // Initialize recorders
  auto rate = static_cast<unsigned int>(sensor_->UpdateRate());
  auto ns = getClassName<GazeboMultiVideoMonitorPlugin>();
  for (const auto &name : names) {
    recorders_[name] =
        std::make_shared<GazeboVideoRecorder>(ros_node_, rate, ns, name);
    recorders_[name]->load(world_, sdf_recorder);
    // NOTE Only the group directory should have the timestamp
    recorders_[name]->setAddTimestampInFilename(false);
  }
}

void GazeboMultiVideoMonitorPlugin::Reset() {
  std::lock_guard<std::mutex> lock(recorders_mutex_);
  if (sensor_->isRecording()) stopRecording(true);
}

void GazeboMultiVideoMonitorPlugin::initRos() {
  GazeboMonitorBasePlugin::initRos();

  // Get start recording service name
  if (not sdf_->HasElement("startRecordingService"))
    gzthrow(logger_prefix_ + "Failed to get startRecordingService");
  auto start_service_name = sdf_->Get<std::string>("startRecordingService");

  // Get stop recording service name
  if (not sdf_->HasElement("stopRecordingService"))
    gzthrow(logger_prefix_ + "Failed to get stopRecordingService");
  auto stop_service_name = sdf_->Get<std::string>("stopRecordingService");

  // Initialize recording services
  start_recording_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
      start_service_name,
      std::bind(&GazeboMultiVideoMonitorPlugin::startRecordingServiceCallback,
                this, std::placeholders::_1, std::placeholders::_2));
  stop_recording_service_ =
      ros_node_
          ->create_service<gazebo_video_monitor_interfaces::srv::StopRecording>(
              stop_service_name,
              std::bind(
                  &GazeboMultiVideoMonitorPlugin::stopRecordingServiceCallback,
                  this, std::placeholders::_1, std::placeholders::_2));
}

void GazeboMultiVideoMonitorPlugin::onNewImages(
    const ImageDataPtrVector &images) {
  std::unique_lock<std::mutex> lock(recorders_mutex_, std::try_to_lock);
  if (not sensor_->isRecording() or not lock.owns_lock()) return;

  for (const auto &image : images) recorders_[image->name]->addFrame(image);
}

bool GazeboMultiVideoMonitorPlugin::stopRecording(
    bool discard, boost::filesystem::path group_directory) {
  sensor_->setRecording(false);

  bool success = true;
  for (auto &recorder : recorders_) {
    auto filename = group_directory / recorder.first;
    auto path = recorder.second->stop(discard, filename.string());
    if (path.empty() and not discard) success = false;
  }

  return success;
}

bool GazeboMultiVideoMonitorPlugin::startRecordingServiceCallback(
    const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
    std_srvs::srv::Empty::Response::SharedPtr /*res*/) {
  std::lock_guard<std::mutex> lock(recorders_mutex_);

  // Stop active recording
  if (sensor_->isRecording()) {
    RCLCPP_WARN_STREAM(
        ros_node_->get_logger(),
        logger_prefix_ << "There is already an active recording; resetting");
    stopRecording(true);
  }

  // Start recording
  file_timestamp_ = getStringTimestamp(std::time(nullptr));
  auto start_time = world_->RealTime();
  for (auto &recorder : recorders_)
    recorder.second->start(save_path_, file_timestamp_, start_time);

  // Set state
  sensor_->setRecording(true);

  return true;
}

bool GazeboMultiVideoMonitorPlugin::stopRecordingServiceCallback(
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

  // Create group directory
  boost::filesystem::path group_directory = req->filename;
  if (add_timestamp_in_filename_) group_directory += "-" + file_timestamp_;
  boost::filesystem::path group_save_path = save_path_ / group_directory;
  if (not req->discard and not createDirectory(group_save_path, ros_node_)) {
    RCLCPP_WARN_STREAM(ros_node_->get_logger(),
                       logger_prefix_ + "Failed to create directory " +
                           group_save_path.string());
    res->success = false;
    return true;
  }

  std::lock_guard<std::mutex> lock(recorders_mutex_);
  res->success = stopRecording(req->discard, group_directory);
  if (not req->discard) res->path = group_save_path.string();
  return true;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboMultiVideoMonitorPlugin)

}  // namespace gazebo
