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

#include <gazebo_video_monitor_plugins/gazebo_video_monitor_plugin.h>
#include <gazebo_video_monitor_plugins/internal/utils.h>

namespace gazebo {

GazeboVideoMonitorPlugin::GazeboVideoMonitorPlugin()
    : GazeboMonitorBasePlugin(getClassName<GazeboVideoMonitorPlugin>()),
      camera_names_({"world_camera", "robot_camera"}) {}

GazeboVideoMonitorPlugin::~GazeboVideoMonitorPlugin() { recorder_->reset(); }

void GazeboVideoMonitorPlugin::Load(sensors::SensorPtr _sensor,
                                    sdf::ElementPtr _sdf) {
  GazeboMonitorBasePlugin::Load(_sensor, _sdf);

  // Confirm cameras
  if (sensor_->getCameraNames() != camera_names_)
    gzthrow(logger_prefix_ + "Wrong cameras configuration; please " +
            "provide two cameras with names: " + toString(camera_names_));

  // Confirm camera reference config for robot camera
  if (not getCameraRefConfig(camera_names_[1]))
    gzwarn << logger_prefix_ << camera_names_[1]
           << " camera reference configuration is not provided\n";

  // Initialize recorder
  recorder_ = std::make_shared<GazeboVideoRecorder>(
      ros_node_, static_cast<unsigned int>(sensor_->UpdateRate()),
      getClassName<GazeboVideoMonitorPlugin>());
  if (not sdf_->HasElement("recorder"))
    gzthrow(logger_prefix_ + "Failed to get recorder");
  recorder_->load(world_, sdf_->GetElement("recorder"));
}

void GazeboVideoMonitorPlugin::Reset() {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  if (sensor_->isRecording()) stopRecording(true);
}

void GazeboVideoMonitorPlugin::initRos() {
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
  start_recording_service_ = ros_node_->create_service<
      gazebo_video_monitor_interfaces::srv::StartGvmRecording>(
      start_service_name,
      std::bind(&GazeboVideoMonitorPlugin::startRecordingServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  stop_recording_service_ =
      ros_node_
          ->create_service<gazebo_video_monitor_interfaces::srv::StopRecording>(
              stop_service_name,
              std::bind(&GazeboVideoMonitorPlugin::stopRecordingServiceCallback,
                        this, std::placeholders::_1, std::placeholders::_2));
}

void GazeboVideoMonitorPlugin::onNewImages(const ImageDataPtrVector &images) {
  std::unique_lock<std::mutex> lock(recorder_mutex_, std::try_to_lock);
  if (not sensor_->isRecording() or not lock.owns_lock()) return;

  if (world_as_main_view_)
    recorder_->addFrame(images[0], disable_window_ ? nullptr : images[1]);
  else
    recorder_->addFrame(images[1], disable_window_ ? nullptr : images[0]);
}

std::string GazeboVideoMonitorPlugin::stopRecording(bool discard,
                                                    std::string filename) {
  sensor_->setRecording(false);
  return recorder_->stop(discard, filename);
}

bool GazeboVideoMonitorPlugin::startRecordingServiceCallback(
    const gazebo_video_monitor_interfaces::srv::StartGvmRecording::Request::
        SharedPtr req,
    gazebo_video_monitor_interfaces::srv::StartGvmRecording::Response::
        SharedPtr /*res*/) {
  std::lock_guard<std::mutex> lock(recorder_mutex_);

  // Stop active recording
  if (sensor_->isRecording()) {
    RCLCPP_WARN_STREAM(
        ros_node_->get_logger(),
        logger_prefix_ << "There is already an active recording; resetting");
    stopRecording(true);
  }

  // Start recording
  recorder_->start(save_path_, getStringTimestamp(std::time(nullptr)),
                   world_->RealTime());

  // Set state
  disable_window_ = req->disable_window;
  world_as_main_view_ = req->world_as_main_view;
  sensor_->setRecording(true);

  return true;
}

bool GazeboVideoMonitorPlugin::stopRecordingServiceCallback(
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

GZ_REGISTER_SENSOR_PLUGIN(GazeboVideoMonitorPlugin)

}  // namespace gazebo
