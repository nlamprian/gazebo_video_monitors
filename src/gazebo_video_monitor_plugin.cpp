#include <algorithm>
#include <chrono>
#include <ctime>
#include <ratio>
#include <sstream>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/RenderEvents.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <gazebo_video_monitor_plugins/gazebo_video_monitor_plugin.h>

namespace gazebo {

GazeboVideoMonitorPlugin::GazeboVideoMonitorPlugin()
    : number_of_initial_attach_retries_(0),
      spinner_(1, &callback_queue_),
      log_wall_time_(false),
      world_as_main_view_(false),
      add_timestamp_in_filename_(true) {}

GazeboVideoMonitorPlugin::~GazeboVideoMonitorPlugin() {
  if (deferred_init_thread_.joinable()) deferred_init_thread_.join();
  video_encoder_.Reset();
  callback_queue_.clear();
  callback_queue_.disable();
  nh_->shutdown();
}

void GazeboVideoMonitorPlugin::Load(sensors::SensorPtr _sensor,
                                    sdf::ElementPtr _sdf) {
  sdf_ = _sdf;
  world_ = physics::get_world(_sensor->WorldName());

  // Get multicamera sensor
  sensor_ = std::static_pointer_cast<sensors::GvmMulticameraSensor>(_sensor);
  if (sensor_ == nullptr)
    gzthrow("GazeboVideoMonitorPlugin: Failed to get gvm_multicamera sensor");

  // Confirm cameras
  auto camera_names = sensor_->getCameraNames();
  if (camera_names.size() != 2 or camera_names[0] != camera_name_world_ or
      camera_names[1] != camera_name_robot_)
    gzthrow(
        "GazeboVideoMonitorPlugin: Wrong cameras configuration; please provide "
        "two cameras with the names " +
        camera_name_world_ + " and " + camera_name_robot_);

  // Get save path
  if (not sdf_->HasElement("savePath"))
    gzthrow("GazeboVideoMonitorPlugin: Failed to get savePath");
  std::string save_path = sdf_->Get<std::string>("savePath");
  save_path_ = boost::filesystem::path(save_path);
  if (not boost::filesystem::exists(save_path_)) {
    if (not boost::filesystem::create_directory(save_path_))
      gzthrow("GazeboVideoMonitorPlugin: Failed to create save directory " +
              save_path);
    ROS_INFO_STREAM_NAMED("video_monitor",
                          "GazeboVideoMonitorPlugin: Save directory "
                              << save_path_ << " has been created");
  }

  if (sdf_->HasElement("addTimestampInFilename"))
    add_timestamp_in_filename_ = sdf_->Get<bool>("addTimestampInFilename");

  if (sdf_->HasElement("logWallTime"))
    log_wall_time_ = sdf_->Get<bool>("logWallTime");

  if (not ros::isInitialized()) {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
        "in the gazebo_ros package)");
    return;
  }

  nh_ = boost::make_shared<ros::NodeHandle>();
  nh_->setCallbackQueue(&callback_queue_);
  spinner_.start();

  // Get robot reference model
  if (not sdf_->HasElement("robotReference"))
    gzthrow("GazeboVideoMonitorPlugin: Failed to get robotReference");
  robot_model_config_ =
      parseReferenceModelConfig(sdf_->GetElement("robotReference"), *nh_);
}

void GazeboVideoMonitorPlugin::Init() {
  // Search for the robot model
  // NOTE ModelByName blocks here for some reason, so use Models instead
  auto models = world_->Models();
  bool has_robot_model =
      std::any_of(models.begin(), models.end(),
                  [&name = robot_model_config_.model_name](const auto &model) {
                    return model->GetName() == name;
                  });

  if (has_robot_model) {
    deferred_init_thread_ =
        std::thread(&GazeboVideoMonitorPlugin::initialize, this);
  } else {
    gzdbg << "GazeboVideoMonitorPlugin: Waiting for model "
          << robot_model_config_.model_name << "\n";
    add_entity_connection_ = event::Events::ConnectAddEntity(
        std::bind(&GazeboVideoMonitorPlugin::addEntityEventCallback, this,
                  std::placeholders::_1));
  }
}

void GazeboVideoMonitorPlugin::Reset() {
  std::lock_guard<std::mutex> lock(video_encoder_mutex_);
  if (sensor_->isRecording()) stopRecording(true);
}

void GazeboVideoMonitorPlugin::initialize() {
  // Attach robot camera to robot link
  sensor_->attachToLink(camera_name_robot_, robot_model_config_, true);

  // HACK The camera is not always moved correctly, even though its world pose
  // is reported just fine. It only gets fixed after a few tries
  // TODO Investigate this and handle properly
  if (sdf_->HasElement("numberOfInitialAttachRetries"))
    number_of_initial_attach_retries_ =
        sdf_->Get<int>("numberOfInitialAttachRetries");
  for (int i = 0; i < number_of_initial_attach_retries_; ++i) {
    common::Time::Sleep(common::Time(1.0));
    sensor_->attachToLink(camera_name_robot_, robot_model_config_);
  }

  new_images_connection_ = sensor_->connectNewImages(std::bind(
      &GazeboVideoMonitorPlugin::onNewImages, this, std::placeholders::_1));

  // Initialize set camera service
  if (not sdf_->HasElement("setCameraService"))
    gzthrow("GazeboVideoMonitorPlugin: Failed to get setCameraService");
  sensor_->initRos(nh_, sdf_->Get<std::string>("setCameraService"));

  // Get start recording service name
  if (not sdf_->HasElement("startRecordingService"))
    gzthrow("GazeboVideoMonitorPlugin: Failed to get startRecordingService");
  std::string start_service_name =
      sdf_->Get<std::string>("startRecordingService");

  // Get stop recording service name
  if (not sdf_->HasElement("stopRecordingService"))
    gzthrow("GazeboVideoMonitorPlugin: Failed to get stopRecordingService");
  std::string stop_service_name =
      sdf_->Get<std::string>("stopRecordingService");

  // Initialize recording services
  start_recording_service_ = nh_->advertiseService(
      start_service_name,
      &GazeboVideoMonitorPlugin::startRecordingServiceCallback, this);
  stop_recording_service_ = nh_->advertiseService(
      stop_service_name,
      &GazeboVideoMonitorPlugin::stopRecordingServiceCallback, this);
}

void GazeboVideoMonitorPlugin::addEntityEventCallback(const std::string &name) {
  if (name != robot_model_config_.model_name) return;

  gzdbg << "GazeboVideoMonitorPlugin: Found model "
        << robot_model_config_.model_name << "\n";
  deferred_init_thread_ =
      std::thread(&GazeboVideoMonitorPlugin::initialize, this);
  add_entity_connection_.reset();
}

std::string GazeboVideoMonitorPlugin::getRecordingPath(std::string filename,
                                                       bool add_timestamp) {
  if (add_timestamp) filename += "-" + start_encoder_timestamp_;
  return (save_path_ / filename.append(".mp4")).string();
}

cv::Mat GazeboVideoMonitorPlugin::toCvMat(
    const sensors::GvmMulticameraSensor::ImageDataPtr &data) const {
  cv::Mat image(data->height, data->width, CV_8UC3,
                const_cast<uchar *>(data->data), data->getStep());
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  return image;
}

void GazeboVideoMonitorPlugin::writeWindow(cv::Mat &image_main,
                                           cv::Mat &image_window) {
  // Make the window image 1/3 of the main image and place the
  // window image in the bottom right corner of the main image
  int window_width = static_cast<int>(0.3 * image_main.cols);
  int window_height = static_cast<int>(0.3 * image_main.rows);
  cv::Rect roi(image_main.cols - window_width - 10,
               image_main.rows - window_height - 10, window_width,
               window_height);

  // Create a frame around the window image
  cv::Rect frame_roi(roi.x - 1, roi.y - 1, roi.width + 2, roi.height + 2);
  image_main(frame_roi).setTo(cv::Scalar(0, 0, 0));

  // Write window image on the main image
  cv::Mat window = image_main(roi);
  cv::resize(image_window, image_window, cv::Size(window_width, window_height));
  image_window.copyTo(window);
}

void GazeboVideoMonitorPlugin::writeMetadata(cv::Mat &image) {
  std::string text;
  text = "Sim Time: " + std::to_string(world_->SimTime().Double());
  cv::putText(image, text, {10, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255},
              1, cv::LINE_AA);
  if (log_wall_time_)
    text = "Wall Time: " + std::to_string(common::Time::GetWallTime().Double());
  else
    text = "Real Time: " + std::to_string(world_->RealTime().Double());
  cv::putText(image, text, {10, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255},
              1, cv::LINE_AA);
  common::Time elapsed_time = world_->RealTime() - start_recording_time_;
  text = "Elapsed Time: " + std::to_string(elapsed_time.Double());
  cv::putText(image, text, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255},
              1, cv::LINE_AA);
}

void GazeboVideoMonitorPlugin::onNewImages(
    const std::vector<sensors::GvmMulticameraSensor::ImageDataPtr> &images) {
  cv::Mat image_main, image_window;
  if (world_as_main_view_) {
    image_main = toCvMat(images[0]);
    image_window = toCvMat(images[1]);
  } else {
    image_main = toCvMat(images[1]);
    image_window = toCvMat(images[0]);
  }

  writeWindow(image_main, image_window);
  writeMetadata(image_main);

  // cv::namedWindow("Recording", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Recording", image_main);
  // cv::waitKey(1);

  cv::cvtColor(image_main, image_main, cv::COLOR_RGB2BGR);
  std::unique_lock<std::mutex> lock(video_encoder_mutex_, std::try_to_lock);
  if (sensor_->isRecording() and lock.owns_lock())
    video_encoder_.AddFrame(image_main.data,
                            static_cast<unsigned int>(image_main.cols),
                            static_cast<unsigned int>(image_main.rows));
}

std::string GazeboVideoMonitorPlugin::stopRecording(bool discard,
                                                    std::string filename) {
  sensor_->setRecording(false);

  video_encoder_.Stop();

  std::string path;
  if (discard) {
    ROS_INFO_NAMED("video_monitor",
                   "GazeboVideoMonitorPlugin: Discarding active recording");
  } else {
    auto file = getRecordingPath(filename, add_timestamp_in_filename_);
    if (video_encoder_.SaveToFile(file)) {
      path = file;
      ROS_INFO_STREAM_NAMED(
          "video_monitor",
          "GazeboVideoMonitorPlugin: Recording saved in " << path);
    } else {
      ROS_WARN_NAMED(
          "video_monitor",
          "GazeboVideoMonitorPlugin: Failed to save recording; resetting");
    }
  }
  if (path.empty()) video_encoder_.Reset();

  return path;
}

bool GazeboVideoMonitorPlugin::startRecordingServiceCallback(
    gazebo_video_monitor_plugins::StartRecordingRequest &req,
    gazebo_video_monitor_plugins::StartRecordingResponse & /*res*/) {
  std::lock_guard<std::mutex> lock(video_encoder_mutex_);

  // Get main camera
  world_as_main_view_ = req.world_as_main_view;
  rendering::CameraPtr camera = sensor_->getCamera(
      world_as_main_view_ ? camera_name_world_ : camera_name_robot_);

  // Stop active recording
  if (sensor_->isRecording()) {
    ROS_WARN_NAMED("video_monitor",
                   "GazeboVideoMonitorPlugin: There is already an active "
                   "recording; resetting");
    stopRecording(true);
  }

  // Set start timestamps
  start_recording_time_ = world_->RealTime();
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
  start_encoder_timestamp_ = ss.str();

  // Start recording
  video_encoder_.Start("mp4", getRecordingPath("tmp-recording"),
                       camera->ImageWidth(), camera->ImageHeight(),
                       static_cast<uint>(sensor_->UpdateRate()), 590000);

  // Set state
  sensor_->setRecording(true);

  return true;
}

bool GazeboVideoMonitorPlugin::stopRecordingServiceCallback(
    gazebo_video_monitor_plugins::StopRecordingRequest &req,
    gazebo_video_monitor_plugins::StopRecordingResponse &res) {
  if (not sensor_->isRecording()) {
    ROS_WARN_NAMED(
        "video_monitor",
        "GazeboVideoMonitorPlugin: No active recording; ignoring request");
    res.success = false;
    return true;
  }

  std::lock_guard<std::mutex> lock(video_encoder_mutex_);
  res.path = stopRecording(req.discard, req.filename);
  res.success = not res.path.empty();
  return true;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboVideoMonitorPlugin)

}  // namespace gazebo
