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

#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/RenderEvents.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/sensors/SensorFactory.hh>

#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>

namespace gazebo {
namespace sensors {

GvmMulticameraSensor::GvmMulticameraSensor()
    : Sensor(sensors::IMAGE), rendered_(false), recording_(false) {
  connections.push_back(event::Events::ConnectRender(
      std::bind(&GvmMulticameraSensor::Render, this)));
}

GvmMulticameraSensor::~GvmMulticameraSensor() {}

void GvmMulticameraSensor::Load(const std::string &world_name) {
  Sensor::Load(world_name);

  // Get parent link
  auto index = ParentName().find("::");
  auto model_name = ParentName().substr(0, index);
  auto link_name = ParentName().substr(index + 2);
  link_ = world->ModelByName(model_name)->GetLink(link_name);

  sensor_publisher_ = node->Advertise<msgs::Sensor>("~/sensor");
  visual_publisher_ = node->Advertise<msgs::Visual>("~/visual");
  request_publisher_ = node->Advertise<msgs::Request>("~/request");
}

void GvmMulticameraSensor::Init() {
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE) {
    gzerr << "GvmMulticameraSensor: Failed to init; rendering is disabled\n";
    return;
  }

  if (world->Name().empty()) {
    gzerr << "GvmMulticameraSensor: Failed to init; world name is empty\n";
    return;
  }

  // Initialize scene
  scene = rendering::get_scene(world->Name());
  if (scene == nullptr) {
    scene = rendering::create_scene(world->Name(), false, true);
    if (scene == nullptr) {
      gzerr << "GvmMulticameraSensor: Failed to init; failed to create scene\n";
      return;
    }
  }

  // Initialize cameras
  auto camera_sdf = sdf->GetElement("camera");
  while (camera_sdf) {
    std::string camera_name = camera_sdf->Get<std::string>("name");
    cameras_.emplace(std::piecewise_construct,
                     std::forward_as_tuple(camera_name),
                     std::forward_as_tuple(Name(), camera_sdf, node));
    auto &camera = cameras_.at(camera_name);
    camera.setParent(link_);
    camera.initCamera(scene);
    setCameraVisualization(camera);
    images_.push_back(camera.data);
    camera_sdf = camera_sdf->GetNextElement("camera");
  }

  // Disable clouds and moon
  scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
                     ~rendering::Scene::GZ_SKYX_CLOUDS &
                     ~rendering::Scene::GZ_SKYX_MOON);

  Sensor::Init();
}

bool GvmMulticameraSensor::IsActive() const {
  return Sensor::IsActive() or
         std::any_of(cameras_.begin(), cameras_.end(),
                     [](const std::pair<std::string, CameraData> &camera) {
                       return camera.second.publisher and
                              camera.second.publisher->HasConnections();
                     }) or
         recording_;
}

void GvmMulticameraSensor::initRos(const rclcpp::Node::SharedPtr &node,
                                   const std::string &set_camera_service_name) {
  set_camera_service_ =
      node->create_service<gazebo_video_monitor_interfaces::srv::SetCamera>(
          set_camera_service_name,
          std::bind(&GvmMulticameraSensor::setCameraServiceCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
}

GvmMulticameraSensor *GvmMulticameraSensor::newSensor() {
  return new GvmMulticameraSensor();
}

std::vector<std::string> GvmMulticameraSensor::getCameraNames() const {
  std::vector<std::string> names;
  for (const auto &image : images_) names.push_back(image->name);
  return names;
}

event::ConnectionPtr GvmMulticameraSensor::connectNewImages(
    const std::function<NewImagesFn> &callback) {
  return new_images_.Connect(callback);
}

rendering::CameraPtr GvmMulticameraSensor::getCamera(const std::string &name) {
  if (cameras_.count(name) == 0) return nullptr;
  return cameras_.at(name).camera;
}

bool GvmMulticameraSensor::attachToLink(const std::string &camera_name,
                                        const RefModelConfig &model_config,
                                        bool on_init) {
  if (cameras_.count(camera_name) == 0) {
    gzerr << "GvmMulticameraSensor: Failed to attach camera " << camera_name
          << " to link; camera does not exist\n";
    return false;
  }

  auto model = world->ModelByName(model_config.model_name);
  if (not model) {
    gzwarn << "GvmMulticameraSensor: Failed to attach camera " << camera_name
           << " to link; model " << model_config.model_name
           << " does not exist\n";
    return false;
  }

  auto link = model->GetLink(model_config.link_name);
  if (not link) {
    gzwarn << "GvmMulticameraSensor: Failed to attach camera " << camera_name
           << " to link; link " << model_config.link_name << " in model "
           << model_config.model_name << " does not exist\n";
    return false;
  }

  auto &camera = cameras_.at(camera_name);

  // resetCameraVisualization(camera);

  if (model_config.has_pose)
    camera.attachToLink(link, model_config.pose, on_init);
  else
    camera.attachToLink(link, on_init);

  setCameraVisualization(camera);

  gzdbg << "GvmMulticameraSensor: Attached camera " << camera_name
        << " to link " << link->GetScopedName() << "\n";
  return true;
}

void GvmMulticameraSensor::setRecording(bool recording) {
  recording_ = recording;
}

bool GvmMulticameraSensor::isRecording() const { return recording_; }

void GvmMulticameraSensor::resetCameraVisualization(
    GvmMulticameraSensor::CameraData &camera) {
  if (not camera.is_visualization_inited) return;

  msgs::Visual vmsg;
  vmsg.set_name(camera.camera->Name());
  vmsg.set_id(camera.id);
  vmsg.set_parent_name(camera.parent_name);
  vmsg.set_delete_me(true);
  visual_publisher_->Publish(vmsg);
  common::Time::Sleep(common::Time(0.2));

  // msgs::Request *rmsg =
  //     msgs::CreateRequest("entity_delete", std::to_string(camera.id));
  // request_publisher_->Publish(*rmsg);
  // common::Time::Sleep(common::Time(0.2));

  gzdbg << "GvmMulticameraSensor: Deleted visualization of camera "
        << camera.getName() << "\n";
}

void GvmMulticameraSensor::setCameraVisualization(
    GvmMulticameraSensor::CameraData &camera) {
  if (not Visualize()) return;

  // Update camera visualization
  // NOTE msgs::Sensor: doesn't create a new camera visualization
  // NOTE msgs::Sensor > msgs::Visual(delete_me) > msgs::Sensor:
  // creates a new camera visualization, but keeps the old one
  // NOTE msgs::Sensor > msgs::Visual(update): moves the camera visualization,
  // but the visualization doesn't track the parent visual (trying to manually
  // attach the visual to the parent has no effect)
  // NOTE msgs::Sensor > msgs::Request(entity_delete) > msgs::Sensor:
  // moves the camera visualization, but not the camera
  // NOTE Local CameraVisual: camera visualization doesn't show up
  if (camera.is_visualization_inited) {
    msgs::Visual msg;
    msg.set_name(camera.camera->Name());
    msg.set_id(camera.id);
    msg.set_parent_name(camera.parent_name);
    msg.set_parent_id(camera.parent_id);
    msgs::Set(msg.mutable_pose(), camera.getWorldPose());
    visual_publisher_->Publish(msg);

    gzdbg << "GvmMulticameraSensor: Updated visualization of camera "
          << camera.getName() << "\n";
    return;
  }

  msgs::Sensor msg;
  msg.set_name(camera.camera->Name());
  msg.set_id(camera.id);
  msg.set_type("camera");
  msg.set_parent(camera.parent_name);
  msg.set_parent_id(camera.parent_id);
  msgs::Set(msg.mutable_pose(), camera.relative_pose);

  msg.set_always_on(IsActive());
  msg.set_update_rate(UpdateRate());
  msg.set_visualize(true);

  msgs::CameraSensor *msg_cam = msg.mutable_camera();
  msg_cam->set_horizontal_fov(camera.camera->HFOV().Radian());
  msg_cam->mutable_image_size()->set_x(camera.camera->ImageWidth());
  msg_cam->mutable_image_size()->set_y(camera.camera->ImageHeight());
  msg_cam->set_image_format(camera.camera->ImageFormat());
  msg_cam->set_near_clip(camera.camera->NearClip());
  msg_cam->set_far_clip(camera.camera->FarClip());

  sensor_publisher_->Publish(msg);
  camera.is_visualization_inited = true;

  // Give some time for the camera visualization to be created
  common::Time::Sleep(common::Time(1.0));
  gzdbg << "GvmMulticameraSensor: Enabled visualization of camera "
        << camera.getName() << "\n";
}

void GvmMulticameraSensor::Fini() {
  for (auto &camera : cameras_) camera.second.destroy(scene);
  Sensor::Fini();
}

void GvmMulticameraSensor::Render() {
  if (cameras_.empty() or not IsActive() or not NeedsUpdate()) return;

  for (auto &camera : cameras_) camera.second.camera->Render();

  rendered_ = true;
  lastMeasurementTime = scene->SimTime();
}

bool GvmMulticameraSensor::UpdateImpl(const bool /*force*/) {
  if (not rendered_) return false;

  common::Time current_time = scene->SimTime();
  for (auto &camera : cameras_) {
    camera.second.camera->PostRender();
    camera.second.publishImage(current_time);
    camera.second.data->data = camera.second.camera->ImageData();
  }

  new_images_(images_);

  rendered_ = false;
  return true;
}

bool GvmMulticameraSensor::setCameraServiceCallback(
    const gazebo_video_monitor_interfaces::srv::SetCamera::Request::SharedPtr
        req,
    gazebo_video_monitor_interfaces::srv::SetCamera::Response::SharedPtr res) {
  if (cameras_.count(req->camera_name) == 0) {
    res->message = "Requested camera does not exist";
    res->success = false;
    return true;
  }

  // Prepare model configuration
  RefModelConfig model_config;
  if (req->model_name.empty()) {
    model_config.model_name = link_->GetModel()->GetName();
    model_config.link_name = link_->GetName();
  } else {
    auto model = world->ModelByName(req->model_name);
    if (not model) {
      res->message = "Requested model does not exist";
      res->success = false;
      return true;
    }

    if (not model->GetLink(req->link_name)) {
      res->message = "Requested link does not exist";
      res->success = false;
      return true;
    }

    model_config.model_name = req->model_name;
    model_config.link_name = req->link_name;
  }
  model_config.setPose(req->pose.x, req->pose.y, req->pose.z, req->pose.roll,
                       req->pose.pitch, req->pose.yaw);

  attachToLink(req->camera_name, model_config);

  res->message = "OK";
  res->success = true;
  return true;
}

GZ_REGISTER_STATIC_SENSOR("gvm_multicamera", GvmMulticameraSensor)

}  // namespace sensors
}  // namespace gazebo
