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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_SENSORS_GVM_MULTICAMERA_SENSOR_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_SENSORS_GVM_MULTICAMERA_SENSOR_H

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/util/system.hh>

#include <gazebo_video_monitor_plugins/internal/types.h>
#include <gazebo_video_monitor_interfaces/srv/set_camera.hpp>

namespace gazebo {
namespace sensors {

/**
 * @brief Creates multiple cameras and publishes an event with synchronized
 * images from all of these cameras.
 * @note Provides a ROS interface for dynamically updating the configuration of
 * a camera. If a model is not specified in the request, the model/link
 * configuration defaults to the model/link to which the sensor is attached.
 * @note Expects the usual sensor configuration, but with an arbitrary number of
 * cameras.
 */
class GZ_SENSORS_VISIBLE GvmMulticameraSensor : public Sensor {
 public:
  struct ImageData {
    ImageData(const std::string &name) : name(name) {}

    size_t getStep() const { return static_cast<size_t>(width * depth); }

    const std::string name;
    const unsigned char *data = nullptr;
    int width = 0;
    int height = 0;
    int depth = 0;
    std::string format;
  };

  using ImageDataPtr = std::shared_ptr<ImageData>;

  using NewImagesFn = void(const std::vector<ImageDataPtr> &);

 private:
  struct CameraData {
    CameraData(const std::string &ns, const sdf::ElementPtr &sdf,
               transport::NodePtr &node)
        : id(physics::getUniqueId()),
          ns(ns),
          sdf(sdf),
          is_visualization_inited(false) {
      publisher = node->Advertise<msgs::ImageStamped>(getTopic(), 30);
      data = std::make_shared<ImageData>(getName());
    }

    std::string getName() const { return sdf->Get<std::string>("name"); }

    std::string getScopedName() const { return ns + "::" + getName(); }

    std::string getTopic() const {
      std::string topic_name = "~/" + getScopedName() + "/image";
      common::replaceAll(topic_name, topic_name, "::", "/");
      common::replaceAll(topic_name, topic_name, " ", "_");
      return topic_name;
    }

    ignition::math::Pose3d getWorldPose() const {
#if GAZEBO_MAJOR_VERSION > 10
      return parent_link->WorldPose() * relative_pose;
#else
      return relative_pose * parent_link->WorldPose();
#endif
    }

    void setParent(const physics::LinkPtr &link) {
      parent_name = link->GetScopedName();
      parent_id = link->GetId();
      parent_link = link;
      gzdbg << "GvmMulticameraSensor: Set camera " << getName() << " to parent "
            << parent_name << " with id " << parent_id << "\n";
    }

    void initCamera(const rendering::ScenePtr &scene) {
      // Create camera
      std::string camera_name = getName();
      camera = scene->CreateCamera(getScopedName(), false);
      if (not camera)
        gzthrow("GvmMulticameraSensor: Failed to create camera " + camera_name);
      camera->SetCaptureData(true);

      // Load camera parameters
      camera->Load(sdf);
      if (camera->ImageWidth() == 0 or camera->ImageHeight() == 0)
        gzthrow("GvmMulticameraSensor: Failed to init; camera " + camera_name +
                "has zero size");

      // Initialize camera
      camera->Init();
      camera->CreateRenderTexture(camera->Name() + "_RttTex");

      // Initialize camera pose
      if (not sdf->HasElement("pose"))
        gzthrow("GvmMulticameraSensor: Failed to init; camera " + camera_name +
                " does not have a pose");
      relative_pose = sdf->Get<ignition::math::Pose3d>("pose");
      camera->SetWorldPose(getWorldPose());
      camera->AttachToVisual(parent_id, true);

      // Initialize image message
      msg.mutable_image()->set_width(camera->ImageWidth());
      msg.mutable_image()->set_height(camera->ImageHeight());
      msg.mutable_image()->set_pixel_format(
          common::Image::ConvertPixelFormat(camera->ImageFormat()));
      msg.mutable_image()->set_step(camera->ImageWidth() *
                                    camera->ImageDepth());

      // Initialize image data
      data->width = static_cast<int>(camera->ImageWidth());
      data->height = static_cast<int>(camera->ImageHeight());
      data->depth = static_cast<int>(camera->ImageDepth());
      data->format = camera->ImageFormat();

      gzdbg << "GvmMulticameraSensor: Created camera " << camera->Name()
            << " with id " << id << "\n";
    }

    void destroy(const rendering::ScenePtr &scene) {
      publisher.reset();
      if (scene) scene->RemoveCamera(camera->Name());
      camera.reset();
    }

    void publishImage(const common::Time &current_time) {
      if (not publisher or not publisher->HasConnections()) return;
      msgs::Set(msg.mutable_time(), current_time);
      msg.mutable_image()->set_data(
          camera->ImageData(),
          camera->ImageWidth() * camera->ImageDepth() * camera->ImageHeight());
      publisher->Publish(msg);
    }

    void attachToLink(const physics::LinkPtr &link, bool on_init = false) {
      setParent(link);
      // HACK When attachToLink is called during initialization, SetWorldPose
      // must take the local pose for the camera world pose to be set correctly
      if (on_init)
        camera->SetWorldPose(relative_pose);
      else
        camera->SetWorldPose(getWorldPose());
      camera->AttachToVisual(parent_id, true);
    }

    void attachToLink(const physics::LinkPtr &link,
                      const ignition::math::Pose3d &camera_pose,
                      bool on_init = false) {
      relative_pose = camera_pose;
      attachToLink(link, on_init);
    }

    const uint32_t id;
    const std::string ns;
    const sdf::ElementPtr sdf;
    bool is_visualization_inited;
    std::string parent_name;
    uint32_t parent_id;
    physics::LinkPtr parent_link;
    ignition::math::Pose3d relative_pose;
    transport::PublisherPtr publisher;
    rendering::CameraPtr camera;
    msgs::ImageStamped msg;
    ImageDataPtr data;
  };

 public:
  GvmMulticameraSensor();
  virtual ~GvmMulticameraSensor() override;
  virtual void Load(const std::string &world_name) override;
  virtual void Init() override;
  virtual bool IsActive() const override;
  void initRos(const rclcpp::Node::SharedPtr &node,
               const std::string &set_camera_service_name);
  static sensors::GvmMulticameraSensor *newSensor();
  std::vector<std::string> getCameraNames() const;
  event::ConnectionPtr connectNewImages(
      const std::function<NewImagesFn> &callback);
  rendering::CameraPtr getCamera(const std::string &name);
  bool attachToLink(const std::string &camera_name,
                    const RefModelConfig &model_config, bool on_init = false);
  void setRecording(bool recording);
  bool isRecording() const;

 private:
  void resetCameraVisualization(CameraData &camera);
  void setCameraVisualization(CameraData &camera);
  virtual void Fini() override;
  void Render();
  virtual bool UpdateImpl(const bool force) override;
  bool setCameraServiceCallback(
      const gazebo_video_monitor_interfaces::srv::SetCamera::Request::SharedPtr
          req,
      gazebo_video_monitor_interfaces::srv::SetCamera::Response::SharedPtr res);

  physics::LinkPtr link_;

  transport::PublisherPtr sensor_publisher_;
  transport::PublisherPtr visual_publisher_;
  transport::PublisherPtr request_publisher_;

  event::EventT<NewImagesFn> new_images_;

  std::map<std::string, CameraData> cameras_;
  std::vector<ImageDataPtr> images_;

  bool rendered_;
  bool recording_;

  rclcpp::Service<gazebo_video_monitor_interfaces::srv::SetCamera>::SharedPtr
      set_camera_service_;
};

using GvmMulticameraSensorPtr = std::shared_ptr<GvmMulticameraSensor>;

}  // namespace sensors
}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_SENSORS_GVM_MULTICAMERA_SENSOR_H
