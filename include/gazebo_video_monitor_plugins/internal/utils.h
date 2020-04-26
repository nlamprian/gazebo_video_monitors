#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H

#include <cxxabi.h>
#include <string>

#include <ros/ros.h>

#include <sdf/Element.hh>

#include <gazebo_video_monitor_plugins/internal/types.h>

template <typename ClassName>
static std::string getClassName() {
  int status;
  std::string name = std::string(
      abi::__cxa_demangle(typeid(ClassName).name(), nullptr, nullptr, &status));
  auto pos = name.rfind("::");
  if (pos == std::string::npos) return name;
  return name.substr(name.rfind("::") + 2);
}

static void parseReferenceModelConfig(const sdf::ElementPtr &sdf,
                                      ReferenceModelConfig &config) {
  if (sdf->HasElement("model") and config.model_name.empty())
    config.model_name = sdf->Get<std::string>("model");
  config.link_name =
      sdf->HasElement("link") ? sdf->Get<std::string>("link") : "link";
}

static ReferenceModelConfig parseReferenceModelConfig(
    const sdf::ElementPtr &sdf) {
  ReferenceModelConfig config;
  parseReferenceModelConfig(sdf, config);
  return config;
}

static ReferenceModelConfig parseReferenceModelConfig(
    const sdf::ElementPtr &sdf, ros::NodeHandle &nh) {
  ReferenceModelConfig config;
  if (sdf->HasElement("modelParam")) {
    auto model_param = sdf->Get<std::string>("modelParam");
    if (not nh.getParam(model_param, config.model_name))
      ROS_WARN_STREAM("Failed to retrieve " << model_param << " parameter");
  }
  parseReferenceModelConfig(sdf, config);
  return config;
}

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H
