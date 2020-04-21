#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H

#include <ros/ros.h>

#include <sdf/Element.hh>

#include <gazebo_video_monitor_plugins/internal/types.h>

void parseReferenceModelConfig(const sdf::ElementPtr &sdf,
                               ReferenceModelConfig &config) {
  if (sdf->HasElement("model") and config.model_name.empty())
    config.model_name = sdf->Get<std::string>("model");
  config.link_name =
      sdf->HasElement("link") ? sdf->Get<std::string>("link") : "link";
}

ReferenceModelConfig parseReferenceModelConfig(const sdf::ElementPtr &sdf) {
  ReferenceModelConfig config;
  parseReferenceModelConfig(sdf, config);
  return config;
}

ReferenceModelConfig parseReferenceModelConfig(const sdf::ElementPtr &sdf,
                                               ros::NodeHandle &nh) {
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
