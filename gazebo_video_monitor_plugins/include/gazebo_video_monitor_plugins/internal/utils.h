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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H

#include <cxxabi.h>
#include <chrono>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>

#include <rclcpp/node.hpp>

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

static bool createDirectory(const boost::filesystem::path &path,
                            const rclcpp::Node::SharedPtr &node) {
  if (not boost::filesystem::exists(path)) {
    if (not boost::filesystem::create_directory(path)) return false;
    RCLCPP_INFO_STREAM(node->get_logger(),
                       path << " directory has been created");
  }
  return true;
}

namespace internal {

static void parseRefModelConfig(const sdf::ElementPtr &sdf,
                                const RefModelConfigPtr &config) {
  if (sdf->HasElement("model") and config->model_name.empty())
    config->model_name = sdf->Get<std::string>("model");
  if (sdf->HasElement("link"))
    config->link_name = sdf->Get<std::string>("link");
}

}  // namespace internal

/**
 * @brief Parses a basic reference model configuration.
 * @note Expects the following configuration:
 *   - model: name of the model with which to associate the camera
 *   - link (optional, defaults to 'link'): name of the link to which to attach
 *     the camera
 */
static RefModelConfigConstPtr parseRefModelConfig(const sdf::ElementPtr &sdf) {
  RefModelConfigPtr config = std::make_shared<RefModelConfig>();
  internal::parseRefModelConfig(sdf, config);
  return std::move(config);
}

/**
 * @brief Parses a reference model configuration.
 * @note Expects the following configuration:
 *   - modelParam: name of the parameter on the parameter server that holds the
 *     name of the model with which to associate the camera
 *   - model: name of the model with which to associate the camera. It's ignored
 *     if modelParam is given
 *   - link (optional, defaults to 'link'): name of the link to which to attach
 *     the camera
 */
static RefModelConfigConstPtr parseRefModelConfig(
    const sdf::ElementPtr &sdf, const rclcpp::Node::SharedPtr &node) {
  RefModelConfigPtr config = std::make_shared<RefModelConfig>();
  if (sdf->HasElement("modelParam")) {
    auto model_param = sdf->Get<std::string>("modelParam");
    if (not node->get_parameter(model_param, config->model_name))
      RCLCPP_WARN_STREAM(node->get_logger(),
                         "Failed to retrieve " << model_param << " parameter");
  }
  internal::parseRefModelConfig(sdf, config);
  return std::move(config);
}

static std::string getStringTimestamp(std::time_t t) {
  std::tm tm = *std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
  return ss.str();
}

static std::string toString(const std::vector<std::string> &names,
                            const std::string &delimiter = ", ") {
  std::stringstream ss;
  for (size_t i = 0; i < names.size(); ++i) {
    ss << names[i];
    if (i < names.size() - 1) ss << delimiter;
  }
  return ss.str();
}

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_INTERNAL_UTILS_H
