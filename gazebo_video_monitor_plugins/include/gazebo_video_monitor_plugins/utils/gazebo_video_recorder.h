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

#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_GAZEBO_VIDEO_RECORDER_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_GAZEBO_VIDEO_RECORDER_H

#include <functional>
#include <unordered_map>

#include <boost/filesystem/operations.hpp>
#include <boost/uuid/random_generator.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/VideoEncoder.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_video_monitor_plugins/internal/utils.h>
#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>

namespace gazebo {

/**
 * @brief Supports creating video recordings from a series of frames.
 * @details Apart from the simple camera stream, it features:
 *   - picture-in-picture mode for a second camera stream on top of the main one
 *   - quadrant mode with 4 camera streams shown in parallel next to each other
 *   - logging of time metadata
 * @note Expects the following configuration:
 *   - width: width of the video recording
 *   - height: height of the video recording
 *   - bitRate: bit rate of the video recording
 *   - logMetadata (optional, defaults to false): flag to indicate whether to
 *     write time metadata on the video recording
 *   - logWallTime (optional, defaults to false): flag to indicate whether to
 *     log wall or real time on the video recording (if logMetadata is enabled)
 *   - addTimestampInFilename (optional, defaults to true): flag to indicate
 *     whether to append the start timestamp in the filename of the video
 *     recording
 */
class GazeboVideoRecorder {
  enum class WindowType : uint8_t {
    BOTTOM_RIGHT_CORNER,
    TOP_LEFT_QUADRANT,
    TOP_RIGHT_QUADRANT,
    BOTTOM_LEFT_QUADRANT,
    BOTTOM_RIGHT_QUADRANT
  };

 public:
  GazeboVideoRecorder(const rclcpp::Node::SharedPtr &node, unsigned int fps,
                      const std::string &ns, const std::string &name = "");
  ~GazeboVideoRecorder();
  void load(const physics::WorldPtr &world, const sdf::ElementPtr &sdf);
  void reset();
  void setAddTimestampInFilename(bool state);
  void start(const boost::filesystem::path &save_path,
             const std::string &file_timestamp, const common::Time &start_time);
  std::string stop(bool discard, std::string filename = "");
  void addFrame(
      const sensors::GvmMulticameraSensor::ImageDataPtr &data_main,
      const sensors::GvmMulticameraSensor::ImageDataPtr &data_window = nullptr);
  void addMultiViewFrame(
      const sensors::GvmMulticameraSensor::ImageDataPtr &data_tl = nullptr,
      const sensors::GvmMulticameraSensor::ImageDataPtr &data_tr = nullptr,
      const sensors::GvmMulticameraSensor::ImageDataPtr &data_bl = nullptr,
      const sensors::GvmMulticameraSensor::ImageDataPtr &data_br = nullptr);

 private:
  std::string getPath(std::string filename, bool add_timestamp = false);
  cv::Mat toCvMat(
      const sensors::GvmMulticameraSensor::ImageDataPtr &data) const;
  void writeWindow(cv::Mat &image_main, cv::Mat &image_window, WindowType type);
  void writeMetadata(cv::Mat &image);

  std::string logger_prefix_;

  physics::WorldPtr world_;

  rclcpp::Node::SharedPtr node_;

  unsigned int fps_;
  unsigned int bit_rate_;
  unsigned int width_;
  unsigned int height_;
  bool log_metadata_;
  bool log_wall_time_;
  bool add_timestamp_in_filename_;

  std::unordered_map<WindowType, std::function<cv::Rect(int, int)>> roi_map_;

  common::VideoEncoder video_encoder_;
  boost::filesystem::path save_path_;
  std::string file_timestamp_;
  common::Time start_time_;

  static boost::uuids::random_generator uuid_generator_;
};

using GazeboVideoRecorderPtr = std::shared_ptr<GazeboVideoRecorder>;

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_GAZEBO_VIDEO_RECORDER_H
