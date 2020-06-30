#ifndef GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_GAZEBO_VIDEO_RECORDER_H
#define GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_GAZEBO_VIDEO_RECORDER_H

#include <boost/filesystem/operations.hpp>
#include <boost/uuid/random_generator.hpp>

#include <opencv2/opencv.hpp>

#include <gazebo/common/VideoEncoder.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_video_monitor_plugins/internal/utils.h>
#include <gazebo_video_monitor_plugins/sensors/gvm_multicamera_sensor.h>

namespace gazebo {

/**
 * @brief Supports creating video recordings from a series of frames.
 * @details Features picture-in-picture for a second view and logging of time
 * metadata in the video recording.
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
 public:
  GazeboVideoRecorder(unsigned int fps, const std::string &ns,
                      const std::string &name = "");
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

 private:
  std::string getPath(std::string filename, bool add_timestamp = false);
  cv::Mat toCvMat(
      const sensors::GvmMulticameraSensor::ImageDataPtr &data) const;
  void writeWindow(cv::Mat &image_main, cv::Mat &image_window);
  void writeMetadata(cv::Mat &image);

  std::string logger_prefix_;

  physics::WorldPtr world_;

  unsigned int fps_;
  unsigned int bit_rate_;
  unsigned int width_;
  unsigned int height_;
  bool log_metadata_;
  bool log_wall_time_;
  bool add_timestamp_in_filename_;

  common::VideoEncoder video_encoder_;
  boost::filesystem::path save_path_;
  std::string file_timestamp_;
  common::Time start_time_;

  static boost::uuids::random_generator uuid_generator_;
};

using GazeboVideoRecorderPtr = std::shared_ptr<GazeboVideoRecorder>;

}  // namespace gazebo

#endif  // GAZEBO_VIDEO_MONITOR_PLUGINS_UTILS_GAZEBO_VIDEO_RECORDER_H
