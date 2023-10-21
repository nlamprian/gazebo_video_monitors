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

#include <boost/uuid/uuid_io.hpp>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <gazebo_video_monitor_plugins/utils/gazebo_video_recorder.h>

namespace gazebo {

GazeboVideoRecorder::GazeboVideoRecorder(const rclcpp::Node::SharedPtr &node,
                                         unsigned int fps,
                                         const std::string &ns,
                                         const std::string &name)
    : logger_prefix_(ns + "::" + getClassName<GazeboVideoRecorder>()),
      node_(node),
      fps_(fps),
      log_metadata_(false),
      log_wall_time_(false),
      add_timestamp_in_filename_(true) {
  logger_prefix_ += name.empty() ? (": ") : ("[" + name + "]: ");

  // Initialize regions of interest
  roi_map_[WindowType::BOTTOM_RIGHT_CORNER] = [](int width, int height) {
    int window_width = static_cast<int>(0.3 * width);
    int window_height = static_cast<int>(0.3 * height);
    return cv::Rect(width - window_width - 10, height - window_height - 10,
                    window_width, window_height);
  };
  roi_map_[WindowType::TOP_LEFT_QUADRANT] = [](int width, int height) {
    int window_width = static_cast<int>(0.5 * width);
    int window_height = static_cast<int>(0.5 * height);
    return cv::Rect(0, 0, window_width, window_height);
  };
  roi_map_[WindowType::TOP_RIGHT_QUADRANT] = [](int width, int height) {
    int window_width = static_cast<int>(0.5 * width);
    int window_height = static_cast<int>(0.5 * height);
    return cv::Rect(width / 2, 0, window_width, window_height);
  };
  roi_map_[WindowType::BOTTOM_LEFT_QUADRANT] = [](int width, int height) {
    int window_width = static_cast<int>(0.5 * width);
    int window_height = static_cast<int>(0.5 * height);
    return cv::Rect(0, height / 2, window_width, window_height);
  };
  roi_map_[WindowType::BOTTOM_RIGHT_QUADRANT] = [](int width, int height) {
    int window_width = static_cast<int>(0.5 * width);
    int window_height = static_cast<int>(0.5 * height);
    return cv::Rect(width / 2, height / 2, window_width, window_height);
  };
}

GazeboVideoRecorder::~GazeboVideoRecorder() { video_encoder_.Reset(); }

void GazeboVideoRecorder::load(const physics::WorldPtr &world,
                               const sdf::ElementPtr &sdf) {
  world_ = world;

  if (not sdf->HasElement("width"))
    gzthrow(logger_prefix_ + "Failed to get width");
  width_ = sdf->Get<unsigned int>("width");

  if (not sdf->HasElement("height"))
    gzthrow(logger_prefix_ + "Failed to get height");
  height_ = sdf->Get<unsigned int>("height");

  if (not sdf->HasElement("bitRate"))
    gzthrow(logger_prefix_ + "Failed to get bitRate");
  bit_rate_ = sdf->Get<unsigned int>("bitRate");

  if (sdf->HasElement("logMetadata"))
    log_metadata_ = sdf->Get<bool>("logMetadata");

  if (log_metadata_) {
    if (sdf->HasElement("logWallTime"))
      log_wall_time_ = sdf->Get<bool>("logWallTime");
  }

  if (sdf->HasElement("addTimestampInFilename"))
    add_timestamp_in_filename_ = sdf->Get<bool>("addTimestampInFilename");
}

void GazeboVideoRecorder::reset() { video_encoder_.Reset(); }

void GazeboVideoRecorder::setAddTimestampInFilename(bool state) {
  add_timestamp_in_filename_ = state;
}

void GazeboVideoRecorder::start(const boost::filesystem::path &save_path,
                                const std::string &file_timestamp,
                                const common::Time &start_time) {
  save_path_ = save_path;
  file_timestamp_ = file_timestamp;
  start_time_ = start_time;
  auto path = getPath(boost::uuids::to_string(uuid_generator_()));
  video_encoder_.Start("mp4", path, width_, height_, fps_, bit_rate_);
}

std::string GazeboVideoRecorder::stop(bool discard, std::string filename) {
  video_encoder_.Stop();

  std::string path;
  if (discard) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       logger_prefix_ << "Discarding active recording");
  } else {
    if (filename.empty()) filename = "video";
    auto file = getPath(filename, add_timestamp_in_filename_);
    if (video_encoder_.SaveToFile(file)) {
      path = file;
      RCLCPP_INFO_STREAM(node_->get_logger(),
                         logger_prefix_ << "Recording saved in " << path);
    } else {
      RCLCPP_WARN_STREAM(node_->get_logger(), logger_prefix_
                                                  << "Failed to save recording "
                                                  << file << "; resetting");
    }
  }
  if (path.empty()) video_encoder_.Reset();

  return path;
}

void GazeboVideoRecorder::addFrame(
    const sensors::GvmMulticameraSensor::ImageDataPtr &data_main,
    const sensors::GvmMulticameraSensor::ImageDataPtr &data_window) {
  cv::Mat image_main = toCvMat(data_main);

  if (data_window) {
    cv::Mat image_window = toCvMat(data_window);
    writeWindow(image_main, image_window, WindowType::BOTTOM_RIGHT_CORNER);
  }
  if (log_metadata_) {
    writeMetadata(image_main);
  }

  // cv::namedWindow(logger_prefix_ + "Recording", cv::WINDOW_AUTOSIZE);
  // cv::imshow(logger_prefix_ + "Recording", image_main);
  // cv::waitKey(1);

  cv::cvtColor(image_main, image_main, cv::COLOR_RGB2BGR);
  video_encoder_.AddFrame(image_main.data, static_cast<uint>(image_main.cols),
                          static_cast<uint>(image_main.rows));
}

void GazeboVideoRecorder::addMultiViewFrame(
    const sensors::GvmMulticameraSensor::ImageDataPtr &data_tl,
    const sensors::GvmMulticameraSensor::ImageDataPtr &data_tr,
    const sensors::GvmMulticameraSensor::ImageDataPtr &data_bl,
    const sensors::GvmMulticameraSensor::ImageDataPtr &data_br) {
  cv::Mat image_main(static_cast<int>(height_), static_cast<int>(width_),
                     CV_8UC3, cv::Scalar(0));

  if (data_tl) {
    cv::Mat image_tl = toCvMat(data_tl);
    writeWindow(image_main, image_tl, WindowType::TOP_LEFT_QUADRANT);
  }
  if (data_tr) {
    cv::Mat image_tr = toCvMat(data_tr);
    writeWindow(image_main, image_tr, WindowType::TOP_RIGHT_QUADRANT);
  }
  if (data_bl) {
    cv::Mat image_bl = toCvMat(data_bl);
    writeWindow(image_main, image_bl, WindowType::BOTTOM_LEFT_QUADRANT);
  }
  if (data_br) {
    cv::Mat image_br = toCvMat(data_br);
    writeWindow(image_main, image_br, WindowType::BOTTOM_RIGHT_QUADRANT);
  }
  if (log_metadata_) {
    writeMetadata(image_main);
  }

  cv::cvtColor(image_main, image_main, cv::COLOR_RGB2BGR);
  video_encoder_.AddFrame(image_main.data, static_cast<uint>(image_main.cols),
                          static_cast<uint>(image_main.rows));
}

std::string GazeboVideoRecorder::getPath(std::string filename,
                                         bool add_timestamp) {
  if (add_timestamp) filename += "-" + file_timestamp_;
  return (save_path_ / filename.append(".mp4")).string();
}

cv::Mat GazeboVideoRecorder::toCvMat(
    const sensors::GvmMulticameraSensor::ImageDataPtr &data) const {
  cv::Mat image(data->height, data->width, CV_8UC3,
                const_cast<uchar *>(data->data), data->getStep());
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  return image;
}

void GazeboVideoRecorder::writeWindow(cv::Mat &image_main,
                                      cv::Mat &image_window, WindowType type) {
  cv::Rect roi = roi_map_[type](image_main.cols, image_main.rows);
  cv::Rect window_roi(roi.x + 1, roi.y + 1, roi.width - 2, roi.height - 2);

  // Create a frame around the window image
  image_main(roi).setTo(cv::Scalar(0, 0, 0));

  // Write window image on the main image
  cv::Mat window = image_main(window_roi);
  cv::resize(image_window, image_window,
             cv::Size(window_roi.width, window_roi.height));
  image_window.copyTo(window);
}

void GazeboVideoRecorder::writeMetadata(cv::Mat &image) {
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
  common::Time elapsed_time = world_->RealTime() - start_time_;
  text = "Elapsed Time: " + std::to_string(elapsed_time.Double());
  cv::putText(image, text, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255},
              1, cv::LINE_AA);
}

boost::uuids::random_generator GazeboVideoRecorder::uuid_generator_ =
    boost::uuids::random_generator();

}  // namespace gazebo
