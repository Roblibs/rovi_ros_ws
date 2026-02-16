#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

using sensor_msgs::image_encodings::TYPE_16UC1;
using sensor_msgs::image_encodings::TYPE_32FC1;

std::string expand_user(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  if (!home || std::string(home).empty()) {
    return path;
  }
  if (path == "~") {
    return std::string(home);
  }
  if (path.size() >= 2 && path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

bool is_depth_encoding_supported(const std::string & encoding)
{
  return encoding == TYPE_32FC1 || encoding == TYPE_16UC1;
}

cv::Mat convert_depth_to_mm_u16(const sensor_msgs::msg::Image & msg)
{
  if (msg.encoding == TYPE_16UC1) {
    const auto cv_ptr = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(msg), TYPE_16UC1);
    return cv_ptr->image.clone();
  }

  if (msg.encoding == TYPE_32FC1) {
    const auto cv_ptr = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(msg), TYPE_32FC1);
    const cv::Mat & src = cv_ptr->image;
    cv::Mat out(src.rows, src.cols, CV_16UC1, cv::Scalar(0));
    for (int y = 0; y < src.rows; ++y) {
      const float * in = src.ptr<float>(y);
      uint16_t * o = out.ptr<uint16_t>(y);
      for (int x = 0; x < src.cols; ++x) {
        const float z_m = in[x];
        if (!std::isfinite(z_m) || z_m <= 0.0f) {
          o[x] = 0;
          continue;
        }
        const float mm_f = z_m * 1000.0f;
        if (mm_f >= 65535.0f) {
          o[x] = 65535;
        } else {
          o[x] = static_cast<uint16_t>(std::lround(mm_f));
        }
      }
    }
    return out;
  }

  throw std::runtime_error("Unsupported depth encoding: " + msg.encoding);
}

void atomic_write_png_u16(const std::filesystem::path & dst_path, const cv::Mat & img_u16)
{
  if (img_u16.empty() || img_u16.type() != CV_16UC1) {
    throw std::runtime_error("atomic_write_png_u16 requires CV_16UC1 image");
  }

  std::filesystem::create_directories(dst_path.parent_path());

  const auto tmp_path =
    dst_path.parent_path() / (dst_path.stem().string() + ".tmp" + dst_path.extension().string());
  if (!cv::imwrite(tmp_path.string(), img_u16)) {
    throw std::runtime_error("Failed to write: " + tmp_path.string());
  }

  std::error_code ec;
  std::filesystem::remove(dst_path, ec);
  std::filesystem::rename(tmp_path, dst_path);
}

cv::Vec3f normalize3(const cv::Vec3f & v)
{
  const float n = std::sqrt(v.dot(v));
  if (n <= 1e-9f) {
    return v;
  }
  return v * (1.0f / n);
}

}  // namespace

class FloorCalibrateNode final : public rclcpp::Node
{
public:
  FloorCalibrateNode()
  : rclcpp::Node("rovi_floor_calibrate"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Standard ROS param (used implicitly by node clock).
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", false);
    }

    depth_topic_ = this->declare_parameter("depth_topic", "/camera/depth/image");
    camera_info_topic_ = this->declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    base_frame_ = this->declare_parameter("base_frame", "base_footprint");

    lut_dir_ = expand_user(this->declare_parameter("lut_dir", "~/.ros/rovi/floor"));
    floor_mm_file_ = this->declare_parameter("floor_mm_file", "floor_mm.png");
    t_floor_mm_file_ = this->declare_parameter("t_floor_mm_file", "t_floor_mm.png");
    t_obst1_mm_file_ = this->declare_parameter("t_obst1_mm_file", "t_obst1_mm.png");
    t_obst2_mm_file_ = this->declare_parameter("t_obst2_mm_file", "t_obst2_mm.png");

    capture_duration_s_ = this->declare_parameter("capture_duration_s", 10.0);
    max_frames_ = this->declare_parameter("max_frames", 350);
    min_valid_samples_ = this->declare_parameter("min_valid_samples", 25);

    height_floor_m_ = this->declare_parameter("height_floor_m", 0.02);
    height_obst1_m_ = this->declare_parameter("height_obst1_m", 0.05);
    height_obst2_m_ = this->declare_parameter("height_obst2_m", 0.10);
    generate_obstacle_thresholds_ = this->declare_parameter("generate_obstacle_thresholds", false);

    plane_sample_stride_px_ = this->declare_parameter("plane_sample_stride_px", 8);
    plane_max_points_ = this->declare_parameter("plane_max_points", 8000);

    auto qos = rclcpp::SensorDataQoS();
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      qos,
      std::bind(&FloorCalibrateNode::on_depth, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { last_camera_info_ = msg; });

    RCLCPP_INFO(
      this->get_logger(),
      "Floor calibration:\n"
      "  - depth_topic=%s\n"
      "  - camera_info_topic=%s\n"
      "  - output_dir=%s\n"
      "  - capture_duration_s=%.1f max_frames=%d",
      depth_topic_.c_str(), camera_info_topic_.c_str(), lut_dir_.c_str(),
      capture_duration_s_, max_frames_);
  }

  bool done() const { return done_; }
  int exit_code() const { return exit_code_; }

private:
  void fail(const std::string & message)
  {
    if (done_) {
      return;
    }
    done_ = true;
    exit_code_ = 2;
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  }

  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (done_ || processing_ || !msg) {
      return;
    }
    if (!is_depth_encoding_supported(msg->encoding)) {
      fail("Unsupported depth encoding on " + depth_topic_ + ": " + msg->encoding);
      return;
    }

    if (!start_time_) {
      start_time_ = std::chrono::steady_clock::now();
    }

    cv::Mat depth_mm;
    try {
      depth_mm = convert_depth_to_mm_u16(*msg);
    } catch (const std::exception & e) {
      fail(e.what());
      return;
    }

    if (frames_.empty()) {
      rows_ = depth_mm.rows;
      cols_ = depth_mm.cols;
    } else if (depth_mm.rows != rows_ || depth_mm.cols != cols_) {
      fail("Depth image size changed during capture.");
      return;
    }

    frames_.push_back(std::move(depth_mm));

    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - *start_time_).count();
    if (static_cast<int>(frames_.size()) >= max_frames_ || elapsed >= capture_duration_s_) {
      processing_ = true;
      process();
    }
  }

  void process()
  {
    if (done_) {
      return;
    }

    if (!last_camera_info_) {
      fail("Missing /camera/depth/camera_info (required for calibration).");
      return;
    }

    if (frames_.empty() || rows_ <= 0 || cols_ <= 0) {
      fail("No depth frames captured.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Captured %zu depth frames (%dx%d).", frames_.size(), cols_, rows_);

    cv::Mat floor_mm(rows_, cols_, CV_16UC1, cv::Scalar(0));
    std::vector<uint16_t> samples;
    samples.reserve(frames_.size());

    for (int y = 0; y < rows_; ++y) {
      uint16_t * out = floor_mm.ptr<uint16_t>(y);
      for (int x = 0; x < cols_; ++x) {
        samples.clear();
        for (const auto & f : frames_) {
          const uint16_t v = f.at<uint16_t>(y, x);
          if (v > 0) {
            samples.push_back(v);
          }
        }
        if (static_cast<int>(samples.size()) < min_valid_samples_) {
          out[x] = 0;
          continue;
        }
        const size_t mid = samples.size() / 2;
        std::nth_element(samples.begin(), samples.begin() + static_cast<long>(mid), samples.end());
        out[x] = samples[mid];
      }
    }

    cv::Vec3f n_base;
    float d_base = 0.0f;
    if (!fit_floor_plane(floor_mm, n_base, d_base)) {
      fail("Failed to fit floor plane.");
      return;
    }
    (void)d_base;

    cv::Mat t_floor_mm;
    cv::Mat t_obst1_mm;
    cv::Mat t_obst2_mm;
    try {
      t_floor_mm = build_threshold_map(floor_mm, n_base, static_cast<float>(height_floor_m_));
      if (generate_obstacle_thresholds_) {
        t_obst1_mm = build_threshold_map(floor_mm, n_base, static_cast<float>(height_obst1_m_));
        t_obst2_mm = build_threshold_map(floor_mm, n_base, static_cast<float>(height_obst2_m_));
      }
    } catch (const std::exception & e) {
      fail(std::string("Threshold map build failed: ") + e.what());
      return;
    }

    try {
      const auto out_dir = std::filesystem::path(lut_dir_);
      atomic_write_png_u16(out_dir / floor_mm_file_, floor_mm);
      atomic_write_png_u16(out_dir / t_floor_mm_file_, t_floor_mm);
      if (generate_obstacle_thresholds_) {
        atomic_write_png_u16(out_dir / t_obst1_mm_file_, t_obst1_mm);
        atomic_write_png_u16(out_dir / t_obst2_mm_file_, t_obst2_mm);
      }
    } catch (const std::exception & e) {
      fail(std::string("Failed to write LUTs: ") + e.what());
      return;
    }

    done_ = true;
    exit_code_ = 0;
    RCLCPP_INFO(this->get_logger(), "Calibration complete. LUTs written under '%s'.", lut_dir_.c_str());
  }

  bool fit_floor_plane(const cv::Mat & floor_mm, cv::Vec3f & n_base, float & d_base)
  {
    const auto camera_info = last_camera_info_;
    if (!camera_info) {
      return false;
    }
    const std::string camera_frame = camera_info->header.frame_id;
    if (camera_frame.empty()) {
      fail("camera_info.header.frame_id is empty.");
      return false;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero, tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException & e) {
      fail(std::string("TF lookup failed: ") + e.what());
      return false;
    }

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    tf2::Matrix3x3 R(q);
    const tf2::Vector3 t(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);

    const double fx = camera_info->k[0];
    const double fy = camera_info->k[4];
    const double cx = camera_info->k[2];
    const double cy = camera_info->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      fail("Invalid camera intrinsics (fx/fy <= 0).");
      return false;
    }

    const int stride = std::max(1, plane_sample_stride_px_);
    std::vector<cv::Vec3f> points;
    points.reserve(static_cast<size_t>(plane_max_points_));

    for (int y = 0; y < floor_mm.rows; y += stride) {
      const uint16_t * row = floor_mm.ptr<uint16_t>(y);
      for (int x = 0; x < floor_mm.cols; x += stride) {
        const uint16_t z_mm = row[x];
        if (z_mm == 0) {
          continue;
        }
        const double Z = static_cast<double>(z_mm) / 1000.0;
        const double X = (static_cast<double>(x) - cx) / fx * Z;
        const double Y = (static_cast<double>(y) - cy) / fy * Z;

        tf2::Vector3 p_cam(X, Y, Z);
        const tf2::Vector3 p_base = R * p_cam + t;

        points.emplace_back(
          static_cast<float>(p_base.x()),
          static_cast<float>(p_base.y()),
          static_cast<float>(p_base.z()));

        if (static_cast<int>(points.size()) >= plane_max_points_) {
          break;
        }
      }
      if (static_cast<int>(points.size()) >= plane_max_points_) {
        break;
      }
    }

    if (points.size() < 50) {
      fail("Not enough floor points to fit plane (need >= 50).");
      return false;
    }

    cv::Mat data(static_cast<int>(points.size()), 3, CV_32F);
    for (int i = 0; i < data.rows; ++i) {
      data.at<float>(i, 0) = points[static_cast<size_t>(i)][0];
      data.at<float>(i, 1) = points[static_cast<size_t>(i)][1];
      data.at<float>(i, 2) = points[static_cast<size_t>(i)][2];
    }

    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Vec3f normal(
      pca.eigenvectors.at<float>(2, 0),
      pca.eigenvectors.at<float>(2, 1),
      pca.eigenvectors.at<float>(2, 2));
    normal = normalize3(normal);
    if (normal[2] < 0.0f) {
      normal *= -1.0f;
    }

    const cv::Vec3f mean(
      pca.mean.at<float>(0, 0),
      pca.mean.at<float>(0, 1),
      pca.mean.at<float>(0, 2));

    n_base = normal;
    d_base = -normal.dot(mean);

    RCLCPP_INFO(
      this->get_logger(),
      "Fitted floor plane in %s: n=[%.3f %.3f %.3f], d=%.3f (points=%zu)",
      base_frame_.c_str(), n_base[0], n_base[1], n_base[2], d_base, points.size());
    return true;
  }

  cv::Mat build_threshold_map(
    const cv::Mat & floor_mm,
    const cv::Vec3f & n_base,
    float height_m)
  {
    const auto camera_info = last_camera_info_;
    if (!camera_info) {
      throw std::runtime_error("missing camera_info");
    }
    const std::string camera_frame = camera_info->header.frame_id;
    if (camera_frame.empty()) {
      throw std::runtime_error("camera_info.header.frame_id empty");
    }

    geometry_msgs::msg::TransformStamped tf =
      tf_buffer_.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero, tf2::durationFromSec(0.25));

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);
    tf2::Matrix3x3 R(q);

    // R is base<-camera. We need n_cam = R^T n_base. tf2::Matrix3x3 stores rows.
    const cv::Vec3f n_cam_t(
      static_cast<float>(R[0].x() * n_base[0] + R[1].x() * n_base[1] + R[2].x() * n_base[2]),
      static_cast<float>(R[0].y() * n_base[0] + R[1].y() * n_base[1] + R[2].y() * n_base[2]),
      static_cast<float>(R[0].z() * n_base[0] + R[1].z() * n_base[1] + R[2].z() * n_base[2]));

    const double fx = camera_info->k[0];
    const double fy = camera_info->k[4];
    const double cx = camera_info->k[2];
    const double cy = camera_info->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      throw std::runtime_error("invalid camera intrinsics (fx/fy <= 0)");
    }

    cv::Mat out(floor_mm.rows, floor_mm.cols, CV_16UC1, cv::Scalar(0));

    for (int v = 0; v < floor_mm.rows; ++v) {
      const uint16_t * f = floor_mm.ptr<uint16_t>(v);
      uint16_t * o = out.ptr<uint16_t>(v);
      for (int u = 0; u < floor_mm.cols; ++u) {
        if (f[u] == 0) {
          o[u] = 0;
          continue;
        }
        const float x = static_cast<float>((static_cast<double>(u) - cx) / fx);
        const float y = static_cast<float>((static_cast<double>(v) - cy) / fy);
        const float denom = n_cam_t[0] * x + n_cam_t[1] * y + n_cam_t[2] * 1.0f;
        if (std::abs(denom) < 1e-6f) {
          o[u] = 0;
          continue;
        }
        const float delta_z = std::abs(height_m / denom);
        const float mm_f = delta_z * 1000.0f;
        if (mm_f >= 65535.0f) {
          o[u] = 65535;
        } else {
          o[u] = static_cast<uint16_t>(std::lround(mm_f));
        }
      }
    }

    return out;
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string base_frame_;

  std::string lut_dir_;
  std::string floor_mm_file_;
  std::string t_floor_mm_file_;
  std::string t_obst1_mm_file_;
  std::string t_obst2_mm_file_;

  double capture_duration_s_{10.0};
  int max_frames_{350};
  int min_valid_samples_{25};

  double height_floor_m_{0.02};
  double height_obst1_m_{0.05};
  double height_obst2_m_{0.10};
  bool generate_obstacle_thresholds_{false};

  int plane_sample_stride_px_{8};
  int plane_max_points_{8000};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;

  std::vector<cv::Mat> frames_;
  int rows_{0};
  int cols_{0};

  std::optional<std::chrono::steady_clock::time_point> start_time_;

  bool done_{false};
  bool processing_{false};
  int exit_code_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FloorCalibrateNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    exec.spin_once(std::chrono::milliseconds(50));
  }
  const int code = node->exit_code();
  rclcpp::shutdown();
  return code;
}
