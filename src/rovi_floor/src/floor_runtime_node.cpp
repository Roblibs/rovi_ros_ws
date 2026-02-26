#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

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

std::string trim_copy(const std::string & s)
{
  const auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  size_t b = 0;
  while (b < s.size() && is_space(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && is_space(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

std::string strip_quotes(const std::string & s)
{
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

std::optional<std::string> read_meta_scalar(const std::filesystem::path & path, const std::string & key)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    return std::nullopt;
  }
  std::string line;
  const std::string prefix = key + ":";
  while (std::getline(in, line)) {
    const auto t = trim_copy(line);
    if (t.rfind(prefix, 0) != 0) {
      continue;
    }
    std::string v = trim_copy(t.substr(prefix.size()));
    if (v.empty()) {
      return std::string();
    }
    return strip_quotes(v);
  }
  return std::nullopt;
}

std::string sha256_hex(const std::string & input)
{
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.data()), input.size(), hash);
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < SHA256_DIGEST_LENGTH; ++i) {
    oss << std::setw(2) << static_cast<int>(hash[i]);
  }
  return oss.str();
}

std::string canonical_signature_input(
  const std::string & robot_mode,
  const std::string & base_frame,
  const std::string & camera_frame_id,
  uint32_t width,
  uint32_t height,
  const sensor_msgs::msg::CameraInfo & ci,
  const geometry_msgs::msg::TransformStamped & tf_base_camera)
{
  std::ostringstream oss;
  oss << std::setprecision(17);
  oss << "schema_version=1\n";
  oss << "robot_mode=" << robot_mode << "\n";
  oss << "base_frame=" << base_frame << "\n";
  oss << "camera_frame_id=" << camera_frame_id << "\n";
  oss << "width=" << width << "\n";
  oss << "height=" << height << "\n";
  oss << "distortion_model=" << ci.distortion_model << "\n";
  oss << "K=";
  for (size_t i = 0; i < 9; ++i) { oss << ci.k[i] << (i + 1 < 9 ? "," : "\n"); }
  oss << "D=";
  for (size_t i = 0; i < ci.d.size(); ++i) { oss << ci.d[i] << (i + 1 < ci.d.size() ? "," : "\n"); }
  oss << "R=";
  for (size_t i = 0; i < 9; ++i) { oss << ci.r[i] << (i + 1 < 9 ? "," : "\n"); }
  oss << "P=";
  for (size_t i = 0; i < 12; ++i) { oss << ci.p[i] << (i + 1 < 12 ? "," : "\n"); }
  oss << "t_base_camera="
      << tf_base_camera.transform.translation.x << ","
      << tf_base_camera.transform.translation.y << ","
      << tf_base_camera.transform.translation.z << "\n";
  oss << "q_base_camera_xyzw="
      << tf_base_camera.transform.rotation.x << ","
      << tf_base_camera.transform.rotation.y << ","
      << tf_base_camera.transform.rotation.z << ","
      << tf_base_camera.transform.rotation.w << "\n";
  return oss.str();
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

sensor_msgs::msg::Image mat_to_mono8_msg(const cv::Mat & mask, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::Image out;
  out.header = header;
  out.height = static_cast<uint32_t>(mask.rows);
  out.width = static_cast<uint32_t>(mask.cols);
  out.encoding = sensor_msgs::image_encodings::MONO8;
  out.is_bigendian = false;
  out.step = static_cast<sensor_msgs::msg::Image::_step_type>(mask.cols);
  out.data.assign(mask.datastart, mask.dataend);
  return out;
}

visualization_msgs::msg::Marker make_delete_all_marker(const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

}  // namespace

class FloorRuntimeNode final : public rclcpp::Node
{
public:
  FloorRuntimeNode()
  : rclcpp::Node("rovi_floor_runtime"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Standard ROS param (used implicitly by node clock).
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", false);
    }

    depth_topic_ = this->declare_parameter("depth_topic", "/camera/depth/image");
    camera_info_topic_ = this->declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    mask_topic_ = this->declare_parameter("mask_topic", "/floor/mask");
    topology_topic_ = this->declare_parameter("topology_topic", "/floor/topology");

    base_frame_ = this->declare_parameter("base_frame", "base_footprint");
    robot_mode_ = this->declare_parameter("robot_mode", "real");
    camera_topology_enabled_ = this->declare_parameter("camera_topology_enabled", false);

    floor_model_ = this->declare_parameter("floor_model", "auto");
    unknown_depth_is_floor_ = this->declare_parameter("unknown_depth_is_floor", true);
    floor_band_height_m_ = this->declare_parameter("floor_band_height_m", 0.02);
    max_floor_depth_m_ = this->declare_parameter("max_floor_depth_m", 2.5);
    t_floor_min_mm_ = this->declare_parameter("t_floor_min_mm", 10);
    t_floor_max_mm_ = this->declare_parameter("t_floor_max_mm", 400);

    lut_dir_ = expand_user("~/.ros/rovi/floor");
    floor_mm_file_ = this->declare_parameter("floor_mm_file", "floor_mm.png");
    t_floor_mm_file_ = this->declare_parameter("t_floor_mm_file", "t_floor_mm.png");
    meta_file_ = this->declare_parameter("meta_file", "meta.yaml");

    min_contour_area_px_ = this->declare_parameter("min_contour_area_px", 500);
    contour_stride_px_ = this->declare_parameter("contour_stride_px", 4);
    morph_kernel_px_ = this->declare_parameter("morph_kernel_px", 5);

    auto qos = rclcpp::SensorDataQoS();
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(mask_topic_, qos);

    // Always subscribe to camera_info so we can verify LUT meta signatures even when topology is disabled.
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { last_camera_info_ = msg; });

    if (camera_topology_enabled_) {
      topology_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topology_topic_, qos);
    }

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      qos,
      std::bind(&FloorRuntimeNode::on_depth, this, std::placeholders::_1));
  }

private:
  void throttled_error(const std::string & message)
  {
    const auto now = this->now();
    if (!last_error_time_ || (now - *last_error_time_).seconds() > 2.0) {
      last_error_time_ = now;
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }
  }

  bool try_load_disk_lut(const sensor_msgs::msg::Image & depth_msg)
  {
    const std::filesystem::path dir(lut_dir_);
    const auto floor_path = (dir / floor_mm_file_).string();
    const auto t_path = (dir / t_floor_mm_file_).string();
    const auto meta_path = dir / meta_file_;

    cv::Mat floor = cv::imread(floor_path, cv::IMREAD_UNCHANGED);
    cv::Mat t = cv::imread(t_path, cv::IMREAD_UNCHANGED);
    if (floor.empty() || t.empty()) {
      throttled_error(
        "Missing floor LUTs under '" + lut_dir_ + "'. Expected: " + floor_mm_file_ + " and " + t_floor_mm_file_);
      return false;
    }
    if (floor.type() != CV_16UC1 || t.type() != CV_16UC1) {
      throttled_error("LUT PNGs must be uint16 single-channel (16UC1).");
      return false;
    }
    if (floor.rows != static_cast<int>(depth_msg.height) || floor.cols != static_cast<int>(depth_msg.width)) {
      throttled_error("LUT image size does not match depth image size.");
      return false;
    }
    if (t.rows != floor.rows || t.cols != floor.cols) {
      throttled_error("Threshold LUT size does not match floor LUT size.");
      return false;
    }

    if (std::filesystem::exists(meta_path)) {
      const auto expected_robot_mode = read_meta_scalar(meta_path, "robot_mode");
      if (expected_robot_mode && !expected_robot_mode->empty() && *expected_robot_mode != robot_mode_) {
        throttled_error(
          "LUT meta conflict: robot_mode mismatch (expected " + *expected_robot_mode + ", got " + robot_mode_ + ")");
        return false;
      }

      const auto expected_frame = read_meta_scalar(meta_path, "camera_frame_id");
      if (expected_frame && !expected_frame->empty() && *expected_frame != depth_msg.header.frame_id) {
        throttled_error(
          "LUT meta conflict: camera_frame_id mismatch (expected " + *expected_frame + ", got " +
          depth_msg.header.frame_id + ")");
        return false;
      }

      const auto expected_serial = read_meta_scalar(meta_path, "device_serial");
      if (expected_serial && !expected_serial->empty()) {
        const std::string current_serial = detect_device_serial();
        if (!current_serial.empty() && current_serial != *expected_serial) {
          throttled_error(
            "LUT meta conflict: device_serial mismatch (expected " + *expected_serial + ", got " + current_serial + ")");
          return false;
        }
      }

      const auto expected_sig = read_meta_scalar(meta_path, "signature_sha256");
      if (expected_sig && !expected_sig->empty()) {
        const auto camera_info = last_camera_info_;
        if (!camera_info) {
          throttled_error("LUT meta requires /camera/depth/camera_info for signature verification.");
          return false;
        }
        if (camera_info->header.frame_id.empty()) {
          throttled_error("LUT meta verification failed: camera_info.header.frame_id is empty.");
          return false;
        }

        geometry_msgs::msg::TransformStamped tf_base_camera;
        try {
          tf_base_camera = tf_buffer_.lookupTransform(
            base_frame_, camera_info->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.25));
        } catch (const tf2::TransformException & e) {
          throttled_error(std::string("LUT meta verification TF lookup failed: ") + e.what());
          return false;
        }

        const std::string signature_input = canonical_signature_input(
          robot_mode_, base_frame_, camera_info->header.frame_id, camera_info->width, camera_info->height,
          *camera_info, tf_base_camera);
        const std::string got_sig = sha256_hex(signature_input);
        if (got_sig != *expected_sig) {
          throttled_error(
            "LUT meta conflict: signature_sha256 mismatch (expected " + *expected_sig + ", got " + got_sig + ")");
          return false;
        }
      }
    }

    floor_mm_ = floor;
    t_floor_mm_ = t;
    floor_model_active_ = "lut";
    RCLCPP_INFO(this->get_logger(), "Loaded floor LUTs from '%s'.", lut_dir_.c_str());
    return true;
  }

  bool try_build_plane_model(const sensor_msgs::msg::Image & depth_msg)
  {
    const auto camera_info = last_camera_info_;
    if (!camera_info) {
      throttled_error("floor_model requires /camera/depth/camera_info but none received yet.");
      return false;
    }
    if (camera_info->header.frame_id.empty()) {
      throttled_error("floor_model requires camera_info.header.frame_id but it is empty.");
      return false;
    }
    if (camera_info->width != depth_msg.width || camera_info->height != depth_msg.height) {
      throttled_error("camera_info dimensions do not match depth image dimensions.");
      return false;
    }

    const double fx = camera_info->k[0];
    const double fy = camera_info->k[4];
    const double cx = camera_info->k[2];
    const double cy = camera_info->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      throttled_error("Invalid camera intrinsics (fx/fy <= 0).");
      return false;
    }

    geometry_msgs::msg::TransformStamped tf_base_camera;
    try {
      tf_base_camera = tf_buffer_.lookupTransform(
        base_frame_, camera_info->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException & e) {
      throttled_error(std::string("TF lookup failed (base->camera): ") + e.what());
      return false;
    }

    const std::string signature_input = canonical_signature_input(
      robot_mode_, base_frame_, camera_info->header.frame_id, camera_info->width, camera_info->height,
      *camera_info, tf_base_camera);
    const std::string sig = sha256_hex(signature_input);
    if (plane_model_sig_ && *plane_model_sig_ == sig &&
      floor_mm_.has_value() && t_floor_mm_.has_value() &&
      floor_mm_->rows == static_cast<int>(depth_msg.height) && floor_mm_->cols == static_cast<int>(depth_msg.width))
    {
      return true;
    }

    tf2::Transform T;
    tf2::fromMsg(tf_base_camera.transform, T);
    const tf2::Vector3 t = T.getOrigin();
    const tf2::Matrix3x3 R = T.getBasis();
    const tf2::Vector3 row_z = R.getRow(2);

    const int rows = static_cast<int>(depth_msg.height);
    const int cols = static_cast<int>(depth_msg.width);
    cv::Mat floor(rows, cols, CV_16UC1, cv::Scalar(0));
    cv::Mat t_floor(rows, cols, CV_16UC1, cv::Scalar(0));

    const double height_m = std::max(0.0, floor_band_height_m_);
    const int t_min = std::max(0, t_floor_min_mm_);
    const int t_max = std::max(t_min, t_floor_max_mm_);

    for (int v = 0; v < rows; ++v) {
      uint16_t * f = floor.ptr<uint16_t>(v);
      uint16_t * tt = t_floor.ptr<uint16_t>(v);
      const double y = (static_cast<double>(v) - cy) / fy;
      for (int u = 0; u < cols; ++u) {
        const double x = (static_cast<double>(u) - cx) / fx;
        const double denom = row_z.x() * x + row_z.y() * y + row_z.z();
        if (denom >= -1e-9) {
          f[u] = 0;
          tt[u] = 0;
          continue;
        }
        const double z0 = -t.z() / denom;  // expected depth (Z in camera optical frame) at base z=0.
        if (!(z0 > 0.0) || z0 > max_floor_depth_m_) {
          f[u] = 0;
          tt[u] = 0;
          continue;
        }

        const double z_mm = z0 * 1000.0;
        const int z_i = static_cast<int>(std::lround(std::clamp(z_mm, 0.0, 65535.0)));
        f[u] = static_cast<uint16_t>(z_i);

        // Depth delta (in mm) corresponding to a +/-height_m band in base Z.
        const double t_mm = std::abs(height_m / denom) * 1000.0;
        const int t_i = static_cast<int>(std::lround(std::clamp(t_mm, static_cast<double>(t_min), static_cast<double>(t_max))));
        tt[u] = static_cast<uint16_t>(t_i);
      }
    }

    floor_mm_ = std::move(floor);
    t_floor_mm_ = std::move(t_floor);
    plane_model_sig_ = sig;
    floor_model_active_ = "plane";
    RCLCPP_INFO(
      this->get_logger(),
      "Built plane floor model in-memory (height=%.3fm max_depth=%.2fm unknown_depth_is_floor=%s).",
      floor_band_height_m_, max_floor_depth_m_, unknown_depth_is_floor_ ? "true" : "false");
    return true;
  }

  bool ensure_floor_model_ready(const sensor_msgs::msg::Image & depth_msg)
  {
    const int rows = static_cast<int>(depth_msg.height);
    const int cols = static_cast<int>(depth_msg.width);
    if (floor_mm_.has_value() && t_floor_mm_.has_value()) {
      if (floor_mm_->rows == rows && floor_mm_->cols == cols &&
        t_floor_mm_->rows == rows && t_floor_mm_->cols == cols)
      {
        return true;
      }
      floor_mm_.reset();
      t_floor_mm_.reset();
      plane_model_sig_.reset();
      floor_model_active_.clear();
    }

    const std::string mode = trim_copy(floor_model_);
    const bool want_lut = (mode.empty() || mode == "auto" || mode == "lut");
    const bool want_plane = (mode.empty() || mode == "auto" || mode == "plane");

    if (want_lut && try_load_disk_lut(depth_msg)) {
      return true;
    }
    if (want_plane && try_build_plane_model(depth_msg)) {
      return true;
    }
    return false;
  }

  std::string detect_device_serial()
  {
    if (cached_device_serial_) {
      return *cached_device_serial_;
    }

    for (const char * node_name : {"camera_info_pub", "sim_camera_info_pub"}) {
      try {
        auto client = std::make_shared<rclcpp::SyncParametersClient>(this->shared_from_this(), node_name);
        if (!client->wait_for_service(std::chrono::milliseconds(200))) {
          continue;
        }
        const auto vals = client->get_parameters({"depth_device_serial"});
        if (vals.size() != 1) {
          continue;
        }
        if (vals[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          continue;
        }
        const std::string serial = vals[0].as_string();
        if (!serial.empty()) {
          cached_device_serial_ = serial;
          return serial;
        }
      } catch (const std::exception &) {
        continue;
      }
    }
    cached_device_serial_ = std::string();
    return *cached_device_serial_;
  }

  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (!is_depth_encoding_supported(msg->encoding)) {
      throttled_error("Unsupported depth encoding on " + depth_topic_ + ": " + msg->encoding);
      return;
    }

    if (!ensure_floor_model_ready(*msg)) {
      return;
    }

    cv::Mat depth_mm;
    try {
      depth_mm = convert_depth_to_mm_u16(*msg);
    } catch (const std::exception & e) {
      throttled_error(e.what());
      return;
    }

    const cv::Mat & floor = *floor_mm_;
    const cv::Mat & t_floor = *t_floor_mm_;

    cv::Mat mask(depth_mm.rows, depth_mm.cols, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < depth_mm.rows; ++y) {
      const uint16_t * d = depth_mm.ptr<uint16_t>(y);
      const uint16_t * f = floor.ptr<uint16_t>(y);
      const uint16_t * t = t_floor.ptr<uint16_t>(y);
      uint8_t * m = mask.ptr<uint8_t>(y);
      for (int x = 0; x < depth_mm.cols; ++x) {
        const uint16_t D = d[x];
        const uint16_t F = f[x];
        if (D == 0) {
          m[x] = unknown_depth_is_floor_ ? 255 : 0;
          continue;
        }
        if (F == 0) {
          m[x] = 0;
          continue;
        }
        const int diff = static_cast<int>(D) - static_cast<int>(F);
        const int ad = std::abs(diff);
        m[x] = (ad <= static_cast<int>(t[x])) ? 255 : 0;
      }
    }

    mask_pub_->publish(mat_to_mono8_msg(mask, msg->header));

    if (!camera_topology_enabled_ || !topology_pub_) {
      return;
    }

    if (!last_camera_info_) {
      throttled_error("Topology enabled but no /camera/depth/camera_info received yet.");
      return;
    }

    publish_topology(mask, depth_mm, msg->header);
  }

  void publish_topology(const cv::Mat & floor_mask, const cv::Mat & depth_mm, const std_msgs::msg::Header & header)
  {
    const auto camera_info = last_camera_info_;
    if (!camera_info) {
      return;
    }

    const std::string camera_frame = camera_info->header.frame_id.empty() ? header.frame_id : camera_info->header.frame_id;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(base_frame_, camera_frame, header.stamp, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & e) {
      throttled_error(std::string("TF lookup failed: ") + e.what());
      return;
    }

    tf2::Transform T;
    tf2::fromMsg(tf.transform, T);

    cv::Mat unsafe;
    cv::bitwise_not(floor_mask, unsafe);

    cv::Mat bin;
    cv::threshold(unsafe, bin, 0, 255, cv::THRESH_BINARY);

    const int k = std::max(1, morph_kernel_px_);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(bin, labels, stats, centroids, 8, CV_32S);
    cv::Mat cleaned(bin.rows, bin.cols, CV_8UC1, cv::Scalar(0));
    for (int i = 1; i < n; ++i) {
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area < min_contour_area_px_) {
        continue;
      }
      cleaned.setTo(255, labels == i);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
      visualization_msgs::msg::MarkerArray out;
      std_msgs::msg::Header h;
      h.stamp = header.stamp;
      h.frame_id = base_frame_;
      out.markers.push_back(make_delete_all_marker(h));
      topology_pub_->publish(out);
      return;
    }

    // Pick the largest contour by area for a stable visualization.
    size_t best_idx = 0;
    double best_area = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
      const double a = std::abs(cv::contourArea(contours[i]));
      if (a > best_area) {
        best_area = a;
        best_idx = i;
      }
    }

    std::vector<cv::Point> contour = contours[best_idx];
    if (contour_stride_px_ > 1) {
      std::vector<cv::Point> decimated;
      decimated.reserve(contour.size() / static_cast<size_t>(contour_stride_px_) + 4);
      for (size_t i = 0; i < contour.size(); i += static_cast<size_t>(contour_stride_px_)) {
        decimated.push_back(contour[i]);
      }
      contour = std::move(decimated);
    }

    const double eps = 2.0;
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, eps, true);

    const double fx = camera_info->k[0];
    const double fy = camera_info->k[4];
    const double cx = camera_info->k[2];
    const double cy = camera_info->k[5];
    if (fx <= 0.0 || fy <= 0.0) {
      throttled_error("Invalid camera intrinsics (fx/fy <= 0).");
      return;
    }

    visualization_msgs::msg::Marker m;
    m.header.stamp = header.stamp;
    m.header.frame_id = base_frame_;
    m.ns = "floor_topology";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.01;
    m.color.r = 1.0f;
    m.color.g = 0.1f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);

    m.points.reserve(approx.size() + 1);
    for (const auto & p : approx) {
      const int u = std::clamp(p.x, 0, depth_mm.cols - 1);
      const int v = std::clamp(p.y, 0, depth_mm.rows - 1);
      const uint16_t D = depth_mm.at<uint16_t>(v, u);
      const uint16_t F = floor_mm_.has_value() ? floor_mm_->at<uint16_t>(v, u) : 0;
      const uint16_t Z_mm = (D > 0) ? D : F;
      if (Z_mm == 0) {
        continue;
      }

      const double Z = static_cast<double>(Z_mm) / 1000.0;
      const double X = (static_cast<double>(u) - cx) / fx * Z;
      const double Y = (static_cast<double>(v) - cy) / fy * Z;

      tf2::Vector3 pc(X, Y, Z);
      tf2::Vector3 pb = T * pc;

      geometry_msgs::msg::Point out;
      out.x = pb.x();
      out.y = pb.y();
      out.z = pb.z();
      m.points.push_back(out);
    }

    if (!m.points.empty()) {
      m.points.push_back(m.points.front());
    }

    visualization_msgs::msg::MarkerArray out;
    std_msgs::msg::Header h;
    h.stamp = header.stamp;
    h.frame_id = base_frame_;
    out.markers.push_back(make_delete_all_marker(h));
    out.markers.push_back(m);
    topology_pub_->publish(out);
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string mask_topic_;
  std::string topology_topic_;
  std::string base_frame_;
  std::string robot_mode_;
  bool camera_topology_enabled_{false};

  std::string floor_model_;
  std::string floor_model_active_;
  bool unknown_depth_is_floor_{true};
  double floor_band_height_m_{0.02};
  double max_floor_depth_m_{2.5};
  int t_floor_min_mm_{10};
  int t_floor_max_mm_{400};

  std::string lut_dir_;
  std::string floor_mm_file_;
  std::string t_floor_mm_file_;
  std::string meta_file_;

  int min_contour_area_px_{500};
  int contour_stride_px_{4};
  int morph_kernel_px_{5};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::optional<cv::Mat> floor_mm_;
  std::optional<cv::Mat> t_floor_mm_;
  std::optional<std::string> plane_model_sig_;

  std::optional<rclcpp::Time> last_error_time_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  std::optional<std::string> cached_device_serial_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FloorRuntimeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
