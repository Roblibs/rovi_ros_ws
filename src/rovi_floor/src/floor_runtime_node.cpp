#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

using sensor_msgs::image_encodings::TYPE_16UC1;
using sensor_msgs::image_encodings::TYPE_32FC1;

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

std::string model_signature(
  const std::string & base_frame,
  const std::string & camera_frame,
  const sensor_msgs::msg::CameraInfo & ci,
  const geometry_msgs::msg::TransformStamped & tf_base_camera,
  double h1_m,
  double h2_m,
  double max_range_m)
{
  std::ostringstream oss;
  oss << std::setprecision(17);
  oss << "base_frame=" << base_frame << "\n";
  oss << "camera_frame=" << camera_frame << "\n";
  oss << "width=" << ci.width << "\n";
  oss << "height=" << ci.height << "\n";
  oss << "K=";
  for (size_t i = 0; i < 9; ++i) {
    oss << ci.k[i] << (i + 1 < 9 ? "," : "\n");
  }
  oss << "h1=" << h1_m << "\n";
  oss << "h2=" << h2_m << "\n";
  oss << "max_range=" << max_range_m << "\n";
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

struct ObstacleModel
{
  int rows{0};
  int cols{0};

  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  tf2::Transform T_base_camera{};

  cv::Mat z_h1_mm;  // CV_16UC1, 0 = no intersection / invalid
  cv::Mat z_h2_mm;  // CV_16UC1, 0 = no intersection / invalid

  std::string signature;
};

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
    camera_topology_enabled_ = this->declare_parameter("camera_topology_enabled", false);

    max_range_m_ = this->declare_parameter("max_range_m", 2.5);
    obst_h1_m_ = this->declare_parameter("obst_h1_m", 0.01);
    obst_h2_m_ = this->declare_parameter("obst_h2_m", 0.07);

    min_contour_area_px_ = this->declare_parameter("min_contour_area_px", 500);
    contour_stride_px_ = this->declare_parameter("contour_stride_px", 4);
    morph_kernel_px_ = this->declare_parameter("morph_kernel_px", 5);

    if (max_range_m_ <= 0.0) {
      throw std::runtime_error("max_range_m must be > 0");
    }

    if (obst_h1_m_ < 0.0 || obst_h2_m_ < 0.0) {
      throw std::runtime_error("obst_h1_m/obst_h2_m must be >= 0");
    }
    if (obst_h2_m_ < obst_h1_m_) {
      std::swap(obst_h1_m_, obst_h2_m_);
    }

    auto qos = rclcpp::SensorDataQoS();
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(mask_topic_, qos);

    if (camera_topology_enabled_) {
      topology_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topology_topic_, qos);
    }

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { last_camera_info_ = msg; });

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

  void throttled_warn(const std::string & message)
  {
    const auto now = this->now();
    if (!last_warn_time_ || (now - *last_warn_time_).seconds() > 2.0) {
      last_warn_time_ = now;
      RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    }
  }

  bool ensure_model_ready(const sensor_msgs::msg::Image & depth_msg)
  {
    const auto camera_info = last_camera_info_;
    if (!camera_info) {
      throttled_warn("Waiting for /camera/depth/camera_info (required for obstacle geometry model).");
      return false;
    }
    if (camera_info->header.frame_id.empty() && depth_msg.header.frame_id.empty()) {
      throttled_error("camera_info.header.frame_id and depth.header.frame_id are both empty.");
      return false;
    }
    if (camera_info->width != depth_msg.width || camera_info->height != depth_msg.height) {
      throttled_error("camera_info dimensions do not match depth image dimensions.");
      return false;
    }

    const std::string camera_frame = camera_info->header.frame_id.empty() ? depth_msg.header.frame_id : camera_info->header.frame_id;

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
        base_frame_, camera_frame, tf2::TimePointZero, tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException & e) {
      throttled_error(std::string("TF lookup failed (base->camera): ") + e.what());
      return false;
    }

    const std::string sig = model_signature(
      base_frame_, camera_frame, *camera_info, tf_base_camera, obst_h1_m_, obst_h2_m_, max_range_m_);
    if (model_ && model_->signature == sig) {
      return true;
    }

    tf2::Transform T;
    tf2::fromMsg(tf_base_camera.transform, T);
    const tf2::Vector3 t = T.getOrigin();
    const tf2::Matrix3x3 R = T.getBasis();
    const tf2::Vector3 row_z = R.getRow(2);

    const int rows = static_cast<int>(depth_msg.height);
    const int cols = static_cast<int>(depth_msg.width);

    cv::Mat z1(rows, cols, CV_16UC1, cv::Scalar(0));
    cv::Mat z2(rows, cols, CV_16UC1, cv::Scalar(0));

    const double max_z = max_range_m_;
    for (int v = 0; v < rows; ++v) {
      uint16_t * o1 = z1.ptr<uint16_t>(v);
      uint16_t * o2 = z2.ptr<uint16_t>(v);
      const double y = (static_cast<double>(v) - cy) / fy;
      for (int u = 0; u < cols; ++u) {
        const double x = (static_cast<double>(u) - cx) / fx;
        const double denom = row_z.x() * x + row_z.y() * y + row_z.z();
        if (denom >= -1e-9) {
          continue;
        }

        const double z_h1 = (obst_h1_m_ - t.z()) / denom;
        const double z_h2 = (obst_h2_m_ - t.z()) / denom;

        if (z_h1 > 0.0 && z_h1 <= max_z) {
          const double mm = z_h1 * 1000.0;
          o1[u] = static_cast<uint16_t>(std::lround(std::clamp(mm, 0.0, 65535.0)));
        }
        if (z_h2 > 0.0 && z_h2 <= max_z) {
          const double mm = z_h2 * 1000.0;
          o2[u] = static_cast<uint16_t>(std::lround(std::clamp(mm, 0.0, 65535.0)));
        }
      }
    }

    ObstacleModel m;
    m.rows = rows;
    m.cols = cols;
    m.fx = fx;
    m.fy = fy;
    m.cx = cx;
    m.cy = cy;
    m.T_base_camera = T;
    m.z_h1_mm = std::move(z1);
    m.z_h2_mm = std::move(z2);
    m.signature = sig;
    model_ = std::move(m);

    RCLCPP_INFO(
      this->get_logger(),
      "Built obstacle geometry model (h1=%.3fm h2=%.3fm max_range=%.2fm).",
      obst_h1_m_, obst_h2_m_, max_range_m_);
    return true;
  }

  std::optional<visualization_msgs::msg::Marker> build_contour_marker(
    const cv::Mat & obstacle_mask,
    const cv::Mat & valid_depth_mask,
    const cv::Mat & depth_mm,
    const std_msgs::msg::Header & header,
    int id,
    float r,
    float g,
    float b)
  {
    if (!model_) {
      return std::nullopt;
    }

    cv::Mat bin;
    cv::threshold(obstacle_mask, bin, 0, 255, cv::THRESH_BINARY);

    const int k = std::max(1, morph_kernel_px_);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);

    // Never invent obstacle pixels without depth.
    cv::bitwise_and(bin, valid_depth_mask, bin);

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
      return std::nullopt;
    }

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

    visualization_msgs::msg::Marker m;
    m.header.stamp = header.stamp;
    m.header.frame_id = base_frame_;
    m.ns = "floor_topology";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.01;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);

    const auto & model = *model_;
    m.points.reserve(approx.size() + 1);
    for (const auto & p : approx) {
      const int u = std::clamp(p.x, 0, depth_mm.cols - 1);
      const int v = std::clamp(p.y, 0, depth_mm.rows - 1);
      const uint16_t D = depth_mm.at<uint16_t>(v, u);
      if (D == 0) {
        continue;
      }

      const double Z = static_cast<double>(D) / 1000.0;
      const double X = (static_cast<double>(u) - model.cx) / model.fx * Z;
      const double Y = (static_cast<double>(v) - model.cy) / model.fy * Z;

      tf2::Vector3 pc(X, Y, Z);
      tf2::Vector3 pb = model.T_base_camera * pc;

      geometry_msgs::msg::Point out;
      out.x = pb.x();
      out.y = pb.y();
      out.z = pb.z();
      m.points.push_back(out);
    }

    if (m.points.size() >= 2) {
      m.points.push_back(m.points.front());
      return m;
    }
    return std::nullopt;
  }

  void publish_topology(
    const cv::Mat & mask_h1,
    const cv::Mat & mask_h2,
    const cv::Mat & depth_mm,
    const std_msgs::msg::Header & header)
  {
    if (!camera_topology_enabled_ || !topology_pub_) {
      return;
    }

    cv::Mat valid(depth_mm.rows, depth_mm.cols, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < depth_mm.rows; ++y) {
      const uint16_t * d = depth_mm.ptr<uint16_t>(y);
      uint8_t * v = valid.ptr<uint8_t>(y);
      for (int x = 0; x < depth_mm.cols; ++x) {
        v[x] = (d[x] > 0) ? 255 : 0;
      }
    }

    cv::Mat not_h2;
    cv::bitwise_not(mask_h2, not_h2);
    cv::Mat h1_only;
    cv::bitwise_and(mask_h1, not_h2, h1_only);

    const auto m1 = build_contour_marker(h1_only, valid, depth_mm, header, 1, 1.0f, 0.6f, 0.0f);
    const auto m2 = build_contour_marker(mask_h2, valid, depth_mm, header, 2, 1.0f, 0.1f, 0.1f);

    visualization_msgs::msg::MarkerArray out;
    std_msgs::msg::Header h;
    h.stamp = header.stamp;
    h.frame_id = base_frame_;
    out.markers.push_back(make_delete_all_marker(h));
    if (m1) {
      out.markers.push_back(*m1);
    }
    if (m2) {
      out.markers.push_back(*m2);
    }
    topology_pub_->publish(out);
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

    if (!ensure_model_ready(*msg)) {
      return;
    }

    cv::Mat depth_mm;
    try {
      depth_mm = convert_depth_to_mm_u16(*msg);
    } catch (const std::exception & e) {
      throttled_error(e.what());
      return;
    }

    const auto & model = *model_;
    cv::Mat mask_h1(depth_mm.rows, depth_mm.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat mask_h2;
    if (camera_topology_enabled_) {
      mask_h2 = cv::Mat(depth_mm.rows, depth_mm.cols, CV_8UC1, cv::Scalar(0));
    }

    for (int y = 0; y < depth_mm.rows; ++y) {
      const uint16_t * d = depth_mm.ptr<uint16_t>(y);
      const uint16_t * z1 = model.z_h1_mm.ptr<uint16_t>(y);
      const uint16_t * z2 = model.z_h2_mm.ptr<uint16_t>(y);
      uint8_t * m1 = mask_h1.ptr<uint8_t>(y);
      uint8_t * m2 = camera_topology_enabled_ ? mask_h2.ptr<uint8_t>(y) : nullptr;
      for (int x = 0; x < depth_mm.cols; ++x) {
        const uint16_t D = d[x];
        if (D == 0) {
          m1[x] = 0;
          if (m2) {
            m2[x] = 0;
          }
          continue;
        }

        const uint16_t Z1 = z1[x];
        if (Z1 == 0) {
          m1[x] = 0;
          if (m2) {
            m2[x] = 0;
          }
          continue;
        }

        const bool is_h1 = D <= Z1;
        m1[x] = is_h1 ? 255 : 0;

        if (m2) {
          const uint16_t Z2 = z2[x];
          const bool is_h2 = (Z2 > 0) && (D <= Z2);
          m2[x] = is_h2 ? 255 : 0;
        }
      }
    }

    mask_pub_->publish(mat_to_mono8_msg(mask_h1, msg->header));
    if (camera_topology_enabled_) {
      publish_topology(mask_h1, mask_h2, depth_mm, msg->header);
    }
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string mask_topic_;
  std::string topology_topic_;

  std::string base_frame_;
  bool camera_topology_enabled_{false};

  double max_range_m_{2.5};
  double obst_h1_m_{0.01};
  double obst_h2_m_{0.07};

  int min_contour_area_px_{500};
  int contour_stride_px_{4};
  int morph_kernel_px_{5};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  std::optional<ObstacleModel> model_;

  std::optional<rclcpp::Time> last_error_time_;
  std::optional<rclcpp::Time> last_warn_time_;

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
