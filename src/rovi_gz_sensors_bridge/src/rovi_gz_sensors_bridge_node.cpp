#include <string>

#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <ros_gz_bridge/convert.hpp>

namespace
{

bool has_leading_slash(const std::string & value)
{
  return !value.empty() && value.front() == '/';
}

}  // namespace

class RoviGzSensorsBridgeNode final : public rclcpp::Node
{
public:
  RoviGzSensorsBridgeNode()
  : rclcpp::Node("rovi_gz_sensors_bridge")
  {
    // Standard ROS param (used implicitly by node clock).
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter("use_sim_time", false);
    }

    gz_scan_topic_ = this->declare_parameter("gz_scan_topic", "/rovi/scan");
    gz_imu_topic_ = this->declare_parameter("gz_imu_topic", "/rovi/imu");
    gz_color_topic_ = this->declare_parameter("gz_color_image_topic", "/rovi/camera/color/image");
    gz_depth_topic_ = this->declare_parameter("gz_depth_image_topic", "/rovi/camera/depth/image");

    ros_scan_topic_ = this->declare_parameter("ros_scan_topic", "/scan");
    ros_imu_topic_ = this->declare_parameter("ros_imu_topic", "/imu/data_raw");
    ros_color_topic_ = this->declare_parameter("ros_color_image_topic", "/camera/color/image");
    ros_depth_topic_ = this->declare_parameter("ros_depth_image_topic", "/camera/depth/image");

    scan_frame_id_ = this->declare_parameter("scan_frame_id", "laser_link");
    imu_frame_id_ = this->declare_parameter("imu_frame_id", "imu_link");
    color_frame_id_ = this->declare_parameter("color_frame_id", "camera_color_optical_frame");
    depth_frame_id_ = this->declare_parameter("depth_frame_id", "camera_depth_optical_frame");

    if (!has_leading_slash(ros_scan_topic_) || !has_leading_slash(ros_imu_topic_) ||
      !has_leading_slash(ros_color_topic_) || !has_leading_slash(ros_depth_topic_))
    {
      throw std::runtime_error("ROS topic parameters must be absolute (start with '/')");
    }

    auto qos = rclcpp::SensorDataQoS();
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(ros_scan_topic_, qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(ros_imu_topic_, qos);
    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_color_topic_, qos);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_depth_topic_, qos);

    if (!gz_node_.Subscribe(gz_scan_topic_, &RoviGzSensorsBridgeNode::on_scan, this)) {
      throw std::runtime_error("Failed to subscribe to gz_scan_topic: " + gz_scan_topic_);
    }
    if (!gz_node_.Subscribe(gz_imu_topic_, &RoviGzSensorsBridgeNode::on_imu, this)) {
      throw std::runtime_error("Failed to subscribe to gz_imu_topic: " + gz_imu_topic_);
    }
    if (!gz_node_.Subscribe(gz_color_topic_, &RoviGzSensorsBridgeNode::on_color_image, this)) {
      throw std::runtime_error("Failed to subscribe to gz_color_image_topic: " + gz_color_topic_);
    }
    if (!gz_node_.Subscribe(gz_depth_topic_, &RoviGzSensorsBridgeNode::on_depth_image, this)) {
      throw std::runtime_error("Failed to subscribe to gz_depth_image_topic: " + gz_depth_topic_);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Bridging Gazebo sensors to ROS:\n"
      "  - %s -> %s (frame_id=%s)\n"
      "  - %s -> %s (frame_id=%s)\n"
      "  - %s -> %s (frame_id=%s)\n"
      "  - %s -> %s (frame_id=%s)",
      gz_scan_topic_.c_str(), ros_scan_topic_.c_str(), scan_frame_id_.c_str(),
      gz_imu_topic_.c_str(), ros_imu_topic_.c_str(), imu_frame_id_.c_str(),
      gz_color_topic_.c_str(), ros_color_topic_.c_str(), color_frame_id_.c_str(),
      gz_depth_topic_.c_str(), ros_depth_topic_.c_str(), depth_frame_id_.c_str());
  }

private:
  void on_scan(const gz::msgs::LaserScan & gz_msg)
  {
    sensor_msgs::msg::LaserScan ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_msg.header.frame_id = scan_frame_id_;
    scan_pub_->publish(ros_msg);
  }

  void on_imu(const gz::msgs::IMU & gz_msg)
  {
    sensor_msgs::msg::Imu ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_msg.header.frame_id = imu_frame_id_;
    imu_pub_->publish(ros_msg);
  }

  void on_color_image(const gz::msgs::Image & gz_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_msg.header.frame_id = color_frame_id_;
    color_pub_->publish(ros_msg);
  }

  void on_depth_image(const gz::msgs::Image & gz_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_msg.header.frame_id = depth_frame_id_;
    depth_pub_->publish(ros_msg);
  }

  std::string gz_scan_topic_;
  std::string gz_imu_topic_;
  std::string gz_color_topic_;
  std::string gz_depth_topic_;

  std::string ros_scan_topic_;
  std::string ros_imu_topic_;
  std::string ros_color_topic_;
  std::string ros_depth_topic_;

  std::string scan_frame_id_;
  std::string imu_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_;

  gz::transport::Node gz_node_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RoviGzSensorsBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "[rovi_gz_sensors_bridge] %s\n", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
