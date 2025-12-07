#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class RoviBaseNode : public rclcpp::Node
{
public:
  explicit RoviBaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rovi_base", options),
    steady_clock_(RCL_STEADY_TIME)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    linear_scale_x_ = declare_parameter<double>("linear_scale_x", 1.0);
    linear_scale_y_ = declare_parameter<double>("linear_scale_y", 1.0);
    max_dt_ = declare_parameter<double>("max_dt", 0.5);
    vel_topic_ = declare_parameter<std::string>("vel_topic", "vel_raw");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom_raw");

    if (max_dt_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "max_dt must be > 0, defaulting to 0.5");
      max_dt_ = 0.5;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto vel_qos = rclcpp::SensorDataQoS().keep_last(10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      vel_topic_, vel_qos, std::bind(&RoviBaseNode::handle_velocity, this, _1));

    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, odom_qos);
  }

private:
  void handle_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = msg->linear.x * linear_scale_x_;
    const double vy = msg->linear.y * linear_scale_y_;
    const double wz = msg->angular.z;
    const rclcpp::Time now_steady = steady_clock_.now();

    if (!have_last_time_) {
      last_vel_time_ = now_steady;  // seed timing on the first message
      have_last_time_ = true;
      publish_odom(vx, vy, wz);
      return;
    }

    double dt = (now_steady - last_vel_time_).seconds();
    last_vel_time_ = now_steady;

    if (dt <= 0.0) {
      return;
    }
    if (dt > max_dt_) {
      dt = max_dt_;
    }

    const double delta_heading = wz * dt;
    const double delta_x = (vx * std::cos(heading_) - vy * std::sin(heading_)) * dt;
    const double delta_y = (vx * std::sin(heading_) + vy * std::cos(heading_)) * dt;

    heading_ += delta_heading;
    x_pos_ += delta_x;
    y_pos_ += delta_y;

    publish_odom(vx, vy, wz);
  }

  void publish_odom(double vx, double vy, double wz)
  {
    const rclcpp::Time stamp = get_clock()->now();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = wz;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_->publish(odom);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x_pos_;
    t.transform.translation.y = y_pos_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time last_vel_time_;
  bool have_last_time_{false};

  double linear_scale_x_{1.0};
  double linear_scale_y_{1.0};
  double max_dt_{0.5};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string vel_topic_{"vel_raw"};
  std::string odom_topic_{"odom_raw"};
  bool publish_tf_{true};

  double x_pos_{0.0};
  double y_pos_{0.0};
  double heading_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoviBaseNode>());
  rclcpp::shutdown();
  return 0;
}
