#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class RoviBaseNode : public rclcpp::Node
{
public:
  explicit RoviBaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rovi_base", options),
    steady_clock_(RCL_STEADY_TIME),
    diag_updater_(this)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    linear_scale_x_ = declare_parameter<double>("linear_scale_x", 1.0);
    linear_scale_y_ = declare_parameter<double>("linear_scale_y", 1.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    const double integrator_period_param = declare_parameter<double>("integrator_period", 0.0);
    drop_warn_factor_ = declare_parameter<double>("drop_warn_factor", 3.0);
    vel_topic_ = declare_parameter<std::string>("vel_topic", "vel_raw");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom_raw");
    diag_period_sec_ = declare_parameter<double>("diagnostics_period", 10.0);

    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate must be > 0, defaulting to 10.0");
      publish_rate_ = 10.0;
    }
    const double derived_period = 1.0 / publish_rate_;
    if (integrator_period_param > 0.0) {
      integrator_period_ = integrator_period_param;
      const double diff = std::fabs(integrator_period_ - derived_period);
      const double tol = 0.1 * derived_period;  // warn if >10% mismatch
      if (diff > tol) {
        RCLCPP_WARN(get_logger(),
          "integrator_period (%.4f) differs from 1/publish_rate (%.4f); verify configs are aligned",
          integrator_period_, derived_period);
      }
    } else {
      integrator_period_ = derived_period;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto vel_qos = rclcpp::SensorDataQoS().keep_last(10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      vel_topic_, vel_qos, std::bind(&RoviBaseNode::handle_velocity, this, _1));

    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, odom_qos);

    if (diag_period_sec_ > 0.0) {
      diag_updater_.setHardwareID("rovi_base");
      diag_updater_.setPeriod(diag_period_sec_);
      diag_updater_.add("timing", this, &RoviBaseNode::produce_diagnostics);
    } else {
      RCLCPP_INFO(get_logger(), "Diagnostics disabled (diagnostics_period <= 0)");
    }

    auto period = std::chrono::duration<double>(integrator_period_);
    integrator_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&RoviBaseNode::integrate_step, this));
  }

private:
  void handle_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = msg->linear.x * linear_scale_x_;
    const double vy = msg->linear.y * linear_scale_y_;
    const double wz = msg->angular.z;
    const rclcpp::Time now_steady = steady_clock_.now();

    if (have_last_time_) {
      const double dt = (now_steady - last_vel_time_).seconds();
      if (dt <= 0.0) {
        dt_nonpositive_count_++;
      } else {
        last_arrival_dt_ = dt;
        if (dt < min_dt_) {
          min_dt_ = dt;
        }
        if (dt > max_dt_observed_) {
          max_dt_observed_ = dt;
        }
      }
    }

    last_vel_time_ = now_steady;
    have_last_time_ = true;
    have_velocity_ = true;
    vx_latest_ = vx;
    vy_latest_ = vy;
    wz_latest_ = wz;
    message_count_++;
  }

  void integrate_step()
  {
    if (!have_velocity_) {
      return;
    }

    const double dt = integrator_period_;

    heading_ += wz_latest_ * dt;
    x_pos_ += (vx_latest_ * std::cos(heading_) - vy_latest_ * std::sin(heading_)) * dt;
    y_pos_ += (vx_latest_ * std::sin(heading_) + vy_latest_ * std::cos(heading_)) * dt;

    const double age = (steady_clock_.now() - last_vel_time_).seconds();
    const double drop_threshold = drop_warn_factor_ * integrator_period_;
    if (age > drop_threshold && !drop_warned_) {
      RCLCPP_WARN(get_logger(),
        "vel_raw stale: age=%.3fs > threshold=%.3fs (missed samples?)", age, drop_threshold);
      drop_warned_ = true;
    } else if (age <= drop_threshold) {
      drop_warned_ = false;
    }

    publish_odom(vx_latest_, vy_latest_, wz_latest_);
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

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (!have_last_time_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No vel_raw messages received");
      return;
    }

    const double age = (steady_clock_.now() - last_vel_time_).seconds();
    const double drop_threshold = drop_warn_factor_ * integrator_period_;
    if (age > drop_threshold) {
      status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Last vel_raw older than %.3fs (threshold %.3fs)", age, drop_threshold);
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Timing OK");
    }

    const double min_dt_val = std::isinf(min_dt_) ? 0.0 : min_dt_;
    status.add("message_count", message_count_);
    status.add("last_arrival_dt", last_arrival_dt_);
    status.add("last_age", age);
    status.add("min_arrival_dt", min_dt_val);
    status.add("max_arrival_dt", max_dt_observed_);
    status.add("dt_nonpositive_count", dt_nonpositive_count_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  diagnostic_updater::Updater diag_updater_;
  rclcpp::TimerBase::SharedPtr integrator_timer_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time last_vel_time_;
  bool have_last_time_{false};
  bool have_velocity_{false};

  double linear_scale_x_{1.0};
  double linear_scale_y_{1.0};
  double publish_rate_{10.0};
  double integrator_period_{0.0};
  double drop_warn_factor_{3.0};
  double diag_period_sec_{10.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string vel_topic_{"vel_raw"};
  std::string odom_topic_{"odom_raw"};
  bool publish_tf_{true};

  double vx_latest_{0.0};
  double vy_latest_{0.0};
  double wz_latest_{0.0};
  double x_pos_{0.0};
  double y_pos_{0.0};
  double heading_{0.0};
  double last_arrival_dt_{0.0};
  uint64_t message_count_{0};
  uint64_t dt_nonpositive_count_{0};
  double min_dt_{std::numeric_limits<double>::infinity()};
  double max_dt_observed_{0.0};
  bool drop_warned_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoviBaseNode>());
  rclcpp::shutdown();
  return 0;
}
