// Copyright 2026 Anand Bobba
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_uwb_localization
{

double normalize_angle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to - from);
}

class YawEstimatorNode : public rclcpp::Node
{
public:
  YawEstimatorNode()
  : Node("uwb_yaw_estimator"), smoothed_yaw_(0.0), first_measurement_(true)
  {
    this->declare_parameter("baseline_expected", 0.7);
    this->declare_parameter("baseline_tolerance", 0.2);
    this->declare_parameter("smoothing_alpha", 0.8);
    this->declare_parameter("use_imu_fusion", false);
    this->declare_parameter("imu_alpha", 0.92);

    baseline_expected_ = this->get_parameter("baseline_expected").as_double();
    baseline_tolerance_ = this->get_parameter("baseline_tolerance").as_double();
    smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();
    use_imu_fusion_ = this->get_parameter("use_imu_fusion").as_bool();
    imu_alpha_ = this->get_parameter("imu_alpha").as_double();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/pose",
      10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/uwb/odom", 10);

    front_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/front/pose", 10,
      std::bind(&YawEstimatorNode::front_callback, this, std::placeholders::_1));

    rear_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/rear/pose", 10,
      std::bind(&YawEstimatorNode::rear_callback, this, std::placeholders::_1));

    if (use_imu_fusion_) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&YawEstimatorNode::imu_callback, this, std::placeholders::_1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&YawEstimatorNode::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(), "UWB Yaw Estimator initialized. IMU Fusion: %s",
      use_imu_fusion_ ? "ENABLED" : "DISABLED");
  }

private:
  void front_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    front_msg_ = msg;
    front_time_ = this->now();
  }

  void rear_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    rear_msg_ = msg;
    rear_time_ = this->now();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_msg_ = msg;
    imu_time_ = this->now();
  }

  void timer_callback()
  {
    rclcpp::Time now = this->now();

    bool front_valid = front_msg_ && (now - front_time_).seconds() < 0.5;
    bool rear_valid = rear_msg_ && (now - rear_time_).seconds() < 0.5;
    bool imu_valid = use_imu_fusion_ && imu_msg_ && (now - imu_time_).seconds() < 0.5;

    double current_yaw = last_valid_yaw_;
    double current_cov = last_valid_cov_;

    geometry_msgs::msg::PoseWithCovarianceStamped out_msg;

    // Choose base header
    if (front_valid) {out_msg.header = front_msg_->header;} else if (rear_valid) {
      out_msg.header = rear_msg_->header;
    } else if (imu_valid) {out_msg.header = imu_msg_->header;} else {
      out_msg.header.stamp = now;
      out_msg.header.frame_id = "map";
    }

    if (front_valid && rear_valid) {
      double dx = front_msg_->pose.pose.position.x - rear_msg_->pose.pose.position.x;
      double dy = front_msg_->pose.pose.position.y - rear_msg_->pose.pose.position.y;
      double measured_baseline = std::sqrt(dx * dx + dy * dy);

      // Calculate confidence and covariance
      double confidence = std::clamp(
        1.0 - std::abs(
          measured_baseline - baseline_expected_) / baseline_expected_, 0.0, 1.0);
      double uwb_cov = 0.01 + (1.0 - confidence) * 100.0;

      double raw_uwb_yaw = std::atan2(dy, dx);

      // EMA smooth the UWB yaw
      if (first_measurement_) {
        smoothed_yaw_ = raw_uwb_yaw;
        first_measurement_ = false;
      } else {
        double current_sin = std::sin(smoothed_yaw_);
        double current_cos = std::cos(smoothed_yaw_);
        double raw_sin = std::sin(raw_uwb_yaw);
        double raw_cos = std::cos(raw_uwb_yaw);

        double new_sin = smoothing_alpha_ * raw_sin + (1.0 - smoothing_alpha_) * current_sin;
        double new_cos = smoothing_alpha_ * raw_cos + (1.0 - smoothing_alpha_) * current_cos;

        smoothed_yaw_ = std::atan2(new_sin, new_cos);
      }

      if (imu_valid) {
        // IMU Fusion
        tf2::Quaternion q_imu;
        tf2::fromMsg(imu_msg_->orientation, q_imu);
        tf2::Matrix3x3 m_imu(q_imu);
        double r, p, imu_yaw;
        m_imu.getRPY(r, p, imu_yaw);

        // yaw_fused = alpha * yaw_imu + (1-alpha) * yaw_uwb
        // Safely handled via shortest angular distance
        current_yaw =
          normalize_angle(
          smoothed_yaw_ + imu_alpha_ *
          shortest_angular_distance(smoothed_yaw_, imu_yaw));
        current_cov = uwb_cov * (1.0 - imu_alpha_);  // Decrease covariance when IMU is fused
      } else {
        current_yaw = smoothed_yaw_;
        current_cov = uwb_cov;
      }

      last_valid_yaw_ = current_yaw;
      last_valid_cov_ = current_cov;

      // Center position
      out_msg.pose.pose.position.x =
        (front_msg_->pose.pose.position.x + rear_msg_->pose.pose.position.x) / 2.0;
      out_msg.pose.pose.position.y =
        (front_msg_->pose.pose.position.y + rear_msg_->pose.pose.position.y) / 2.0;
      out_msg.pose.pose.position.z =
        (front_msg_->pose.pose.position.z + rear_msg_->pose.pose.position.z) / 2.0;

      // Average position covariance
      for (int i = 0; i < 36; ++i) {
        out_msg.pose.covariance[i] =
          (front_msg_->pose.covariance[i] + rear_msg_->pose.covariance[i]) / 2.0;
      }

    } else if (front_valid || rear_valid) {
      // One tag dropped - hold previous yaw, inflate covariance
      current_cov += 1.0 * 0.05;  // inflate by 1 rad^2 per second (at 20Hz)
      last_valid_cov_ = current_cov;

      // Use the valid tag's position as an approximation of the robot position
      // (though shifted by half baseline)
      auto valid_msg = front_valid ? front_msg_ : rear_msg_;
      out_msg.pose.pose.position = valid_msg->pose.pose.position;
      out_msg.pose.covariance = valid_msg->pose.covariance;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "One UWB tag dropped! Holding yaw, inflating covariance.");
    } else {
      // Both tags dropped
      if (imu_valid) {
        tf2::Quaternion q_imu;
        tf2::fromMsg(imu_msg_->orientation, q_imu);
        tf2::Matrix3x3 m_imu(q_imu);
        double r, p, imu_yaw;
        m_imu.getRPY(r, p, imu_yaw);

        current_yaw = imu_yaw;
        current_cov = 0.05;  // Trust IMU covariance

        // We have no position estimate from UWB, so we should skip publishing or
        // publish with huge pos covariance. We will skip publishing UWB pose if we
        // don't have position. The EKF needs position from UWB.
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Both UWB tags invalid! Cannot provide pose update.");
        return;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Both UWB tags invalid AND no IMU. Waiting for data...");
        return;
      }
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw);
    out_msg.pose.pose.orientation = tf2::toMsg(q);
    out_msg.pose.covariance[35] = current_cov;

    pose_pub_->publish(out_msg);

    // Optional Odometry output
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header = out_msg.header;
    odom_msg.child_frame_id = "uwb_base";
    odom_msg.pose = out_msg.pose;
    odom_pub_->publish(odom_msg);
  }

  double baseline_expected_;
  double baseline_tolerance_;
  double smoothing_alpha_;
  bool use_imu_fusion_;
  double imu_alpha_;

  double smoothed_yaw_;
  bool first_measurement_;

  double last_valid_yaw_{0.0};
  double last_valid_cov_{100.0};

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr front_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rear_msg_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;

  rclcpp::Time front_time_;
  rclcpp::Time rear_time_;
  rclcpp::Time imu_time_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr front_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rear_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2_uwb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_uwb_localization::YawEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
