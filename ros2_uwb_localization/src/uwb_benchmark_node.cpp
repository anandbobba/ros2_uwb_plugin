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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ros2_uwb_msgs/msg/uwb_error_diagnostics.hpp"

namespace ros2_uwb_localization
{

/**
 * @brief Node that compares UWB estimation with Ground Truth and analyzes error sources.
 */
class BenchmarkNode : public rclcpp::Node
{
public:
  BenchmarkNode()
  : Node("uwb_benchmark_node")
  {
    this->declare_parameter("gt_topic", "/ground_truth/pose");
    this->declare_parameter("est_topic", "/uwb/pose");
    this->declare_parameter("diag_topic", "/uwb/error_diagnostics");
    this->declare_parameter("log_file", "uwb_benchmark.csv");

    gt_topic_ = this->get_parameter("gt_topic").as_string();
    est_topic_ = this->get_parameter("est_topic").as_string();
    diag_topic_ = this->get_parameter("diag_topic").as_string();
    log_file_path_ = this->get_parameter("log_file").as_string();

    gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      gt_topic_, 10, std::bind(&BenchmarkNode::gt_callback, this, std::placeholders::_1));

    est_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      est_topic_, 10, std::bind(&BenchmarkNode::est_callback, this, std::placeholders::_1));

    diag_sub_ = this->create_subscription<ros2_uwb_msgs::msg::UWBErrorDiagnostics>(
      diag_topic_, 10, std::bind(&BenchmarkNode::diag_callback, this, std::placeholders::_1));

    log_file_.open(log_file_path_);
    log_file_ <<
      "timestamp,error_x,error_y,error_z,error_3d,gaussian,nlos,multipath,drift,error_yaw,"
      "covariance_yaw\n";

    RCLCPP_INFO(
      this->get_logger(), "Benchmarking node started. Logging to %s",
      log_file_path_.c_str());
  }

  void generate_final_report()
  {
    if (errors_.empty()) {
      return;
    }

    double rmse = std::sqrt(
      std::accumulate(
        errors_.begin(), errors_.end(), 0.0,
        [](double a, double b) {return a + b * b;}) / errors_.size());
    double mae = std::accumulate(
      errors_.begin(), errors_.end(), 0.0,
      [](double a, double b) {return a + std::abs(b);}) / errors_.size();
    std::vector<double> sorted_errors = errors_;
    std::sort(sorted_errors.begin(), sorted_errors.end());
    double p95 = sorted_errors[static_cast<size_t>(sorted_errors.size() * 0.95)];

    double yaw_rmse = std::sqrt(
      std::accumulate(
        yaw_errors_.begin(), yaw_errors_.end(), 0.0,
        [](double a, double b) {return a + b * b;}) / yaw_errors_.size());
    double yaw_mae = std::accumulate(
      yaw_errors_.begin(), yaw_errors_.end(), 0.0,
      [](double a, double b) {return a + std::abs(b);}) / yaw_errors_.size();

    std::cout << "\n--- UWB Benchmark Results ---\n";
    std::cout << "  Samples : " << errors_.size() << "\n";
    std::cout << "  Pos RMSE: " << rmse << " m\n";
    std::cout << "  Pos MAE : " << mae << " m\n";
    std::cout << "  Pos P95 : " << p95 << " m\n";
    std::cout << "  Yaw RMSE: " << yaw_rmse * 180.0 / M_PI << " deg\n";
    std::cout << "  Yaw MAE : " << yaw_mae * 180.0 / M_PI << " deg\n";

    if (diag_count_ > 0) {
      double total_abs_err = total_gaussian_ + total_nlos_ + total_multipath_ + total_drift_;
      std::cout << "\n--- Error Attribution --- \n";
      std::cout << "  Gaussian  : " << (total_gaussian_ / total_abs_err) * 100.0 << "%\n";
      std::cout << "  NLOS Bias : " << (total_nlos_ / total_abs_err) * 100.0 << "%\n";
      std::cout << "  Multipath : " << (total_multipath_ / total_abs_err) * 100.0 << "%\n";
      std::cout << "  ClockDrift: " << (total_drift_ / total_abs_err) * 100.0 << "%\n";
      std::cout << "  NLOS Rate : " << (static_cast<double>(nlos_events_) / diag_count_) * 100.0 <<
        "%\n";
      if (multipath_series_.size() > 1) {
        double mean = total_multipath_ / multipath_series_.size();
        double num = 0, den = 0;
        for (size_t i = 0; i < multipath_series_.size() - 1; ++i) {
          num += (multipath_series_[i] - mean) * (multipath_series_[i + 1] - mean);
          den += (multipath_series_[i] - mean) * (multipath_series_[i] - mean);
        }
        std::cout << "  Multipath Correlation (lag-1): " << (den > 0 ? num / den : 0) << "\n";
      }
    }
    std::cout << "-----------------------------\n";
  }

  ~BenchmarkNode() = default;

private:
  void gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_gt_ = msg;
  }

  void est_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!last_gt_) {
      return;
    }

    double dx = msg->pose.pose.position.x - last_gt_->pose.pose.position.x;
    double dy = msg->pose.pose.position.y - last_gt_->pose.pose.position.y;
    double dz = msg->pose.pose.position.z - last_gt_->pose.pose.position.z;
    double d3d = std::sqrt(dx * dx + dy * dy + dz * dz);

    tf2::Quaternion q_gt, q_est;
    tf2::fromMsg(last_gt_->pose.pose.orientation, q_gt);
    tf2::fromMsg(msg->pose.pose.orientation, q_est);
    tf2::Matrix3x3 m_gt(q_gt), m_est(q_est);
    double r1, p1, y_gt, r2, p2, y_est;
    m_gt.getRPY(r1, p1, y_gt);
    m_est.getRPY(r2, p2, y_est);

    double d_yaw = y_est - y_gt;
    while (d_yaw > M_PI) {d_yaw -= 2.0 * M_PI;}
    while (d_yaw < -M_PI) {d_yaw += 2.0 * M_PI;}

    errors_.push_back(d3d);
    yaw_errors_.push_back(d_yaw);
    // We log the latest diagnostic values if they correspond to the same time (approx)
    log_file_ << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ","
              << dx << "," << dy << "," << dz << "," << d3d << ","
              << last_gaussian_ << "," << last_nlos_ << "," << last_multipath_ << "," <<
      last_drift_ << ","
              << d_yaw << "," << msg->pose.covariance[35] << "\n";
  }

  void diag_callback(const ros2_uwb_msgs::msg::UWBErrorDiagnostics::SharedPtr msg)
  {
    last_gaussian_ = std::abs(msg->gaussian_error);
    last_nlos_ = std::abs(msg->nlos_bias);
    last_multipath_ = std::abs(msg->multipath_error);
    last_drift_ = std::abs(msg->clock_drift_error);

    total_gaussian_ += last_gaussian_;
    total_nlos_ += last_nlos_;
    total_multipath_ += last_multipath_;
    total_drift_ += last_drift_;

    if (msg->nlos_bias > 0.1) {
      nlos_events_++;
    }
    multipath_series_.push_back(msg->multipath_error);
    diag_count_++;
  }

  std::string gt_topic_;
  std::string est_topic_;
  std::string diag_topic_;
  std::string log_file_path_;
  std::ofstream log_file_;

  nav_msgs::msg::Odometry::SharedPtr last_gt_;
  std::vector<double> errors_;
  std::vector<double> yaw_errors_;

  // Research stats
  double total_gaussian_ = 0, total_nlos_ = 0, total_multipath_ = 0, total_drift_ = 0;
  double last_gaussian_ = 0, last_nlos_ = 0, last_multipath_ = 0, last_drift_ = 0;
  int nlos_events_ = 0;
  int diag_count_ = 0;
  std::vector<double> multipath_series_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr est_sub_;
  rclcpp::Subscription<ros2_uwb_msgs::msg::UWBErrorDiagnostics>::SharedPtr diag_sub_;
};


}  // namespace ros2_uwb_localization


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_uwb_localization::BenchmarkNode>();
  rclcpp::spin(node);
  node->generate_final_report();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
