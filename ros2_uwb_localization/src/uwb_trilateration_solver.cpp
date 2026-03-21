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
#include <chrono>
#include <cmath>

#include "ros2_uwb_localization/trilateration_solver.hpp"

namespace ros2_uwb_localization
{

TrilaterationSolver::TrilaterationSolver()
: Node("uwb_trilateration_solver")
{
  this->declare_parameter("output_frame", "map");
  this->declare_parameter("min_anchors", 3);
  this->declare_parameter("optimization_iterations", 15);
  this->declare_parameter("convergence_threshold", 1.0e-5);
  this->declare_parameter("range_std_dev", 0.1);
  this->declare_parameter("outlier_threshold", 2.0);
  this->declare_parameter("outlier_iterations", 2);
  this->declare_parameter("two_d_mode", true);   // Constrain z when anchors are coplanar
  this->declare_parameter("tag_height", 0.35);   // Height of the UWB tag on the robot (m)

  output_frame_ = this->get_parameter("output_frame").as_string();
  min_anchors_ = this->get_parameter("min_anchors").as_int();
  optimization_iterations_ = this->get_parameter("optimization_iterations").as_int();
  convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
  range_std_dev_ = this->get_parameter("range_std_dev").as_double();
  outlier_threshold_ = this->get_parameter("outlier_threshold").as_double();
  outlier_iterations_ = this->get_parameter("outlier_iterations").as_int();
  two_d_mode_ = this->get_parameter("two_d_mode").as_bool();
  tag_height_ = this->get_parameter("tag_height").as_double();

  if (two_d_mode_) {
    RCLCPP_INFO(
      this->get_logger(),
      "2D mode enabled: z constrained to %.3f m", tag_height_);
  }

  load_anchors_from_params();

  // Integrated Multi-Range subscription
  multi_range_sub_ = this->create_subscription<ros2_uwb_msgs::msg::UWBMultiRange>(
    "/uwb/ranges_filtered", 10,
    std::bind(&TrilaterationSolver::multi_range_callback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/uwb/pose", 10);

  current_estimate_ = Eigen::Vector3d::Zero();

  // Dynamic parameter callback
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        if (param.get_name() == "min_anchors") {
          min_anchors_ = param.as_int();
        } else if (param.get_name() == "optimization_iterations") {
          optimization_iterations_ = param.as_int();
        } else if (param.get_name() == "convergence_threshold") {
          convergence_threshold_ = param.as_double();
        } else if (param.get_name() == "range_std_dev") {
          range_std_dev_ = param.as_double();
        } else if (param.get_name() == "outlier_threshold") {
          outlier_threshold_ = param.as_double();
        } else if (param.get_name() == "outlier_iterations") {
          outlier_iterations_ = param.as_int();
        }
      }
      return result;
    });

  RCLCPP_INFO(this->get_logger(), "UWB Trilateration Solver initialized");
}

void TrilaterationSolver::load_anchors_from_params()
{
  for (int i = 0; i < 32; ++i) {
    std::string prefix = "anchors.anchor_" + std::to_string(i);
    std::string id_param = prefix + ".id";
    std::string pos_param = prefix + ".position";

    if (!this->has_parameter(id_param)) {
      this->declare_parameter<std::string>(id_param, "");
    }
    if (!this->has_parameter(pos_param)) {
      this->declare_parameter<std::vector<double>>(pos_param, std::vector<double>{});
    }

    std::string anchor_id = this->get_parameter(id_param).as_string();
    std::vector<double> pos = this->get_parameter(pos_param).as_double_array();

    if (anchor_id.empty() || pos.size() != 3) {
      break;
    }

    Anchor anchor;
    anchor.id = anchor_id;
    anchor.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    anchors_[anchor_id] = anchor;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %zu UWB anchors from parameters", anchors_.size());
}

void TrilaterationSolver::multi_range_callback(
  const ros2_uwb_msgs::msg::UWBMultiRange::SharedPtr msg)
{
  std::vector<Anchor> active;
  for (const auto & r : msg->ranges) {
    auto it = anchors_.find(r.anchor_id);
    if (it != anchors_.end()) {
      // Create a temporary "active" copy with the specific range value for this solve
      Anchor a = it->second;
      // We pass the measurement values to the solver inner loop
      active.push_back(a);
    }
  }

  if (static_cast<int>(active.size()) < min_anchors_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Insufficient active anchors (%zu/%d). Localization paused.",
      active.size(), min_anchors_);
    return;
  }

  RCLCPP_INFO_ONCE(
    this->get_logger(), "Solver receiving ranges for %zu anchors. Localization active.",
    active.size());
  solve_trilateration(msg, active);
}

void TrilaterationSolver::solve_trilateration(
  const ros2_uwb_msgs::msg::UWBMultiRange::SharedPtr & msg,
  const std::vector<Anchor> & active)
{
  const size_t n = active.size();
  std::vector<bool> is_outlier(n, false);

  if (two_d_mode_) {
    // ─── 2D Gauss-Newton: solve for x, y only; z = tag_height_ ───
    Eigen::Vector2d xy(current_estimate_.x(), current_estimate_.y());
    double z_fixed = tag_height_;
    Eigen::Matrix2d H2 = Eigen::Matrix2d::Zero();

    for (int iter = 0; iter < optimization_iterations_; ++iter) {
      int valid_count = 0;
      for (size_t j = 0; j < n; ++j) {
        if (!is_outlier[j]) {
          valid_count++;
        }
      }
      if (valid_count < min_anchors_) {return;}

      Eigen::MatrixXd J(valid_count, 2);
      Eigen::VectorXd r(valid_count);
      int row = 0;
      for (size_t j = 0; j < n; ++j) {
        if (is_outlier[j]) {continue;}
        Eigen::Vector3d diff(
          xy.x() - active[j].position.x(),
          xy.y() - active[j].position.y(),
          z_fixed - active[j].position.z());
        double d_est = diff.norm();
        if (d_est < 1e-6) {d_est = 1e-6;}
        J(row, 0) = diff.x() / d_est;
        J(row, 1) = diff.y() / d_est;
        r(row) = msg->ranges[j].range - d_est;
        row++;
      }

      Eigen::Matrix2d JtJ = J.transpose() * J;
      Eigen::Vector2d dxy = JtJ.ldlt().solve(J.transpose() * r);
      xy += dxy;

      if (iter < outlier_iterations_) {
        for (size_t j = 0; j < n; ++j) {
          if (is_outlier[j]) {continue;}
          Eigen::Vector3d diff(
            xy.x() - active[j].position.x(),
            xy.y() - active[j].position.y(),
            z_fixed - active[j].position.z());
          double residual = std::abs(msg->ranges[j].range - diff.norm());
          if (residual > outlier_threshold_) {
            is_outlier[j] = true;
          }
        }
      }

      if (dxy.norm() < convergence_threshold_) {
        H2 = JtJ;
        break;
      }
      if (iter == optimization_iterations_ - 1) {
        H2 = JtJ;
      }
    }

    current_estimate_ = Eigen::Vector3d(xy.x(), xy.y(), z_fixed);
    if (std::abs(H2.determinant()) < 1e-10) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "2D solver: near-singular geometry");
      return;
    }
    Eigen::Matrix2d cov2 = std::pow(range_std_dev_, 2) * H2.inverse();

    // Build 3x3 covariance with very high z uncertainty
    Eigen::Matrix3d cov3 = Eigen::Matrix3d::Zero();
    cov3(0, 0) = cov2(0, 0);
    cov3(0, 1) = cov2(0, 1);
    cov3(1, 0) = cov2(1, 0);
    cov3(1, 1) = cov2(1, 1);
    cov3(2, 2) = 1e6;  // z is fixed, not estimated
    publish_pose(msg->header.stamp, current_estimate_, cov3);

  } else {
    // ─── 3D Gauss-Newton (original implementation) ───
    Eigen::Vector3d x = current_estimate_;
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    for (int iter = 0; iter < optimization_iterations_; ++iter) {
      int valid_count = 0;
      for (size_t j = 0; j < n; ++j) {
        if (!is_outlier[j]) {
          valid_count++;
        }
      }
      if (valid_count < min_anchors_) {return;}

      Eigen::MatrixXd J(valid_count, 3);
      Eigen::VectorXd r(valid_count);
      int row = 0;
      for (size_t j = 0; j < n; ++j) {
        if (is_outlier[j]) {continue;}
        Eigen::Vector3d diff = x - active[j].position;
        double d_est = diff.norm();
        if (d_est < 1e-6) {d_est = 1e-6;}
        J.row(row) = diff.transpose() / d_est;
        r(row) = msg->ranges[j].range - d_est;
        row++;
      }

      Eigen::Matrix3d JtJ = J.transpose() * J;
      Eigen::Vector3d dx = JtJ.ldlt().solve(J.transpose() * r);
      x += dx;

      if (iter < outlier_iterations_) {
        for (size_t j = 0; j < n; ++j) {
          if (is_outlier[j]) {continue;}
          double residual = std::abs(msg->ranges[j].range - (x - active[j].position).norm());
          if (residual > outlier_threshold_) {
            is_outlier[j] = true;
          }
        }
      }

      if (dx.norm() < convergence_threshold_) {
        H = JtJ;
        break;
      }
      if (iter == optimization_iterations_ - 1) {
        H = JtJ;
      }
    }

    current_estimate_ = x;
    if (std::abs(H.determinant()) < 1e-10) {return;}
    Eigen::Matrix3d cov_pos = std::pow(range_std_dev_, 2) * H.inverse();
    publish_pose(msg->header.stamp, x, cov_pos);
  }
}

void TrilaterationSolver::publish_pose(
  const rclcpp::Time & stamp, const Eigen::Vector3d & pos, const Eigen::Matrix3d & cov)
{
  auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  msg.header.stamp = stamp;
  msg.header.frame_id = output_frame_;

  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();
  msg.pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg.pose.covariance[static_cast<size_t>(i * 6 + j)] = cov(i, j);
    }
  }
  for (int i = 3; i < 6; ++i) {
    msg.pose.covariance[static_cast<size_t>(i * 6 + i)] = 1e6;
  }

  pose_pub_->publish(msg);
}

}  // namespace ros2_uwb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<ros2_uwb_localization::TrilaterationSolver>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
