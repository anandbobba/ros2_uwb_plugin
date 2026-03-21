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

#ifndef ROS2_UWB_LOCALIZATION__TRILATERATION_SOLVER_HPP_
#define ROS2_UWB_LOCALIZATION__TRILATERATION_SOLVER_HPP_

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_uwb_msgs/msg/uwb_multi_range.hpp"

namespace ros2_uwb_localization
{

struct Anchor
{
  std::string id;
  Eigen::Vector3d position;
};

/**
 * @brief Standalone trilateration solver node.
 * Subscribes to /uwb/ranges_filtered (UWBMultiRange) and performs Gauss-Newton
 * iterative optimization to find the robot position.
 * Outlier rejection is applied based on solver residuals.
 */
class TrilaterationSolver : public rclcpp::Node
{
public:
  TrilaterationSolver();

private:
  void load_anchors_from_params();
  void multi_range_callback(const ros2_uwb_msgs::msg::UWBMultiRange::SharedPtr msg);

  void solve_trilateration(
    const ros2_uwb_msgs::msg::UWBMultiRange::SharedPtr & msg,
    const std::vector<Anchor> & active_anchors);

  void publish_pose(
    const rclcpp::Time & stamp, const Eigen::Vector3d & pos,
    const Eigen::Matrix3d & cov);

  // Anchor configuration
  std::map<std::string, Anchor> anchors_;

  // ROS interfaces
  rclcpp::Subscription<ros2_uwb_msgs::msg::UWBMultiRange>::SharedPtr multi_range_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // Solver state
  Eigen::Vector3d current_estimate_;

  // Parameters
  int optimization_iterations_;
  double convergence_threshold_;
  double range_std_dev_;
  double outlier_threshold_;
  int outlier_iterations_;
  int min_anchors_;
  std::string output_frame_;
  bool two_d_mode_;     // If true, solve for x,y only (z fixed to tag_height_)
  double tag_height_;   // Fixed z of the tag in 2D mode (m)
};

}  // namespace ros2_uwb_localization

#endif  // ROS2_UWB_LOCALIZATION__TRILATERATION_SOLVER_HPP_
