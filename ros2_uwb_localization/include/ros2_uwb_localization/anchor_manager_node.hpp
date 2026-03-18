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

#ifndef ROS2_UWB_LOCALIZATION__ANCHOR_MANAGER_NODE_HPP_
#define ROS2_UWB_LOCALIZATION__ANCHOR_MANAGER_NODE_HPP_

#include <Eigen/Dense>
#include <tf2_ros/static_transform_broadcaster.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_uwb_localization
{

struct AnchorConfig
{
  std::string id;
  Eigen::Vector3d position;
};

/// @brief Manages UWB anchor positions and broadcasts their TF frames.
///
/// Loads anchor definitions from ROS2 parameters (set via anchors.yaml),
/// broadcasts static transforms for each anchor, and provides anchor
/// position data to other nodes via a shared-memory-style parameter server.
class AnchorManagerNode : public rclcpp::Node
{
public:
  AnchorManagerNode();

  /// @brief Returns loaded anchor configurations (for testing / composition).
  const std::vector<AnchorConfig> & get_anchors() const {return anchors_;}

private:
  void load_anchors_from_params();
  void broadcast_transforms();

  std::vector<AnchorConfig> anchors_;
  std::string world_frame_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2_uwb_localization

#endif  // ROS2_UWB_LOCALIZATION__ANCHOR_MANAGER_NODE_HPP_
