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

//
// Anchor Manager Node
// -------------------
// Loads UWB anchor positions from ROS2 parameters and broadcasts static TF
// frames for each anchor. This enables other nodes (trilateration, visualization)
// to discover anchor positions via TF or parameter sharing.

#include "ros2_uwb_localization/anchor_manager_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace ros2_uwb_localization
{

AnchorManagerNode::AnchorManagerNode()
: Node("anchor_manager")
{
  // Declare world frame parameter
  this->declare_parameter("world_frame", "map");
  world_frame_ = this->get_parameter("world_frame").as_string();

  this->declare_parameter("broadcast_rate", 10.0);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  load_anchors_from_params();
  broadcast_transforms();

  // Re-broadcast periodically for late-joining nodes (sim-clock timer)
  double rate = this->get_parameter("broadcast_rate").as_double();
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration::from_seconds(1.0 / rate),
    std::bind(&AnchorManagerNode::broadcast_transforms, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Anchor Manager initialized with %zu anchors in frame '%s'",
    anchors_.size(), world_frame_.c_str());
}

void AnchorManagerNode::load_anchors_from_params()
{
  // Try to load anchors from parameters. Supports both:
  // 1. Map-based: anchors.anchor_0...
  // 2. Index-based: anchors.0... (more idiomatic for lists)
  
  for (int i = 0; i < 32; ++i) {
    std::string anchor_id = "";
    std::vector<double> pos;

    // Check possible prefixes
    std::vector<std::string> prefixes = {
      "anchors.anchor_" + std::to_string(i),
      "anchors." + std::to_string(i)
    };

    bool found = false;
    for (const auto& prefix : prefixes) {
      std::string id_param = prefix + ".id";
      std::string pos_param = prefix + ".position";

      if (!this->has_parameter(id_param)) {
        this->declare_parameter<std::string>(id_param, "");
      }
      if (!this->has_parameter(pos_param)) {
        this->declare_parameter<std::vector<double>>(pos_param, std::vector<double>{});
      }

      anchor_id = this->get_parameter(id_param).as_string();
      pos = this->get_parameter(pos_param).as_double_array();

      if (!anchor_id.empty() && pos.size() == 3) {
        found = true;
        break;
      }
    }

    if (!found) continue;

    AnchorConfig anchor;
    anchor.id = anchor_id;
    anchor.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    anchors_.push_back(anchor);

    RCLCPP_INFO(
      this->get_logger(),
      "  Anchor '%s': [%.2f, %.2f, %.2f]",
      anchor.id.c_str(), pos[0], pos[1], pos[2]);
  }

  if (anchors_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No anchors loaded! Check anchors.yaml configuration.");
  }
}

void AnchorManagerNode::broadcast_transforms()
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  auto stamp = this->now();

  for (const auto & anchor : anchors_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = world_frame_;
    t.child_frame_id = anchor.id;

    t.transform.translation.x = anchor.position.x();
    t.transform.translation.y = anchor.position.y();
    t.transform.translation.z = anchor.position.z();

    // Anchors have identity rotation
    t.transform.rotation.w = 1.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;

    transforms.push_back(t);
  }

  tf_broadcaster_->sendTransform(transforms);
}

}  // namespace ros2_uwb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_uwb_localization::AnchorManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
