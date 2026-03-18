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
// UWB Visualization Node
// ----------------------
// Publishes RViz MarkerArray messages for:
//   1. Anchor position cubes (color-coded per anchor)
//   2. Anchor ID text labels
//   3. Range measurement circles (real-time range visualization)
//
// Anchors are loaded from the same shared parameters as the other nodes.

#include "ros2_uwb_localization/uwb_visualization_node.hpp"

#include <chrono>
#include <cmath>

namespace ros2_uwb_localization
{

// Predefined anchor colors (RGBA)
static const std::array<std::array<float, 4>, 8> ANCHOR_COLORS = {{
  {1.0f, 0.2f, 0.2f, 1.0f},     // Red
  {0.2f, 0.8f, 0.2f, 1.0f},     // Green
  {0.2f, 0.4f, 1.0f, 1.0f},     // Blue
  {1.0f, 0.8f, 0.0f, 1.0f},     // Yellow
  {0.8f, 0.2f, 0.8f, 1.0f},     // Magenta
  {0.0f, 0.9f, 0.9f, 1.0f},     // Cyan
  {1.0f, 0.5f, 0.0f, 1.0f},     // Orange
  {0.6f, 0.6f, 0.6f, 1.0f},     // Gray
}};

UWBVisualizationNode::UWBVisualizationNode()
: Node("uwb_visualization")
{
  // Declare parameters
  this->declare_parameter("world_frame", "map");
  this->declare_parameter("anchor_marker_scale", 0.3);
  this->declare_parameter("anchor_marker_alpha", 0.9);
  this->declare_parameter("anchor_label_offset", 0.5);
  this->declare_parameter("show_range_circles", true);
  this->declare_parameter("range_circle_alpha", 0.15);
  this->declare_parameter("range_circle_segments", 64);
  this->declare_parameter("publish_rate", 5.0);

  world_frame_ = this->get_parameter("world_frame").as_string();
  anchor_scale_ = this->get_parameter("anchor_marker_scale").as_double();
  anchor_alpha_ = this->get_parameter("anchor_marker_alpha").as_double();
  label_offset_ = this->get_parameter("anchor_label_offset").as_double();
  show_range_circles_ = this->get_parameter("show_range_circles").as_bool();
  range_circle_alpha_ = this->get_parameter("range_circle_alpha").as_double();
  range_circle_segments_ = this->get_parameter("range_circle_segments").as_int();

  load_anchors_from_params();

  // Publisher
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/uwb/markers", 10);

  // Timer — sim-clock timer so marker stamps match Gazebo time
  double rate = this->get_parameter("publish_rate").as_double();
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration::from_seconds(1.0 / rate),
    std::bind(&UWBVisualizationNode::timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Visualization node initialized with %zu anchors", anchors_.size());
}

void UWBVisualizationNode::load_anchors_from_params()
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

    AnchorVizData a;
    a.id = anchor_id;
    a.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    anchors_.push_back(a);

    setup_range_subscriber(anchor_id, i);
  }
}

void UWBVisualizationNode::setup_range_subscriber(
  const std::string & anchor_id, int index)
{
  std::string topic = "/uwb/range_" + std::to_string(index);
  size_t anchor_idx = static_cast<size_t>(index);

  range_subs_[anchor_id] = this->create_subscription<sensor_msgs::msg::Range>(
    topic, rclcpp::SensorDataQoS(),
    [this, anchor_idx](const sensor_msgs::msg::Range::SharedPtr msg) {
      if (anchor_idx < anchors_.size()) {
        anchors_[anchor_idx].last_range = msg->range;
        anchors_[anchor_idx].has_range = true;
      }
    });
}

void UWBVisualizationNode::timer_callback()
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (size_t i = 0; i < anchors_.size(); ++i) {
    // Anchor cube marker
    marker_array.markers.push_back(
      make_anchor_marker(anchors_[i], marker_id++));

    // Text label
    marker_array.markers.push_back(
      make_label_marker(anchors_[i], marker_id++));

    // Range circle (if enabled and data available)
    if (show_range_circles_ && anchors_[i].has_range) {
      marker_array.markers.push_back(
        make_range_circle_marker(anchors_[i], marker_id++));
    }
  }

  marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker UWBVisualizationNode::make_anchor_marker(
  const AnchorVizData & anchor, int marker_id) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = world_frame_;
  m.header.stamp = this->now();
  m.ns = "uwb_anchors";
  m.id = marker_id;
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.pose.position.x = anchor.position.x();
  m.pose.position.y = anchor.position.y();
  m.pose.position.z = anchor.position.z();
  m.pose.orientation.w = 1.0;

  m.scale.x = anchor_scale_;
  m.scale.y = anchor_scale_;
  m.scale.z = anchor_scale_;

  // Color per anchor
  size_t idx = static_cast<size_t>(&anchor - anchors_.data());
  const auto & c = ANCHOR_COLORS[idx % ANCHOR_COLORS.size()];
  m.color.r = c[0];
  m.color.g = c[1];
  m.color.b = c[2];
  m.color.a = static_cast<float>(anchor_alpha_);

  m.lifetime = rclcpp::Duration::from_seconds(0);    // Persistent
  return m;
}

visualization_msgs::msg::Marker UWBVisualizationNode::make_label_marker(
  const AnchorVizData & anchor, int marker_id) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = world_frame_;
  m.header.stamp = this->now();
  m.ns = "uwb_labels";
  m.id = marker_id;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.pose.position.x = anchor.position.x();
  m.pose.position.y = anchor.position.y();
  m.pose.position.z = anchor.position.z() + label_offset_;
  m.pose.orientation.w = 1.0;

  m.scale.z = 0.25;    // Text height

  // Display anchor ID and range if available
  if (anchor.has_range) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s\n%.2fm", anchor.id.c_str(), anchor.last_range);
    m.text = buf;
  } else {
    m.text = anchor.id;
  }

  m.color.r = 1.0f;
  m.color.g = 1.0f;
  m.color.b = 1.0f;
  m.color.a = 1.0f;

  m.lifetime = rclcpp::Duration::from_seconds(0);
  return m;
}

visualization_msgs::msg::Marker UWBVisualizationNode::make_range_circle_marker(
  const AnchorVizData & anchor, int marker_id) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = world_frame_;
  m.header.stamp = this->now();
  m.ns = "uwb_ranges";
  m.id = marker_id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.pose.orientation.w = 1.0;

  m.scale.x = 0.02;    // Line width

  // Color per anchor (semi-transparent)
  size_t idx = static_cast<size_t>(&anchor - anchors_.data());
  const auto & c = ANCHOR_COLORS[idx % ANCHOR_COLORS.size()];
  m.color.r = c[0];
  m.color.g = c[1];
  m.color.b = c[2];
  m.color.a = static_cast<float>(range_circle_alpha_);

  // Generate circle points at anchor height projected to ground plane
  double r = anchor.last_range;
  double z = anchor.position.z();

  for (int s = 0; s <= range_circle_segments_; ++s) {
    double theta = 2.0 * M_PI * static_cast<double>(s) /
      static_cast<double>(range_circle_segments_);
    geometry_msgs::msg::Point pt;
    pt.x = anchor.position.x() + r * std::cos(theta);
    pt.y = anchor.position.y() + r * std::sin(theta);
    pt.z = z;
    m.points.push_back(pt);
  }

  m.lifetime = rclcpp::Duration::from_seconds(0.5);    // Fade when stale
  return m;
}

}  // namespace ros2_uwb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_uwb_localization::UWBVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
