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

#ifndef ROS2_UWB_LOCALIZATION__UWB_VISUALIZATION_NODE_HPP_
#define ROS2_UWB_LOCALIZATION__UWB_VISUALIZATION_NODE_HPP_

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ros2_uwb_localization
{

/// @brief RViz visualization of UWB anchor positions and live range circles.
///
/// Publishes MarkerArray for:
///   - Anchor position cubes (color-coded)
///   - Anchor ID text labels
///   - Range measurement circles (when range data is available)
class UWBVisualizationNode : public rclcpp::Node
{
public:
  UWBVisualizationNode();

private:
  struct AnchorVizData
  {
    std::string id;
    Eigen::Vector3d position;
    double last_range = 0.0;
    bool has_range = false;
  };

  void load_anchors_from_params();
  void setup_range_subscriber(const std::string & anchor_id, int index);
  void timer_callback();

  visualization_msgs::msg::Marker make_anchor_marker(
    const AnchorVizData & anchor, int marker_id) const;
  visualization_msgs::msg::Marker make_label_marker(
    const AnchorVizData & anchor, int marker_id) const;
  visualization_msgs::msg::Marker make_range_circle_marker(
    const AnchorVizData & anchor, int marker_id) const;

  std::vector<AnchorVizData> anchors_;
  std::string world_frame_;

  // Appearance params
  double anchor_scale_;
  double anchor_alpha_;
  double label_offset_;
  bool show_range_circles_;
  double range_circle_alpha_;
  int range_circle_segments_;

  // ROS interfaces
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2_uwb_localization

#endif  // ROS2_UWB_LOCALIZATION__UWB_VISUALIZATION_NODE_HPP_
