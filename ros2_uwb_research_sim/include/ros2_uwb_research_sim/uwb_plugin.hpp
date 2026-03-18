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

#ifndef ROS2_UWB_RESEARCH_SIM__UWB_PLUGIN_HPP_
#define ROS2_UWB_RESEARCH_SIM__UWB_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include <ignition/gazebo/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "ros2_uwb_msgs/msg/uwb_range.hpp"
#include "ros2_uwb_msgs/msg/uwb_error_diagnostics.hpp"
#include "ros2_uwb_research_sim/msg/uwb_research_data.hpp"

#include "ros2_uwb_research_sim/channel_model.hpp"

namespace ros2_uwb_research_sim
{

class UWBPlugin : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPostUpdate
{
public:
  UWBPlugin();
  ~UWBPlugin() override;

  // ISystemConfigure interface
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager & eventMgr) override;

  // ISystemPostUpdate interface
  void PostUpdate(
    const ignition::gazebo::UpdateInfo & info,
    const ignition::gazebo::EntityComponentManager & ecm) override;

private:
  // ROS2 components
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<ros2_uwb_msgs::msg::UWBRange>::SharedPtr standardized_range_pub_;
  rclcpp::Publisher<ros2_uwb_msgs::msg::UWBErrorDiagnostics>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<ros2_uwb_research_sim::msg::UWBResearchData>::SharedPtr research_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Simulation components
  ignition::gazebo::Entity host_entity_;
  ignition::gazebo::Entity target_entity_;
  std::string target_name_;
  std::string frame_id_;

  // Optional fixed target position from SDF (avoids entity lookup issues)
  bool has_fixed_target_pos_{false};
  Eigen::Vector3d fixed_target_pos_;

  std::unique_ptr<ChannelModel> channel_;

  double update_frequency_;
  std::chrono::steady_clock::duration last_update_time_;

  // Threading components
  std::thread ros_thread_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::atomic<bool> stop_flag_{false};
};

}  // namespace ros2_uwb_research_sim

#endif  // ROS2_UWB_RESEARCH_SIM__UWB_PLUGIN_HPP_
