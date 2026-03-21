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

#include "ros2_uwb_research_sim/uwb_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

GZ_ADD_PLUGIN(
  ros2_uwb_research_sim::UWBPlugin,
  gz::sim::System,
  ros2_uwb_research_sim::UWBPlugin::ISystemConfigure,
  ros2_uwb_research_sim::UWBPlugin::ISystemPostUpdate)

namespace ros2_uwb_research_sim
{

UWBPlugin::UWBPlugin()
  : update_frequency_(10.0),
  last_update_time_(std::chrono::steady_clock::duration::zero())
{
}

UWBPlugin::~UWBPlugin()
{
  stop_flag_ = true;
  if (ros_executor_) {
    ros_executor_->cancel();
  }
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
}

void UWBPlugin::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & /*ecm*/,
  gz::sim::EventManager & /*eventMgr*/)
{
  host_entity_ = entity;

  // Initialize ROS2 Node if not already initialized
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Read SDF parameters
  target_name_ = sdf->Get<std::string>("target_entity", "anchor").first;
  update_frequency_ = sdf->Get<double>("update_rate", 10.0).first;

  // Predictable node name for each anchor link
  std::string node_name = "uwb_plugin_" + target_name_;
  
  // Use sim time for synchronization with Gazebo clock
  auto options = rclcpp::NodeOptions();
  options.append_parameter_override("use_sim_time", true);
  ros_node_ = std::make_shared<rclcpp::Node>(node_name, options);

  // Channel Noise Parameters
  ChannelParams params;
  if (sdf->HasElement("noise_profile")) {
    std::string profile = sdf->Get<std::string>("noise_profile");
    params = ChannelModel::getParamsByProfile(profile);
    gzmsg << "UWB Plugin: Using noise profile [" << profile << "]" << std::endl;
  } else {
    // Individual parameter overrides (legacy)
    params.gaussian_sigma = sdf->Get<double>("gaussian_std", 0.05).first;
    params.nlos_lambda = sdf->Get<double>("nlos_lambda", 2.0).first;
    params.nlos_prob = sdf->Get<double>("nlos_prob", 0.1).first;
    params.multipath_alpha = sdf->Get<double>("multipath_alpha", 0.8).first;
    params.multipath_sigma = sdf->Get<double>("multipath_std", 0.02).first;
    params.clock_drift_sigma = sdf->Get<double>("clock_drift_std", 0.001).first;
  }

  channel_ = std::make_unique<ChannelModel>(params);

  // ROS2 Publisher
  std::string topic = sdf->Get<std::string>("topic", "/uwb/range").first;
  range_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Range>(topic, 10);

  std::string std_msg_topic = "/uwb/range";
  standardized_range_pub_ = ros_node_->create_publisher<ros2_uwb_msgs::msg::UWBRange>(
    std_msg_topic, 10);

  diagnostics_pub_ = ros_node_->create_publisher<ros2_uwb_msgs::msg::UWBErrorDiagnostics>(
    "/uwb/error_diagnostics", 10);

  std::string research_topic = sdf->Get<std::string>("research_topic", topic + "/research").first;
  research_pub_ = ros_node_->create_publisher<ros2_uwb_research_sim::msg::UWBResearchData>(
    research_topic, 10);

  // Get frame_id from SDF or default to entity name
  frame_id_ = sdf->Get<std::string>("frame_id", std::to_string(entity)).first;

  // Optional: read target position directly from SDF
  // This avoids entity lookup issues with static models in IGN Fortress
  if (sdf->HasElement("target_position")) {
    auto pos = sdf->Get<gz::math::Vector3d>("target_position");
    fixed_target_pos_ = Eigen::Vector3d(pos.X(), pos.Y(), pos.Z());
    has_fixed_target_pos_ = true;
    gzmsg << "UWB Plugin: Using fixed target position (" << pos.X()
           << ", " << pos.Y() << ", " << pos.Z() << ")" << std::endl;
  }

  // Declare ROS parameters for runtime reconfiguration
  std::string initial_profile = sdf->HasElement("noise_profile") ? sdf->Get<std::string>("noise_profile") : "custom";
  ros_node_->declare_parameter("noise_profile", initial_profile);

  // Parameter callback to handle runtime changes
  parameter_callback_handle_ = ros_node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        if (param.get_name() == "noise_profile") {
          std::string profile = param.as_string();
          try {
            ChannelParams new_params = ChannelModel::getParamsByProfile(profile);
            channel_->setParams(new_params);
            RCLCPP_INFO(ros_node_->get_logger(), "Switched to noise profile: %s", profile.c_str());
          } catch (...) {
            RCLCPP_WARN(ros_node_->get_logger(), "Failed to load profile: %s", profile.c_str());
            result.successful = false;
            result.reason = "Invalid noise profile name";
          }
        }
      }
      return result;
    });

  // Start background ROS thread
  ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ros_executor_->add_node(ros_node_);
  ros_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !stop_flag_) {
      ros_executor_->spin();
    }
  });

  gzmsg << "UWB Plugin configured for entity [" << entity << "]. "
        << "Target: [" << target_name_ << "]" << std::endl;
}

void UWBPlugin::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  if (info.paused) {return;}

  // Control publish rate
  double dt = std::chrono::duration<double>(info.simTime - last_update_time_).count();
  if (dt < (1.0 / update_frequency_)) {return;}
  last_update_time_ = info.simTime;

  // Get host position
  auto p_host_ign = gz::sim::worldPose(host_entity_, ecm).Pos();
  Eigen::Vector3d p_host(p_host_ign.X(), p_host_ign.Y(), p_host_ign.Z());

  // Get target position: use fixed position if available, else entity lookup
  Eigen::Vector3d p_target;
  if (has_fixed_target_pos_) {
    p_target = fixed_target_pos_;
  } else {
    // Find target entity by name if not found yet
    if (target_entity_ == gz::sim::kNullEntity) {
      target_entity_ = ecm.EntityByComponents(
        gz::sim::components::Name(target_name_),
        gz::sim::components::Model());
      if (target_entity_ == gz::sim::kNullEntity) {
        target_entity_ = ecm.EntityByComponents(
          gz::sim::components::Name(target_name_));
      }
      if (target_entity_ == gz::sim::kNullEntity) {return;}
    }
    auto p_target_ign = gz::sim::worldPose(target_entity_, ecm).Pos();
    p_target = Eigen::Vector3d(p_target_ign.X(), p_target_ign.Y(), p_target_ign.Z());
  }

  // Compute range with noise
  auto m = channel_->computeDetailedMeasurement(p_host, p_target);
  double range_val = m.measured;

  // Optional: Log once to verify math
  static int log_count = 0;
  if (log_count++ % 100 == 0) {
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Target: %s at (%.2f, %.2f, %.2f) | True Dist: %.3f",
      target_name_.c_str(), p_target.x(), p_target.y(), p_target.z(), m.d_true);
  }

  // Publish ROS2 message
  auto msg = sensor_msgs::msg::Range();
  
  // Convert Ignition Gazebo simTime to ROS2 rclcpp::Time
  auto sim_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  rclcpp::Time current_sim_time(sim_nanos, RCL_ROS_TIME);
  
  msg.header.stamp = current_sim_time;
  msg.header.frame_id = frame_id_;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;   // Using Range for UWB
  msg.field_of_view = 6.28;
  msg.min_range = 0.1;
  msg.max_range = 100.0;
  msg.range = static_cast<float>(range_val);

  range_pub_->publish(msg);

  // Publish standardized range
  auto std_range_msg = ros2_uwb_msgs::msg::UWBRange();
  std_range_msg.header = msg.header;
  std_range_msg.anchor_id = target_name_;
  std_range_msg.range = range_val;
  std_range_msg.std_dev = channel_->getParams().gaussian_sigma;
  
  // Basic RSSI model: -40dBm at 1m, dropping by 20dB per decade
  double d_ref = std::max(1.0, m.d_true);
  std_range_msg.rssi = -40.0 - 20.0 * std::log10(d_ref);
  std_range_msg.fpp = std_range_msg.rssi - 3.0; // Assume FPP is slightly lower
  
  std_range_msg.is_nlos = (m.nlos_bias > 0.1);
  standardized_range_pub_->publish(std_range_msg);

  // Publish Research Data
  auto res_msg = ros2_uwb_research_sim::msg::UWBResearchData();
  res_msg.header = msg.header;
  res_msg.anchor_id = target_name_;
  res_msg.ground_truth = m.d_true;
  res_msg.measured = m.measured;
  res_msg.gaussian_noise = m.gaussian_noise;
  res_msg.nlos_bias = m.nlos_bias;
  res_msg.multipath_error = m.multipath_error;
  res_msg.clock_drift = m.clock_drift;
  research_pub_->publish(res_msg);

  // Publish Error Diagnostics
  auto diag_msg = ros2_uwb_msgs::msg::UWBErrorDiagnostics();
  diag_msg.header = msg.header;
  diag_msg.anchor_id = target_name_;
  diag_msg.true_distance = m.d_true;
  diag_msg.measured_distance = m.measured;
  diag_msg.gaussian_error = m.gaussian_noise;
  diag_msg.nlos_bias = m.nlos_bias;
  diag_msg.multipath_error = m.multipath_error;
  diag_msg.clock_drift_error = m.clock_drift;
  diag_msg.total_error = m.measured - m.d_true;
  diagnostics_pub_->publish(diag_msg);
}

}  // namespace ros2_uwb_research_sim
