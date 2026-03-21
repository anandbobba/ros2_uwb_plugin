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

#ifndef ROS2_UWB_LOCALIZATION__RANGE_PREPROCESSOR_HPP_
#define ROS2_UWB_LOCALIZATION__RANGE_PREPROCESSOR_HPP_

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "ros2_uwb_msgs/msg/uwb_range.hpp"
#include "ros2_uwb_msgs/msg/uwb_multi_range.hpp"

namespace ros2_uwb_localization
{

/**
 * @brief Node that aggregates single UWB range measurements into synchronized MultiRange messages.
 * It listens to /uwb/range (standardized UWBRange) and publishes aggregated /uwb/ranges_filtered.
 * This acts as a sensor abstraction layer, allowing the solver to work with a single topic.
 */
class RangePreprocessor : public rclcpp::Node
{
public:
  RangePreprocessor();

private:
  void range_callback(const ros2_uwb_msgs::msg::UWBRange::SharedPtr msg);
  void timer_callback();

  // ROS2 components
  rclcpp::Subscription<ros2_uwb_msgs::msg::UWBRange>::SharedPtr range_sub_;
  rclcpp::Publisher<ros2_uwb_msgs::msg::UWBMultiRange>::SharedPtr multi_range_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  struct CachedRange
  {
    ros2_uwb_msgs::msg::UWBRange last_msg;
    rclcpp::Time last_update;
    bool valid;
  };

  std::map<std::string, CachedRange> range_cache_;

  // Parameters
  double range_timeout_;
  double min_range_;
  double max_range_;
  double min_rssi_;
  std::string base_frame_;
};

}  // namespace ros2_uwb_localization

#endif  // ROS2_UWB_LOCALIZATION__RANGE_PREPROCESSOR_HPP_
