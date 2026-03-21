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

#include "ros2_uwb_localization/range_preprocessor.hpp"

namespace ros2_uwb_localization
{
RangePreprocessor::RangePreprocessor()
: Node("uwb_range_preprocessor")
{
  this->declare_parameter("publish_rate", 20.0);
  this->declare_parameter("range_timeout", 0.5);
  this->declare_parameter("min_range", 0.0);
  this->declare_parameter("max_range", 100.0);
  this->declare_parameter("min_rssi", -100.0);
  this->declare_parameter("base_frame", "uwb_base");

  range_timeout_ = this->get_parameter("range_timeout").as_double();
  min_range_ = this->get_parameter("min_range").as_double();
  max_range_ = this->get_parameter("max_range").as_double();
  min_rssi_ = this->get_parameter("min_rssi").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();

  // Standardized raw range subscription
  range_sub_ = this->create_subscription<ros2_uwb_msgs::msg::UWBRange>(
    "/uwb/range", 10,
    std::bind(&RangePreprocessor::range_callback, this, std::placeholders::_1));

  // Multi-range filtered output
  multi_range_pub_ = this->create_publisher<ros2_uwb_msgs::msg::UWBMultiRange>(
    "/uwb/ranges_filtered", 10);

  // Dynamic parameter callback
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        if (param.get_name() == "range_timeout") {
          range_timeout_ = param.as_double();
        } else if (param.get_name() == "min_range") {
          min_range_ = param.as_double();
        } else if (param.get_name() == "max_range") {
          max_range_ = param.as_double();
        } else if (param.get_name() == "min_rssi") {
          min_rssi_ = param.as_double();
        }
      }
      return result;
    });
  double rate = this->get_parameter("publish_rate").as_double();
  RCLCPP_INFO(
    this->get_logger(),
    "UWB Range Preprocessor initialized at %.1f Hz (Event-Driven)", rate);
  RCLCPP_INFO(
    this->get_logger(),
    " - Params: timeout=%.2fs, min_range=%.1f, max_range=%.1f",
    range_timeout_, min_range_, max_range_);
}

void RangePreprocessor::range_callback(const ros2_uwb_msgs::msg::UWBRange::SharedPtr msg)
{
  // Sanity check filtering
  if (msg->range < min_range_ || msg->range > max_range_ || std::isnan(msg->range)) {
    return;
  }

  // RSSI filtering (for robustness against weak signals/multipath)
  if (msg->rssi < min_rssi_) {
    return;
  }

  // Cache by anchor ID
  CachedRange entry;
  entry.last_msg = *msg;
  entry.last_update = this->now();
  entry.valid = true;

  range_cache_[msg->anchor_id] = entry;

  // Log every 10 messages to avoid spam but confirm life
  static int count = 0;
  if (++count % 10 == 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received range from [%s]: %.3f m",
      msg->anchor_id.c_str(), msg->range);
  }

  // EVENT-DRIVEN PUBLISH: Immediately aggregate and publish
  auto out_msg = ros2_uwb_msgs::msg::UWBMultiRange();
  // Inherit exactly the timestamp of the triggering range
  out_msg.header.stamp = msg->header.stamp;
  out_msg.header.frame_id = base_frame_;

  rclcpp::Time current_time(msg->header.stamp);

  // Aggregate all fresh ranges
  int active_anchors = 0;
  for (auto & [id, cache] : range_cache_) {
    rclcpp::Time cache_time(cache.last_msg.header.stamp);
    double age = std::abs((current_time - cache_time).seconds());
    if (age <= range_timeout_) {
      out_msg.ranges.push_back(cache.last_msg);
      active_anchors++;
    }
  }

  if (!out_msg.ranges.empty()) {
    multi_range_pub_->publish(out_msg);
  }
}

}  // namespace ros2_uwb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<ros2_uwb_localization::RangePreprocessor>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
