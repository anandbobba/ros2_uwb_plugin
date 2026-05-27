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

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace ros2_uwb_localization
{

class YawEstimatorNode : public rclcpp::Node
{
public:
  YawEstimatorNode() : Node("uwb_yaw_estimator"), smoothed_yaw_(0.0), first_measurement_(true)
  {
    this->declare_parameter("baseline_expected", 0.7);
    this->declare_parameter("baseline_tolerance", 0.2);
    this->declare_parameter("smoothing_alpha", 0.8);

    baseline_expected_ = this->get_parameter("baseline_expected").as_double();
    baseline_tolerance_ = this->get_parameter("baseline_tolerance").as_double();
    smoothing_alpha_ = this->get_parameter("smoothing_alpha").as_double();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/pose", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/uwb/odom", 10);

    front_sub_.subscribe(this, "/uwb/front/pose");
    rear_sub_.subscribe(this, "/uwb/rear/pose");

    sync_.reset(new Sync(MySyncPolicy(10), front_sub_, rear_sub_));
    sync_->registerCallback(std::bind(&YawEstimatorNode::pose_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "UWB Yaw Estimator initialized");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& front_msg,
                     const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& rear_msg)
  {
    double dx = front_msg->pose.pose.position.x - rear_msg->pose.pose.position.x;
    double dy = front_msg->pose.pose.position.y - rear_msg->pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (std::abs(distance - baseline_expected_) > baseline_tolerance_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Baseline validation failed! Expected: %.2f, Got: %.2f", baseline_expected_, distance);
      return;
    }

    double raw_yaw = std::atan2(dy, dx);

    if (first_measurement_) {
      smoothed_yaw_ = raw_yaw;
      first_measurement_ = false;
    } else {
      // Avoid wrap-around issues by smoothing sin and cos
      double current_sin = std::sin(smoothed_yaw_);
      double current_cos = std::cos(smoothed_yaw_);
      double raw_sin = std::sin(raw_yaw);
      double raw_cos = std::cos(raw_yaw);
      
      double new_sin = smoothing_alpha_ * raw_sin + (1.0 - smoothing_alpha_) * current_sin;
      double new_cos = smoothing_alpha_ * raw_cos + (1.0 - smoothing_alpha_) * current_cos;
      
      smoothed_yaw_ = std::atan2(new_sin, new_cos);
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, smoothed_yaw_);

    // Create the combined pose message
    auto out_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    out_msg.header = front_msg->header;
    
    // Position: Center between front and rear tags
    out_msg.pose.pose.position.x = (front_msg->pose.pose.position.x + rear_msg->pose.pose.position.x) / 2.0;
    out_msg.pose.pose.position.y = (front_msg->pose.pose.position.y + rear_msg->pose.pose.position.y) / 2.0;
    out_msg.pose.pose.position.z = (front_msg->pose.pose.position.z + rear_msg->pose.pose.position.z) / 2.0;
    out_msg.pose.pose.orientation = tf2::toMsg(q);

    // Covariances
    for (int i = 0; i < 36; ++i) {
      out_msg.pose.covariance[i] = (front_msg->pose.covariance[i] + rear_msg->pose.covariance[i]) / 2.0;
    }
    // Set yaw covariance based on baseline tolerance roughly
    out_msg.pose.covariance[35] = 0.05; // ~ 0.22 rad std dev

    pose_pub_->publish(out_msg);

    // Optional Odometry output
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header = out_msg.header;
    odom_msg.child_frame_id = "uwb_base";
    odom_msg.pose = out_msg.pose;
    odom_pub_->publish(odom_msg);
  }

  double baseline_expected_;
  double baseline_tolerance_;
  double smoothing_alpha_;
  double smoothed_yaw_;
  bool first_measurement_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> front_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> rear_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseWithCovarianceStamped, geometry_msgs::msg::PoseWithCovarianceStamped> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
};

}  // namespace ros2_uwb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_uwb_localization::YawEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
