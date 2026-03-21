# Copyright 2026 Anand Bobba
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Circular autopilot for UWB research evaluation."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class ResearchAutopilot(Node):
    """ROS2 node that publishes velocity commands to move the robot in a circle."""

    def __init__(self):
        """Initialize the autopilot node and start the command timer."""
        super().__init__('research_autopilot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('linear_vel', 0.5)
        self.declare_parameter('angular_vel', 0.3)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Research Autopilot started. Robot will move in a circle.')

    def timer_callback(self):
        """Publish the velocity command based on current parameters."""
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_vel').value
        msg.angular.z = self.get_parameter('angular_vel').value
        self.publisher_.publish(msg)


def main(args=None):
    """Initialize ROS2, spin the ResearchAutopilot node, then shut down cleanly."""
    rclpy.init(args=args)
    node = ResearchAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
