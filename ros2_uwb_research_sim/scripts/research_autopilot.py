#!/usr/bin/env python3
# Copyright 2026 Anand Bobba
# Circular autopilot for UWB research evaluation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ResearchAutopilot(Node):
    def __init__(self):
        super().__init__('research_autopilot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('linear_vel', 0.5)
        self.declare_parameter('angular_vel', 0.3)
        self.get_logger().info('Research Autopilot started. Robot will move in a circle.')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_vel').value
        msg.angular.z = self.get_parameter('angular_vel').value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ResearchAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
