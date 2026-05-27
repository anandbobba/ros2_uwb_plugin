#!/usr/bin/env python3

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

"""ROS2 node that subscribes to UWB research data and logs it to CSV."""

import csv
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from ros2_uwb_research_sim.msg import UWBResearchData


class UWBResearcher(Node):
    """ROS2 node that logs UWB research data to CSV and reports live metrics."""

    def __init__(self):
        """Initialize the UWBResearcher node, open CSV log, and start subscription."""
        super().__init__('uwb_researcher')

        # Parameters
        self.declare_parameter('log_dir', 'logs')
        self.declare_parameter('experiment_name', 'default_exp')

        log_dir = self.get_parameter('log_dir').value
        exp_name = self.get_parameter('experiment_name').value

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.csv_path = os.path.join(log_dir, f'{exp_name}_{int(time.time())}.csv')
        self.get_logger().info(f'Logging to {self.csv_path}')

        # CSV Setup
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'anchor_id', 'ground_truth', 'measured',
            'gaussian_noise', 'nlos_bias', 'multipath_error', 'clock_drift', 'error'
        ])

        # Subscription
        # Subscribe to all research data topics using a wildcard or specific subscription
        # For simplicity, we assume we subscribe to a combined topic or multiple topics
        self.sub = self.create_subscription(
            UWBResearchData,
            '/uwb/research_data',  # Can be remapped in launch
            self.callback,
            10
        )

        self.data_buffer = []

    def callback(self, msg):
        """Write incoming UWB measurement to CSV and periodically log metrics."""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        error = msg.measured - msg.ground_truth

        row = [
            timestamp, msg.anchor_id, msg.ground_truth, msg.measured,
            msg.gaussian_noise, msg.nlos_bias, msg.multipath_error, msg.clock_drift, error
        ]

        self.csv_writer.writerow(row)
        self.data_buffer.append(error)

        if len(self.data_buffer) % 100 == 0:
            self.compute_live_metrics()

    def compute_live_metrics(self):
        """Compute and log RMSE, MAE, bias, and std from accumulated error buffer."""
        errors = np.array(self.data_buffer)
        rmse = np.sqrt(np.mean(errors**2))
        mae = np.mean(np.abs(errors))
        bias = np.mean(errors)
        std = np.std(errors)

        self.get_logger().info(
            f'Metrics: RMSE={rmse:.3f}, MAE={mae:.3f}, Bias={bias:.3f}, Std={std:.3f}')

    def __del__(self):
        """Close the CSV log file on node destruction."""
        self.csv_file.close()


def main(args=None):
    """Initialize ROS2, spin the UWBResearcher node, then shut down cleanly."""
    rclpy.init(args=args)
    node = UWBResearcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
