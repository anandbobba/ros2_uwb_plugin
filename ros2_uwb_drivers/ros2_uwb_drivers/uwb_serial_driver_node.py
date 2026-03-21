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

import rclpy
from rclpy.node import Node
from ros2_uwb_msgs.msg import UWBRange
import serial
import threading


class UWBSerialDriver(Node):
    """
    ROS2 Node for interfacing with UWB hardware over serial.

    Reads range data from a serial port and publishes UWBRange messages.
    """

    def __init__(self):
        """Initialize parameters, publishers, and serial connection."""
        super().__init__('uwb_serial_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'uwb_tag')
        self.declare_parameter('std_dev', 0.05)
        self.declare_parameter('topic', '/uwb/range')
        self.declare_parameter('use_mock', False)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.std_dev = self.get_parameter('std_dev').value
        self.topic_name = self.get_parameter('topic').value
        self.use_mock = self.get_parameter('use_mock').value

        # Publisher
        self.publisher = self.create_publisher(UWBRange, self.topic_name, 10)

        # Serial Connection
        self.ser = None
        if not self.use_mock:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
                self.get_logger().info(
                    f"Connected to UWB hardware at {self.port} ({self.baud} baud)"
                )
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Could not open port {self.port}. Is the hardware plugged in?"
                )
                self.get_logger().error(
                    "TIP: Use 'ls /dev/tty*' to find your port, or set 'use_mock: True' in config."
                )
                raise e
        else:
            self.get_logger().info("UWB Driver started in MOCK MODE (generating dummy data)")

        # Thread for reading / generating mock data
        self.stop_flag = False
        self.thread = threading.Thread(target=self.read_loop)
        self.thread.start()

    def read_loop(self):
        """Run the main read loop from the serial port."""
        import time
        import random
        while rclpy.ok() and not self.stop_flag:
            if not self.use_mock and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            self.parse_line(line)
                except Exception as e:
                    self.get_logger().warn(f"Error reading serial data: {e}")
            elif self.use_mock:
                # Generate realistic mock data: ANCHOR_ID,RANGE,RSSI
                mock_range = 5.0 + random.uniform(-0.1, 0.1)
                mock_line = f"uwb_anchor_0,{mock_range:.2f},-60"
                self.parse_line(mock_line)
                time.sleep(0.1)  # 10Hz

    def parse_line(self, line):
        """
        Parse a single line of serial data.

        Identifies expected formats like ANCHOR_ID,RANGE,RSSI.
        """
        parts = line.split(',')
        if len(parts) >= 2:
            try:
                anchor_id = parts[0].strip()
                range_val = float(parts[1].strip())
                rssi = float(parts[2].strip()) if len(parts) > 2 else 0.0

                # Basic validation
                if range_val < 0 or range_val > 100.0:
                    return

                msg = UWBRange()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.anchor_id = anchor_id
                msg.range = range_val
                msg.rssi = rssi
                msg.std_dev = self.std_dev

                self.publisher.publish(msg)

            except ValueError:
                self.get_logger().debug(f"Malformed serial data: {line}")

    def destroy(self):
        """Safely stop the reading thread and close the serial port."""
        self.stop_flag = True
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if hasattr(self, 'ser'):
            self.ser.close()
        super().destroy_node()


def main(args=None):
    """Entry point for the UWB Serial Driver node."""
    rclpy.init(args=args)
    try:
        node = UWBSerialDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
