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

#
# Generate Synthetic Demo Bag
# ---------------------------
# Creates a ROS2 bag with synthetic UWB range + odometry data — no Gazebo
# or real hardware required. The robot travels a circular path.
#
# Output: ~/ros2_uwb_demo_bag/  (standard SQLite3 bag format)
#
# Usage:
#   ros2 run ros2_uwb_localization generate_demo_bag.py
#   ros2 run ros2_uwb_localization generate_demo_bag.py --output /tmp/my_bag
#   ros2 run ros2_uwb_localization generate_demo_bag.py --duration 120 --radius 4.0

"""Generate a synthetic ROS2 bag with UWB ranges and odometry for demo/replay."""

import argparse
import math
import os
import random
import shutil
import sys
import time

try:
    from rclpy.serialization import serialize_message
    import rosbag2_py
    from sensor_msgs.msg import Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Quaternion
    from rosgraph_msgs.msg import Clock
except ImportError as e:
    print(f'[ERROR] Missing ROS2 Python packages: {e}')
    print('        Run:  source /opt/ros/humble/setup.bash')
    print('              source <workspace>/install/setup.bash')
    sys.exit(1)


# ---------------------------------------------------------------------------
# Anchor layout (must match anchors.yaml)
# ---------------------------------------------------------------------------
ANCHORS = [
    ('uwb_anchor_0', (5.0, 5.0, 2.0)),
    ('uwb_anchor_1', (-5.0, 5.0, 2.0)),
    ('uwb_anchor_2', (5.0, -5.0, 2.0)),
    ('uwb_anchor_3', (-5.0, -5.0, 2.0)),
]

# ---------------------------------------------------------------------------
# Noise model (Gaussian only, to keep demo clean)
# ---------------------------------------------------------------------------
GAUSSIAN_SIGMA = 0.05   # metres
CLOCK_DRIFT_SIGMA = 0.001  # metres/step


def _dist3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _yaw_to_quat(yaw):
    """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _ns_from_s(t_s):
    """Convert float seconds to (sec, nanosec) tuple."""
    sec = int(t_s)
    nanosec = int((t_s - sec) * 1e9)
    return sec, nanosec


# ---------------------------------------------------------------------------
# Topic writers
# ---------------------------------------------------------------------------

def write_bag(output_dir, duration_s, radius_m, dt, rng):
    """Write the synthetic bag to output_dir."""
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    storage_opts = rosbag2_py.StorageOptions(
        uri=output_dir, storage_id='sqlite3')
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_opts, converter_opts)

    # Register topics
    _add_topic(writer, '/clock', 'rosgraph_msgs/msg/Clock')
    _add_topic(writer, '/odom', 'nav_msgs/msg/Odometry')
    for i in range(len(ANCHORS)):
        _add_topic(writer, f'/uwb/range_{i}', 'sensor_msgs/msg/Range')

    n_steps = int(duration_s / dt)
    period = duration_s              # seconds for one full circle
    drift = 0.0

    print(f'  Generating {n_steps} steps × {len(ANCHORS)} anchors '
          f'({duration_s:.0f}s at {1.0 / dt:.0f} Hz)...')

    for step in range(n_steps):
        t_s = step * dt
        sec, nanosec = _ns_from_s(t_s)
        stamp_ns = int(t_s * 1e9)

        # --- Clock ---
        clk_msg = Clock()
        clk_msg.clock.sec = sec
        clk_msg.clock.nanosec = nanosec
        writer.write('/clock', serialize_message(clk_msg), stamp_ns)

        # --- Robot circular trajectory ---
        angle = 2 * math.pi * t_s / period
        rx = radius_m * math.cos(angle)
        ry = radius_m * math.sin(angle)
        rz = 0.0
        yaw = angle + math.pi / 2.0  # tangent direction

        odom_msg = Odometry()
        odom_msg.header.stamp.sec = sec
        odom_msg.header.stamp.nanosec = nanosec
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = rx
        odom_msg.pose.pose.position.y = ry
        odom_msg.pose.pose.position.z = rz
        odom_msg.pose.pose.orientation = _yaw_to_quat(yaw)
        # Linear velocity: tangential v = 2πr / T
        v = 2 * math.pi * radius_m / period
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = 2 * math.pi / period
        writer.write('/odom', serialize_message(odom_msg), stamp_ns)

        # --- UWB ranges ---
        drift += rng.gauss(0, CLOCK_DRIFT_SIGMA)
        robot_pos = (rx, ry, rz)
        for idx, (anchor_id, anchor_pos) in enumerate(ANCHORS):
            true_range = _dist3(robot_pos, anchor_pos)
            measured = true_range + rng.gauss(0, GAUSSIAN_SIGMA) + drift

            range_msg = Range()
            range_msg.header.stamp.sec = sec
            range_msg.header.stamp.nanosec = nanosec
            range_msg.header.frame_id = anchor_id
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 6.28
            range_msg.min_range = 0.1
            range_msg.max_range = 100.0
            range_msg.range = float(max(0.1, measured))
            writer.write(f'/uwb/range_{idx}',
                         serialize_message(range_msg), stamp_ns)

    del writer   # flush & close
    print(f'  Bag written to: {output_dir}/')


def _add_topic(writer, topic, msg_type):
    meta = rosbag2_py.TopicMetadata(
        name=topic,
        type=msg_type,
        serialization_format='cdr')
    writer.create_topic(meta)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args():
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Generate a synthetic ROS2 UWB demo bag (no Gazebo needed)')
    parser.add_argument(
        '--output', type=str,
        default=os.path.expanduser('~/ros2_uwb_demo_bag'),
        help='Output bag directory (default: ~/ros2_uwb_demo_bag)')
    parser.add_argument(
        '--duration', type=float, default=60.0,
        help='Simulation duration in seconds (default: 60.0)')
    parser.add_argument(
        '--radius', type=float, default=3.0,
        help='Robot circular path radius in metres (default: 3.0)')
    parser.add_argument(
        '--rate', type=float, default=10.0,
        help='UWB measurement rate in Hz (default: 10.0)')
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Random seed (default: 42)')
    return parser.parse_args()


def main():
    """Run main entry point for bag generation."""
    args = parse_args()
    rng = random.Random(args.seed)
    dt = 1.0 / args.rate

    print('\nUWB Synthetic Demo Bag Generator')
    print(f'  Output     : {args.output}')
    print(f'  Duration   : {args.duration:.0f} s')
    print(f'  Radius     : {args.radius:.1f} m')
    print(f'  Rate       : {args.rate:.0f} Hz')
    print(f'  Anchors    : {len(ANCHORS)}')
    print(f'  Noise σ    : {GAUSSIAN_SIGMA:.3f} m (Gaussian)')
    print('')

    t0 = time.monotonic()
    write_bag(args.output, args.duration, args.radius, dt, rng)
    elapsed = time.monotonic() - t0

    print(f'  Done in {elapsed:.1f}s.\n')
    print('Replay this bag with:')
    print(f'  ros2 launch ros2_uwb_localization play_bag.launch.py '
          f'bag_path:={args.output}\n')


if __name__ == '__main__':
    main()
