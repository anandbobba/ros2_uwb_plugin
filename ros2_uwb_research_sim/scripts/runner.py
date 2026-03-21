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

"""Run a UWB simulation experiment from a YAML config and collect CSV results."""

import os
import signal
import subprocess
import time

import yaml


def run_experiment(config_file):
    """Launch simulation and researcher node for the duration specified in config_file."""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    exp_name = config['experiment']['name']
    duration = config['experiment']['duration']

    print(f'Starting Experiment: {exp_name} for {duration} seconds...')

    # 1. Start Simulation with remapped parameters (simplified for demo)
    # In a real setup, we would generate a custom SDF or pass params to launch
    launch_cmd = [
        'ros2', 'launch', 'ros2_uwb_research_sim', 'sim.launch.py'
    ]

    # 2. Start Researcher Node
    csv_dir = os.path.abspath('logs')
    res_dir = os.path.abspath(os.path.join('results', exp_name))
    for d in [csv_dir, res_dir]:
        os.makedirs(d, exist_ok=True)

    print(f'Logs: {csv_dir}')
    print(f'Results: {res_dir}')

    logger_cmd = [
        'ros2', 'run', 'ros2_uwb_research_sim', 'researcher_node.py',
        '--ros-args',
        '-p', f'experiment_name:={exp_name}',
        '-p', f'log_dir:={csv_dir}'
    ]

    sim_proc = subprocess.Popen(launch_cmd)
    print('Waiting 15s for Gazebo to start...')
    time.sleep(15)

    log_proc = subprocess.Popen(logger_cmd)

    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        pass
    finally:
        print('Stopping experiment...')
        log_proc.send_signal(signal.SIGINT)
        sim_proc.send_signal(signal.SIGINT)
        log_proc.wait()
        sim_proc.wait()

    # 4. Process results (automatically find latest CSV)
    csv_files = [f for f in os.listdir(csv_dir) if f.startswith(exp_name)]
    if csv_files:
        latest_csv = sorted(csv_files)[-1]
        csv_path = os.path.join(csv_dir, latest_csv)
        print(f'Processing data from {csv_path}...')
        subprocess.run([
            'ros2', 'run', 'ros2_uwb_research_sim', 'plotter.py',
            csv_path,
            res_dir
        ])


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print('Usage: python3 runner.py <config_path>')
    else:
        run_experiment(sys.argv[1])
