#!/usr/bin/env bash

set -e

source install/setup.bash

ros2 daemon start >/dev/null 2>&1 || true

ROS_LOCALHOST_ONLY=1 ros2 launch rovi_bringup offline_view.launch.py

