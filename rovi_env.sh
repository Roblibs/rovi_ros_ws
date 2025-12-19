#!/usr/bin/env bash

# rovi_ros_ws shell helpers
#
# Add to your real ~/.bashrc:
#   source "$HOME/dev/Roblibs/rovi_ros_ws/rovi_env.sh"

ROVI_ROS_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROVI_ROS_WS_DIR

# System ROS (Jazzy)
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# Common ROS settings
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

rovi_dds_default() {
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
  unset ROS_DISCOVERY_SERVER
}

# Default DDS mode for this workspace:
rovi_dds_default

ws() {
  cd "${ROVI_ROS_WS_DIR}" || return
  # shellcheck disable=SC1091
  source install/setup.bash
  if [ -f .venv/bin/activate ]; then
    # shellcheck disable=SC1091
    source .venv/bin/activate
  fi
}

teleop() {
  ws && ros2 launch rovi_bringup teleop.launch.py "$@"
}

mapping() {
  ws && ros2 launch rovi_bringup mapping.launch.py "$@"
}

localization() {
  ws && ros2 launch rovi_bringup localization.launch.py "$@"
}

nav() {
  ws && ros2 launch rovi_bringup nav.launch.py "$@"
}

offline_view() {
  ws && ros2 launch rovi_bringup offline_view.launch.py "$@"
}

talk() {
  ros2 run demo_nodes_cpp talker "$@"
}

listen() {
  ros2 run demo_nodes_py listener "$@"
}
