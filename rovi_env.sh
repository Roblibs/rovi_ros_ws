#!/usr/bin/env bash

# rovi_ros_ws shell helpers
#
# Add to your real ~/.bashrc:
#   source "$HOME/dev/Roblibs/rovi_ros_ws/rovi_env.sh"

ROVI_ROS_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROVI_ROS_WS_DIR

build() {
  cd "${ROVI_ROS_WS_DIR}" || return
  colcon build "$@"
}

setup() {
  if [ ! -f "${ROVI_ROS_WS_DIR}/install/setup.bash" ]; then
    echo "[rovi_env] Missing ${ROVI_ROS_WS_DIR}/install/setup.bash (run: build)" >&2
    return 1
  fi
  # shellcheck disable=SC1091
  source "${ROVI_ROS_WS_DIR}/install/setup.bash"
}

activate() {
  if [ ! -f "${ROVI_ROS_WS_DIR}/.venv/bin/activate" ]; then
    echo "[rovi_env] Missing ${ROVI_ROS_WS_DIR}/.venv/bin/activate (run: uv sync)" >&2
    return 1
  fi
  # shellcheck disable=SC1091
  source "${ROVI_ROS_WS_DIR}/.venv/bin/activate"
}

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
  setup && activate && ros2 launch rovi_bringup teleop.launch.py "$@"
}

mapping() {
  setup && activate && ros2 launch rovi_bringup mapping.launch.py "$@"
}

localization() {
  setup && activate && ros2 launch rovi_bringup localization.launch.py "$@"
}

nav() {
  setup && activate && ros2 launch rovi_bringup nav.launch.py "$@"
}

view_offline() {
  setup && ros2 launch rovi_bringup offline_view.launch.py "$@"
}

view_teleop() {
  setup && rviz2 -d "$(ros2 pkg prefix rovi_description)/share/rovi_description/rviz/rovi.rviz" "$@"
}

view_map() {
  setup && rviz2 -d "$(ros2 pkg prefix rovi_description)/share/rovi_description/rviz/rovi_map.rviz" "$@"
}

talk() {
  ros2 run demo_nodes_cpp talker "$@"
}

listen() {
  ros2 run demo_nodes_py listener "$@"
}
