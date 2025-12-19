#!/usr/bin/env bash

# rovi_ros_ws shell helpers

if [ -z "${ROVI_ROS_WS_DIR:-}" ]; then
  echo "[rovi_env] ROVI_ROS_WS_DIR is not set." >&2
  return 1 2>/dev/null || exit 1
fi

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

clean() {
  rm -rf build/ install/ log/
}

build() {
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

ws() {
  cd "${ROVI_ROS_WS_DIR}" || return
  setup
  #optioanl activate, silent skip
  if [ -f "${ROVI_ROS_WS_DIR}/.venv/bin/activate" ]; then
    source "${ROVI_ROS_WS_DIR}/.venv/bin/activate"
  fi
}

teleop() {
  ros2 launch rovi_bringup teleop.launch.py "$@"
}

mapping() {
  ros2 launch rovi_bringup mapping.launch.py "$@"
}

localization() {
  ros2 launch rovi_bringup localization.launch.py "$@"
}

nav() {
  ros2 launch rovi_bringup nav.launch.py "$@"
}

view() {
  rviz2 -d "install/rovi_description/share/rovi_description/rviz/rovi_map.rviz" "$@"
}

view_teleop() {
  rviz2 -d "install/rovi_description/share/rovi_description/rviz/rovi_odom.rviz" "$@"
}

view_offline() {
  ROS_LOCALHOST_ONLY=1 ros2 launch rovi_bringup offline_view.launch.py "$@"
}

talk() {
  ros2 run demo_nodes_cpp talker "$@"
}

listen() {
  ros2 run demo_nodes_py listener "$@"
}
