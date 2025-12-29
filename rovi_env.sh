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

ensure_ros2_daemon() {
  local daemon_status
  daemon_status="$(ros2 daemon status 2>/dev/null || true)"
  if [[ "${daemon_status}" == *"not running"* ]]; then
    ros2 daemon start >/dev/null 2>&1 || true
  fi
}

# Default DDS mode for this workspace:
rovi_dds_default
ensure_ros2_daemon

clean() {
  rm -rf build/ install/ log/
}

stop() {
  python3 "${ROVI_ROS_WS_DIR}/tools/rovi_stop.py" "$@"
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
  #if [ -f "${ROVI_ROS_WS_DIR}/.venv/bin/activate" ]; then
  #  source "${ROVI_ROS_WS_DIR}/.venv/bin/activate"
  #fi
}

record() {
  ros2 run rovi_bringup rovi_bag record "$@"
}

play() {
  ros2 run rovi_bringup rovi_bag play "$@"
}

teleop() {
  ros2 launch rovi_bringup rovi.launch.py robot_mode:=real stack:=teleop rviz:=false "$@"
}

keyboard() {
  python3 "${ROVI_ROS_WS_DIR}/tools/rovi_keyboard.py" "$@"
}

mapping() {
  ros2 launch rovi_bringup rovi.launch.py robot_mode:=real stack:=mapping rviz:=false "$@"
}

localization() {
  ros2 launch rovi_bringup rovi.launch.py robot_mode:=real stack:=localization rviz:=false "$@"
}

nav() {
  ros2 launch rovi_bringup rovi.launch.py robot_mode:=real stack:=nav rviz:=false "$@"
}

sim() {
  local mode="${1:-mapping}"
  if [ $# -gt 0 ]; then
    shift
  fi

  case "${mode}" in
    teleop|mapping|localization|nav)
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch rovi_bringup rovi.launch.py \
        robot_mode:=sim \
        stack:="${mode}" \
        joy_enabled:=false \
        "$@"
      ;;
    gazebo|backend)
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch rovi_sim gazebo_sim.launch.py "$@"
      ;;
    *)
      echo "[rovi_env] Usage: sim {teleop|mapping|localization|nav|gazebo} [ros2 launch args...]" >&2
      return 2
      ;;
  esac
}

view() {
  local mode="${1:-nav}"
  if [ $# -gt 0 ]; then
    shift
  fi

  case "${mode}" in
    nav)
      rviz2 -d "install/rovi_description/share/rovi_description/rviz/rovi_nav.rviz" "$@"
      ;;
    mapping)
      rviz2 -d "install/rovi_description/share/rovi_description/rviz/rovi_map.rviz" "$@"
      ;;
    teleop)
      rviz2 -d "install/rovi_description/share/rovi_description/rviz/rovi_odom.rviz" "$@"
      ;;
    offline)
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch rovi_bringup rovi.launch.py \
        robot_mode:=offline \
        stack:=offline \
        "$@"
      ;;
    *)
      echo "[rovi_env] Usage: view {nav|mapping|teleop|offline} [...args]" >&2
      return 2
      ;;
  esac
}

bridge() {
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml "$@"
}

talk() {
  ros2 run demo_nodes_cpp talker "$@"
}

listen() {
  ros2 run demo_nodes_py listener "$@"
}

install_ros_deps() {
  local -a pkgs=(
    libsdformat14
    ros-jazzy-joy
    ros-jazzy-teleop-twist-joy
    ros-jazzy-twist-mux
    ros-jazzy-rosbag2
    ros-jazzy-rosbag2-compression-zstd
    ros-jazzy-diagnostic-updater
    python3-psutil
    ros-jazzy-robot-state-publisher
    ros-jazzy-joint-state-publisher-gui
    ros-jazzy-rviz2
    ros-jazzy-foxglove-bridge
    ros-jazzy-rplidar-ros
    ros-jazzy-slam-toolbox
    ros-jazzy-robot-localization
    ros-jazzy-nav2-bringup
    ros-jazzy-nav2-rviz-plugins
    ros-jazzy-imu-filter-madgwick
  )

  if [ "$(id -u)" -eq 0 ]; then
    apt update
    apt install -y "${pkgs[@]}"
  else
    sudo apt update
    sudo apt install -y "${pkgs[@]}"
  fi
}
