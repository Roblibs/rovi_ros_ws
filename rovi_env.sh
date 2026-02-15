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

if [ "${ROVI_SKIP_OPENNI2:-}" != "1" ]; then
  export OPENNI2_REDIST=$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer
  if [ -d "$OPENNI2_REDIST" ]; then
    ln -sf "$OPENNI2_REDIST/libOpenNI2.so" "$OPENNI2_REDIST/libOpenNI2.so.0" 2>/dev/null || true
    export LD_LIBRARY_PATH=$OPENNI2_REDIST:$LD_LIBRARY_PATH
  fi
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
#ensure_ros2_daemon

clean() {
  rm -rf "${ROVI_ROS_WS_DIR}/build" "${ROVI_ROS_WS_DIR}/install" "${ROVI_ROS_WS_DIR}/log"
}

rovi_stop() {
  python3 "${ROVI_ROS_WS_DIR}/tools/rovi_stop.py" "$@"
}

if ! alias stop >/dev/null 2>&1 && ! declare -F stop >/dev/null 2>&1; then
  function stop { rovi_stop "$@"; }
fi

build() {
  local colcon_cmd=""
  if [ -x "/usr/bin/colcon" ]; then
    colcon_cmd="/usr/bin/colcon"
  elif command -v colcon >/dev/null 2>&1; then
    colcon_cmd="$(command -v colcon)"
  fi
  if [ -z "${colcon_cmd}" ]; then
    echo "[rovi_env] Missing colcon command (install ROS Jazzy colcon packages)." >&2
    return 1
  fi
  local -a extra_args=()
  if [ "${ROVI_SKIP_OPENNI2:-}" = "1" ]; then
    extra_args+=(--packages-skip openni2_camera)
  fi
  # Build policy: keep system Python as the authority for CMake-based builds
  # (rosidl, etc), and never depend on the workspace venv for builds.
  #
  # Runtime can still use the venv (e.g. for Python deps), but build must be
  # stable against venv package drift.
  (
    _remove_path_entry() {
      local entry="$1"
      local wrapped=":${PATH}:"
      wrapped="${wrapped//:${entry}:/:}"
      wrapped="${wrapped#:}"
      wrapped="${wrapped%:}"
      PATH="${wrapped}"
    }

    # Strip venv from PATH for this build invocation only (even if a venv was
    # activated earlier, or its bin path was manually added).
    _remove_path_entry "${ROVI_ROS_WS_DIR}/.venv/bin"
    if [ -n "${VIRTUAL_ENV:-}" ]; then
      _remove_path_entry "${VIRTUAL_ENV}/bin"
      unset VIRTUAL_ENV
    fi
    unset PYTHONHOME

    local -a cmake_args=(
      --cmake-args
      -DPython3_EXECUTABLE=/usr/bin/python3
    )
    "${colcon_cmd}" build "${extra_args[@]}" "${cmake_args[@]}" "$@"
  )
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

ensure_venv() {
  if [ ! -f "${ROVI_ROS_WS_DIR}/.venv/bin/activate" ]; then
    echo "[rovi_env] Missing ${ROVI_ROS_WS_DIR}/.venv/bin/activate (run: uv sync)" >&2
    return 1
  fi
  if [ "${VIRTUAL_ENV:-}" != "${ROVI_ROS_WS_DIR}/.venv" ]; then
    # shellcheck disable=SC1091
    source "${ROVI_ROS_WS_DIR}/.venv/bin/activate"
  fi
}

ws() {
  cd "${ROVI_ROS_WS_DIR}" || return
  setup
  ensure_venv
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

camera() {
  ros2 launch rovi_bringup rovi.launch.py robot_mode:=real stack:=camera rviz:=false "$@"
}

calib() {
  ros2 run camera_calibration cameracalibrator \
    --size 8x5 --square 0.028 \
    --ros-args \
    --remap image:=/camera/color/image \
    --remap camera/set_camera_info:=/camera/color/v4l2_camera/set_camera_info \
    "$@"
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
    teleop|camera|mapping|localization|nav)
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
      echo "[rovi_env] Usage: sim {teleop|camera|mapping|localization|nav|gazebo} [ros2 launch args...]" >&2
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
      rviz_cfg="${ROVI_ROS_WS_DIR}/src/rovi_description/rviz/rovi_nav.rviz"
      rviz2 -d "${rviz_cfg}" "$@"
      ;;
    mapping)
      rviz_cfg="${ROVI_ROS_WS_DIR}/src/rovi_description/rviz/rovi_map.rviz"
      rviz2 -d "${rviz_cfg}" "$@"
      ;;
    teleop)
      rviz_cfg="${ROVI_ROS_WS_DIR}/src/rovi_description/rviz/rovi_odom.rviz"
      rviz2 -d "${rviz_cfg}" "$@"
      ;;
    camera)
      rviz_cfg="${ROVI_ROS_WS_DIR}/src/rovi_description/rviz/rovi_camera.rviz"
      rviz2 -d "${rviz_cfg}" "$@"
      ;;
    offline)
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch rovi_bringup rovi.launch.py \
        robot_mode:=offline \
        stack:=offline \
        "$@"
      ;;
    *)
      echo "[rovi_env] Usage: view {nav|mapping|teleop|camera|offline} [...args]" >&2
      return 2
      ;;
  esac
}

log(){
  ros2 run rqt_console rqt_console "$@"
}

foxglove() {
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
    ros-jazzy-joy
    ros-jazzy-teleop-twist-joy
    ros-jazzy-twist-mux
    ros-jazzy-ros-gz-sim
    ros-jazzy-ros-gz-bridge
    ros-jazzy-rosbag2
    ros-jazzy-rosbag2-compression-zstd
    ros-jazzy-diagnostic-updater
    python3-psutil
    ros-jazzy-robot-state-publisher
    ros-jazzy-joint-state-publisher-gui
    ros-jazzy-rviz2
    ros-jazzy-foxglove-bridge
    ros-jazzy-camera-info-manager
    ros-jazzy-v4l2-camera
    ros-jazzy-camera-calibration
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
