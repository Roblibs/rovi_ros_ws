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

_rovi_filter_prefix_paths() {
  # When `install/` is deleted (e.g. after `clean`) but the current shell still
  # has the overlay in AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH, `colcon` will warn
  # about non-existent prefix entries. Filter them out to keep the environment
  # consistent with what's on disk.
  local install_prefix="${ROVI_ROS_WS_DIR}/install"

  _filter_colon_var() {
    local var_name="$1"
    local value="${!var_name:-}"
    [ -z "${value}" ] && return 0

    local -a parts=()
    local out=""
    IFS=':' read -r -a parts <<<"${value}"
    for p in "${parts[@]}"; do
      [ -z "${p}" ] && continue
      case "${p}" in
        "${install_prefix}"|${install_prefix}/*) continue ;;
      esac
      if [ -z "${out}" ]; then
        out="${p}"
      else
        out="${out}:${p}"
      fi
    done
    export "${var_name}=${out}"
  }

  _filter_colon_var AMENT_PREFIX_PATH
  _filter_colon_var CMAKE_PREFIX_PATH
  _filter_colon_var COLCON_PREFIX_PATH
}

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

_rovi_args_have_gateway_enabled() {
  local arg
  for arg in "$@"; do
    if [[ "${arg}" == gateway_enabled:=* ]]; then
      return 0
    fi
  done
  return 1
}

_rovi_gateway_service_active() {
  command -v systemctl >/dev/null 2>&1 || return 1
  systemctl is-active --quiet rovi-gateway.service >/dev/null 2>&1
}

_rovi_port_listening() {
  local port="$1"
  command -v ss >/dev/null 2>&1 || return 1
  ss -lnt 2>/dev/null | rg -q ":${port}\\b"
}

_rovi_launch_real_stack() {
  if [ $# -lt 1 ]; then
    echo "[rovi_env] Usage: _rovi_launch_real_stack <stack> [ros2 launch args...]" >&2
    return 2
  fi

  local stack="$1"
  shift || true

  local -a launch_args=(
    robot_mode:=real
    stack:="${stack}"
    rviz:=false
  )

  # If the always-on gateway plane is already running under systemd,
  # make stack launches "stack-only" automatically to avoid a second gateway.
  if _rovi_args_have_gateway_enabled "$@"; then
    echo "[rovi_env] gateway_enabled explicitly set by caller; not auto-adjusting." >&2
  else
    if _rovi_gateway_service_active; then
      echo "[rovi_env] rovi-gateway.service is active; adding gateway_enabled:=false (stack-only launch)" >&2
      launch_args+=(gateway_enabled:=false)
      if ! _rovi_port_listening 50051; then
        echo "[rovi_env] WARN: rovi-gateway.service is active but :50051 is not listening yet; display/UI may be blank until it comes up." >&2
      fi
    else
      echo "[rovi_env] rovi-gateway.service is not active; leaving gateway_enabled default (this launch will start a gateway plane)." >&2
    fi
  fi

  ros2 launch rovi_bringup rovi.launch.py "${launch_args[@]}" "$@"
}

# Default DDS mode for this workspace:
rovi_dds_default
#ensure_ros2_daemon

clean() {
  rm -rf "${ROVI_ROS_WS_DIR}/build" "${ROVI_ROS_WS_DIR}/install" "${ROVI_ROS_WS_DIR}/log"
  _rovi_filter_prefix_paths
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
    _rovi_filter_prefix_paths

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

ws() {
  cd "${ROVI_ROS_WS_DIR}" || return
  setup
}

record() {
  ros2 run rovi_bringup rovi_bag record "$@"
}

play() {
  ros2 run rovi_bringup rovi_bag play "$@"
}

teleop() {
  _rovi_launch_real_stack teleop "$@"
}

camera() {
  _rovi_launch_real_stack camera "$@"
}

calib_color() {
  ros2 run camera_calibration cameracalibrator \
    --size 8x5 --square 0.028 \
    --ros-args \
    --remap image:=/camera/color/image \
    --remap camera/set_camera_info:=/camera/color/v4l2_camera/set_camera_info \
    "$@"
}

calib() {
  echo "[rovi_env] 'calib' is deprecated; use 'calib_color'." >&2
  calib_color "$@"
}

keyboard() {
  python3 "${ROVI_ROS_WS_DIR}/tools/rovi_keyboard.py" "$@"
}

mapping() {
  _rovi_launch_real_stack mapping "$@"
}

localization() {
  _rovi_launch_real_stack localization "$@"
}

nav() {
  _rovi_launch_real_stack nav "$@"
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
      echo "[rovi_env] Usage: sim {teleop|camera|mapping|localization|nav|gazebo|backend} [ros2 launch args...]" >&2
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

check() {
  command -v systemctl >/dev/null 2>&1 || {
    echo "[rovi_env] systemctl not found; systemd service checks are unavailable on this host." >&2
    return 1
  }

  local services_dir="${ROVI_ROS_WS_DIR}/services"
  local -a units=()
  if [ -d "${services_dir}" ]; then
    local f
    for f in "${services_dir}"/rovi-*.service; do
      [ -e "${f}" ] || continue
      units+=("$(basename "${f}")")
    done
  fi

  if [ "${#units[@]}" -eq 0 ]; then
    echo "[rovi_env] No known rovi-*.service units found under ${services_dir}." >&2
    return 1
  fi

  local unit
  for unit in "${units[@]}"; do
    local active enabled sub_state result main_pid exec_status

    active="$(systemctl is-active "${unit}" 2>/dev/null || true)"
    enabled="$(systemctl is-enabled "${unit}" 2>/dev/null || echo "n/a")"

    if [ "${active}" = "unknown" ] || [ -z "${active}" ]; then
      active="not-found"
      sub_state="-"
      result="-"
      main_pid="-"
      exec_status="-"
    else
      sub_state="$(systemctl show -p SubState --value "${unit}" 2>/dev/null || echo "-")"
      result="$(systemctl show -p Result --value "${unit}" 2>/dev/null || echo "-")"
      main_pid="$(systemctl show -p MainPID --value "${unit}" 2>/dev/null || echo "-")"
      exec_status="$(systemctl show -p ExecMainStatus --value "${unit}" 2>/dev/null || echo "-")"
      [ -z "${sub_state}" ] && sub_state="-"
      [ -z "${result}" ] && result="-"
      [ -z "${main_pid}" ] && main_pid="-"
      [ -z "${exec_status}" ] && exec_status="-"
    fi

    printf "%s active=%s sub=%s result=%s enabled=%s pid=%s code=%s\n" \
      "${unit}" "${active}" "${sub_state}" "${result}" "${enabled}" "${main_pid}" "${exec_status}"
  done
}

_rovi_require_openni2() {
  if [ "${ROVI_SKIP_OPENNI2:-}" = "1" ]; then
    echo "[rovi_env] OpenNI2 helpers disabled (ROVI_SKIP_OPENNI2=1)." >&2
    return 2
  fi
  if [ -z "${OPENNI2_REDIST:-}" ] || [ ! -d "${OPENNI2_REDIST}" ]; then
    echo "[rovi_env] OPENNI2_REDIST is not set or missing." >&2
    echo "[rovi_env] Expected: \$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer" >&2
    return 2
  fi
  return 0
}

_rovi_build_openni2_tool() {
  local src_rel="$1"
  local out_rel="$2"

  _rovi_require_openni2 || return $?

  local src="${ROVI_ROS_WS_DIR}/${src_rel}"
  local out="${ROVI_ROS_WS_DIR}/${out_rel}"

  command -v g++ >/dev/null 2>&1 || {
    echo "[rovi_env] Missing g++ (install build-essential)." >&2
    return 1
  }

  if [ ! -f "${src}" ]; then
    echo "[rovi_env] Missing source: ${src}" >&2
    return 1
  fi

  if [ -x "${out}" ] && [ "${out}" -nt "${src}" ]; then
    return 0
  fi

  local inc="${HOME}/OpenNI/OpenNI_2.3.0/sdk/Include"
  if [ ! -d "${inc}" ]; then
    echo "[rovi_env] Missing OpenNI2 headers: ${inc}" >&2
    return 1
  fi

  (
    cd "$(dirname "${out}")" || exit 1
    g++ -std=c++17 "$(basename "${src}")" \
      -I"${inc}" -L"${OPENNI2_REDIST}" \
      -lOpenNI2 -Wl,-rpath,"${OPENNI2_REDIST}" \
      -O2 -o "$(basename "${out}")"
  )
}

depth() {
  local sub="${1:-}"
  if [ -n "${sub}" ]; then
    shift
  fi

  case "${sub}" in
    list)
      _rovi_require_openni2 || return $?
      "${ROVI_ROS_WS_DIR}/tools/depth/openni2_list_modes" "$@"
      ;;
    snapshot|snap)
      _rovi_build_openni2_tool tools/depth/openni2_snapshot_depth.cpp tools/depth/openni2_snapshot_depth || return $?
      "${ROVI_ROS_WS_DIR}/tools/depth/openni2_snapshot_depth" "$@"
      ;;
    ros_snapshot|ros-snapshot)
      python3 "${ROVI_ROS_WS_DIR}/tools/depth/ros_snapshot.py" "$@"
      ;;
    view|viewer)
      _rovi_require_openni2 || return $?
      "${OPENNI2_REDIST}/NiViewer" "$@"
      ;;
    *)
      echo "[rovi_env] Usage: depth {list|snapshot|ros_snapshot|view} [...args]" >&2
      echo "[rovi_env] Examples:" >&2
      echo "  depth list" >&2
      echo "  depth snapshot --out-dir output/openni2_snapshot" >&2
      echo "  depth ros_snapshot --topic /camera/depth/image_raw --out-dir output/cam_snapshot_ros" >&2
      echo "  depth view" >&2
      return 2
      ;;
  esac
}

tools() {
  local domain="${1:-}"
  if [ -n "${domain}" ]; then
    shift
  fi

  case "${domain}" in
    depth)
      depth "$@"
      ;;
    *)
      echo "[rovi_env] Usage: tools depth <subcommand> [...args]" >&2
      echo "[rovi_env] Example: tools depth snapshot" >&2
      return 2
      ;;
  esac
}
