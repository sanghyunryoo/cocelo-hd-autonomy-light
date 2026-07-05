#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  launch.sh [--real|--sim] [--no-drivers] [options] [-- <extra ros args>]

Modes:
  --real              Real robot mode. Starts Livox driver and Point-LIO. Default.
  --sim               Simulation mode. Does not start Livox driver, uses sim time,
                      and reads simulated /<prefix>/livox/lidar + /<prefix>/livox/imu.
  --no-drivers        Start autonomy_light only; disables both Livox driver and Point-LIO.

Options:
  --config FILE       Override config yaml.
  --sim-topic-prefix PREFIX
                      Simulation topic prefix. Default: /f4 or AUTONOMY_LIGHT_SIM_TOPIC_PREFIX.
  --raw-lidar-topic TOPIC
                      Override raw LiDAR topic.
  --raw-imu-topic TOPIC
                      Override raw IMU topic.

Examples:
  ./launch.sh --real
  ./launch.sh --sim
  ./launch.sh --sim --sim-topic-prefix /f16
  ./launch.sh --no-drivers
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_PREFIX_DIR="$(cd -- "${SCRIPT_DIR}/../../.." 2>/dev/null && pwd || true)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
MODE="real"
NO_DRIVERS="false"
SIM_TOPIC_PREFIX="${AUTONOMY_LIGHT_SIM_TOPIC_PREFIX:-/f4}"
RAW_LIDAR_TOPIC=""
RAW_IMU_TOPIC=""

if [[ -n "${AUTONOMY_LIGHT_CONFIG:-}" ]]; then
  CONFIG_FILE="${AUTONOMY_LIGHT_CONFIG}"
elif [[ -f "${SCRIPT_DIR}/config/autonomy_light.yaml" ]]; then
  CONFIG_FILE="${SCRIPT_DIR}/config/autonomy_light.yaml"
else
  CONFIG_FILE="${SCRIPT_DIR}/../../share/autonomy_light/config/autonomy_light.yaml"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --real|real)
      MODE="real"
      shift
      ;;
    --sim|sim|simulation:=true|simulation:=1|simulation:=yes|simulation:=on)
      MODE="sim"
      shift
      ;;
    simulation:=false|simulation:=0|simulation:=no|simulation:=off)
      MODE="real"
      shift
      ;;
    --no-drivers)
      NO_DRIVERS="true"
      shift
      ;;
    --config)
      CONFIG_FILE="${2:?--config requires a file path}"
      shift 2
      ;;
    --config=*)
      CONFIG_FILE="${1#--config=}"
      shift
      ;;
    --sim-topic-prefix)
      SIM_TOPIC_PREFIX="${2:?--sim-topic-prefix requires a prefix}"
      shift 2
      ;;
    --sim-topic-prefix=*)
      SIM_TOPIC_PREFIX="${1#--sim-topic-prefix=}"
      shift
      ;;
    --raw-lidar-topic)
      RAW_LIDAR_TOPIC="${2:?--raw-lidar-topic requires a topic}"
      shift 2
      ;;
    --raw-lidar-topic=*)
      RAW_LIDAR_TOPIC="${1#--raw-lidar-topic=}"
      shift
      ;;
    --raw-imu-topic)
      RAW_IMU_TOPIC="${2:?--raw-imu-topic requires a topic}"
      shift 2
      ;;
    --raw-imu-topic=*)
      RAW_IMU_TOPIC="${1#--raw-imu-topic=}"
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      break
      ;;
  esac
done

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
fi

if [[ -f "${SOURCE_WORKSPACE_DIR}/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${SOURCE_WORKSPACE_DIR}/install/setup.bash"
  set -u
elif [[ -n "${INSTALL_PREFIX_DIR}" && -f "${INSTALL_PREFIX_DIR}/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${INSTALL_PREFIX_DIR}/setup.bash"
  set -u
fi

EXTRA_ROS_ARGS=()
if [[ "${MODE}" == "sim" ]]; then
  sim_prefix="${SIM_TOPIC_PREFIX%/}"
  [[ "${sim_prefix}" == "/" ]] && sim_prefix=""
  RAW_LIDAR_TOPIC="${RAW_LIDAR_TOPIC:-${sim_prefix}/livox/lidar}"
  RAW_IMU_TOPIC="${RAW_IMU_TOPIC:-${sim_prefix}/livox/imu}"
  EXTRA_ROS_ARGS+=(
    "-p" "start_lidar_driver:=false"
    "-p" "start_point_lio:=true"
    "-p" "use_sim_time:=true"
    "-p" "child_use_sim_time:=true"
    "-p" "raw_lidar_topic:=${RAW_LIDAR_TOPIC}"
    "-p" "raw_imu_topic:=${RAW_IMU_TOPIC}"
  )
else
  EXTRA_ROS_ARGS+=(
    "-p" "start_lidar_driver:=true"
    "-p" "start_point_lio:=true"
    "-p" "use_sim_time:=false"
    "-p" "child_use_sim_time:=false"
  )
fi

if [[ "${NO_DRIVERS}" == "true" ]]; then
  EXTRA_ROS_ARGS+=("-p" "start_lidar_driver:=false" "-p" "start_point_lio:=false")
fi

echo "autonomy-light mode=${MODE} config=${CONFIG_FILE}"
if [[ "${MODE}" == "sim" ]]; then
  echo "simulation topics: lidar=${RAW_LIDAR_TOPIC} imu=${RAW_IMU_TOPIC}"
fi

exec ros2 run autonomy_light autonomy_light \
  --ros-args \
  --params-file "${CONFIG_FILE}" \
  "${EXTRA_ROS_ARGS[@]}" \
  "$@"
