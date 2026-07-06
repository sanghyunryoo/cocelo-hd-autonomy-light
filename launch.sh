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
  --livox-interface IFACE
                      MID360 wired interface. Default: enP8p1s0.
  --livox-host-ip CIDR
                      MID360 host IP/CIDR. Default: 192.168.1.50/24.
  --livox-lidar-ip IP
                      MID360 LiDAR IP. Default: 192.168.1.12.
  --skip-livox-network
                      Do not configure MID360 host network/config.

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
LIVOX_INTERFACE="${AUTONOMY_LIGHT_LIVOX_INTERFACE:-}"
LIVOX_HOST_IP="${AUTONOMY_LIGHT_LIVOX_HOST_IP:-}"
LIVOX_LIDAR_IP="${AUTONOMY_LIGHT_LIVOX_LIDAR_IP:-}"
LIVOX_CONFIG_PATH="${AUTONOMY_LIGHT_LIVOX_CONFIG_PATH:-}"
SKIP_LIVOX_NETWORK="${AUTONOMY_LIGHT_SKIP_LIVOX_NETWORK:-false}"

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
    --livox-interface)
      LIVOX_INTERFACE="${2:?--livox-interface requires an interface name}"
      shift 2
      ;;
    --livox-interface=*)
      LIVOX_INTERFACE="${1#--livox-interface=}"
      shift
      ;;
    --livox-host-ip)
      LIVOX_HOST_IP="${2:?--livox-host-ip requires an IP or CIDR}"
      shift 2
      ;;
    --livox-host-ip=*)
      LIVOX_HOST_IP="${1#--livox-host-ip=}"
      shift
      ;;
    --livox-lidar-ip)
      LIVOX_LIDAR_IP="${2:?--livox-lidar-ip requires an IP}"
      shift 2
      ;;
    --livox-lidar-ip=*)
      LIVOX_LIDAR_IP="${1#--livox-lidar-ip=}"
      shift
      ;;
    --skip-livox-network)
      SKIP_LIVOX_NETWORK="true"
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

read_livox_network_config() {
  local config_file="$1"
  /usr/bin/python3 - "${config_file}" <<'PY'
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}

params = ((data.get("autonomy_light") or {}).get("ros__parameters")) or {}

def value(name, default=""):
    item = params.get(name, default)
    return "" if item is None else str(item).strip()

print(value("livox_interface", "enP8p1s0"))
print(value("livox_host_ip", "192.168.1.50/24"))
print(value("livox_lidar_ip", "192.168.1.12"))
print(value("lidar_frame", "livox_frame"))
PY
}

interface_has_ip() {
  local iface="$1"
  local local_host="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | cut -d/ -f1 | grep -Fxq "${local_host}"
}

interface_has_cidr() {
  local iface="$1"
  local cidr="$2"
  ip -o -4 addr show dev "${iface}" | awk '{print $4}' | grep -Fxq "${cidr}"
}

write_livox_mid360_config() {
  local output_path="$1"
  local host_ip="$2"
  local lidar_ip="$3"
  mkdir -p "$(dirname "${output_path}")"
  /usr/bin/python3 - "${output_path}" "${host_ip}" "${lidar_ip}" <<'PY'
import json
import sys

output_path, host_ip, lidar_ip = sys.argv[1:4]
config = {
    "lidar_summary_info": {"lidar_type": 8},
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100,
            "push_msg_port": 56200,
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500,
        },
        "host_net_info": {
            "cmd_data_ip": host_ip,
            "cmd_data_port": 56101,
            "push_msg_ip": host_ip,
            "push_msg_port": 56201,
            "point_data_ip": host_ip,
            "point_data_port": 56301,
            "imu_data_ip": host_ip,
            "imu_data_port": 56401,
            "log_data_ip": "",
            "log_data_port": 56501,
        },
    },
    "lidar_configs": [
        {
            "ip": lidar_ip,
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0,
            },
        }
    ],
}
with open(output_path, "w", encoding="utf-8") as stream:
    json.dump(config, stream, indent=2)
    stream.write("\n")
PY
}

configure_livox_network() {
  if [[ "${MODE}" != "real" || "${NO_DRIVERS}" == "true" ]]; then
    return
  fi
  case "${SKIP_LIVOX_NETWORK}" in
    true|1|yes|on)
      return
      ;;
  esac

  if [[ -n "${LIVOX_CONFIG_PATH}" ]]; then
    return
  fi

  if ! command -v ip >/dev/null 2>&1; then
    echo "warning: 'ip' command not found; skipping Livox fixed network setup." >&2
    return
  fi

  local cfg_iface cfg_host_cidr cfg_lidar_ip cfg_frame_id
  if [[ -f "${CONFIG_FILE}" ]]; then
    mapfile -t livox_config < <(read_livox_network_config "${CONFIG_FILE}")
    cfg_iface="${livox_config[0]:-}"
    cfg_host_cidr="${livox_config[1]:-}"
    cfg_lidar_ip="${livox_config[2]:-}"
    cfg_frame_id="${livox_config[3]:-}"
    LIVOX_INTERFACE="${LIVOX_INTERFACE:-${cfg_iface}}"
    LIVOX_HOST_IP="${AUTONOMY_LIGHT_LIVOX_HOST_IP:-${LIVOX_HOST_IP:-${cfg_host_cidr}}}"
    LIVOX_LIDAR_IP="${AUTONOMY_LIGHT_LIVOX_LIDAR_IP:-${LIVOX_LIDAR_IP:-${cfg_lidar_ip}}}"
    LIVOX_FRAME_ID="${cfg_frame_id:-livox_frame}"
  else
    LIVOX_FRAME_ID="livox_frame"
  fi

  local host_cidr="${LIVOX_HOST_IP:-192.168.1.50/24}"
  local lidar_ip="${LIVOX_LIDAR_IP:-192.168.1.12}"
  local host_ip="${host_cidr%%/*}"
  [[ "${host_cidr}" == */* ]] || host_cidr="${host_cidr}/24"

  local iface="${LIVOX_INTERFACE:-enP8p1s0}"
  if [[ ! -d "/sys/class/net/${iface}" ]]; then
    echo "warning: Livox fixed network interface does not exist: ${iface}" >&2
    echo "warning: set --livox-interface or livox_interface in the autonomy-light config." >&2
    return
  fi

  sudo ip link set "${iface}" up
  if ! interface_has_cidr "${iface}" "${host_cidr}"; then
    if ! interface_has_ip "${iface}" "${host_ip}"; then
      echo "Configuring Livox MID360 host network: ${iface} ${host_cidr}"
      sudo ip addr add "${host_cidr}" dev "${iface}"
    fi
  fi

  LIVOX_CONFIG_PATH="/tmp/autonomy_light_livox_mid360_config.json"
  write_livox_mid360_config "${LIVOX_CONFIG_PATH}" "${host_ip}" "${lidar_ip}"
  echo "Livox MID360 network: iface=${iface} host=${host_ip} lidar=${lidar_ip} config=${LIVOX_CONFIG_PATH}"
}

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

configure_livox_network

if [[ -n "${LIVOX_CONFIG_PATH}" && "${MODE}" == "real" && "${NO_DRIVERS}" != "true" ]]; then
  LIVOX_DRIVER_COMMAND=(
    "ros2"
    "run"
    "livox_ros_driver2"
    "livox_ros_driver2_node"
    "--ros-args"
    "-r"
    "__node:=livox_lidar_publisher"
    "-p"
    "xfer_format:=1"
    "-p"
    "multi_topic:=0"
    "-p"
    "data_src:=0"
    "-p"
    "publish_freq:=10.0"
    "-p"
    "output_data_type:=0"
    "-p"
    "frame_id:=${LIVOX_FRAME_ID:-mid360}"
    "-p"
    "lvx_file_path:=/tmp/autonomy_light_livox.lvx"
    "-p"
    "user_config_path:=${LIVOX_CONFIG_PATH}"
    "-p"
    "cmdline_input_bd_code:=livox0000000001"
  )
  LIVOX_DRIVER_COMMAND_PARAM="$(
    /usr/bin/python3 - "${LIVOX_DRIVER_COMMAND[@]}" <<'PY'
import json
import sys

print(json.dumps(sys.argv[1:]))
PY
  )"
  EXTRA_ROS_ARGS+=("-p" "lidar_driver_command:=${LIVOX_DRIVER_COMMAND_PARAM}")
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
