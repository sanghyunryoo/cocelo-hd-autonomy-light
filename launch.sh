#!/usr/bin/env bash
# Start Point-LIO odometry and saved-map localization (LOCALIZATION mode).
set -e -o pipefail

usage() {
  cat <<'EOF'
Usage: ./launch.sh --map <global_map_refined.pcd> [--sim|--real] [additional ROS launch arguments]

--map is required in LOCALIZATION mode. The supplied PCD is used as the global
prior; Point-LIO remains the local odometry source.
EOF
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${script_dir}/runtime_env.sh" ]]; then
  source "${script_dir}/runtime_env.sh"
else
  source "${script_dir}/scripts/runtime_env.sh"
fi
autonomy_light_prepare_runtime "${script_dir}"
set -u

map_file=""
use_sim_time=true
launch_args=()
while (($#)); do
  case "$1" in
    --map)
      shift
      if (($# == 0)); then
        echo "error: --map requires a PCD path" >&2
        exit 2
      fi
      map_file="$1" ;;
    --sim) use_sim_time=true ;;
    --real) use_sim_time=false ;;
    --help|-h) usage; exit 0 ;;
    *) launch_args+=("$1") ;;
  esac
  shift
done

if [[ -z "${map_file}" ]]; then
  echo "error: --map <global_map_refined.pcd> is required" >&2
  usage >&2
  exit 2
fi
if [[ ! -f "${map_file}" ]]; then
  echo "error: map file does not exist: ${map_file}" >&2
  exit 2
fi
map_file="$(realpath -e -- "${map_file}")"

exec ros2 launch autonomy_light full_slam.launch.py \
  full_slam:=false saved_map_file:="${map_file}" use_sim_time:="${use_sim_time}" \
  "${launch_args[@]}"
