#!/usr/bin/env bash
# Start Point-LIO + loop closure + map saving (FULL_SLAM mode).
set -e -o pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${script_dir}/runtime_env.sh" ]]; then
  source "${script_dir}/runtime_env.sh"
else
  source "${script_dir}/scripts/runtime_env.sh"
fi
autonomy_light_prepare_runtime "${script_dir}"
set -u

use_sim_time=true
launch_args=()
while (($#)); do
  case "$1" in
    --sim) use_sim_time=true ;;
    --real) use_sim_time=false ;;
    --help|-h)
      cat <<'EOF'
Usage: ./mapping.sh [--sim|--real] [additional ROS launch arguments]

Starts FULL_SLAM. Saved maps use the paths configured in config/full_slam.yaml;
run this command from a writable directory when those paths are relative.
EOF
      exit 0 ;;
    *) launch_args+=("$1") ;;
  esac
  shift
done

exec ros2 launch autonomy_light full_slam.launch.py \
  full_slam:=true use_sim_time:="${use_sim_time}" "${launch_args[@]}"
