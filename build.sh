#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${package_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-foxy}"

source "/opt/ros/${ros_distro}/setup.bash"
set -u
cd "${workspace_dir}"

colcon --log-base "${workspace_dir}/log" build \
  --packages-select autonomy_light \
  --symlink-install \
  --build-base "${workspace_dir}/build" \
  --install-base "${workspace_dir}/install" \
  "$@"
