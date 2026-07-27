#!/usr/bin/env bash
# Shared by the source-tree and installed mapping/launch entry points.

autonomy_light_prepare_runtime() {
  local caller_dir="$1"
  local ros_distro="${ROS_DISTRO:-}"

  if [[ -f "${caller_dir}/package.xml" ]]; then
    AUTONOMY_LIGHT_SOURCE_TREE=true
    AUTONOMY_LIGHT_WORKSPACE="$(cd -- "${caller_dir}/../.." && pwd)"
    AUTONOMY_LIGHT_PREFIX="${AUTONOMY_LIGHT_WORKSPACE}/install/autonomy_light"
    AUTONOMY_LIGHT_SETUP="${AUTONOMY_LIGHT_WORKSPACE}/install/setup.bash"
    AUTONOMY_LIGHT_BUILD_SCRIPT="${caller_dir}/build.sh"
  else
    AUTONOMY_LIGHT_SOURCE_TREE=false
    AUTONOMY_LIGHT_PREFIX="$(cd -- "${caller_dir}/../.." && pwd)"
    AUTONOMY_LIGHT_SETUP="${AUTONOMY_LIGHT_PREFIX}/share/autonomy_light/local_setup.bash"
  fi

  if [[ -z "${ros_distro}" || ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    ros_distro=""
    local candidate
    for candidate in foxy humble; do
      if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
        ros_distro="${candidate}"
        break
      fi
    done
  fi
  if [[ -z "${ros_distro}" ]]; then
    echo "error: ROS 2 Foxy or Humble was not found under /opt/ros" >&2
    return 1
  fi

  source "/opt/ros/${ros_distro}/setup.bash"
  if [[ ! -f "${AUTONOMY_LIGHT_SETUP}" ]]; then
    if [[ "${AUTONOMY_LIGHT_SOURCE_TREE}" == true ]]; then
      "${AUTONOMY_LIGHT_BUILD_SCRIPT}"
    else
      echo "error: installed autonomy_light setup is missing: ${AUTONOMY_LIGHT_SETUP}" >&2
      return 1
    fi
  fi
  source "${AUTONOMY_LIGHT_SETUP}"

  # GTSAM is built in this workspace's private .deps prefix. libgtsam's
  # dependency on libmetis-gtsam is transitive, so the executable RUNPATH does
  # not cover it; expose the prefix to the dynamic loader explicitly.
  if [[ "${AUTONOMY_LIGHT_SOURCE_TREE}" == true ]]; then
    local gtsam_library_dir
    for gtsam_library_dir in \
      "${AUTONOMY_LIGHT_WORKSPACE}/.deps/install-system/lib" \
      "${AUTONOMY_LIGHT_WORKSPACE}/.deps/install/lib"; do
      if [[ -f "${gtsam_library_dir}/libmetis-gtsam.so" ]]; then
        export LD_LIBRARY_PATH="${gtsam_library_dir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
        break
      fi
    done
  fi
}
