#!/usr/bin/env bash
# Build a relocatable .deb from the copy-based autonomy_light install tree.
set -e -o pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${package_dir}/../.." && pwd)"
package_name="cocelo-autonomy-light"
package_version="$(sed -n 's|^[[:space:]]*<version>\(.*\)</version>[[:space:]]*$|\1|p' "${package_dir}/package.xml" | head -n 1)"
output_dir="${workspace_dir}/artifacts"
depends_override=""

usage() {
  cat <<'EOF'
Usage: ./package_deb.sh [--version VERSION] [--output-dir DIRECTORY] [--depends DEPENDENCIES]

Builds autonomy_light with a copy-based install and writes a .deb. The package
installs to /opt/cocelo/autonomy_light. ROS 2 and the non-APT third-party
runtime dependencies (livox_ros_driver2, GTSAM, nano_gicp) must be available on
the target system. --depends overrides the generated Debian Depends field.
EOF
}

while (($#)); do
  case "$1" in
    --version)
      shift
      package_version="${1:-}"
      ;;
    --output-dir)
      shift
      output_dir="${1:-}"
      ;;
    --depends)
      shift
      depends_override="${1:-}"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  if (($# == 0)); then
    echo "error: missing value for option" >&2
    exit 2
  fi
  shift
done

if [[ -z "${package_version}" ]]; then
  echo "error: package version is empty" >&2
  exit 2
fi
if ! command -v dpkg-deb >/dev/null; then
  echo "error: dpkg-deb is required (install dpkg)" >&2
  exit 1
fi

ros_distro="${ROS_DISTRO:-}"
if [[ -z "${ros_distro}" || ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  ros_distro=""
  for candidate in foxy humble; do
    if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
      ros_distro="${candidate}"
      break
    fi
  done
fi
if [[ -z "${ros_distro}" ]]; then
  echo "error: ROS 2 Foxy or Humble was not found under /opt/ros" >&2
  exit 1
fi

if [[ -z "${depends_override}" ]]; then
  depends_override="ros-${ros_distro}-ros-base, ros-${ros_distro}-geometry-msgs, ros-${ros_distro}-nav-msgs, ros-${ros_distro}-pcl-conversions, ros-${ros_distro}-pcl-ros, ros-${ros_distro}-sensor-msgs, ros-${ros_distro}-tf2-ros, ros-${ros_distro}-visualization-msgs"
fi

"${package_dir}/build.sh" --release
install_prefix="${workspace_dir}/install-release/autonomy_light"
if [[ ! -f "${install_prefix}/share/autonomy_light/local_setup.bash" ]]; then
  echo "error: release install is missing: ${install_prefix}" >&2
  exit 1
fi
if find "${install_prefix}" -type l -print -quit | rg -q .; then
  echo "error: release install contains symlinks; refusing to package it" >&2
  exit 1
fi

architecture="$(dpkg --print-architecture)"
mkdir -p -- "${output_dir}"
output_file="${output_dir}/${package_name}_${package_version}_${architecture}.deb"
stage_dir="$(mktemp -d "${TMPDIR:-/tmp}/${package_name}.XXXXXX")"
trap 'rm -rf -- "${stage_dir}"' EXIT

payload_prefix="${stage_dir}/opt/cocelo/autonomy_light"
mkdir -p "${stage_dir}/DEBIAN" "${payload_prefix}" "${stage_dir}/usr/share/doc/${package_name}"
cp -a "${install_prefix}/." "${payload_prefix}/"

cat > "${stage_dir}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${package_version}
Section: robotics
Priority: optional
Architecture: ${architecture}
Maintainer: autonomy_light maintainer <todo@example.com>
Depends: ${depends_override}
Description: Point-LIO based SLAM and elevation mapping runtime
 Installed under /opt/cocelo/autonomy_light.
EOF
cat > "${stage_dir}/usr/share/doc/${package_name}/README.Debian" <<EOF
Source the ROS 2 underlay, then source this package:
  source /opt/ros/${ros_distro}/setup.bash
  source /opt/cocelo/autonomy_light/share/autonomy_light/local_setup.bash

livox_ros_driver2, GTSAM, and nano_gicp are deployment dependencies supplied
by the target ROS environment.
EOF

dpkg-deb --build --root-owner-group "${stage_dir}" "${output_file}"
echo "created: ${output_file}"
