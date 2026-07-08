#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/package_deb.sh [options]

Build a source-free Jetson runtime .deb for autonomy-light.

Assumption:
  The target already has Ubuntu and ROS 2 Humble installed under /opt/ros/humble.
  This package bundles the autonomy-light runtime, generated custom message
  interface, vendored Livox ROS driver2 install tree, Point-LIO runtime, docs,
  examples, and Livox-SDK2 shared library when found under /usr/local/lib.

Options:
  --version VERSION       Debian package version. Default: package.xml version.
  --revision REV          Debian revision suffix. Default: 1.
  --output-dir DIR        Output directory. Default: dist.
  --ros-distro NAME       ROS distro. Default: ROS_DISTRO or humble.
  --skip-build            Package the existing install tree without rebuilding.
  --no-strip              Do not strip runtime binaries/libraries.
  -h, --help              Show this help.

The package installs:
  /opt/cocelo/autonomy-light/install   ROS 2 runtime install tree
  /etc/cocelo/autonomy-light           Editable runtime config
  /usr/bin/autonomy-light              Runtime launcher
  /usr/bin/autonomy-light-doctor       Receiver/network self-check helper
  /usr/bin/autonomy-light-heightmap-example
                                      HeightMap subscriber SDK example
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd -- "${REPO_DIR}/../.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
VERSION=""
REVISION="1"
OUTPUT_DIR="${REPO_DIR}/dist"
SKIP_BUILD="false"
DO_STRIP="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --version)
      VERSION="${2:?--version requires a value}"
      shift 2
      ;;
    --version=*)
      VERSION="${1#--version=}"
      shift
      ;;
    --revision)
      REVISION="${2:?--revision requires a value}"
      shift 2
      ;;
    --revision=*)
      REVISION="${1#--revision=}"
      shift
      ;;
    --output-dir)
      OUTPUT_DIR="${2:?--output-dir requires a directory}"
      shift 2
      ;;
    --output-dir=*)
      OUTPUT_DIR="${1#--output-dir=}"
      shift
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:?--ros-distro requires a name}"
      shift 2
      ;;
    --ros-distro=*)
      ROS_DISTRO_NAME="${1#--ros-distro=}"
      shift
      ;;
    --skip-build)
      SKIP_BUILD="true"
      shift
      ;;
    --no-strip)
      DO_STRIP="false"
      shift
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

read_package_version() {
  python3 - "${REPO_DIR}/package.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
print(root.findtext("version", "0.1.0"))
PY
}

if [[ -z "${VERSION}" ]]; then
  VERSION="$(read_package_version)"
fi

ARCH="$(dpkg --print-architecture)"
PACKAGE_VERSION="${VERSION}-${REVISION}"
PACKAGE_NAME="cocelo-autonomy-light"
STAGE_ROOT="$(mktemp -d /tmp/${PACKAGE_NAME}.XXXXXX)"
trap 'rm -rf "${STAGE_ROOT}"' EXIT

if [[ "${SKIP_BUILD}" != "true" ]]; then
  "${REPO_DIR}/build.sh" --packages livox_ros_driver2 autonomy_light
fi

INSTALL_ROOT="${WORKSPACE_DIR}/install"
for pkg in autonomy_light livox_ros_driver2; do
  if [[ ! -d "${INSTALL_ROOT}/${pkg}" ]]; then
    echo "error: missing install tree: ${INSTALL_ROOT}/${pkg}" >&2
    echo "hint: run ${REPO_DIR}/build.sh first, or omit --skip-build." >&2
    exit 1
  fi
done

mkdir -p \
  "${STAGE_ROOT}/DEBIAN" \
  "${STAGE_ROOT}/opt/cocelo/autonomy-light/install" \
  "${STAGE_ROOT}/opt/cocelo/autonomy-light/lib" \
  "${STAGE_ROOT}/etc/cocelo/autonomy-light" \
  "${STAGE_ROOT}/usr/bin" \
  "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}"

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -e "${src}" ]]; then
    cp -aL "${src}" "${dst}"
  fi
}

copy_install_tree() {
  local src="$1"
  local dst_parent="$2"
  local base
  base="$(basename "${src}")"

  mkdir -p "${dst_parent}"
  (
    cd "$(dirname "${src}")"
    find "${base}" \
      \( \( -type l ! -exec test -e {} \; \) -o -name __pycache__ -o -name '*.pyc' \) -prune -o \
      -print0 |
      tar --null --dereference --no-recursion --files-from - -cf -
  ) | (
    cd "${dst_parent}"
    tar -xf -
  )
}

for file in setup.bash setup.sh setup.zsh local_setup.bash local_setup.sh local_setup.zsh \
  _local_setup_util_sh.py COLCON_IGNORE .colcon_install_layout
do
  copy_if_exists "${INSTALL_ROOT}/${file}" "${STAGE_ROOT}/opt/cocelo/autonomy-light/install/"
done

copy_install_tree "${INSTALL_ROOT}/autonomy_light" "${STAGE_ROOT}/opt/cocelo/autonomy-light/install/"
copy_install_tree "${INSTALL_ROOT}/livox_ros_driver2" "${STAGE_ROOT}/opt/cocelo/autonomy-light/install/"

mkdir -p "${STAGE_ROOT}/opt/cocelo/autonomy-light/install/share/colcon-core/packages"
copy_if_exists \
  "${INSTALL_ROOT}/share/colcon-core/packages/autonomy_light" \
  "${STAGE_ROOT}/opt/cocelo/autonomy-light/install/share/colcon-core/packages/"
copy_if_exists \
  "${INSTALL_ROOT}/share/colcon-core/packages/livox_ros_driver2" \
  "${STAGE_ROOT}/opt/cocelo/autonomy-light/install/share/colcon-core/packages/"

cp -aL "${REPO_DIR}/config/autonomy_light.yaml" \
  "${STAGE_ROOT}/etc/cocelo/autonomy-light/autonomy_light.yaml"

cp -aL "${REPO_DIR}/README.md" "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/README.md"
if [[ -d "${REPO_DIR}/docs" ]]; then
  cp -aL "${REPO_DIR}/docs/." "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/"
fi
if [[ -d "${REPO_DIR}/examples" ]]; then
  mkdir -p "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/examples"
  cp -aL "${REPO_DIR}/examples/." "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/examples/"
  find "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/examples" \
    \( -name __pycache__ -o -name '*.pyc' \) -exec rm -rf {} +
fi

if [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]]; then
  cp -aL /usr/local/lib/liblivox_lidar_sdk_shared.so \
    "${STAGE_ROOT}/opt/cocelo/autonomy-light/lib/"
fi

cat > "${STAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/runtime_assumptions.txt" <<EOF
cocelo-autonomy-light runtime assumptions
=========================================

This .deb is intended to be self-contained for autonomy-light-specific runtime
artifacts while assuming the target system already provides:

- Ubuntu userspace for ${ARCH}
- ROS 2 ${ROS_DISTRO_NAME} installed at /opt/ros/${ROS_DISTRO_NAME}
- Python 3
- sudo and iproute2 for real MID360 network setup

Bundled in this package:

- autonomy_light binaries and launch/config resources
- autonomy_light/msg/HeightMap generated C++/Python type support
- vendored livox_ros_driver2 binaries and message type support
- Point-LIO runtime binary installed by autonomy_light
- Livox-SDK2 shared library when available at build time
- receiver SDK example and documentation

The package deliberately does not bundle /opt/ros/${ROS_DISTRO_NAME} or system
libraries such as glibc/libstdc++.
EOF

cat > "${STAGE_ROOT}/usr/bin/autonomy-light" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO_NAME}"
export AUTONOMY_LIGHT_CONFIG="\${AUTONOMY_LIGHT_CONFIG:-/etc/cocelo/autonomy-light/autonomy_light.yaml}"
export LD_LIBRARY_PATH="/opt/cocelo/autonomy-light/lib:\${LD_LIBRARY_PATH:-}"

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  set +u
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
else
  echo "error: /opt/ros/${ROS_DISTRO_NAME}/setup.bash not found" >&2
  exit 1
fi

set +u
source "/opt/cocelo/autonomy-light/install/setup.bash"
set -u

exec /opt/cocelo/autonomy-light/install/autonomy_light/lib/autonomy_light/launch.sh "\$@"
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/autonomy-light"

cat > "${STAGE_ROOT}/usr/bin/autonomy-light-doctor" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO_NAME}"
export LD_LIBRARY_PATH="/opt/cocelo/autonomy-light/lib:\${LD_LIBRARY_PATH:-}"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "/opt/cocelo/autonomy-light/install/setup.bash"
set -u

echo "== autonomy-light runtime =="
echo "config: \${AUTONOMY_LIGHT_CONFIG:-/etc/cocelo/autonomy-light/autonomy_light.yaml}"
echo

echo "== message interface =="
ros2 interface show autonomy_light/msg/HeightMap
echo

echo "== height map manual debug config =="
CONFIG_FILE="\${AUTONOMY_LIGHT_CONFIG:-/etc/cocelo/autonomy-light/autonomy_light.yaml}"
if grep -q '^[[:space:]]*height_map_debug:' "\${CONFIG_FILE}" 2>/dev/null; then
  grep -A8 '^[[:space:]]*height_map_debug:' "\${CONFIG_FILE}" || true
else
  echo "height_map_debug not configured; defaults: manual_mode=false manual_value=0.48"
fi
echo

echo "== external domain topics =="
ROS_DOMAIN_ID=0 ros2 topic list | egrep 'autonomy_light|/tf|/path' || true
echo

echo "== local_map leakage check on external domain =="
if ROS_DOMAIN_ID=0 ros2 topic list | grep -q '^/point_lio/local_map$'; then
  echo "warning: /point_lio/local_map is visible on ROS_DOMAIN_ID=0"
else
  echo "ok: /point_lio/local_map is not visible on ROS_DOMAIN_ID=0"
fi
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/autonomy-light-doctor"

cat > "${STAGE_ROOT}/usr/bin/autonomy-light-heightmap-example" <<EOF
#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO_NAME}"
export ROS_DOMAIN_ID="\${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="/opt/cocelo/autonomy-light/lib:\${LD_LIBRARY_PATH:-}"

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "/opt/cocelo/autonomy-light/install/setup.bash"
set -u

exec python3 /usr/share/doc/${PACKAGE_NAME}/examples/height_map_subscriber.py "\$@"
EOF
chmod 0755 "${STAGE_ROOT}/usr/bin/autonomy-light-heightmap-example"

cat > "${STAGE_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${PACKAGE_VERSION}
Section: robotics
Priority: optional
Architecture: ${ARCH}
Maintainer: Cocelo <todo@example.com>
Depends: bash, sudo, iproute2, python3
Description: Cocelo autonomy-light Jetson runtime
 Source-free runtime bundle for MID360 Livox driver, Point-LIO mapping, and
 control-facing autonomy-light height map outputs. This package assumes ROS 2
 ${ROS_DISTRO_NAME} is already installed on the target system.
EOF

cat > "${STAGE_ROOT}/DEBIAN/conffiles" <<'EOF'
/etc/cocelo/autonomy-light/autonomy_light.yaml
EOF

cat > "${STAGE_ROOT}/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e
echo "cocelo-autonomy-light installed."
echo "Edit config: /etc/cocelo/autonomy-light/autonomy_light.yaml"
echo "Run: autonomy-light --real"
echo "Check: autonomy-light-doctor"
EOF
chmod 0755 "${STAGE_ROOT}/DEBIAN/postinst"

if [[ "${DO_STRIP}" == "true" ]]; then
  while IFS= read -r file; do
    if file "${file}" | grep -Eq 'ELF .* (executable|shared object)'; then
      strip --strip-unneeded "${file}" 2>/dev/null || true
    fi
  done < <(find "${STAGE_ROOT}/opt/cocelo/autonomy-light" -type f)
fi

find "${STAGE_ROOT}" -type d -exec chmod 0755 {} +
find "${STAGE_ROOT}/opt/cocelo/autonomy-light" -type f -name '*.sh' -exec chmod 0755 {} +
find "${STAGE_ROOT}/opt/cocelo/autonomy-light/install" -type f \
  \( -path '*/lib/autonomy_light/*' -o -path '*/lib/livox_ros_driver2/*' \) \
  -exec chmod 0755 {} +

mkdir -p "${OUTPUT_DIR}"
DEB_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}_${PACKAGE_VERSION}_${ARCH}.deb"
dpkg-deb --build --root-owner-group "${STAGE_ROOT}" "${DEB_PATH}"

echo
echo "Created: ${DEB_PATH}"
echo
if [[ "${ARCH}" != "arm64" ]]; then
  echo "Warning: this package is ${ARCH}, not arm64. It will not run on Jetson arm64."
  echo "Build on an arm64 Jetson or an arm64 CI/builder to create a Jetson package."
  echo
fi
echo "Install on target ${ARCH} system:"
echo "  sudo apt install ./$(basename "${DEB_PATH}")"
echo "  autonomy-light --real"
echo
echo "Runtime assumption: ROS 2 ${ROS_DISTRO_NAME} is pre-installed at /opt/ros/${ROS_DISTRO_NAME}."
