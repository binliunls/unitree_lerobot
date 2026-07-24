#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

gscam2_src="${GSCAM2_SRC:-/home/unitree/dev/gscam2}"
ros_distro="${ROS_DISTRO:-jazzy}"
package_name="${PACKAGE_NAME:-ros-${ros_distro}-gscam2}"
output_dir="${OUTPUT_DIR:-${repo_root}/dist}"
revision="${PACKAGE_REVISION:-1}"
arch="${DEB_ARCH:-$(dpkg --print-architecture)}"

if [[ ! -f "${gscam2_src}/package.xml" ]]; then
  echo "Expected gscam2 package.xml at ${gscam2_src}/package.xml" >&2
  exit 2
fi

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "ROS setup file not found: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 2
fi

version="$(
  python3 - "${gscam2_src}/package.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
print(root.findtext("version"))
PY
)"

if [[ -z "${version}" ]]; then
  echo "Could not read package version from ${gscam2_src}/package.xml" >&2
  exit 2
fi

deb_version="${version}-${revision}"
work_dir="$(mktemp -d)"

cleanup() {
  rm -rf "${work_dir}"
}
trap cleanup EXIT

workspace="${work_dir}/ws"
package_root="${work_dir}/${package_name}_${deb_version}_${arch}"
install_prefix="${package_root}/opt/ros/${ros_distro}"

mkdir -p "${workspace}/src/gscam2" "${package_root}/DEBIAN" "${output_dir}"

tar -C "${gscam2_src}" \
  --exclude='.git' \
  --exclude='build' \
  --exclude='install' \
  --exclude='log' \
  -cf - . | tar -C "${workspace}/src/gscam2" -xf -

set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u

colcon --log-base "${workspace}/log" build \
  --base-paths "${workspace}/src/gscam2" \
  --build-base "${workspace}/build" \
  --install-base "${install_prefix}" \
  --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

installed_size="$(
  du -sk "${package_root}" | awk '{print $1}'
)"

cat > "${package_root}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${deb_version}
Section: misc
Priority: optional
Architecture: ${arch}
Maintainer: Unitree Camera Bring-up <unitree@example.invalid>
Depends: ros-${ros_distro}-ament-cmake, ros-${ros_distro}-camera-calibration-parsers, ros-${ros_distro}-camera-info-manager, ros-${ros_distro}-class-loader, ros-${ros_distro}-rclcpp, ros-${ros_distro}-rclcpp-components, ros-${ros_distro}-rclpy, ros-${ros_distro}-sensor-msgs, libgstreamer1.0-0, libgstreamer-plugins-base1.0-0
Installed-Size: ${installed_size}
Description: ROS 2 Jazzy gscam2 camera bridge
 gscam2 built from local source for Jetson camera bring-up.
 It publishes ROS sensor_msgs/Image and CameraInfo from GStreamer pipelines.
EOF

dpkg-deb --build "${package_root}" "${output_dir}/${package_name}_${deb_version}_${arch}.deb"

cat <<EOF

Built ${output_dir}/${package_name}_${deb_version}_${arch}.deb

Install on another ${arch} ROS ${ros_distro} host with:
  sudo apt install ./${package_name}_${deb_version}_${arch}.deb

After install:
  source /opt/ros/${ros_distro}/setup.bash
  ros2 pkg executables gscam2
EOF
