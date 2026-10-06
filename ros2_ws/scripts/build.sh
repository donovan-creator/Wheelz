#!/usr/bin/env bash
set -euo pipefail
workspace=$(cd "$(dirname "$0")/.." && pwd)
if [ -z "${ROS_DISTRO:-}" ]; then
  . /etc/os-release
  case "$VERSION_ID" in
    24.04) distro=jazzy ;;
    26.04) distro=lyrical ;;
    *) echo "Source a supported ROS 2 setup.bash first." >&2; exit 1 ;;
  esac
  set +u
  source "/opt/ros/$distro/setup.bash"
  set -u
fi
cd "$workspace"
colcon build --symlink-install --packages-select wheelz_ros
printf '\nIn this terminal: source "%s/install/setup.bash"\n' "$workspace"
