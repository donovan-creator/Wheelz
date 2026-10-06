#!/usr/bin/env bash
# Run inside Ubuntu/WSL. Installs ROS and host dependencies, never flashes hardware.
set -euo pipefail
. /etc/os-release
case "$VERSION_ID" in
  24.04) distro=jazzy ;;
  26.04) distro=lyrical ;;
  *) echo "Supported: Ubuntu 24.04 (Jazzy) or 26.04 (Lyrical)." >&2; exit 1 ;;
esac
if [ "$(id -u)" -ne 0 ]; then
  exec sudo bash "$0"
fi
export DEBIAN_FRONTEND=noninteractive
if [ ! -f /etc/apt/sources.list.d/ros2.sources ] && ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  apt-get update
  apt-get install -y --no-install-recommends ca-certificates curl python3
  source_deb=$(mktemp /tmp/wheelz-ros-source-XXXXXX.deb)
  trap 'rm -f "$source_deb"' EXIT
  python3 - "$UBUNTU_CODENAME" "$source_deb" <<'PY'
import json
import sys
import urllib.request
release = json.load(urllib.request.urlopen(
    'https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest',
    timeout=30))
suffix = '.' + sys.argv[1] + '_all.deb'
asset = next(a for a in release['assets']
             if a['name'].startswith('ros2-apt-source_') and a['name'].endswith(suffix))
print('Installing repository configuration:', asset['name'])
urllib.request.urlretrieve(asset['browser_download_url'], sys.argv[2])
PY
  dpkg -i "$source_deb"
fi
apt-get update
apt-get install -y --no-install-recommends \
  "ros-$distro-ros-base" "ros-$distro-cv-bridge" \
  "ros-$distro-teleop-twist-keyboard" "ros-$distro-image-transport-plugins" \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-opencv
printf '\nROS installed. In each Ubuntu terminal: source /opt/ros/%s/setup.bash\n' "$distro"
