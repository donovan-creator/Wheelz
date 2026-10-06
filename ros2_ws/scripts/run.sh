#!/usr/bin/env bash
set -euo pipefail
workspace=$(cd "$(dirname "$0")/.." && pwd)
if [ ! -f "$workspace/install/setup.bash" ]; then
  echo "Build first: bash $workspace/scripts/build.sh" >&2
  exit 1
fi
set +u
source "$workspace/install/setup.bash"
set -u
exec ros2 launch wheelz_ros bringup.launch.py "$@"
