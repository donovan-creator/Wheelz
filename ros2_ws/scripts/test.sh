#!/usr/bin/env bash
set -euo pipefail
workspace=$(cd "$(dirname "$0")/.." && pwd)
cd "$workspace"
set +u
source install/setup.bash
set -u
colcon test --packages-select wheelz_ros --python-testing pytest --event-handlers console_direct+
colcon test-result --verbose
