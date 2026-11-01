#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$project_dir/ros2_ws/install/setup.bash"
set -u
pallet_python=${PALLET_PYTHON:-$project_dir/.venv/bin/python}
exec "$pallet_python" "$project_dir/scripts/align_pallet.py" "$@"
