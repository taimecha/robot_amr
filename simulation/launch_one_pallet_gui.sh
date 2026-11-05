#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case_values="$(python3 "$repo_dir/simulation/one_pallet_case.py" "$@")"
read -r case_distance case_lateral case_yaw spawn_x spawn_y spawn_yaw yolo_conf <<< "$case_values"
export TAI_ONE_PALLET_DISTANCE_M="$case_distance"
export TAI_ONE_PALLET_LATERAL_M="$case_lateral"
export TAI_ONE_PALLET_YAW_DEG="$case_yaw"
export TAI_ONE_PALLET_YOLO_CONF="$yolo_conf"
echo "Pallet: distance=${case_distance} m, lateral=${case_lateral} m, yaw=${case_yaw} deg; spawn=(${spawn_x}, ${spawn_y}, ${spawn_yaw} rad)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$repo_dir/ros2_ws/install/setup.bash" ]]; then
  source "$repo_dir/ros2_ws/install/setup.bash"
elif [[ -f /tmp/tai_docking_install/setup.bash ]]; then
  source /tmp/tai_docking_install/setup.bash
else
  echo 'Build first: cd ros2_ws && colcon build --symlink-install' >&2
  exit 1
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export GZ_PARTITION="${GZ_PARTITION:-tai_one_pallet_${ROS_DOMAIN_ID}}"
export QT_X11_NO_MITSHM=1
export TAI_SIM_CAMERA_WIDTH=640
export TAI_SIM_CAMERA_HEIGHT=480
export TAI_SIM_CAMERA_RATE=5
export TAI_SIM_DISABLE_POINTS=1
export TAI_SIM_BRIDGE_PALLET_POSE=1
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo 'A desktop DISPLAY or WAYLAND_DISPLAY is needed for Gazebo, RViz and camera window.' >&2
  exit 1
fi
yolo_python="/home/tai/astra_pallets/.venv/bin/python"
if [[ ! -x "$yolo_python" ]]; then
  echo "YOLO Python environment missing: $yolo_python" >&2
  exit 1
fi
"$yolo_python" "$repo_dir/simulation/one_pallet_visualizer.py" --ros-args -p use_sim_time:=true &
visualizer_pid=$!
python3 "$repo_dir/simulation/pallet_camera_window.py" &
viewer_pid=$!
cleanup() { kill "$visualizer_pid" "$viewer_pid" 2>/dev/null || true; }
trap cleanup EXIT
ros2 launch tai_robot_one gazebo_sim.launch.py \
  "world:=$repo_dir/ros2_ws/worlds/docking_one_pallet.sdf" \
  "rviz_config:=$repo_dir/ros2_ws/config/docking_one_pallet.rviz" \
  "spawn_x:=$spawn_x" "spawn_y:=$spawn_y" "spawn_yaw:=$spawn_yaw" \
  use_rviz:=true headless:=false use_phone_teleop:=false
