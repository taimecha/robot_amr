#!/usr/bin/env bash
# Isolated simulation only; never source a hardware domain or start real devices.
set -e
R3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R3_ROOT"
mode="${1:-gui}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in gui|world|run|mission) ;; *)
  printf 'Usage: %s gui | world [--distance m --lateral m --yaw-deg deg] | run | mission\n' "$0"; exit 0;;
esac
source /opt/ros/jazzy/setup.bash
if [[ ! -f /tmp/tai_r3_ros_install/setup.bash ]]; then
  colcon --log-base /tmp/tai_r3_ros_log build --base-paths ros2_ws --packages-select tai_robot_one \
    --build-base /tmp/tai_r3_ros_build --install-base /tmp/tai_r3_ros_install --symlink-install
fi
source /tmp/tai_r3_ros_install/setup.bash
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=tai_r3_sim_only_89
export TAI_SIM_ROBOT_XACRO=/tmp/tai_r3_gazebo_assets/description/robot.urdf.xacro
export TAI_SIM_BRIDGE_PALLET_POSE=1 TAI_SIM_DISABLE_POINTS=1
export TAI_SIM_CAMERA_WIDTH=320 TAI_SIM_CAMERA_HEIGHT=240 TAI_SIM_CAMERA_RATE=5
if [[ "$mode" == gui ]]; then exec python3 simulation/r3_gui.py "$@"; fi
r3_child_pid=''
cleanup() {
  if [[ -n "$r3_child_pid" ]]; then
    kill -INT -- "-$r3_child_pid" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 -- "-$r3_child_pid" 2>/dev/null; then break; fi
      sleep .1
    done
    kill -TERM -- "-$r3_child_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
case_file=''; plan=''; out=''; no_open=false
if [[ "$mode" == world ]]; then
  exec 9>/tmp/tai_r3_world.lock
  if ! flock -n 9 || pgrep -f '^gz sim.*tai_r3_gazebo_assets/world.sdf' >/dev/null; then
    printf 'Đã có world r3 đang mở. Đóng world cũ trước khi đổi tư thế.\n' >&2; exit 1
  fi
  python3 simulation/r3_world_guard.py
  case_file="$(mktemp /tmp/tai_r3_case.XXXXXX.json)"
  python3 simulation/r3_case.py "$@" --output "$case_file"
  python3 simulation/r3/generate_gazebo_assets.py --case "$case_file"
  rm "$case_file"
  mapfile -t spawn < <(python3 - <<'PY'
import json
c=json.load(open('/tmp/tai_r3_gazebo_assets/case.json'))
for v in c['base_world']:print(v)
print(str(c.get('headless',False)).lower())
PY
)
  setsid ros2 launch tai_robot_one gazebo_sim.launch.py headless:="${spawn[3]}" \
    use_rviz:=false use_phone_teleop:=false world:=/tmp/tai_r3_gazebo_assets/world.sdf \
    spawn_x:="${spawn[0]}" spawn_y:="${spawn[1]}" spawn_yaw:="${spawn[2]}" &
  r3_child_pid=$!
  wait "$r3_child_pid"
else
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --case) case_file="$2"; shift 2;; --plan) plan="$2"; shift 2;;
      --output) out="$2"; shift 2;; --no-open) no_open=true; shift;;
      *) printf 'Unknown option: %s\n' "$1" >&2; exit 2;;
    esac
  done
  case_file="${case_file:-/tmp/tai_r3_gazebo_assets/case.json}"
  python3 - "$case_file" <<'PY'
import json,sys
from pathlib import Path
active=Path('/tmp/tai_r3_gazebo_assets/case.json')
if not active.exists():raise SystemExit('Chưa mở world cho ca thử. Chạy world hoặc giao diện trước.')
if json.load(open(sys.argv[1]))['case_id']!=json.loads(active.read_text())['case_id']:
    raise SystemExit('Ca thử khác world đang mở. Cần mở lại world với tư thế mới.')
PY
  if [[ -z "$out" ]]; then out="$R3_ROOT/simulation/output/r3_user/$(TZ=Asia/Ho_Chi_Minh date +%Y%m%d_%H%M%S)_${mode}_$$"; fi
  mkdir -p "$out"
  if [[ -z "$plan" ]]; then
    python3 simulation/r3_prepare.py --case "$case_file" --output "$out/plan"
    plan="$out/plan/frozen_plan.json"
  fi
  python3 - "$plan" "$case_file" <<'PY'
import json,sys
p=json.load(open(sys.argv[1]));c=json.load(open(sys.argv[2]))
if json.loads(p['target_json']).get('case_id')!=c['case_id']:
    raise SystemExit('Frozen plan không thuộc đúng ca thử đang mở.')
PY
  setsid python3 simulation/r3_gazebo_scorer.py --output "$out" --case "$case_file" > "$out/scorer.log" 2>&1 &
  r3_child_pid=$!
  # Give the independent recorder time to subscribe before the first command.
  sleep 1
  r3_options=(--longitudinal-stop-report-only)
  if [[ "$mode" == run ]]; then r3_options+=(--tracking-only); fi
  printf 'Output: %s\n' "$out"
  python3 simulation/r3_gazebo_trial.py --gazebo-execute --plan "$plan" --output "$out" "${r3_options[@]}"
  cleanup
  wait "$r3_child_pid" 2>/dev/null || true
  r3_child_pid=''
  python3 simulation/r3_score_gazebo.py "$out" --plan "$plan" --longitudinal-stop-report-only
  printf '\nPlot: %s/trajectory.png\nResult: %s/mission_result.json\n' "$out" "$out"
  if ! "$no_open" && command -v xdg-open >/dev/null; then xdg-open "$out/trajectory.png" >/dev/null 2>&1 & fi
fi
