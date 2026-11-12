#!/usr/bin/env bash
# One terminal: start missing sources, recognize, plan, track, save image, stop owned nodes.
set -eo pipefail
capture_only=0
case "${1:-}" in
  --capture-only) capture_only=1; shift ;;
  --help|-h)
    echo 'Usage: bash simulation/run_r3_live_trial.sh [--capture-only]'
    echo '--capture-only: chốt RGB-D và lưu bằng chứng; không khởi chạy bộ bám quỹ đạo.'
    exit 0 ;;
esac
[[ $# -eq 0 ]] || { echo "Tham số không hỗ trợ: $*" >&2; exit 2; }
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
source /opt/ros/jazzy/setup.bash
source camera/detect_astra/ros2_ws/install/setup.bash
source camera/detect_astra/data/alignment_robot_ws/install/setup.bash
set -u
[[ -t 0 && -t 1 ]] || { echo 'Chạy lệnh này trong terminal tương tác của VS Code.' >&2; exit 1; }

for device in /dev/tai_drive /dev/tai_imu /dev/tai_lidar; do
  [[ -e "$device" ]] || { echo "Thiếu thiết bị $device" >&2; exit 1; }
done
[[ -x camera/detect_astra/.venv/bin/python ]] || { echo 'Thiếu Python camera' >&2; exit 1; }
[[ -d camera/detect_astra/data/runtime_models/best_v3_ncnn_model ]] || {
  echo 'Thiếu model NCNN pallet' >&2; exit 1;
}

mkdir -p simulation/output/r3_live
exec 9>simulation/output/r3_live/.trial.lock
flock -n 9 || { echo 'Đã có một lượt r3 đang chạy' >&2; exit 1; }
run_dir=$(mktemp -d "$root/simulation/output/r3_live/$(date +%Y%m%d_%H%M%S)_XXXXXX")
plan="$run_dir/plan/frozen_plan.json"
camera_pid=''
bringup_pid=''
perception_pid=''
safety_pid=''

group_running() {
  local group=$1
  ps -eo pgid=,stat= | awk -v wanted="$group" \
    '$1 == wanted && $2 !~ /^Z/ { found = 1 } END { exit !found }'
}

stop_owned() {
  local pid=$1
  [[ -n "$pid" ]] || return 0
  if group_running "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..20}; do
      group_running "$pid" || break
      sleep .2
    done
    if group_running "$pid"; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      for _ in {1..10}; do
        group_running "$pid" || break
        sleep .2
      done
    fi
    if group_running "$pid"; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local result=$?
  trap - EXIT
  stop_owned "$safety_pid"
  stop_owned "$perception_pid"
  stop_owned "$bringup_pid"
  stop_owned "$camera_pid"
  if [[ -f "$run_dir/trial/mission_result.json" &&
        ! -s "$run_dir/trial/executed_odom_base.csv" ]]; then
    echo 'Lượt chạy không có mẫu odom; không thể tạo ảnh ba quỹ đạo.' >&2
    [[ $result -ne 0 ]] || result=3
  fi
  if [[ -f "$plan" && -s "$run_dir/trial/executed_odom_base.csv" &&
        -f "$run_dir/trial/mission_result.json" ]]; then
    if camera/detect_astra/.venv/bin/python simulation/r3_render_real_result.py \
       --plan "$plan" --trial "$run_dir/trial" \
       --output "$run_dir/trajectory_result.png"; then
      echo "Ảnh ba quỹ đạo: $run_dir/trajectory_result.png"
    else
      echo 'Không vẽ được ảnh từ log odom; xem các tệp trong thư mục lượt chạy.' >&2
      [[ $result -ne 0 ]] || result=3
    fi
  fi
  echo "Kết quả và log: $run_dir"
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

if pgrep -f 'ros2_control_node( |$)|cmd_vel_stamper( |$)|sllidar_node( |$)|r3_real_obstacle_filter.py|r3_real_trial.py' >/dev/null; then
  echo 'Đã có node điều khiển xe hoặc LiDAR ngoài lệnh này; dừng lượt đó trước.' >&2
  exit 1
fi
if pgrep -f 'run_pallet_perception.py|publish_pallet_target.py|publish_pallet_detection.py' >/dev/null; then
  echo 'Đã có node đo pallet khác chạy; dừng lượt đó trước.' >&2
  exit 1
fi

if pgrep -f 'astra_camera_node( |$)' >/dev/null; then
  echo 'Dùng lại camera Astra đang chạy.'
  ros2 param get /camera/camera depth_registration | rg -q 'True$' || {
    echo 'Camera đang chạy nhưng depth_registration chưa bật.' >&2; exit 1;
  }
else
  echo 'Mở camera Astra...'
  ASTRA_POINT_CLOUD=false setsid bash camera/detect_astra/scripts/open_registered_astra.sh \
    >"$run_dir/camera.log" 2>&1 &
  camera_pid=$!
fi

echo 'Mở phần cứng, odom và LiDAR dừng vật cản...'
setsid ros2 launch camera/detect_astra/launch/r3_real_bringup.launch.py \
  >"$run_dir/bringup.log" 2>&1 &
bringup_pid=$!

echo 'Mở YOLO và node đo pallet RGB-D...'
setsid bash camera/detect_astra/scripts/open_pallet_target.sh \
  --model camera/detect_astra/data/runtime_models/best_v3_ncnn_model \
  --measurement-bias-yaml camera/detect_astra/config/calibration/pallet_target_bias_robot_2026_11_11.yaml \
  --measurement-evidence-dir "$run_dir/camera_evidence" \
  --heading-floor-reference --max-fps 2 --track-max-fps 0 --preview-fps 2 --cpu-threads 2 >"$run_dir/perception.log" 2>&1 &
perception_pid=$!

while true; do
  kill -0 "$bringup_pid" 2>/dev/null || { echo 'Bringup đã dừng; xem bringup.log' >&2; exit 1; }
  kill -0 "$perception_pid" 2>/dev/null || { echo 'Nhận diện đã dừng; xem perception.log' >&2; exit 1; }
  python3 simulation/r3_perception_switch.py recognize --wait-timeout 120
  if python3 simulation/r3_live_prepare.py --output "$run_dir/plan" --timeout 0; then
    break
  else
    prepare_rc=$?
    if [[ $prepare_rc -ne 3 ]]; then
      exit "$prepare_rc"
    fi
    echo 'RGB-D mất dấu trước khi khóa đích; bật lại YOLO và đo mới, xe vẫn đứng yên.'
    sleep .5
  fi
done
python3 simulation/r3_real_preflight.py --plan "$plan" --allow-provisional
kill -0 "$perception_pid" 2>/dev/null || {
  echo 'Node đo pallet đã dừng trước khi tắt tracking; xem perception.log' >&2
  exit 1
}
python3 simulation/r3_perception_switch.py pause --wait-timeout 15
echo 'Đã chốt vị trí pallet và tắt YOLO; dừng tracking pallet để giảm tải Pi.'
echo "Ảnh kế hoạch hai tâm: $run_dir/plan/trajectory.svg"
if [[ $capture_only -eq 1 ]]; then
  [[ -s "$run_dir/camera_evidence/index.json" ]] || {
    echo 'Chưa lưu được RGB-D gốc; xem perception.log.' >&2
    exit 3
  }
  echo 'Đã lưu phép đo để kiểm tra hiệu chuẩn. Bộ bám quỹ đạo chưa được khởi chạy.'
  echo "RGB-D gốc và vùng nhận diện: $run_dir/camera_evidence/index.json"
  exit 0
fi

setsid ros2 launch camera/detect_astra/launch/r3_safety.launch.py plan:="$plan" \
  >"$run_dir/safety.log" 2>&1 &
safety_pid=$!
sleep 2
kill -0 "$safety_pid" 2>/dev/null || { echo 'Bộ dừng vật cản đã dừng; xem safety.log' >&2; exit 1; }

set +e
python3 simulation/r3_real_trial.py --plan "$plan" --output "$run_dir/trial" \
  --allow-provisional --execute --stop-at-c 2>&1 | tee "$run_dir/trial.log"
trial_rc=${PIPESTATUS[0]}
set -e
[[ $trial_rc -eq 0 ]] || exit "$trial_rc"
if [[ ! -f "$run_dir/trial/mission_result.json" ]]; then
  echo 'Lượt chạy đã hủy trước khi xe bắt đầu.' >&2
  exit 2
fi
mission_status=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' \
  "$run_dir/trial/mission_result.json")
if [[ "$mission_status" == C_DIAGNOSTIC_DONE ]]; then
  echo 'Đã dừng chẩn đoán trước C; chưa đưa càng vào pallet. Xem góc odom cuối và ảnh.'
elif [[ "$mission_status" != DONE_TRACKING ]]; then
  echo "Xe dừng ở trạng thái $mission_status; ảnh sẽ ghi quỹ đạo odom đã đi." >&2
  exit 2
else
  echo 'Xe đã tới B theo điều kiện odom; đang lưu ảnh ba quỹ đạo.'
fi
