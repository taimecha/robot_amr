# Giao diện mô phỏng và nhận diện

Từ gốc repo, mở giao diện desktop:

```bash
./simulation/launch_r3_sim.sh gui
```

Chạy xong muốn đổi vị trí: sửa ba ô → **Áp dụng / chạy lại vị trí** → **Tạo quỹ đạo** → **Chạy bám mô phỏng**. Không cần reset terminal. Đổi ca sẽ xóa đường/kết quả của ca cũ; đồ thị trước khi tạo đường hiển thị chú thích chưa có quỹ đạo. Nút Đo lại camera chỉ thử nhận diện lại tại vị trí hiện tại, không dịch chuyển xe và không thay thế bước Tạo quỹ đạo.

Đồ thị giao diện và hình mặc định cho ca tự nhập dùng **world cố định**: pallet giữ nguyên vị trí/hướng, xe thay đổi theo ba ô nhập. Hình xám là xe/càng lúc đầu; vẫn chỉ có hai đường planned/thực tế. Đích RGB-D đo được không xoay hình pallet vật lý. CSV điều khiển và log vẫn theo odom. [Ảnh kiểm tra hệ world](../output/r3_world_view_fix/three_start_poses.png).

Pallet r3 nay dùng màu vật liệu **#25A18E** theo yêu cầu. World đang mở cần bấm Áp dụng lại để nạp màu. Thử A/B cùng vị trí1,5m/model/ngưỡng cho0 phát hiện ở cả màu cũ và màu mới; đổi màu riêng lẻ chưa làm YOLO nhận được. [Ảnh và báo cáo thử màu](../output/r3_color_25a18e/README.md).

Nhập **khoảng cách tâm gốc càng đến mặt pallet** (m, dọc pháp tuyến), **lệch ngang so tâm lỗ vào** (m, dương sang trái pallet), **góc lệch** (độ, dương quay trái). 0° nghĩa càng hướng vào pallet và mặt phẳng gốc càng song song mặt pallet. Số nhập hữu hạn không có min/max; nhận cả số âm và góc ngoài ±180°, góc được quy về cùng hướng vật lý. Bản đồ planner và sàn mô phỏng mở rộng theo tư thế. Chấp nhận số nhập không bảo đảm tư thế không va chạm hoặc solver tìm được đường.

1. Bấm **Áp dụng tư thế / mở world**. Có thể bật cửa sổ Gazebo 3D; mặc định xem camera và đường ngay trong ứng dụng.
2. Xem **Camera trực tiếp**, **Ảnh nhận diện cuối** và kết quả YOLO/RGB-D: loại pallet, độ tin cậy, khoảng cách, lệch ngang, góc, trạng thái và mốc chụp. Chưa đo được thì hiện dấu “—” cùng lý do.
3. Chọn **Pose nhập** để thử planner/follower theo kịch bản, hoặc **RGB-D đã khóa** để dùng phép đo camera đã ổn định. Không thay số đo camera bằng tư thế nhập. YOLO hiện chưa nhận ra pallet hình hộp của world kiểm tra; nguồn RGB-D chưa có pose khóa. Không đổi model hay thêm detector thay thế.
4. Bấm **Tạo quỹ đạo**, xem đường xanh, rồi **Chạy bám mô phỏng**. Giao diện chỉ vẽ hai đường của **tâm gốc càng**: bộ tạo quỹ đạo và thực tế Gazebo (truth độc lập). Odom vẫn ghi CSV phục vụ đối chiếu, không thêm đường lên giao diện. Khi tạo xong đường hoặc bắt đầu chạy, tắt đo pallet; camera tiếp tục truyền ảnh và giữ kết quả đo đứng yên cuối. Recorder truth không đưa dữ liệu vào bộ bám.
5. Bấm **DỪNG MÔ PHỎNG** để hủy. Đổi tư thế bằng Áp dụng lại; giao diện chỉ đóng world do chính nó mở. Nếu có world r3 khác, đóng world đó trước. Đóng giao diện sẽ dừng các tiến trình nó tạo.

Không có thời hạn chạy xe tổng cố định. Giữ các điều kiện dừng vật cản, dữ liệu stale, va chạm và giới hạn v≤0,15 m/s, ngang≤20 mm, góc≤0,5°. Giữ goal danh định gốc càng 20 mm; sai số dọc chỉ báo cáo theo yêu cầu. Nút Chạy không nâng càng. `mission` qua CLI vẫn thử nâng80mm; vấn đề thiếu hành trình nâng đã báo trước chưa được xử lý bằng đổi controller.

Chạy hai terminal vẫn được; terminal 1 sẽ hỏi ba giá trị nếu chưa truyền:

```bash
./simulation/launch_r3_sim.sh world
# Hoặc: ./simulation/launch_r3_sim.sh world --distance 2 --lateral .25 --yaw-deg 10
# Terminal 2:
./simulation/launch_r3_sim.sh run
```

Mỗi lượt xuất vào `simulation/output/r3_user/`. Hình mặc định `trajectory.png/svg` có hai đường. Xem [ảnh camera và giao diện](../output/r3_gui_validation/gui_live_camera.png), [ảnh lượt lệch góc đã chấm](../output/r3_gui_validation/gui_camera_paths.png) và [kết quả kiểm chứng](../output/r3_gui_validation/README.md). Ca thẳng đạt; ca 2m / +0,25m / +10° hoàn tất nhưng FAIL ngang truth khoảng30,4mm, không tuyên bố mọi tư thế đều đạt.

# r3-approved-lattice implementation

Read `../../pi5_docking_diagnostics/TRIEN_KHAI_R3.md` for the actual results and limitations.
The approved implementation uses the **installed Nav2 SmacPlannerLattice** with differential primitives,
**installed Nav2 ConstrainedSmoother** on translating approach segments only, then an exact fork-axis line.
The line is never optimized. No alternative planner is used on failure.

The existing `ShadowFollower` Pure Pursuit equation steers translation. Stationary yaw correction calls the
existing `pallet_alignment_core.stationary_heading_command`; only its existing equation was extracted.
Speed profiling respects0.15m/s, omega0.20rad/s, acceleration0.08m/s² and angular acceleration0.40rad/s²
from the existing demo/alignment config. There is no minimum-speed floor. The fork transform, target,
geometry and primitive/config hashes are frozen with each plan. A new output directory is required to
avoid overwriting reviewed plans. A curvature discontinuity at A is handled by a stopped phase boundary,
not by changing the exact terminal line.

## Run exact-input kinematic simulation

Requires ROS2 Jazzy, nav2_smac_planner, nav2_constrained_smoother, nav2_costmap_2d, pluginlib,
CMake/C++17, nlohmann_json, Python numpy/matplotlib. The C++ helper is built under `/tmp` and loads the
real plugins; it has no drive/lift command publishers.

```bash
source /opt/ros/jazzy/setup.bash
python3 simulation/r3_sim.py
python3 -m unittest discover -s simulation -p 'test_*.py'
python3 simulation/r3_native/run_tests.py
```

Outputs go to a fresh `simulation/output/r3_runs/<local timestamp>/` directory. `fixture.json` explicitly
marks estimated fork/pillar positions and synthetic base/housing offsets. Do not use it as real calibration.
The5 exact-pose cases do not establish RGB-D accuracy, wheel slip, contact, or hardware acceptance.
A separate historical quintic comparison is read-only:

```bash
python3 simulation/r3/compare_baseline.py <kinematic-run-directory>
```

## Run isolated Gazebo

Build the current package as documented in `../README.md`, using a separate `/tmp/tai_r3_ros_install`.
Generate the simulation-only URDF/world with approved dimensions:

```bash
python3 simulation/r3/generate_gazebo_assets.py
source /opt/ros/jazzy/setup.bash
source /tmp/tai_r3_ros_install/setup.bash
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=tai_r3_sim_only_89
export TAI_SIM_ROBOT_XACRO=/tmp/tai_r3_gazebo_assets/description/robot.urdf.xacro
export TAI_SIM_BRIDGE_PALLET_POSE=1 TAI_SIM_DISABLE_POINTS=1
ros2 launch tai_robot_one gazebo_sim.launch.py headless:=true \
  use_rviz:=false use_phone_teleop:=false \
  world:=/tmp/tai_r3_gazebo_assets/world.sdf spawn_x:=0.0225 spawn_y:=0.10
```

Use this spawn only with the `straight` exact-input case. Start the independent scorer **before** the
trial; omit --duration for an unlimited recorder (the launcher handles its lifecycle). In terminals with the same isolated environment:

```bash
python3 simulation/r3_gazebo_scorer.py --output /tmp/my_r3_trial
python3 simulation/r3_gazebo_trial.py --gazebo-execute \
  --plan <kinematic-run>/straight/frozen_plan.json --output /tmp/my_r3_trial
python3 simulation/r3_score_gazebo.py /tmp/my_r3_trial \
  --plan <kinematic-run>/straight/frozen_plan.json
```

The controller checks domain/partition, LOCALHOST discovery, simulated controller clock, single
clock/odom/joint publishers and a matching start pose before creating command publishers. The scorer
reads named Gazebo model poses in a separate process and never feeds control. Full contact-force and
RGB-D-obstacle instrumentation are not implemented in this test adapter. All Gazebo results remain
simulation evidence, not real-operation approval.

Stop the launch after the trial and verify that its Gazebo server has exited before starting another
world in the same partition. A server may survive its launcher; mixed clocks make a run invalid.

## Real stationary-camera preview

`r3_camera_preview.py` has no drive/lift publishers or execute flag. It accepts an exported stationary
camera session, confirmed calibration and geometry, requires post-YOLO-off distinct RGB-D samples,
freezes the target, and exports a planned-only preview. It rejects the synthetic fixture, missing
extrinsics, or an independently measured yaw bound above0.5°. It cannot certify absolute accuracy
from sample stability alone. `corrected_base_scalars` and `optical_SE3` are separate adapters, so an
optical transform is never applied twice to existing corrected scalars.

```bash
source /opt/ros/jazzy/setup.bash
python3 simulation/r3_camera_preview.py --measurement-json <real-session.json> \
  --calibration <confirmed-calibration.json> --geometry <confirmed-geometry.json> \
  --output <fresh-preview-directory>
```

The live ROS acquisition lifecycle now has a separate supervised trial path in
`../r3_live_prepare.py` and `../run_r3_live_trial.sh`. It captures recent stationary RGB-D,
including a fresh valid frame after YOLO stops,
uses the camera's one +2.5 cm lateral correction, and creates a new provisional plan from current
odom. The pallet heading uses a fresh floor-referenced front-plane estimate instead of the old
fixed depth-skew coefficient. The A→C path finishes its bounded quintic bend 12 cm before C and stays straight to C; C→B is also straight. Live A→C plans use signed curvature feedforward plus heading and lateral error feedback at the nearest path reference. The same law continues through the final 12 cm, retaining lateral correction instead of switching abruptly from Pure Pursuit to heading-only control. The one-command real script passes `--stop-at-c`: it brakes before C, waits up to 2 s for the wheels to settle, and ends without a second camera measurement, straight insertion, or lift. During motion it uses the EKF /odom pose for both X/Y and control yaw. Raw IMU yaw is logged for comparison. Missing IMU or more than 2° disagreement between raw IMU and fused odom yaw stops the trial. The geometry is explicitly
approximate (`confirmed_real_geometry=false`) and requires `--allow-provisional`; the trial speed
is limited to 0.10 m/s (operator requested), with curvature, acceleration and braking speed limits. The guarded follower and pallet-masked LiDAR scan are in
`../r3_real_trial.py` and `../r3_real_obstacle_filter.py`. It does not command the lift.
The one-terminal script starts the missing camera/drive/LiDAR/perception processes and stops only
those it owns. After the run, `../r3_render_real_result.py` saves `trajectory_result.png` and SVG
showing the initial robot, pallet, planned base path, planned fork-root path, and recorded odom base
path. The final robot outline uses odom X/Y and control yaw (EKF odom for new trials, raw IMU for older logs); the title shows both final angles. The image title records `DONE_TRACKING` or the actual stop state. The initial target must be
fresh when planning and starting the safety filter; the Enter review wait has no time limit and
does not remeasure the pallet. Motion still requires current odom, IMU and LiDAR at A. The stop report saves final IMU and odom heading errors, their disagreement, estimated lateral position and fork-tip clearance. The plotted orange final robot is a sensor-based estimate, not a physical measurement of fork-to-pallet alignment.
RGB-D floor correction is now shared by position and heading; earlier trials mixed a static position correction with a per-frame heading correction. The floor method still assumes zero camera roll and unverified camera mounting yaw. It does not establish absolute alignment from temporal stability. Acquisition stores bounded raw capture evidence under `camera_evidence/`, scalar inputs under `plan/capture_samples.json`, and raw wheel/fused odom/IMU under `trial/odom_imu_diagnostics.jsonl`. Capture timestamp differences and calibration assumptions are explicit.

For a stationary reference check at A without starting trajectory tracking:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --capture-only
```

This starts acquisition/hardware odom sources, saves evidence after planning, then cleans up its processes. Use a physically checked pallet pose that remains visible to the camera. The old empirical forward residual is not independently validated for the new depth correction.

The current pictured entered-pose plan remains blocked. See `../../camera/detect_astra/ALIGNMENT.md`.

## Read-only preflight for the pictured A→C→B plan

From the repository root, inspect the frozen plan used for the base/fork-root image:

```bash
python3 simulation/r3_real_preflight.py --plan \
  simulation/output/r3_camera_measured_preview/centered_20261111_145707/plan_refined/frozen_plan.json
```

The command makes zero robot commands and exits with code 2 while any static real-plan blocker remains.
The pictured plan currently uses an entered pose and unconfirmed synthetic geometry. The historical
alignment launch has been retired; its command does not execute this A→C→B plan.

## Firmware

Both standalone `cang_nang_ha` and production `bno055_imu_monitor` implement bounded1mm HOME_SETTLE,
then zero. Lower-active startup requires release/reapproach. Normal downtravel remains blocked by the
lower switch and no longer changes the settled zero at the earlier trigger. Watchdog, faults, upper stop
and existing debounce values are retained. Native tests compile the actual lift source with deterministic
IO stubs; they verify exact step count, final zero/lift80mm and fault paths, not physical step loss/timing.
Both projects have been built for ESP32; neither has been flashed.

### Empirical fork-axis lateral residual (2026-11-12)

The separate residual profile `real_trial_lateral_calibration.json` is currently **disabled (0.000 m)**. A +0.025 m axis shift was tried from approximate operator observations, then disabled after trials `153159_MwqW6S` and `153415_nTMQCp` produced a reversed leftward miss and worse heading. The original physical camera mounting correction remains +0.025 m.

A perpendicular offset depends on measured pallet heading as well as translation: at 1.4 m, one degree of heading error changes the offset by about 2.4 cm. Do not identify a constant translational calibration from one approximate perpendicular measurement. The separate report and once-only correction validation remain available, but the live default applies no additional residual. New invocations recognize and freeze a fresh target. Historical plans are unchanged.
