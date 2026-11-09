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

The live ROS acquisition lifecycle and robot mission launcher are not connected to this preview adapter.
No new real-motion launcher is provided. Existing historical real launch files do not become r3-compliant
by these changes; do not treat their defaults as execution approval.

## Firmware

Both standalone `cang_nang_ha` and production `bno055_imu_monitor` implement bounded1mm HOME_SETTLE,
then zero. Lower-active startup requires release/reapproach. Normal downtravel remains blocked by the
lower switch and no longer changes the settled zero at the earlier trigger. Watchdog, faults, upper stop
and existing debounce values are retained. Native tests compile the actual lift source with deterministic
IO stubs; they verify exact step count, final zero/lift80mm and fault paths, not physical step loss/timing.
Both projects have been built for ESP32; neither has been flashed.
