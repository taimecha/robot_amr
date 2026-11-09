> Bản triển khai hiện tại **r3-approved-lattice**: xem [r3/README.md](r3/README.md) và [báo cáo](../pi5_docking_diagnostics/TRIEN_KHAI_R3.md). Các demo quintic/85mm/12mm và kết quả lịch sử bên dưới không phải luồng r3 và không được dùng làm nghiệm thu mới.

Giao diện mới: `./simulation/launch_r3_sim.sh gui` — camera/nhận diện, nhập khoảng cách–ngang–góc theo **tâm gốc càng**, xem đúng hai đường planner/thực tế và chủ động chạy mô phỏng. [Cách dùng và giới hạn](r3/README.md).

# Mô phỏng docking trên laptop Ubuntu

Chạy từ gốc repo:

```bash
python3 simulation/run_all.py
python3 -m unittest discover -s simulation -p 'test_*.py' -v
```

Xem `output/preview.svg` để thấy đường cong đến điểm **pre-dock** và timeline nhận diện. `path.csv`, `grid.csv`, `timeline.csv`, `shadow_follower.csv`, `summary.json` chứa dữ liệu để kiểm tra hoặc vẽ lại. Script chỉ dùng Python stdlib, không import ROS, không tạo publisher, không mở cổng mạng. Dữ liệu cảm biến và hành lang trong preview là giả lập. `illustrative_feasible` chỉ kiểm tra chiều tiến, độ cong giả định tối đa 2.5 1/m và hành lang rộng giả định 1.2 m; **không chứng minh càng lọt lỗ pallet**.

Kết quả ví dụ **giả định hiệu chuẩn độc lập đã đạt**: chờ dữ liệu tốt đến sau giây thứ 8; tắt detector và chờ heartbeat; xác minh bằng phép đo RGB-D mới; READY ở 10.2 s nhưng không tự di chuyển. Sau thao tác bắt đầu giả lập, shadow follower đi đến pre-dock và dừng ở `WAIT_INSERTION_VERIFICATION`. Các kịch bản mất odom, mất depth, vật cản, reset odom, xung đột nguồn lệnh và hủy đều khóa STOP trong mô hình; không tự chạy lại.

`quality_replay.py` chạy A/B trên cùng 41 frame giả: raw, chỉ frame gate, frame+Depth, và frame+Depth+robust pose. Nó ghi lý do loại frame, Depth median/MAD và sai số tuyệt đối so với truth giả. Bias X 10 mm vẫn còn sau lọc, vượt budget ví dụ 5 mm; nhánh đó **không được READY**. `plan_sim.py` quét 72 trường hợp lateral ±2/5/10 cm, yaw ±2/5/10°, bốn khoảng approach và thêm một ca vật cản. Nó tìm tangent scale cho quintic + straight, kiểm tra footprint/càng ở từng mẫu, curvature và ngân sách bất định giả; kết quả nằm trong `plan_grid.csv` và `plan_report.json`.

`run_all.py` sinh `manifest.json`, `metrics.csv`, `events.jsonl`, `summary.json` và [báo cáo mô phỏng](output/report.md). Các file mang provenance `synthetic simulation`, không phải log Pi.

`integrated_scenario.py` nối frame gate → kiểm sai số tuyệt đối → acquisition → khóa đích → sàng lọc đường → shadow follower. Kịch bản còn bias 10 mm đứng ở `ACQUIRING`; kịch bản đã sửa bias bằng ground truth **giả** đi tới `WAIT_INSERTION_VERIFICATION`. Điều chỉnh bias trong script là cách tạo hai trường hợp mô phỏng, không phải hồ sơ hiệu chuẩn cho robot.

## Đối chiếu với source hiện tại

- HEAD `9056b5f` đúng mốc tài liệu. Máy đang chạy là laptop x86_64 với ROS 2 Jazzy. Chưa xác minh phiên bản source, tham số, thiết bị hay dịch vụ **đang chạy trên Pi 5** vì chưa có phiên SSH Pi xác định trong môi trường này.
- Trong checkout này, `pallet_acquisition.py` đã bỏ deadline tổng 5 s; `pallet_rgbd_health.py` chỉ dùng độ mới, số mẫu và cửa sổ ổn định khi đứng yên. Acquisition mặc định chặn ở `ACQUIRING` vì ROS pipeline chưa có bằng chứng hiệu chuẩn độc lập để đặt `accuracy_validated=True`. Unit/ROS tests inject giá trị này **chỉ cho dữ liệu giả**. Không tự bật gate đó trên robot.
- `align_pallet.py` còn subscribe `/pallet/target` khi chạy và `pallet_alignment_real.launch.py` dùng `--execute`. Chưa có FROZEN_TARGET supervisor/planner/follower production; không dùng launch đó để thử ý tưởng frozen-target trên robot thật.
- `pallet_alignment_collision.yaml` chỉ dùng `/scan_filtered` làm nguồn chặn. Đường vào pallet cần xác minh vùng mù của LiDAR/Depth và hành lang insertion riêng.
- Hình học mô phỏng lấy mốc từ repo: càng dài 0.390 m, mũi càng cách mặt trước chassis 0.455 m; `base_controller` dùng bán kính bánh 0.0875 m, track 0.428 m, timeout lệnh 0.25 s. Preview chưa tính chính xác lỗ pallet, biến dạng, slip và tải.

## Gazebo trên laptop

Đã build package `tai_robot_one` từ checkout hiện tại và launch `gazebo_sim.launch.py` headless với `ROS_DOMAIN_ID=88`, `GZ_PARTITION=tai_docking_preview_88`, tắt RViz và phone teleop. Ba controller active; `/odom`, `/joint_states`, `/scan`, `/imu`, RGB, Depth và CameraInfo có mặt. Gửi một lệnh `/cmd_vel` một lần trong Gazebo rồi thấy vận tốc odom trở về gần 0, phù hợp timeout 0.25 s. Đây là smoke test mô hình robot/cảm biến, **chưa phải vòng docking tích hợp trong Gazebo**.

Lệnh chạy lại Gazebo (không dùng launch robot thật):

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/tai_docking_log build --base-paths ros2_ws \
  --packages-select tai_robot_one --build-base /tmp/tai_docking_build \
  --install-base /tmp/tai_docking_install --symlink-install
source /tmp/tai_docking_install/setup.bash
export ROS_DOMAIN_ID=88 GZ_PARTITION=tai_docking_preview_88
ros2 launch tai_robot_one gazebo_sim.launch.py headless:=true \
  use_rviz:=false use_phone_teleop:=false
```

Gazebo smoke test xác nhận mô hình robot/cảm biến; bộ quét planner và kiểm tra acquisition vẫn chạy offline hoặc với input ROS giả. Để thử docking tích hợp trong Gazebo tiếp theo cần nối acquisition chất lượng dữ liệu, lifecycle YOLO/RGB-D và target lock vào planner/follower dựa trên dữ liệu Gazebo. Không được đưa `--execute` của launch robot thật vào bước này. Tiêu chí và giả định hình học phải đối chứng bằng ground truth trong Gazebo.

Đã thêm một lượt **pre-dock** chạy bằng odometry Gazebo: `gazebo_dock_trial.py` dùng đường cong đến `(0.90, 0.16)` m, gửi lệnh chỉ trong domain 88/partition riêng và dừng trước mặt pallet. Có thể [xem lại chuyển động](output/gazebo_replay.html), xem [đồ thị Gazebo](output/gazebo_trial.svg) hoặc đọc `output/gazebo_trial.csv`. Lượt đó dùng đích giả đặt sẵn, chưa nối pipeline nhận diện camera; vì thế không phải một nhiệm vụ docking end-to-end. Khi chạy lại, sau khi launch Gazebo và xác nhận `/controller_manager use_sim_time=true`, dùng:

```bash
export ROS_DOMAIN_ID=88 GZ_PARTITION=tai_docking_preview_88
python3 simulation/gazebo_dock_trial.py --gazebo-execute
python3 simulation/run_all.py
```

## Demo trực quan: một pallet vuông dưới sàn

Pallet trong `ros2_ws/worlds/docking_one_pallet.sdf` là **một vật đứng yên**, không có kệ: mặt vuông 35,5 × 35,5 cm, dày 1 cm; mặt dưới ở cao 10 cm so với sàn; bốn chân 5 × 5 × 10 cm ở các góc. Robot đi vào từ mặt trước rộng 35,5 cm, giữa hai chân có lỗ rộng 25,5 cm. Cặp càng rộng ngoài 15,5 cm, nên còn 5 cm danh nghĩa mỗi bên khi căn chính giữa. Chưa tính sai số nhận diện, điều khiển, biến dạng hoặc va chạm ba chiều khi chèn càng.

Lần đầu build package ROS từ gốc repo:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/tai_docking_log build --base-paths ros2_ws \
  --packages-select tai_robot_one --build-base /tmp/tai_docking_build \
  --install-base /tmp/tai_docking_install --symlink-install
```

Mở đồng thời **Gazebo, RViz và cửa sổ nhận diện pallet** từ gốc repo:

```bash
./simulation/launch_one_pallet_gui.sh
```

Để đặt tư thế xe lúc xuất phát, nhập trực tiếp ba giá trị trong terminal:

```bash
./simulation/launch_one_pallet_gui.sh --interactive --yolo-conf 0.10
```

Hoặc truyền tham số để chạy lại một trường hợp cụ thể:

```bash
./simulation/launch_one_pallet_gui.sh --distance 1.65 --lateral -0.05 --yaw-deg 5 --yolo-conf 0.10
```

`--distance` là khoảng cách theo trục tiến từ tâm `base_link` của xe đến **mặt trước** pallet (m). `--lateral` là vị trí xe so với trục giữa pallet: dương bên trái, âm bên phải (m). `--yaw-deg` là góc xe so với hướng đưa càng thẳng vào pallet: dương quay trái (độ). Pallet giữ nguyên tại `(2,0.10)` m trong Gazebo; vị trí và hướng xe thay đổi theo ba giá trị này. Mỗi lần thử cần đóng lượt Gazebo cũ rồi mở lại launcher; terminal thứ hai chạy `python3 simulation/gazebo_dock_trial.py --one-pallet --gazebo-execute` như bên dưới. Giá trị ngoài phạm vi 1,45–3,5 m, ±0,35 m, ±20° hoặc không lập được đường cong sẽ bị từ chối trước khi xe chạy.

Nếu pallet thấy trong ảnh nhưng YOLO không chọn vì confidence thấp, có thể thử thêm `--yolo-conf 0.10` **chỉ trong mô phỏng** để đánh giá độ nhạy; mặc định là `0.25`. Màn hình hiển thị confidence của kết quả được chọn. Xe vẫn chờ nếu không có kết quả và chỉ lập đường khi tọa độ RGB-D gần vị trí pallet thật trong world Gazebo. Không dùng ngưỡng hạ thấp này làm hiệu chuẩn cho robot thật.

RViz hiển thị `/pallet_demo/planned_path` (đường xanh) và chấm dừng trước pallet. Cửa sổ camera dùng **YOLO thật `best_v3.pt`** trên ảnh Gazebo 640 × 480, hiển thị mask, bbox, số đo RGB-D, vị trí tương đối xe → pallet `(X tiến, Y trái, yaw)`, FPS RGB, khoảng thời gian giữa các frame và thời gian suy luận YOLO. Góc yaw được đo từ mép trước của mặt pallet trong Depth: chia điểm mặt pallet thành các dải ngang, lấy biên gần và khớp đường thẳng bền vững. Chỉ khóa đích sau ít nhất 5 tư thế RGB-D hợp lệ có góc/vị trí ổn định, tắt YOLO rồi xác minh bằng hai phép đo RGB-D mới; góc ca thử chỉ dùng để kiểm tra độc lập sai số tối đa 3°. Pallet mô phỏng màu xanh. Môi trường YOLO mặc định là `/home/tai/astra_pallets/.venv`; PyTorch dùng GPU nếu CUDA sẵn có.

Khi xe đứng yên, node nhận diện và đo tâm mặt pallet bằng Depth đã đăng ký, yêu cầu ba kết quả ổn định rồi **tắt suy luận YOLO**. Nó xác minh thêm hai frame RGB-D mới sau khi YOLO tắt, khóa tọa độ pallet trong `odom`, công bố `/pallet_demo/target_locked` và lập đường từ mục tiêu đã khóa. Lúc robot chạy, không còn suy luận YOLO hoặc đo lại pallet; `(X, Y, yaw)` trên màn hình chỉ cập nhật từ odometry xe so với tọa độ đã khóa. `/pallet_demo/status` cho biết pha, FPS và độ trễ. Có thể xem trạng thái trong terminal dùng cùng `ROS_DOMAIN_ID=88`:

```bash
ros2 topic echo /pallet_demo/status --once --full-length
```

World Gazebo cung cấp phép kiểm tra độc lập **chỉ trong mô phỏng**: sai số tâm RGB-D so với vị trí pallet SDF phải dưới 5 cm mới khóa. Đây chưa phải chứng nhận độ chính xác cho Pi 5 hoặc robot thật. Bộ điều khiển chỉ bắt đầu khi nhận topic khóa và đường đã lập; không chèn càng hoặc nâng pallet.

Để xem robot chạy theo đúng đường đó đến điểm chờ, mở terminal thứ hai:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/tai_docking_install/setup.bash
export ROS_DOMAIN_ID=88 GZ_PARTITION=tai_one_pallet_88
python3 simulation/gazebo_dock_trial.py --one-pallet --gazebo-execute
```

Robot dừng trước khi đầu càng chạm mặt pallet và ghi `simulation/output/one_pallet_trial.csv`; không chèn càng, không nâng. Muốn chạy lại từ đầu, đóng demo bằng Ctrl-C rồi mở lại lệnh launch. Giới hạn tốc độ bám đường hiện tại là 0,10 m/s, gia tốc 0,08 m/s². Lượt thử ngày 2026-11-05 sau khi tăng tốc: robot chạy 191 mẫu, tốc độ lệnh tối đa 0,10 m/s, đến `(0,875; 0,101)` m sau 9,5 s và dừng `WAIT_INSERTION_VERIFICATION`; đầu càng cách mặt trước pallet khoảng 19,2 cm. YOLO tắt trước chuyển động, không chèn càng hoặc nâng.

Để thử **chèn càng thẳng và nâng pallet bằng vật lý Gazebo** trong cùng demo, mở launcher ở terminal 1, rồi ở terminal 2:

```bash
cd ~/tai_robot_one_community
source /opt/ros/jazzy/setup.bash
source /tmp/tai_docking_install/setup.bash
export ROS_DOMAIN_ID=88 GZ_PARTITION=tai_one_pallet_88
python3 simulation/gazebo_dock_trial.py --one-pallet --gazebo-execute --insert-and-lift
```

RViz hiển thị đường pre-dock màu xanh và đoạn chèn thẳng màu cam. Xe dừng tại điểm pre-dock, kiểm tra góc/lệch ngang, chèn thẳng ở tối đa 0,04 m/s đến khi **gốc càng cách mặt trước pallet khoảng 1–2 cm**, rồi ra lệnh nâng khớp 85 mm. Mục tiêu đặt tại 12 mm; đầu càng dài 390 mm nên nhô khoảng 23 mm qua mép sau pallet dài 355 mm. Chỉ trong mô phỏng, điểm dừng cuối dùng pose Gazebo của pallet để tránh sai số vài cm từ phép đo RGB-D đã khóa. Pallet trong world là vật thể động 1 kg; bộ điều khiển chỉ báo `DONE_LIFTED` khi pose Gazebo cho thấy pallet cao hơn ban đầu 45–65 mm và khớp nâng gần đạt vị trí yêu cầu. Sai lệch, dữ liệu cũ hoặc hết thời gian sẽ dừng `STOP_LATCHED`. CSV ghi thêm `lift_joint_m`, `pallet_z_m` và `fork_root_gap_m`. Đây là phép thử mô phỏng, không phải lệnh áp dụng trên robot thật.

Lượt chạy kiểm chứng với `--distance 1.7 --lateral -0.08 --yaw-deg 8 --yolo-conf 0.10` đạt `DONE_LIFTED`: **gốc càng cách mặt trước pallet 13,6 mm**, pallet cao **50,7 mm**, khớp nâng 77,4 mm tại lúc hoàn tất (CSV `output/one_pallet_trial.csv`). Khi chạy chế độ này, đích RGB-D đã khóa phải lệch dưới 30 mm so với pallet của ca mô phỏng trước khi xe bắt đầu chèn càng.

Khi thử các vị trí nhập khác nhau, bộ điều khiển còn lưu mỗi lượt thành CSV riêng trong `simulation/output/one_pallet_trials/`, với khoảng cách, lệch ngang, góc và thời điểm trong tên file. File `one_pallet_trial.csv` giữ kết quả mới nhất. Nếu YOLO không khóa được pallet, xe tiếp tục chờ và người vận hành có thể hủy bằng Ctrl-C; không có thời hạn tổng cho giai đoạn nhận diện.

## LiDAR trong demo một pallet

LiDAR A1M8 trong URDF vẫn dùng extrinsic đã lưu từ lượt kiểm chứng xe thật ngày 2026-09-23: `base_footprint → lidar_link` là `x=-0,2115 m, y=0, z=0,385 m, yaw=-2°`. Chỉ thân vuông trong mô hình được xoay bù `+2°` để nhìn thẳng với chassis; TF và hướng quét vật lý không đổi. RViz demo hiển thị `/scan_filtered_rviz`, đúng với đầu ra của `scan_self_filter`, và nhãn ghi cao độ 0,385 m.

Kiểm tra 5 scan trong Gazebo world một pallet: mỗi `/scan` thô có 97 điểm tự phản xạ gần cột/thân xe ở khoảng x≈0,51 m tính từ LiDAR; `/scan_filtered` loại đủ 97 điểm mỗi scan. Pallet cao tối đa 0,11 m, thấp hơn mặt phẳng quét 0,385 m, nên world chỉ có pallet có thể không còn điểm LiDAR nào sau lọc. Bộ lọc không tạo khả năng phát hiện pallet thấp; camera Depth/scan an toàn thấp là phần riêng. Tài liệu kiểm chứng lịch sử: `ros2_ws/docs/handoff_2026-09-19.md`; chưa xác nhận cấu hình đang chạy trên Pi 5 trong phiên này vì các địa chỉ SSH lưu trên laptop không kết nối được.
