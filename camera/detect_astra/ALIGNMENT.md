# Căn xe trước pallet — lượt thử 1,20 m

## Điều kiện đạt

- Gốc càng cách mặt trước pallet **1,20 m**, sai số theo X tối đa **±2 cm**.
- Tâm lỗ lệch so với trục giữa càng **dưới 2 cm**: `abs(Y) < 0.02 m`.
- Hướng càng so với hướng vào lỗ **dưới 2°**. Giữ nguyên giá trị góc đã lọc, không ép về 0°.
- Các điều kiện cùng đạt và xe đứng yên ít nhất **0,5 s**, với dữ liệu mới liên tục.
- Dừng tại đây. Chưa đưa càng vào pallet, chạy trong vùng mù hoặc nâng hàng.

Cấu hình: `config/pallet_alignment.yaml`. Bộ căn chạy 20 Hz, tốc độ tối đa 0,08 m/s, quay tối đa 0,20 rad/s. Mặc định không lùi. Truyền động vi sai không đi ngang được; phải phối hợp quay và tiến. Nếu đã quá sát điểm chờ mà vẫn cần dịch ngang, bộ căn dừng và báo `REPOSITION_REQUIRED`, cần đặt lại xe xa hơn. Không tự cố xoay khiến pallet vượt vùng góc đo ±18° hoặc góc nhìn tâm vượt ±25°.

## Mở bộ căn CHẠY THẬT tới khoảng cách chờ 1,20 m

Giữ ba terminal camera, nhận diện/đo và web đang chạy. Dừng terminal bộ căn cũ bằng Ctrl+C nếu đang có `Mode: DRY RUN`. Trong terminal đó chạy:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_alignment_real.sh
```

Script tự build tăng dần package robot và lidar vào `data/alignment_robot_ws`, rồi mở điều khiển bánh, IMU/EKF `/odom`, lidar `/scan_filtered`, bộ tránh vật cản và bộ căn `--execute`. Không dùng Nav2 goal hoặc nâng càng. Không mở thêm teleop, Nav2 hoặc bộ điều khiển bánh khác trong cùng phiên.

Chờ web hiện **CHẠY THẬT**. Nếu lượt nhận diện trước đã hết hạn/mất dấu, bấm **Nhận diện lại (3–5 giây)**. Khi có mục tiêu hợp lệ và đủ điều kiện, bấm **Bắt đầu căn xe**: YOLO tắt, xe căn X/Y/hướng và dừng khi gốc càng cách mặt pallet **1,20 m ±2 cm**, lệch ngang **dưới 2 cm**, góc **dưới 2°**. Khởi động script chưa tự bắt đầu căn. Nút **Dừng căn xe** hoặc Ctrl+C terminal bộ chạy thật dừng hệ thống.

`open_pallet_alignment.sh` là lệnh **kiểm tra không phát lệnh xe**; dùng `open_pallet_alignment_real.sh` cho thử chuyển động thực đã thống nhất. Wrapper chạy thật từ chối mở trùng một bộ căn/bộ điều khiển bánh/lidar đang chạy và kiểm tra quyền đọc/ghi ba cổng `/dev/tai_drive`, `/dev/tai_imu`, `/dev/tai_lidar`.

## Nhận diện tự động rồi chờ bấm căn

Sau khi model sẵn sàng, YOLO nhận diện ở 2 FPS trong 3–5 giây. Cần ít nhất ba kết quả cùng lớp/vùng, confidence ≥0,75, đồng thời có ít nhất ba phép đo RGB-D mới ổn định trong ≥0,2 s. Từ giây 3 đủ điều kiện thì tự tắt YOLO và tiếp tục đo RGB-D. Đến giây 5 chưa đủ thì tự tắt, báo nhận diện lại và không cho bắt đầu căn. Nhận diện hoàn tất không tự gọi bộ căn.

Khi bấm **Bắt đầu căn xe** (hoặc nút kiểm tra trong dry run), bộ căn gọi `/pallet/prepare_alignment` để tắt YOLO và đợi khung hình đang suy luận hoàn tất. Sau xác nhận này, bỏ các số đo cũ và chờ ít nhất ba phép đo chụp mới, cùng odometry/lớp bảo vệ hợp lệ, rồi mới bắt đầu. Nút dừng có thể hủy ngay yêu cầu bắt đầu đang chờ; phản hồi trễ không được làm xe tự chạy lại. Khi đang chuẩn bị hoặc căn, yêu cầu bật YOLO bị từ chối. Thiếu heartbeat xác nhận YOLO tắt sẽ dừng bộ căn.

RGB-D cần chạy trong giai đoạn nhận diện để kiểm chứng hình học và tiếp tục chạy độc lập khi YOLO tắt. Mất dấu yêu cầu bấm **Nhận diện lại (3–5 giây)**; không tự bật YOLO. Các goal phục vụ đi đến vùng nhận diện, tiến càng vào pallet và nâng hàng để thực hiện sau; điểm chờ 1,20 m hiện tại chỉ dùng cho bước căn đã thống nhất.

## Nguồn đo và mốc tọa độ

Chỉ dùng `/pallet/target` hiện tại từ bộ đo RGB + Depth đã đăng ký và áp dụng hồ sơ bù. Không đọc Depth thô, không lấy khoảng cách từ YOLO. Y và hướng lỗ lấy từ các scalar đã hiệu chỉnh; `header.frame_id` là tên nguồn ảnh, **không biến đổi các scalar này như điểm XYZ trong hệ optical**.

- Tâm xe: gốc `base_link`; hình chiếu XY trùng `base_footprint`, tâm khung xe nhìn từ trên xuống.
- Mặt trước khung: X = **0,3005 m** từ tâm xe.
- Mặt phẳng gốc càng: X = **0,3655 m** từ tâm xe, gồm 0,3005 + 0,065 m.
- Tâm lỗ trong hệ xe: `X = chassis_front_forward_m + 0.3005`, `Y = opening_lateral_m`.
- Khoảng gốc càng tới mặt lỗ theo X: `X - 0.3655`.
- Hướng vào lỗ: `entry_heading_rad = -face_yaw_rad`, dương quay trái xe.

Dùng `/odom` từ EKF, hệ `odom → base_footprint` (cũng chấp nhận child `base_link` vì cùng XY/yaw). Nội suy odometry tại **thời điểm chụp** để đưa tâm lỗ và hướng vào `odom`. Lọc vị trí/hướng trong `odom` giúp tránh độ trễ góc lọc khi chính xe đang quay. Không có odometry thì không tạo điểm đích giả hoặc chạy căn.

Điểm chờ của tâm xe là `G = C - (1.20 + 0.3655) × n`, với C là tâm lỗ trong odom, n là hướng xe dự kiến tại điểm chờ. Nếu hướng hiện tại đã lệch dưới 2°, có thể giữ hướng này và căn vị trí; vẫn kiểm tra góc thực và Y ở điểm cuối. Vì vậy vị trí chờ có thể lệch đôi chút so với đường pháp tuyến lý tưởng của pallet trong dung sai góc đã chọn.

Phải có ít nhất ba phép đo mới hợp lệ, hai chân mỗi chân ≥60 mẫu, lệch RGB/Depth ≤50 ms và hình học đúng vùng đã dùng khi đo. Tuổi phép đo tối đa **250 ms**, odometry **200 ms**. Lỗi đo lập tức đưa lệnh về 0; thiếu phép đo quá 0,75 s yêu cầu bấm bắt đầu lại. Sai/reset odometry hoặc mất lớp bảo vệ dừng và bỏ trạng thái chạy. Không dùng FPS JPEG của web để điều khiển.

## Bước đầu: chỉ xem lệnh, xe đứng yên

Giữ camera và bộ nhận diện/đo chạy như README/TRACKING.md. Bộ đo cần tham số
`--measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml`
để sử dụng hồ sơ bù đã đo; chế độ chạy thật sẽ từ chối phép đo thiếu dấu `target_bias_applied`.
Trong terminal mới:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_alignment.sh
```

Nếu web đang chạy bản cũ: Ctrl+C **terminal web**, sau đó mở lại:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_web.sh
```

Truy cập web như trước, mục **Căn xe trước pallet**. Mặc định hiển thị **KIỂM TRA** và nút **Kiểm tra lệnh căn (xe đứng yên)**. Nếu chưa có `/odom`, báo chờ odometry và vô hiệu nút bắt đầu. Bộ căn ở chế độ này **không tạo publisher** tới chuỗi điều khiển động cơ. Bấm kiểm tra sẽ xuất vận tốc đề xuất, không làm xe chạy và không mô phỏng xe tự di chuyển; nếu xe đứng nguyên thì sai số sẽ giữ nguyên.

Các topic xem trên laptop (source ROS và overlay interface):

```bash
ros2 topic echo /pallet/alignment/status
ros2 topic echo /pallet/alignment/proposed_cmd_vel
```

RViz: Fixed Frame `odom`; thêm Pose `/pallet/alignment/goal` và MarkerArray `/pallet/alignment/markers`. Mũi tên từ điểm chờ của tâm xe tới tâm lỗ có thời hạn 0,4 s; biến mất khi không còn dữ liệu hợp lệ. Pose là điểm chờ, **không phải lệnh Nav2**; khi không còn dữ liệu, không sử dụng Pose cũ. Không tự gửi Nav2 goal từ node này.

## Chuẩn bị chạy thật (thực hiện sau khi kiểm tra)

Cần bộ điều khiển bánh, EKF/IMU, TF và laser đang hoạt động. Chạy riêng bộ căn, không mở teleop hoặc Nav2 cùng lúc. Các lệnh dưới đây là chuẩn bị cho thử chuyển động; triển khai mã không tự chạy chúng.

Nếu chưa có bản build package robot hiện tại, build vào workspace riêng, không ghi đè bản cài cũ:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/build_pallet_alignment_robot.sh
```

Mỗi terminal phần cứng/laser phải source bản build này và workspace driver đã có:

```bash
source /opt/ros/jazzy/setup.bash
source /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra/data/alignment_robot_ws/install/local_setup.bash
```

Terminal phần cứng — lệnh này kích hoạt hệ truyền động; chỉ dùng khi đã sẵn sàng thử:

```bash
ros2 launch tai_robot_one real_hardware.launch.py \
  cmd_vel_input_topic:=/cmd_vel_safe use_cm029_teleop:=false \
  use_lift:=false home_lift_on_activate:=false use_rviz:=false
```

Terminal laser, nếu chưa chạy:

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/tai_lidar frame_id:=lidar_link
```

Terminal lọc scan:

```bash
ros2 launch tai_robot_one scan_self_filter.launch.py
```

Terminal lớp tránh vật cản riêng:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra/launch/pallet_alignment_safety.launch.py
```

Dừng terminal bộ căn đang ở chế độ kiểm tra bằng Ctrl+C, sau đó mới mở chế độ chạy thật:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_alignment.sh --execute
```

Chỉ khi web hiện **CHẠY THẬT**, có số đo và đủ điều kiện, nút **Bắt đầu căn xe** mới dùng được. Khởi động node chưa làm xe di chuyển. Nút bật/tắt YOLO vẫn chỉ điều khiển nhận diện. Nút **Dừng căn xe** hoặc dịch vụ sau đưa lệnh về 0:

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /pallet/alignment/set_enabled std_srvs/srv/SetBool '{data: false}'
```

Đường lệnh: `/pallet/alignment/cmd_vel` → `pallet_alignment_collision_monitor` → `/cmd_vel_safe` → `cmd_vel_stamper` → `/base_controller/cmd_vel`. Bộ căn xác minh monitor ACTIVE, tham số vùng dừng/scan, tham số stamper và mỗi đoạn chỉ có đúng một publisher được chỉ định. Laser quá 300 ms hoặc xuất hiện nguồn lệnh cạnh tranh sẽ dừng căn. Vùng dừng bao gồm thân/càng cộng biên 15 cm; hai điểm laser vào vùng này làm monitor dừng xe. Laser 2D chỉ bảo vệ vật cản cắt mặt phẳng quét, không đảm bảo thấy mọi vật thấp quanh càng. Lượt đầu tắt lùi tự động; `--execute` từ chối cấu hình bật lùi.

## Xác minh

Kiểm thử thuần động học và ROS chạy trong domain 231, tách khỏi robot. Mô phỏng hội tụ với Y = ±5 cm, ±10 cm và góc ±5°; kiểm tra điều kiện dưới 2 cm/dưới 2°, mất dữ liệu, tốc độ/gia tốc, không tự lùi và dừng bằng nút. Bài ROS dùng **Nav2 Collision Monitor thật nhưng đầu nhận cuối giả**, xác nhận chặn chạy khi thiếu lớp bảo vệ, có lệnh khi đủ điều kiện, dừng vì vật cản và nguồn lệnh cạnh tranh. Đây không phải kết quả thử xe chuyển động ngoài thực tế; độ bám RGB-D khi xe quay/tiến và độ chính xác động học cần kiểm tra ở lượt thử đầu.

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
.venv/bin/python -m unittest discover -s tests -v
```

### Xác minh luồng tự tắt YOLO

Đã chạy 59 kiểm thử đạt sau thay đổi, gồm kiểm tra cửa sổ 3–5 giây, confidence thấp/mất Depth/hết hạn không xác nhận, không tự căn sau nhận diện, yêu cầu seed mới khi nhận diện lại, đợi suy luận dở và phép đo chụp mới trước khi căn, chặn bật YOLO khi căn, hủy yêu cầu bắt đầu đang chờ và bỏ phản hồi trễ. Kiểm thử ROS dùng dữ liệu tổng hợp trong domain 231 và không kết nối động cơ. Giao diện được kiểm tra cú pháp JavaScript cùng trạng thái đếm thời gian, chờ YOLO dừng, nút dừng và khóa nhận diện lại. Chưa chạy thử xe chuyển động với luồng mới.

### Chuẩn bị package cho chạy thật

Đã build thành công `tai_robot_one` và `sllidar_ros2` vào workspace riêng trên Pi, kiểm tra resolve toàn bộ launch bằng `--show-args`, sinh URDF phần cứng (đúng `/dev/tai_drive`, `use_lift=false`, không home càng) và kiểm tra thư viện phần cứng không thiếu liên kết. Các thao tác xác minh này không mở cổng serial hay phát lệnh động cơ. Chưa kiểm chứng chuyển động thật của luồng căn; người vận hành mở bộ chạy thật rồi bấm căn như trên.
