# Triển khai r3-approved-lattice —08/11/2026

> Cập nhật 11/11/2026: Người dùng muốn thử xe thật theo hình A→C→B mới nhất, dùng odom bám đường; LiDAR và RGB-D chỉ dừng vật cản, bỏ qua đúng vùng pallet đã khóa sau khi YOLO tắt. Bộ căn thật cũ đã vô hiệu hóa. Đã thêm bộ lọc scan và bộ bám odom cho lượt thử không nâng, nhưng chưa cho xe chạy: kiểm tra chỉ đọc bằng `python3 simulation/r3_real_preflight.py --plan simulation/output/r3_camera_measured_preview/centered_20261111_145707/plan_refined/frozen_plan.json` báo BLOCKED vì hình dùng pose nhập/hình học mô phỏng. Lượt thử NCNN cùng odom/LiDAR/RGB-D thấy pallet nhưng RGB-D mất dấu sau khi tắt YOLO; cổng xác nhận độ chính xác tuyệt đối vẫn chưa đạt. Đã sửa luồng nhận diện để YOLO tự tắt sau khi nhận diện/đo ổn định dù cổng hiệu chuẩn chưa đạt, nhưng vẫn giữ trạng thái `CALIBRATION_REQUIRED` và chặn chuyển động. Các kết quả Gazebo bên dưới thuộc lượt thử trước, không chứng nhận kế hoạch mới.

> Cập nhật tiêu chí mô phỏng theo người dùng: chấp nhận phần tạo/bám quỹ đạo, sai số dừng dọc chỉ báo cáo và không chặn nghiệm thu phần bám. Giữ goal danh định gốc càng20mm, v≤0.15m/s, ngang≤20mm, góc≤0.5°. Nâng80mm chưa được miễn. Chạy xem: `./simulation/launch_r3_sim.sh world` rồi `./simulation/launch_r3_sim.sh run` ở terminal thứ hai.

> Ghi chú lịch sử 08/11: **thiết bị thật để sau; kiểm tra mô phỏng đạt trước**. Chỉ đạo ngày 11/11 ở trên thay thế ghi chú này. [Phân tích lỗi mô phỏng cũ](../simulation/output/r3_gazebo_final/failure_analysis.json).

Đã triển khai và chạy mô phỏng đúng chuỗi **Smac State Lattice vi sai/skid-steer → Nav2 Constrained Smoother cho approach → line cuối chính xác theo trục càng**. **Chưa nghiệm thu: Gazebo FAIL gap thực và lift; camera thật INCOMPLETE.** Không chạy robot thật, flash firmware, commit/push hoặc đổi bộ bám/PID. Các mục xóa cục bộ có sẵn trong git status vẫn được giữ nguyên.

## Thông số người dùng đã xác nhận trong phiên này

| Đại lượng | Giá trị |
|---|---|
| Pallet |355×355mm;4 chân50×50mm; trống cao100mm; mặt dày gần30mm; tổng khoảng130mm |
| Càng | dài385mm, phủ bì cặp155mm, dày3mm |
| Cao mặt trên càng |75mm tại trigger;74mm sau HOME_SETTLE1mm;154mm sau nâng80mm |
| Gap gốc càng sau dừng | mục tiêu20mm, chấp nhận15–25mm (±5mm) |
| Giới hạn |0≤v≤0.15m/s;vy=0; không lùi; góc≤0.5°; ngang≤20mm, ưu tiên≤10mm |
| Nguồn gia tốc/omega | pallet_alignment.yaml:0.08m/s²,0.20rad/s,0.40rad/s²; không có speed floor mới |

## Kết quả động học: đầu vào pose chuẩn, odom tổng hợp

Đây là mô hình unicycle lý tưởng, không phải đo RGB-D, Gazebo hoặc xe thật. Cả5 ca đạt các điều kiện bám/line/stop trong mô hình; kết quả nhiệm vụ vẫn INCOMPLETE vì chưa chứng minh hình học thật và chưa chạy lift trong mô hình này.

| Ca | Độ dài(m) | max v(m/s) | max góc line(°) | max ngang line(mm) | gap cuối(mm) |
|---|---:|---:|---:|---:|---:|
|straight|1.414747|0.150|0.063705|0.2901|19.9894|
|left|1.416693|0.150|0.185142|0.8427|20.0344|
|right|1.420631|0.150|0.149789|0.6819|19.9744|
|yaw_left|1.417036|0.150|0.181718|0.8271|20.0338|
|yaw_right|1.420368|0.150|0.153251|0.6976|19.9738|

Gia tốc lệnh đã kiểm lại: tối đa0.08m/s² và0.40rad/s². Profile giới hạn tốc độ theo độ cong/omega và giảm tốc trước cua;0.15m/s là trần, không là tốc độ bắt buộc mọi đoạn. Dừng ở A để đổi độ cong và dùng lại chức năng căn góc sẵn có trước entry. Mỗi ca có một pha căn góc thực thi, dù lattice raw không cần pure rotation trong5 đầu vào này.

Đường lattice ngắn hơn baseline quintic ở5 so sánh cùng S/A/goal trong fixture này; các file `baseline_comparison.json` có số liệu. Không khẳng định tối ưu toàn cục; baseline không là fallback.

- [Hình planned thẳng: tâm xe + gốc càng + envelope +4 chân](../simulation/output/r3_acceptance/straight/planned.png)
- [Hình overlay ca góc trái8°](../simulation/output/r3_acceptance/yaw_left/overlay_planned_odom.png)
- [Profile reference v/yaw](../simulation/output/r3_acceptance/yaw_left/reference_speed_heading.png)
- [Kết quả5 ca](../simulation/output/r3_acceptance/mission_result.json)

## Gazebo với feedback odom/IMU và truth scorer riêng

Lượt cuối có **1166/1166 mẫu** được ghép với truth, sai khác timestamp tối đa20.00ms. Controller chỉ đọc target khóa, odom/IMU và obstacle stop; scorer đọc named model poses trong tiến trình khác. Không dùng truth để chỉnh điểm dừng/insertion.

| Tiêu chí | Kết quả cuối | Đánh giá |
|---|---|---|
| Gap theo odom sau dừng |19.986mm | chỉ là ước lượng |
| Gap truth tại lúc xe dừng, trước nâng |41.618mm | **FAIL**, yêu cầu15–25mm |
| Max góc line theo truth |0.092659° | đạt0.5° |
| Max ngang line theo truth |0.7643mm | đạt20mm |
| Lift feedback, target80mm |73.803mm | **FAIL**, timeout |
| Pallet rise truth cuối |47.744mm | chỉ quan sát scorer |

Khoảng cách thực không đạt dù bám odom tốt. Không lấy đó làm lý do kéo goal theo truth hoặc hiệu chỉnh PID/bán kính bánh/ma sát để làm bài thử PASS. Controller nâng mô phỏng hiện dùng kp=1200,ki=0,kd=80 và bù trọng lực moving_mass=4kg; world có pallet1kg. Thiếu bù tải có thể giải thích sai lệch nâng tĩnh (suy luận từ source), nhưng chưa chỉnh thiết kế/thông số controller nâng.

- [Overlay planned/odom/truth cho cả B và F, có zoom điểm dừng](../simulation/output/r3_gazebo_final/overlay_planned_odom_truth.png)
- [Góc/ngang/gap/lift với truth riêng](../simulation/output/r3_gazebo_final/heading_lateral_gap_lift_truth.png)
- [Planned/odom của cả tâm xe và gốc càng](../simulation/output/r3_gazebo_final/overlay_planned_odom.png)
- [Kết quả nhiệm vụ đã chấm](../simulation/output/r3_gazebo_final/mission_result.json)

Một lượt trung gian đã bị loại vì Gazebo cũ còn sống sau khi đóng launch, trộn2 world chung partition. Lượt cuối chạy từ world sạch, có kiểm tra single publisher/clock epoch trước lệnh; dữ liệu lượt lỗi không nằm trong nghiệm thu này. Tất cả server mô phỏng do phiên này tạo được dừng khi bàn giao.

## Giới hạn và camera thật

`simulation/r3/fixture.json` có nhãn SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION. X_F=0.3655m lấy từ repo; bản fixture hiện đặt fork Y=0 theo xác nhận càng nằm giữa khung xe và housing Y=+20mm theo quan hệ F-H=-20mm. Tọa độ mắt RGB/optical và T_BF thật vẫn chưa đo đầy đủ. Tine widths53mm, centers±51mm,4 chân ở góc là hình học repo còn cần xác nhận. Pipeline/plots dùng cùng transform và hash; không cộng20mm vào pixel hoặc mặc định housing=optical.

Ở lượt 08/11, laptop chỉ thấy Microdia Integrated_Webcam_HD qua USB. Đến 11/11, Astra đã chạy trên Pi và có phép đo đứng yên, nhưng chưa export được session RGB-D khóa hợp lệ cho r3; hình A→C→B hiện dùng pose nhập theo số đo. T_BC, housing reference, full T_BF thật và các mốc nhánh/chân còn thiếu; không dựng đường chạy thật từ fixture. Hồ sơ camera cũ có max sai số yaw khoảng0.7769°, chưa chứng minh0.5°. RGB-D simulation case cũng chưa chạy và được ghi INCOMPLETE riêng, không trộn với pose chuẩn.

`simulation/r3_camera_preview.py` là adapter preview **không publisher drive/lift**, nhận session đo đứng yên đã export và calibration/geometry xác nhận. Nó tách optical SE(3) khỏi scalar đã hiệu chỉnh base, yêu cầu mẫu riêng sau YOLO off, khóa target/hash rồi lập planned-only. Chưa nối live acquisition lifecycle/mission launcher robot thật; các launch thật cũ vẫn là baseline, không trở thành r3-compliant.

## Source và kiểm chứng

- `simulation/r3_nav2_backend/`: tải trực tiếp hai plugin Nav2 đã cài; diff primitives5cm, reverse expansion=false, internal Smac smoothing=false; giữ direction holders trước Constrained solve, giữ phase quay, không smooth line.
- `simulation/r3_core.py`: SE(2)/SE(3), goal fork/base, frame/metadata/hash, exact line và swept geometry lấy mẫu≤2mm/0.1°. Kiểm hình học fixture không thay contact-force instrumentation.
- `simulation/r3_sim.py`:5 ca, fixed-phase adapter, Pure Pursuit hiện tại, speed/acceleration caps và artifact export.
- `camera/detect_astra/scripts/pallet_alignment_core.py`: chỉ tách lại luật stationary yaw đã có thành hàm dùng chung;85 tests camera/ROS đạt.
- Cả firmware càng riêng và IMU+càng: HOME_SETTLE đúng100steps=1mm rồi zero; giữ upper/fault/watchdog/debounce riêng từng firmware; normal lower không đặt lại zero tại trigger cũ.

Đã đạt: **25 tests mô phỏng**, **85 tests camera/ROS**, native C++ tests trên lift source thật của2 firmware, build ROS package và2 build ESP32. Không flash. Native stub tests xác nhận logic/pulse count, không chứng minh cơ khí/step loss. [Log kiểm tra](../simulation/output/r3_validation/simulation_tests.txt).

Chạy lại và định dạng input: [simulation/r3/README.md](../simulation/r3/README.md). Các nguồn API đã đối chiếu: [Nav2 Jazzy State Lattice](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/) và source/header ConstrainedSmoother của Nav2 Jazzy.

Việc còn phải hoàn tất: xử lý nguyên nhân sai khác odom/truth và nâng80mm trong mô phỏng, xác nhận geometry/extrinsics thực, chạy case RGB-D, rồi đo camera đứng yên và cho người dùng xem planned. Bất kỳ thay thiết kế/controller/giới hạn nào phải được duyệt trước; không có quyền chạy robot thật từ các kết quả trên.

## Giao diện và thử tư thế tự nhập (bổ sung cùng phiên)

Đã thêm `./simulation/launch_r3_sim.sh gui`: ảnh camera trực tiếp, ảnh nhận diện cuối, kết quả YOLO/RGB-D, ba ô khoảng cách–ngang–góc tham chiếu **tâm gốc càng**, nút tạo/xem trước/chạy/dừng và đúng hai đường planner/thực tế F. Không min/max số hữu hạn, không deadline tổng chạy xe; giữ điều kiện stale/vật cản và các thông số đã chốt. Hai nguồn Pose nhập và RGB-D khóa độc lập; tắt phép đo pallet trước khi chạy. Map/sàn được dựng theo case, scorer đăng ký world→odom theo đúng spawn/yaw; không dùng truth điều khiển.

Đã chạy36 tests mô phỏng và chạy luồng nút giao diện trên Gazebo. Ca thẳng mới đạt tracking (max yaw0,02862°, ngang1,1182mm). Ca2m, ngang+0,25m, góc+10° hoàn tất nhưng FAIL ngang truth30,394mm >20mm; không nới yêu cầu hoặc thay follower. Planner tạo được cả ca35° và ca20m. YOLO hiện nhận0 pallet trên hình hộp mô phỏng; RGB-D chưa khóa nên kiểm chứng nhận diện/RGB-D end-to-end vẫn INCOMPLETE. Xem [bằng chứng và ảnh giao diện](../simulation/output/r3_gui_validation/README.md) và [cách dùng](../simulation/r3/README.md). Thiết bị thật vẫn hoãn theo người dùng.

## Thử màu pallet theo yêu cầu #25A18E

Đã đổi riêng ambient/diffuse của5 visual pallet r3, giữ hình học/collision, YOLO best_v3 và confidence0.25. Thử render tĩnh riêng (không xe/controller) cùng camera, ánh sáng, gốc càng cách1,5m: cả xanh cũ và #25A18E đều0 phát hiện. Khâu nhận diện/RGB-D vẫn INCOMPLETE; không cấp pose giả vào GUI. 37 tests mô phỏng đạt, gồm kiểm tra XML chỉ thay màu. [Ảnh và bằng chứng](../simulation/output/r3_color_25a18e/README.md).

## Sửa hiển thị pallet cố định trong world

Giao diện trước đây vẽ trong odom có trục đổi theo yaw xe ban đầu, khiến pallet trông bị nghiêng khi đổi góc xe. Đã chuyển riêng lớp hiển thị (cả hai đường F) sang world bằng base_world của ca, pallet vật lý lấy từ pallet_world cố định. Vẽ hình xe/càng ban đầu để thấy thay đổi pose. Không sửa đích camera, planner/follower, ROS TF, CSV hoặc scoring. 40 tests mô phỏng đạt. [Ảnh, kiểm tra GUI và replay log đã ghi](../simulation/output/r3_world_view_fix/README.md). Không chạy xe mới cho thay đổi này.
