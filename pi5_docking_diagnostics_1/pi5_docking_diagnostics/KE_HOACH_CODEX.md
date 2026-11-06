# Kế hoạch cập nhật: đo pallet trên robot thật khi đứng yên trước, rồi kiểm tra chuyển động và docking

Ngày cập nhật: 06/11/2026. Nguồn: https://github.com/taimecha/robot_amr
Commit đã đọc: `9398733b98e6f777f9d02285e7d837ba361073f3` (06/11/2026, 09:10 giờ Việt Nam).
Thư mục người dùng sẽ cập nhật trên máy họ: `/home/tai/Downloads/pi5_docking_diagnostics`.
Bản này thay thế kế hoạch cũ. Tài liệu ở `reference/` là lịch sử, không dùng làm chỉ dẫn hiện hành. Không giả định thư mục Downloads nằm trên Pi: Codex phải xác định host hiện tại, SSH target và repo thực trước khi sửa/chạy.

## 1. Mục tiêu, thứ tự và phạm vi đang thực hiện

Ưu tiên mới: dùng camera Astra Pro và Pi 5 thật, robot ĐỨNG YÊN, kiểm tra nhận diện và pose pallet thật tương tự trình tự mô phỏng trước. Chỉ sau khi chứng minh sai số X/Y/yaw phù hợp mới tiếp tục các giai đoạn của kế hoạch trước.

| Gate | Công việc | Quyền chuyển động |
|---|---|---|
| G0 | Kiểm kê source, thiết bị, topic, calibration, thu baseline | Không |
| G1 | Nhận diện/đo/lọc pallet thật khi đứng yên, đối chiếu thước/mốc chuẩn | Không |
| G2 | Kiểm chứng tắt YOLO, pose mới sau tắt và khóa đích tĩnh | Không |
| G3 | Kiểm tra truyền động + camera khi chạy ngắn 30 cm–1 m | Người vận hành bắt đầu từng lượt |
| G4 | Preview quỹ đạo từ pose đã đo và shadow follower | Không |
| G5 | Bám đường trên vùng trống rồi đến pre-dock, không chạm pallet | Sau G3/G4 đạt và đã chọn controller |
| G6 | Đưa càng theo đoạn thẳng, xác nhận đủ sâu, sau đó thử nâng | Sau nghiệm thu riêng hình học, bảo vệ và xác nhận insertion |

Ngay bây giờ triển khai và chạy G0–G2 trước. Chuẩn bị phần sau nhưng không tự bỏ qua gate. Khi G2 đạt, tiếp tục phần chỉ đọc/offline không cần hỏi lại các lựa chọn thường lệ; chỉ bắt đầu lượt chuyển động khi người vận hành chủ động ra lệnh trong vùng đã chuẩn bị. Chưa chốt planner/controller thay người dùng.

## 2. Tiến độ thực tế đã có — tận dụng, không làm lại từ đầu

- Mô phỏng Gazebo: YOLO `best_v3.pt`, đo tâm và yaw bằng Depth, lọc pose qua `StablePoseWindow`, ít nhất 5 pose phù hợp rồi tắt YOLO, xác minh thêm 2 cặp mới, khóa X/Y/yaw trong odom.
- `simulation/pallet_rgbd_filter.py`: khớp mép trước bằng least squares có trọng số robust, loại outlier, cửa sổ pose bounded. Chưa phải quality gate nhòe/rung/ánh sáng đầy đủ.
- `simulation/one_pallet_case.py`: nhận heading đã đo để lập đường. Có đường quintic + straight. Follower demo vẫn là Pure Pursuit tối giản, KHÔNG phải Nav2 RPP production.
- `camera/detect_astra/scripts/pallet_rgbd_estimator.py`: pipeline đo hai chân/lỗ đã có bù Depth, vị trí và tùy chọn floor-referenced heading; bổ sung median góc mặc định 5 frame, 4/5 inlier trong 2.5°, loại mẫu góc nhảy nhưng giữ RGB tracker khi vị trí ổn. X/Y vẫn dùng Depth mới hiện tại.
- `pallet_acquisition.py`: bỏ deadline tổng, `MAX_SECONDS=None`; có gate `accuracy_validated=False` mặc định. Chưa có chứng cứ độc lập nên mặc định chưa READY. Đây là hành vi phải xử lý bằng dữ liệu nghiệm thu, không đổi cờ thành True thủ công để vượt gate.
- Có 15 unit tests mô phỏng đã chạy đạt trong lần review, chưa kiểm tra đầy đủ ROS/camera thật/Pi trong phiên này.
- Đoạn insertion của demo còn dùng pose Gazebo để tính khoảng còn lại và xác minh nâng. Không thể lấy kết quả đó làm nghiệm thu frozen-target trên xe thật.

Trước làm: đọc AGENTS.md nếu có, git status/HEAD/diff và source mới; repo đã đổi tên nhưng path cài trên Pi có thể vẫn tên cũ. Không reset/hard checkout hoặc ghi đè thay đổi của người dùng. Ghi khác biệt giữa local runtime và commit trên.

## 3. Không sao chép giả định mô phỏng sang camera thật

1. Gazebo demo đo tâm MẶT TRÊN và yaw mép mặt trên; pipeline thật đo tâm LỖ/CHÂN và hướng mặt trước. Hai điểm mục tiêu không tự bằng nhau. Xác định tọa độ cần cho càng: tâm hành lang giữa các lỗ, mặt trước pallet, chiều sâu cắm.
2. Trong demo: vùng z≈8–14 cm hoặc 9.8–12 cm, rộng mặt ≥19 cm, pallet 35.5 cm, pitch/extrinsic hằng số. KHÔNG dùng các giá trị đó cho pallet/camera thật chưa đo. Đưa hình học đo được vào config có phiên bản.
3. Gazebo Depth `32FC1` (m), camera thật có thể `16UC1` (mm). Đọc encoding/endian/step thực, chuyển đơn vị một lần. Không reshape giả định hoặc nhầm camera_link với optical.
4. Giữ K/D, RGB–Depth registration, Depth correction và hồ sơ bù đã có. Không cộng bù góc hai lần khi dùng `--heading-floor-reference`; đọc `HEADING_REFERENCE.md` và `pallet_rgbd_estimator.measure()`.
5. Không đổi estimator đang dùng chỉ để giống hình demo. Baseline A dùng pipeline thật hiện tại; nhánh B thử thuật toán mép mới nếu mặt pallet nhìn đủ. Cùng dữ liệu đầu vào, cùng ground truth. Chỉ chọn nhánh B nếu sai số thật tốt hơn và không tăng false-lock.
6. Chỉ dùng ground truth để chấm/hiệu chuẩn có kiểm soát, không nạp pose thật vào estimator hoặc planner. Bật chứng nhận calibration sau nghiệm thu là khác với cấp đáp án tọa độ từng ca thử.

## 4. G0 — chuẩn bị hệ đo chỉ đọc

- Xác nhận Pi 5, OS/ROS Jazzy/RMW, RAM/nguồn/tản nhiệt, camera/USB, model/runtime NCNN/PyTorch thực tế. Không chuyển inference sang laptop.
- Kiểm tra source/overlay đang chạy đúng version có `--heading-filter-frames`; không chỉ sửa source nhưng chạy install cũ. Lưu model/calibration/config hashes.
- Khởi động camera/perception riêng, dừng web/RViz remote/stream JPEG không cần thiết. Không launch `real_hardware.launch.py`, không `--execute`, không ARM/ENABLE/HOME càng chỉ để đo pallet.
- Nếu /odom/IMU cần chạy: đọc trạng thái hiện có hoặc dùng node cảm biến riêng không enable actuator. G1 đo base-relative vẫn thực hiện được khi chưa có /odom; ghi ODOM_UNAVAILABLE, không dựng TF giả và không chốt target odom ở G2.
- Monitor G0–G2 không tạo publisher tới /cmd_vel, /cmd_vel_safe, base controller, càng; không mở serial actuator. Kiểm tra graph không có nguồn lệnh xe hoạt động; zero command không tự bảo đảm actuator vô hiệu.
- Đánh dấu bánh/thân và gá camera; để xe yên cơ học. Kiểm tra nhiều frame không có chuyển động thân qua marker/IMU/odom nếu có; odom=0 không loại trừ gá camera rung.
- Chạy host_health + raw camera baseline nhẹ ≥120 s; ghi mọi sensor error, không yêu cầu FPS acquisition cao.

## 5. G1 — phép đo vị trí/góc pallet thật khi xe đứng yên

### 5.1 Hệ tọa độ và ground truth

Dùng base_link/base_footprint: X tiến, Y trái, yaw dương quay trái. Tách:
- X tới MẶT TRƯỚC pallet và Y tới TÂM LỖ được chọn, không dùng tâm mặt trên lẫn lộn.
- `face_yaw_rad` của mặt pallet và `entry_heading_rad` hướng đưa càng; theo repo entry=-face, phải kiểm chứng dấu bằng một ca trái và một ca phải.
- Bearing tới tâm lỗ không phải yaw mặt pallet. Không dùng abs(yaw) để điều khiển.
- Gốc đo khoảng cách: mặt trước chassis/gốc càng/mũi càng/tâm base phải ghi rõ. Hiệu chỉnh rigid transform một lần; scalar đã bù không được transform như optical XYZ lần hai.

Người vận hành đo bằng thước/laser và mốc vuông góc sàn/marker có kiểm chứng, lưu độ bất định dụng cụ. Nếu xác định góc từ hai điểm mặt pallet: đo tọa độ dọc X_left và X_right tại hai vị trí Y cách nhau B đã biết trong cùng hệ xe; face_yaw=atan2(X_left-X_right,B) theo quy ước mặt phẳng của repo. Không thay bằng hai khoảng cách xiên từ laser mà vẫn dùng công thức này.

Mỗi ca phải có GT X/Y/yaw thực tế + uncertainty; không tự coi ca được đặt tên “0 độ” là góc bằng 0. Thiếu GT vẫn ghi dữ liệu được nhưng kết quả ACCURACY_NOT_VERIFIED, không PASS. Marker dùng làm tham chiếu cần xác minh kích thước/pose, không coi camera thứ hai tự là chuẩn.

### 5.2 Các tư thế thử

Robot không chạy ở G1. Giữ xe đứng yên, người vận hành đặt pallet vào các vị trí khác nhau; ghi lại GT sau mỗi lần di chuyển pallet và reset phiên acquisition. Nếu phải dịch chuyển xe bằng tay, tách phiên và invalid target/odom epoch cũ.

Bắt đầu 3 ca: chính giữa và gần song song; lệch trái nhẹ; lệch phải nhẹ. Sau khi đo đúng hệ/dấu mới mở rộng:
- Khoảng cách quanh vùng sẽ dùng để khóa đích (người dùng khoảng 1.1–1.2 m theo mốc thực), thêm 1–2 khoảng xa hơn còn đủ hai chân/mặt trong FOV; không áp khoảng spawn base 1.45–3.5 m của demo làm chuẩn.
- Lệch ngang 0, ±5 cm, ±10 cm trong FOV.
- Góc tương đối 0, ±5°, ±10°; ±15° chỉ nếu hình học và dữ liệu hỗ trợ.
- Không cần toàn bộ tích Descartes ngay: chọn 9–15 ca bao phủ hai dấu và khoảng cách. Ít nhất 3 lần acquisition độc lập mỗi ca; đo 60–120 s mỗi lượt sau warmup. Đây là quy mô chẩn đoán, chưa phải chứng nhận sản phẩm.
- Tập ca dùng hiệu chỉnh tách khỏi tập kiểm tra giữ lại; không fit bias trên mọi ca rồi báo chính những ca đó đạt.

Dùng `templates/stationary_cases.csv`, không điền số đo giả. `case_id`, frame/mốc, model/profile hash, ánh sáng và pallet ID phải gắn vào mỗi run.

### 5.3 Luồng nhận diện/lọc ban đầu

Không giới hạn tổng thời gian acquisition, không yêu cầu FPS cao. Không giảm resolution/model chỉ để tăng FPS; cũng không tự đổi model đã có. Chạy inference chậm vẫn phải giữ timestamp nguồn và độ mới phép đo đúng semantics; frame cũ không trở thành mới nhờ đóng lại stamp. Thu đủ mẫu mới trên cửa sổ tối thiểu là điều kiện chất lượng, không là deadline hết hạn.

Baseline pipeline thật trước, sau đó bật đối chứng:
- Gate timestamp/đồng bộ/encoding/registration, frame mới duy nhất.
- Gate nhòe (Laplacian/Tenengrad), clipping/tối và chuyển động khi đứng yên, có confidence về texture; ngưỡng dựa baseline, không copy 80 hoặc 1.5 px từ synthetic replay.
- Depth: loại zero/NaN/out-of-range, đủ mẫu hai chân/ROI, median/MAD và residual plane/edge nếu có; không hole-fill mép lỗ hay dùng Depth cũ bù điểm thiếu.
- Lọc góc có sẵn 5 frame; nhánh pose-window mới kết hợp X/Y/yaw sau khi hệ/mốc thống nhất. Không áp hai median liên tiếp tùy tiện; ghi raw/filtered stamps và effective delay.
- Spatial smoothing hay lọc thêm frame là ứng viên A/B, không bắt buộc. Tránh làm mờ RGB khiến mép/mask thay đổi. Pose mượt nhưng bias tăng là FAIL.
- Bộ đệm hữu hạn, reset khi đổi pallet/pose/gá/calibration, dữ liệu gián đoạn hoặc xe bị xê dịch. Angle aggregation phải xử lý wrap nếu mở rộng khỏi miền nhỏ ±20°.

### 5.4 Đầu ra terminal và báo cáo

Mỗi giây in: case/state, X_raw/Y_raw/yaw_raw, X_filtered/Y_filtered/yaw_filtered, GT và error, số mẫu hợp lệ/bị loại + lý do, MAD/peak-to-peak, age/sync, CPU từng lõi, nhiệt/throttle. Chưa có GT in N/A, không điền 0. Không stream ảnh; ảnh/ROI/rosbag cửa sổ ngắn chỉ lưu cục bộ nếu cần replay.
Ghi đủ mẫu trước gate để thấy rejection/false lock, không chỉ giữ phần đẹp. Median từ cửa sổ phải kèm oldest/newest stamp, window span, outlier count; không diễn giải thành độ chính xác độc lập giảm 1/sqrt(N).

### 5.5 Điều kiện đạt G1

Tách 3 thứ: nhận diện đúng pallet, repeatability (nhiễu), absolute accuracy (bias). G1 chỉ đạt khi cả ba đáp ứng vùng làm việc và error budget đã định từ hình học thật.
Gọi c_min là độ hở nhỏ nhất từng càng/lỗ, D là quãng/projection hiệu dụng tới đoạn insertion; cần eY + D*tan(eYaw) + odom_drift_budget + tracking_budget + mechanical_margin < c_min. Dùng giới hạn bảo thủ/percentile đã khai báo, không chỉ mean.
Ví dụ D=1.2 m thì 1° thêm khoảng 21 mm; vì vậy dung sai cũ 2 cm/2° không tự đủ. Đo chiều rộng lỗ, bề rộng/độ lệch hai càng, cao độ và độ dài thật trước phân bổ.
Trong `acceptance_policy.json`, budget chưa đo để null => kết quả UNCONFIGURED/ACCURACY_NOT_VERIFIED. Codex được đề xuất budget sau đo và cho người dùng xem; không tự nới để PASS. Nếu dụng cụ chuẩn có uncertainty lớn hơn budget cần, báo chưa đủ khả năng kiểm chứng.
Tốc độ nhận diện KHÔNG là tiêu chí pass G1; tuổi data và đồng bộ vẫn cần kiểm soát riêng. Tách thời gian inference với tuổi cặp RGB-D phục vụ đo sau detector dừng. Mốc 50 ms sync/250 ms result của repo cần giữ hoặc chứng minh cách xử lý đúng, không tăng để che queue.

## 6. G2 — chuyển giao sau tắt YOLO và chứng nhận khóa đích, vẫn đứng yên

Tránh vòng lặp “cần READY để đo accuracy nhưng cần accuracy mới READY”: G1 observer đọc candidate/raw pose và /pallet/target độc lập với acquisition READY. Nếu chưa có candidate pose xuất ra, thêm topic/log read-only; không tắt accuracy gate để lấy số.
Sau G1: tạo qualification record chứa model/profile hashes, camera/pallet geometry, miền làm việc, dataset IDs, held-out errors, uncertainty, tiêu chí pass và reviewer. Record không chứa tọa độ đáp án từng ca để điều khiển. Khi record khớp runtime mới được xét `accuracy_validated`, không hardcode True như test synthetic; thay gá/model/profile hoặc ngoài miền thì invalid.
Trình tự:
1. Nhận diện đứng yên, đủ bằng chứng target identity + fresh geometry + stability.
2. Xác nhận YOLO dừng/inference hoàn tất/heartbeat tắt; giữ đo RGB-D tạm thời.
3. Thu thêm cửa sổ pose mới sau stop, kiểm tra X/Y/yaw và consistency với cửa sổ trước. Demo dùng hai frame; robot thật chỉ dùng hai nếu đã nghiệm thu đủ, mặc định số mẫu/thời lượng cấu hình theo evidence G1.
4. Chốt immutable target từ phép đo mới (không GT), gắn target_id, odom epoch, capture window, calibration/qualification hashes và uncertainty.
5. G2 xuất `STATIC_QUALIFIED` hoặc `STATIC_REVIEW_REQUIRED`, KHÔNG tự cho chạy.
6. Chế độ frozen-target: trước chuyển động tắt PALLET detector/tracker/pose estimator, kiểm tra không còn update target; obstacle-only RGB-D/LiDAR vẫn có thể chạy riêng. Ở G2 dừng tại đây và kiểm tra target không đổi khi nhìn màn hình; chưa phát lệnh.

Base-relative đo chính xác nhưng chưa có odom hợp lệ thì vẫn có thể đạt G1; G2 khóa odom phải chờ timestamp/TF/epoch đúng. Không biến pose base thành odom bằng identity giả. AMCL/map correction không được kéo target local đã khóa.

## 7. Các chương trình/khung cần Codex hoàn thiện

| File trong gói | Vai trò | Hiện trạng gói |
|---|---|---|
| host_health.py | tài nguyên Pi cơ bản | chạy được, read-only, stdlib; chưa đủ monitor toàn bộ |
| metrics.py | thống kê finite samples | chạy được |
| stationary_pallet_probe.py | observer G0/G1, raw/filtered vs GT, không phụ thuộc READY | code giả mới |
| qualify_static_target.py | hồ sơ nghiệm thu và handoff/khóa ở G2 | code giả mới |
| frame_quality_gate.py | chất lượng frame/Depth/pose và A/B | code giả đã chỉnh theo source mới |
| rgbd_probe.py | timestamp/queue/Depth health | code giả |
| camera_motion_probe.py | rung/blur bù chuyển động | code giả |
| drive_probe.py | cmd/encoder/PWM/IMU | code giả |
| short_motion_runner.py | dry-run rồi bài chạy ngắn | code giả, chưa điều khiển |
| frozen_path_proposal.py | đề xuất planner/follower từ target đo | code giả |
| report_run.py | tổng hợp gate/results | code giả |

Tận dụng `verify_pallet_handoff.py`, `record_pallet_calibration.py`, tests và interfaces của repo; không nhân đôi pipeline camera chỉ để in terminal. Kiểm tra tool handoff cũ còn yêu cầu >=10 Hz hoặc tên phase cũ không; phân biệt MONITOR_SESSION_DURATION với acquisition timeout. Cửa sổ ghi 120 s kết thúc có thể trả INCOMPLETE, không được làm acquisition tổng hết hạn.
Scaffold không phải chương trình chạy thật; Codex phải hoàn thiện, fake/replay test rồi mới nối robot. Không tự chạy file Gazebo có `--gazebo-execute` hoặc `--insert-and-lift` trên Pi.

## 8. Tài nguyên và mạng: giữ yêu cầu cũ, áp dụng từ G0

Dữ liệu chỉ xử lý/lưu trên Pi; SSH terminal 1 Hz, không JPEG/web/RViz remote/ROS bridge. Chuẩn bị dependency/model trước phiên đo. Codex cloud/extension cần mạng: tách triển khai online khỏi phiên đo cô lập; dùng SSH thường để chạy thử offline.
DDS localhost cho mọi process sau restart cùng ROS_DOMAIN_ID, kiểm tra RMW/static peers/discovery config; LOCALHOST không đủ bảo đảm cấm egress toàn máy. Nếu cần đúng yêu cầu chỉ SSH: firewall tạm IPv4/IPv6 cho loopback và phản hồi đúng SSH tuple, chặn kết nối IP khác; không miễn trừ toàn bộ established cloud/DNS/NTP. Kiểm kê SSH/counters, lưu rules và rollback cục bộ trước áp dụng, không flush rules hệ thống. Không tự làm rớt SSH để chứng minh cô lập. Capture/counters ít nhất 60 s, ghi NOT_ISOLATED nếu không thực hiện được. Không có raw stream ra laptop.
CPU per-core/process/thread, IRQ/USB, PSI, RAM/swap, I/O, nhiệt/xung, active và historical throttle, kernel USB errors. Không có vcgencmd/sysfs thì N/A. Không pin core/đổi priority/threads ngay: baseline -> A/B từng thay đổi. Không gán mọi lỗi cho CPU tổng.
Frame time: header/clock domain, receive monotonic, queue wait, processing, result publish/receive, unique stamp. Thiếu capture time thật thì gọi driver stamp. Không suy packet loss từ frame gap.

## 9. G3 — sau static qualification mới chạy ngắn chẩn đoán

Xác nhận source hardware/udev/firmware và single command owner; đo deadzone có tải hiện tại, đối chiếu target/measured FL FR RL RR + PWM + IMU. Lệnh demo 0.10 m/s và omega 0.20 rad/s chưa được chứng minh phù hợp xe thật; lịch sử có breakaway >0.10 và 0.25–0.35 rad/s chỉ là giả thuyết cần đo lại. Không ép lệnh nhỏ lên min speed tự động hoặc đổi PID để test pass.

Ma trận tiến trong vùng trống, đủ xa pallet: 30 cm ->50 cm ->1 m. Đổi hướng mượt 0->±5°->0, sau đạt mới ±10° và ±15°, chiều dài phù hợp Rmin/gia tốc/footprint. Thiếu quãng không ép góc, trả SKIP có lý do. Quay về heading 0 không có nghĩa về cùng line; đo lệch ngang đúng đường dự kiến. Không tự lùi trả xe.
DIAGNOSTIC: giữ đo RGB-D pallet chỉ để chẩn đoán, không lái theo kết quả. Mỗi case tối thiểu 5 lượt mỗi chiều khi đủ điều kiện; người vận hành bắt đầu từng lượt. Bù ego-motion bằng timestamp/extrinsic/depth/marker trước tính rung; flow thô không là rung. Palette odom residual phải gần cố định nhưng odom drift cũng là nguồn lỗi, cần mốc ngoài đối chứng.
Stop/latch khi stale safety/odom, epoch reset, command conflict, lease/SSH watchdog mất, obstacle/corridor violation hoặc operator stop; không auto-resume. Lớp bảo vệ thấp độc lập logger/SSH. Không tắt safety để benchmark.
Acquisition không yêu cầu FPS; DIAGNOSTIC chuyển động vẫn đo valid Hz/age/gap và so mức cần để đánh giá. Không sửa nghĩa metric để vượt ngưỡng. Ghi baseline không monitor nặng, monitor nhẹ, recorder ngắn, full sensors, detector on/off.

## 10. G4–G5 — đường từ pose thật đã khóa và bám tới pre-dock

Không dựng lại phần mô phỏng đã có. Tận dụng quintic+straight, thêm kiểm chứng trên miền thật: đủ độ dài sửa ngang/yaw, arc-length/derivative/cusp, kappa/rate, wheel speed/accel/jerk, full swept footprint gồm càng/tải và vùng chưa quan sát. Không chỉ check corner sampling thưa hoặc tâm robot.
Tọa độ đầu cuối là tâm HÀNH LANG LỖ và hướng đo, không tâm mặt trên chưa biến đổi. Planner/follower runtime không được đọc Gazebo world pose, spawn yaw, OnePalletCase ground truth hoặc bảng GT; ground truth chỉ vào scorer không có quyền điều khiển. Tránh API có fallback heading sang case yaw khi đo thiếu: reject INVALID_TARGET.
Quintic+straight là ứng viên ưu tiên; clothoid là đối chứng. Follower demo không gọi là RPP. Đưa so sánh Pure Pursuit đã có / RPP docking riêng / phương án khác để người dùng chọn trước khi tích hợp motor; MPC chưa ưu tiên khi chưa có bằng chứng cần.
Muốn cong nhẹ mà khoảng sửa không đủ thì REPOSITION_REQUIRED; không quay tại chỗ rồi đi mà bỏ yêu cầu người dùng. RPP riêng phải kiểm logic rotate-to-heading/recovery, giữ collision check, không thay Nav2 profile toàn cục. Kèm speed envelope, stop distance, tiến độ đơn điệu, kiểm X/Y/yaw ở cuối, không chỉ XY.
Chạy shadow (không motor) -> sàn trống có GT -> pre-dock chưa tiếp xúc pallet. Lúc G5 FROZEN_TARGET: YOLO và PALLET RGB-D measurement/tracking đã tắt; odom/IMU feedback bám đường, RGB-D obstacle-only và LiDAR có thể chặn/dừng, không sửa target/replan/auto-resume.

## 11. G6 — insertion và nâng, sau cùng

Mô hình lỗ/càng/hành lang và tải thật, height/length/dừng đo độc lập. Kiểm insertion gate khi chưa vào: Y/yaw/uncertainty trong budget; không xoay sửa hướng khi càng đã cắm trong lỗ. Tính điểm dừng từ pose camera khóa và chiều sâu thật, không từ GT từng thời điểm.
Pallet là target nhưng vẫn có thân/đố gỗ cản: không xóa toàn mask/pallet khỏi costmap hoặc tắt Collision Monitor để vào được. Corridor theo state phải được nghiệm thu. Camera mù gần theo người dùng ~1.1 m và LiDAR cao không đảm bảo thấy vật quanh càng; nếu thiếu coverage phải bổ sung giải pháp xác nhận/bảo vệ phù hợp trước insertion thật, không suy “ảnh không thấy cản” là đường trống.
Nâng chỉ sau xe dừng, càng đúng cao độ, homed/endstop hợp lệ, insertion depth được xác nhận bằng phương án độc lập đã nghiệm thu, tải/ổn định cho phép. Base lock khi nâng. Chưa có xác nhận thì WAIT_INSERTION_VERIFICATION, không AUTO_LIFT vì odom tới goal.

## 12. Đầu ra và điều kiện tiếp tục

Mỗi run: manifest.json (host/commit/diff/config/model/calibration/GT/clock), metrics.csv raw/filtered, events.jsonl, summary.json và report.md. Bảo toàn invalid/rejected/timeout counts, missing N/A; file raw ngắn có cap dung lượng trên Pi.
G0–G2 bàn giao trước: schema tọa độ, báo cáo error vs GT theo ca, before/after filter, bias theo khoảng/góc, dấu yaw, drift khi đứng yên, repeatability, effective age, CPU/USB, qualification record hoặc lý do BLOCKED, và chính xác lệnh chạy chỉ đọc trên host đã xác minh.
G2 chưa đạt không chạy G3–G6. G2 đạt không tự chứng nhận chuyển động; còn G3 odom/slip/deadzone và G4/G5 track error. Mọi kết quả phải ghi provenance REAL_PI_STATIONARY / REAL_PI_MOTION / SYNTHETIC / GAZEBO; không trộn report cũ làm chứng cứ runtime.

## 13. File và nguồn đối chiếu

Repo pinned: https://github.com/taimecha/robot_amr/tree/9398733b98e6f777f9d02285e7d837ba361073f3
Đọc camera/detect_astra/{README.md,CALIBRATION.md,HEADING_REFERENCE.md,TRACKING.md}; scripts/{pallet_rgbd_estimator.py,pallet_acquisition.py,publish_pallet_detection.py,pallet_perception_config.py,verify_pallet_handoff.py}; simulation/{README.md,pallet_rgbd_filter.py,one_pallet_visualizer.py,gazebo_dock_trial.py,one_pallet_case.py}.
TRACKING.md và README còn đoạn lịch sử 3–5s/3 frame; code hiện tại và yêu cầu trong kế hoạch này ưu tiên. Codex cập nhật docs sau khi triển khai, không dùng ghi chú cũ để phục hồi deadline đã bỏ.
Đây là kế hoạch/các hợp đồng code giả, không phải chứng nhận đã chạy G0–G6 trên robot. Chưa sửa repo GitHub, chưa kết nối hay gửi lệnh tới robot.
