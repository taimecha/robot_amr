> LỊCH SỬ — đã được thay thế bởi KE_HOACH_CODEX.md ngày 06/11/2026. Không triển khai theo thứ tự cũ.

# Kế hoạch cho Codex trên VS Code / SSH vào Pi 5

Ngày: 05/11/2026. Dự án: https://github.com/taimecha/tai_robot_one_community
Mốc mã nguồn đã đọc: 9056b5f (01/11/2026). Trước khi làm phải kiểm tra HEAD, AGENTS.md, diff và cấu hình đang chạy trên Pi; không coi snapshot này là mã đang được triển khai.

## 1. Yêu cầu và giới hạn triển khai

Làm chương trình chạy và lưu dữ liệu tại Pi 5; chỉ xem bảng tóm tắt terminal qua SSH. Không stream ảnh, ROS/DDS, web, telemetry hay gửi dữ liệu đến laptop/mạng ngoài. Không tự đổi PID, hiệu chuẩn, firmware hoặc tham số an toàn để làm kết quả đạt. Không tự di chuyển hoặc nâng ngay sau khởi động chương trình. Bộ khung kèm theo không điều khiển động cơ; các phần TODO phải được triển khai và kiểm thử trước.

Hai chế độ hoàn toàn riêng:
- DIAGNOSTIC: đo RGB-D/pallet liên tục trong các bài thử ngắn, chỉ để phân tích; không dùng phép đo đó để điều khiển đường thử.
- FROZEN_TARGET: đứng yên nhận diện và đo, khóa pose pallet trong odom, lập đường một lần. Khi bắt đầu chạy: dừng YOLO và toàn bộ bộ đo/tracking pallet. RGB-D/LiDAR nếu còn chạy chỉ phục vụ chặn/dừng vật cản. Không cập nhật đích từ ảnh. Encoder + IMU/EKF vẫn phản hồi để bám đường.

Chưa chốt planner/controller. Codex được làm công cụ kiểm tra và giao diện, mô phỏng các ứng viên; không tự chọn triển khai bộ điều khiển chạy thật thay người dùng. Không thay RPP điều hướng hiện tại của Nav2 bằng cấu hình docking toàn cục.

## 1.1. Nhận diện không giới hạn thời gian — áp dụng cho cả hai kế hoạch

Yêu cầu cập nhật: KHÔNG dùng cửa sổ 3–5 giây hoặc deadline tổng để tự tắt YOLO, chấp nhận pallet hay báo hết thời gian nhận diện. `acquisition_timeout_s = null` nghĩa không giới hạn; không dùng 0 nếu code cũ hiểu là timeout ngay. Không áp max_duration của bài chạy xe lên state nhận diện. Người vận hành có thể hủy bất kỳ lúc nào; xe đứng yên trong toàn bộ quá trình này.

Điều kiện kết thúc dựa trên chất lượng dữ liệu:
1. Đúng pallet được chọn, vùng/lớp nhất quán, không nhập nhằng với hàng hoặc pallet khác. Khi có nhiều pallet, chưa phân biệt rõ thì chờ chọn mục tiêu, không khóa tùy tiện.
2. YOLO đủ số quan sát mới và nhất quán; RGB-D có đủ phép đo mới hợp lệ, đồng bộ, đủ mẫu Depth trên cả hai chân và đăng ký/hiệu chuẩn đúng. Không đếm frame lặp hoặc kết quả cũ thành bằng chứng mới.
3. X/Y/yaw và hình học pallet ổn định trong cửa sổ trượt gần nhất. Bộ đệm hữu hạn; dùng số mẫu tối thiểu kèm thời lượng quan sát tối thiểu để tránh vài frame gần nhau tạo kết luận giả. Các ngưỡng min_samples, min_stable_window_s, valid_ratio, MAD/peak-to-peak và uncertainty phải cấu hình công khai, được nghiệm thu; cửa sổ ổn định là điều kiện chất lượng, KHÔNG phải giới hạn thời gian nhận diện tối đa.
4. Ngân sách sai số phù hợp độ hở càng/lỗ. Độ lặp lại tốt không chứng minh vị trí tuyệt đối đúng: kiểm tra hồ sơ bù/extrinsic bằng thước/marker độc lập, không tự nới ngưỡng hoặc ép góc về 0 để nhận mục tiêu.
5. Khi đổi mục tiêu, xe bị xê dịch, odom reset, có nhảy pose hoặc dữ liệu quá hạn: xóa bằng chứng ổn định liên quan, tiếp tục chờ dữ liệu mới. Giữ timeout từng frame, watchdog, kiểm tra sensor health và quyền hủy; chỉ bỏ deadline tổng nhận diện. Lỗi thiết bị không thể phục hồi phải báo SENSOR_FAULT rõ, không giả vờ đang nhận diện hoặc tự cho xe chạy.

Sau khi đủ điều kiện: yêu cầu tắt YOLO, đợi inference đang chạy kết thúc và xác nhận heartbeat trạng thái tắt; thu đủ phép đo RGB-D mới sau xác nhận này để kiểm tra lại độ ổn định. Nếu mất mục tiêu/không còn đạt: trở về ACQUIRING và bật lại detector theo supervisor của phiên nhận diện, không khóa pose cũ. Không tái bật detector sau khi đích đã khóa và xe đang chạy; lúc đó phải dừng và bắt đầu phiên mới nếu cần đo lại.

| Kế hoạch | Sau khi nhận diện đủ và ổn định | Khi xe chạy |
|---|---|---|
| DIAGNOSTIC — kiểm tra camera | Tắt YOLO sau xác nhận, giữ đo/tracking RGB-D | Đo liên tục để chấm rung, trễ và sai số; phép đo không điều khiển đường thử |
| FROZEN_TARGET — chạy theo đường cố định | Tắt YOLO, chốt pose/uncertainty trong odom sau kiểm chứng RGB-D | Tắt toàn bộ đo/tracking PALLET trước khi chạy; RGB-D/LiDAR obstacle-only vẫn giữ theo cấu hình bảo vệ |

Terminal 1 Hz hiển thị state ACQUIRING/STABILIZING/STOPPING_YOLO/VERIFYING_RGBD/READY/CANCELED/SENSOR_FAULT, thời gian đã chờ, số mẫu mới, valid ratio, dao động X/Y/yaw, tuổi dữ liệu và lý do chưa đạt. Không có countdown hết hạn; READY không tự phát lệnh chạy, vẫn cần người vận hành bắt đầu lượt đã chuẩn bị.

Codex cần sửa bộ acquisition/supervisor và unit/ROS tests hiện có: chờ quá 5 giây vẫn ACQUIRING khi dữ liệu chưa đủ; đủ điều kiện sau thời gian dài mới READY; dữ liệu không ổn định không bao giờ tự READY; mất Depth/duplicate frame/đổi pallet/hủy yêu cầu phải xử lý đúng; không inference hoặc tự restart YOLO trong FROZEN_TARGET đang chạy. Tất cả đây là yêu cầu triển khai, chưa phải thay đổi firmware/phần mềm đã chạy trên Pi.

## 1.2. Chọn frame tốt và hợp nhất pose khi nhận diện ban đầu

Áp dụng chung cho cả hai kế hoạch, CHỈ trong giai đoạn xe đứng yên nhận diện/chốt pose. Đây là yêu cầu triển khai bổ sung, không có nghĩa đã thêm bộ lọc vào robot.

Luồng: RGB/Depth gốc -> kiểm tra chất lượng/timestamp -> YOLO + xác định ROI -> loại mẫu Depth xấu -> ước lượng pose từng cặp -> loại ngoại lai và tổng hợp robust nhiều cặp -> kiểm tra độ ổn định + ngân sách sai số -> chốt đích. Giữ yêu cầu không giới hạn tổng thời gian nhận diện.

Ưu tiên do người dùng chốt: ĐỘ CHÍNH XÁC CAO, KHÔNG yêu cầu FPS cao trong ACQUIRE_STATIONARY/VERIFYING_RGBD. Không dùng ngưỡng >=10 Hz ở mục chẩn đoán chuyển động để loại kết quả nhận diện đứng yên. FPS acquisition chỉ là metric; không giảm resolution/resize ảnh hoặc ép dùng model nhẹ chỉ để đạt tốc độ. Có thể chạy YOLO chậm, chọn frame chất lượng cao, dùng ảnh đủ độ phân giải và thu thêm nhiều cặp độc lập theo thời gian; không đặt thời gian hoàn tất tối đa. Benchmark model hiện tại và biến thể nặng hơn nếu cần, chọn theo SAI SỐ TÂM/GÓC và tỷ lệ chọn đúng pallet so với ground truth, không chỉ confidence/mAP hay FPS. Giữ deadline dữ liệu/safety riêng: nếu xử lý lâu thì dùng timestamp gốc và ghép pose tại thời điểm chụp; sau khi detector ngừng cần phép đo RGB-D mới trước khi chốt, không nới freshness để nhận ảnh cũ. Mẫu bổ sung có thể giảm nhiễu ngẫu nhiên, không sửa bias hiệu chuẩn hoặc tự bảo đảm độ chính xác cao.

Ba lớp bắt buộc:
1. Frame quality gate: loại frame cũ, trùng, lệch RGB/Depth, thiếu CameraInfo/registration, nhòe mạnh, cháy sáng/tối hoặc chuyển động camera vượt giới hạn đứng yên. Đánh giá sharpness/blur bằng variance of Laplacian hoặc Tenengrad trong ROI, kèm mức sáng/clipping/texture. Ngưỡng phải hiệu chỉnh cùng resolution/ánh sáng/ROI và ground truth; textureless không tự gán là nhòe. Giữ số liệu trước gate, tỷ lệ bị loại và lý do; không chỉ thống kê các frame đẹp. Nếu đang chờ nhận diện mà chưa có ROI, dùng vùng quan tâm rộng cấu hình trước, sau đó đánh giá lại ROI chân pallet.
2. Depth robust trong ROI: zero/NaN/out-of-range bị loại; chọn vùng nội thất chân có erosion hợp lý tránh trộn nền/mép. Tính median + MAD, yêu cầu số mẫu từng chân và valid ratio trên diện tích ROI gốc để erosion không làm tỷ lệ trông tốt giả. Nếu fit mặt phẳng, RANSAC + residual/inlier coverage/hình học hai chân, không fit nhầm sàn/hàng. Không hole-fill qua biên lỗ/chân; không dùng depth cũ hoặc pixel nội suy như bằng chứng đo thật. Spatial filter giữ biên chỉ là ứng viên A/B, không mặc định bật toàn frame.
3. Pose robust theo thời gian: cửa sổ hữu hạn các cặp RGB-D mới hợp lệ của cùng pallet, cùng calibration/odom epoch. Loại ngoại lai bằng median/MAD hoặc robust residual gate; tổng hợp X/Y bằng robust location và yaw bằng circular statistics/unwrap quanh tham chiếu (không trung bình trực tiếp +179/-179 độ). Kết hợp trọng số chất lượng có chặn trên; không dùng riêng YOLO confidence làm độ tin cậy khoảng cách. Chọn vị trí đại diện gần thời điểm chốt, công khai oldest/newest stamp và độ trải cửa sổ; không đóng dấu kết quả trung bình lịch sử như phép đo tức thời. Do frame liên tiếp tương quan, không báo sai số giảm 1/sqrt(N) hoặc covariance độc lập nếu chưa kiểm chứng.

Không làm mờ mạnh RGB rồi đo mép: có thể xóa chi tiết/đẩy biên, thay phân phối ảnh đầu vào YOLO. Ưu tiên chọn frame rõ và lọc kết quả hình học; median/bilateral/guided filter chỉ thêm nếu replay + thước độc lập chứng minh giảm sai số mà không gây bias. Temporal depth smoothing chỉ cân nhắc lúc camera/pallet đứng yên, giữ mask dữ liệu thật và độ tuổi; chuyển động/reset/đổi mục tiêu phải xóa lịch sử. Không giữ hình cũ thành quan sát mới để vượt quality gate.

Trong DIAGNOSTIC chuyển động: mặc định giữ pipeline raw hiện có để không che rung/nhiễu, xuất nhánh raw và filtered riêng nếu đối chứng, ghi processing latency. Trong FROZEN_TARGET đang chạy: cả bộ lọc/chốt pose pallet này và tracker đều tắt; obstacle-only pipeline riêng không được lấp vùng mù bằng lịch sử.

Terminal 1 Hz: frames_received, unique, quality_rejected_by_reason, accepted_pairs, left/right raw_depth_valid_ratio, depth_MAD, pose_raw_spread, pose_filtered_spread, outlier_fraction, stability_window_span, pose_age, ready_reason. Bộ đệm giới hạn; log nhiều frame chỉ ở cửa sổ cục bộ ngắn.

A/B trên CÙNG dữ liệu: raw hiện tại / chỉ quality gate / gate+robust depth / gate+robust pose. So độ chính xác tuyệt đối bằng thước/marker đã kiểm chứng, tỷ lệ sẵn sàng, thời gian để đủ điều kiện, các lần chốt sai, CPU và p99 age. Số đo mượt hơn nhưng lệch thước nhiều hơn là FAIL. Khi thiếu mẫu tốt thì tiếp tục chờ hoặc báo cần chỉnh ánh sáng/gá camera, không tự nới gate. Test bước nhảy thật, đổi pallet, một chân mất depth, timestamp cũ, ảnh nhòe, textureless, yaw wrap và reset bộ nhớ.

Ví dụ tham khảo: RealSense SDK có spatial/temporal depth filters (https://dev.realsenseai.com/docs/post-processing-filters/); OpenCV có đánh giá sharpness trong calibration và robust pose estimation. Đây là ví dụ kỹ thuật thực tế, KHÔNG xác nhận API RealSense chạy trực tiếp trên Astra Pro. Với Astra phải dùng đúng driver/đơn vị/registration và tự đối chứng bộ lọc thích hợp.

## 2. Trình tự Codex cần thực hiện

1. Kiểm kê repo, topic/interface/QoS thực, timestamp camera, phiên bản ROS/RMW, cổng ESP, kích thước càng/pallet, camera mount và CPU/RAM/nguồn/USB. Lưu manifest + commit + diff + config hash.
2. Triển khai monitor chỉ đọc và replay trước. Test missing data, NaN, clock jump, duplicate/out-of-order stamp, queue overflow; luôn báo N/A hoặc FAIL, không tự điền số liệu.
3. Chạy đứng yên và ma trận tải; đưa báo cáo nguyên nhân có bằng chứng.
4. Triển khai runner chuyển động có giám sát và chạy dry-run trước. Sau đó người vận hành bắt đầu từng lượt trong vùng trống. Giữ collision monitor, watchdog, nguồn lệnh duy nhất và dừng cứng hoạt động.
5. Chạy bài thử ngắn; chẩn đoán rung ảnh, dữ liệu, CPU và truyền động.
6. Đưa bảng so sánh planner/controller, preview đường bằng CSV + bảng terminal; chờ người dùng chốt trước tích hợp chạy thật.
7. Sau khi chốt: shadow follower -> chạy đường trên sàn không pallet -> mock pallet có độ hở lớn -> đưa càng thật -> thử nâng riêng -> ghép nhiệm vụ.

## 3. Pi không gửi dữ liệu ra mạng, chỉ giữ SSH

LOCALHOST của DDS chỉ là một lớp, không đủ cho yêu cầu cấm egress toàn máy.
- Chuẩn bị dependency TRƯỚC phiên cô lập. Trong phiên thử không download, update, upload, gửi báo cáo cloud. Codex/extension cloud có thể cần mạng: chuẩn bị mã trước; không chạy dịch vụ đó trong phép đo cô lập. Điều khiển phiên thử bằng SSH thường.
- Dừng web preview, RViz remote, ROS bridge, image server, ros2 topic echo/hz từ máy khác, đồng bộ/cloud telemetry tự khởi động. Chỉ dừng dịch vụ xác định; lưu cách khôi phục.
- Restart tất cả tiến trình ROS liên quan với cùng ROS_DOMAIN_ID và ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; xóa cấu hình static peers/discovery server ngoài máy sau khi kiểm kê. Kiểm tra tính tương thích RMW đang dùng. Đọc schema thực trước đặt env.
- Nếu cần bảo đảm không egress: tạo firewall tạm trên Pi, cả IPv4 và IPv6: cho loopback; chỉ cho response thuộc đúng kết nối SSH hiện tại với laptop; chặn kết nối mới và các lưu lượng IP khác trên interface mạng. Không cho toàn bộ ESTABLISHED tồn tại trước phiên vì có thể còn cloud session. DHCP/DNS/NTP không được miễn trừ ngầm; dùng địa chỉ hiện có trong cửa sổ thử.
- Trước áp dụng: lưu rules, xác định SSH tuple, đặt rollback tự động cục bộ, giữ terminal dự phòng; kiểm tra SSH còn sống rồi mới hoàn tất. Không flush firewall hệ thống hoặc vô hiệu hóa bảo vệ sẵn có. Nếu không đủ quyền: ghi NOT_ISOLATED, không báo đã đáp ứng.
- Xác minh bằng counters firewall + capture eth/wlan cả hai chiều ít nhất 60 s khi camera/ROS đang chạy. Phân biệt ARP/link-layer với IP application egress; mọi ngoại lệ phải ghi rõ. Không coi không có subscriber remote là bằng chứng không có gói mạng.
- Lưu dữ liệu trong thư mục run trên Pi. Xem summary tối đa 1 Hz qua SSH, không in từng frame. Dừng thử rồi mới khôi phục firewall/mạng; file vẫn nằm trên Pi.

## 4. Bộ chương trình cần triển khai từ scaffolds

| File | Nhiệm vụ | Nhịp/tải |
|---|---|---|
| host_health.py | CPU từng lõi/process/thread, nhiệt, xung, throttle/undervoltage, RAM/swap/PSI, disk I/O, USB reset, network counters | 1 Hz, read-only |
| rgbd_probe.py | RGB/depth/camera_info rate, stamp, age, sync, duplicate, queue, processing time, valid depth ROI | xử lý nhẹ; không JPEG |
| camera_motion_probe.py | chuyển động nền, blur, phần dư sau bù chuyển động xe, độ trôi pose pallet odom | ảnh xám nhỏ, 10–15 Hz hoặc replay |
| drive_probe.py | cmd các tầng, encoder FL/FR/RL/RR, PWM, IMU/EKF, tracking error, deadzone | đọc tốc độ gốc, summary 1 Hz |
| short_motion_runner.py | quãng ngắn, đổi hướng nhẹ; mặc định dry-run, không chạy trong bộ khung | 20–50 Hz sau triển khai |
| frozen_path_proposal.py | hợp đồng planner/follower và pseudo-code; KHÔNG phát lệnh | preview offline |
| report_run.py | summary.json + metrics.csv + report.md; so sánh A/B | offline |

Tận dụng code có sẵn như verify_pallet_handoff.py, pallet_rgbd_health.py, test_target/pallet interfaces; không chạy hai bộ subscribe ảnh nặng nếu có thể thêm hook trong tiến trình đang xử lý. Tách monitor ảnh nặng ra replay để đo observer overhead.

## 5. Dữ liệu cần ghi và cách tính

### 5.1 Timestamp, đồng bộ và mất dữ liệu
Mỗi frame: capture_stamp_raw, capture_stamp_ros, clock_domain, receive_monotonic_ns, callback_start/end, pair_enqueue/dequeue, processing_start/end, output_stamp, seq/frame_id nếu nguồn có. Nếu driver không expose mốc chụp thật, ghi là driver_header_stamp; không gọi đó là sensor capture latency. Không tự sửa timestamp để pass.
Tuổi frame tính từ ROS clock và header chỉ khi cùng clock domain; thời lượng xử lý/inter-arrival dùng monotonic. Ghi /use_sim_time, clock offset/reset. Tách driver-to-callback, queue wait, compute, result age. Không suy ra network packet loss từ việc thiếu frame hoặc stamp gap.
Báo p50/p95/p99/max + count + duration; phân biệt frame nguồn, frame nhận, frame ghép, frame valid, frame unique. QoS sensor best-effort, depth queue ngắn; số frame drop/overwrite phải đếm. CameraInfo đôi khi có stamp cố định: kiểm tra hành vi driver, không áp ngưỡng sync ảnh lên metadata tĩnh nếu sai semantics.

### 5.2 Depth + RGB
Ghép theo timestamp với ngưỡng 50 ms làm mốc ban đầu từ dự án, không phải tiêu chuẩn sensor chung. RGB/depth resolution, registration, K/D và đơn vị depth phải xác nhận. Trên ROI hai chân: valid ratio, median, MAD, p95 residual, số mẫu từng chân, clipping/zero/NaN/saturation. Khi xe chạy, khoảng cách phải đổi; không đánh giá bằng độ lệch chuẩn depth thô toàn lượt. Dùng ROI được warp/chiếu đúng và so phép đo với chuyển động tương đối dự đoán; nếu ROI không còn theo vật thì gắn ROI_INVALID.

### 5.3 Rung camera và sai số pallet theo frame
- Khi đứng yên: ảnh xám, nền tĩnh có texture (tường/marker), loại mask pallet, người và càng. LK optical flow + kiểm tra forward/backward + RANSAC affine/homography. Ghi global shift/rotation, inlier ratio, forward-backward error, Laplacian blur và exposure nếu có.
- Khi xe tiến/quay: raw optical flow là chuyển động mong muốn, KHÔNG gọi ngay là rung. Bù bằng camera extrinsic + chuyển động IMU/EKF tại timestamp ảnh; với translation cần depth hoặc mặt phẳng nền vì parallax, không chỉ một homography cho mọi vật. Có thể dùng bảng marker/AprilTag cố định để đo pose camera độc lập.
- Rung: residual tần số cao và nhòe sau bù chuyển động, loại khung thiếu texture/ít inlier. Báo px và deg có hiệu chuẩn; không đổi px ra mm bằng một hệ số cho mọi độ sâu.
- IMU trên thân không đo đầy đủ rung tương đối của gá camera. Nếu nghi gá lỏng, dùng marker/IMU gắn camera hoặc thử gá chắc hơn có đối chứng; không kết luận chỉ từ IMU thân.
- Pose pallet: p_odom = T_odom_base(t_capture) * T_base_camera * p_camera. Với scalar đã bù trong repo phải theo đúng hệ quy chiếu của scalar, không transform optical lần hai. Pallet đứng yên nên p_odom và yaw_odom phải gần cố định; tính sai số so baseline robust, không chỉ so frame liền trước.
- Đối chiếu residual rung/blur với residual pose, sync age, CPU và depth invalid bằng lag correlation. Tương quan không chứng minh nhân quả: làm A/B gá camera/ánh sáng/tải CPU để xác nhận.

### 5.4 Truyền động
Lấy cmd docking -> monitor -> stamper -> controller output -> wheel target -> wheel measured -> PWM -> IMU/EKF. Không so cmd đầu chuỗi với vận tốc bánh mà bỏ qua acceleration limiter/collision stop.
Với radius r, effective track b: v_wheel=r*(mean_right+mean_left)/2; w_wheel=r*(mean_right-mean_left)/b. Ghi odom encoder không độc lập với encoder; đối chiếu w với gyro Z và quãng đường với thước/marker độc lập. Đo b và r hiệu dụng đã calibrate; không lấy hình học URDF làm chân lý.
Đo deadzone có tải hai chiều, hysteresis, steady error, settling, PWM saturation, chênh trước/sau và trái/phải. Không sửa PID trước khi có kết quả. Đặc biệt kiểm tra lệnh docking cũ 0.08 m/s, 0.20 rad/s so với ngưỡng thắng ma sát được ghi trước đây >0.10 m/s và 0.25–0.35 rad/s; số cũ chỉ là giả thuyết cần đo lại.

## 6. Ma trận thử và trình tự

Mỗi profile đứng yên tối thiểu 120 s, tối thiểu 3 lượt; đảo thứ tự A/B để giảm ảnh hưởng nóng máy. Warmup giống nhau. Bài ngắn lặp tối thiểu 5 lượt mỗi chiều; số này chỉ là mốc chẩn đoán, không chứng nhận độ tin cậy.

| Profile | Tải | Mục đích |
|---|---|---|
| A0 | camera driver + probe nhẹ, đứng yên | baseline USB/camera |
| A1 | A0 + acquisition/đo pallet, chưa web | thêm chi phí perception |
| A2 | đứng yên, YOLO tắt, đo RGB-D tiếp | tách inference khỏi phép đo |
| A3 | A2 + LiDAR + hardware/EKF + collision, không phát lệnh | full stack local |
| A4 | A3 + recorder ảnh ngắn, rồi tắt recorder | observer + disk overhead |
| B0 | full stack + chạy ngắn, DIAGNOSTIC | chất lượng đo khi tiến |
| B1 | B0 + heading lệch nhẹ trái/phải | ảnh khi tiến và quay |
| C0 | đo đầu rồi tắt pallet perception; đường thử không tiếp xúc pallet | FROZEN_TARGET |
| C1 | C0 + RGB-D obstacle-only so với chỉ cấu hình bảo vệ đã nghiệm thu | chi phí obstacle pipeline |

A0/A1 không thay đổi dây USB hay cấp nguồn giữa lượt; thử cáp/hub/gá khác là một yếu tố riêng. Không tắt lớp bảo vệ cần thiết trong bài có chuyển động để benchmark nhanh hơn.

Bài chuyển động trong vùng TRỐNG, cách pallet đủ xa:
1. 0.30 m thẳng, dừng; 0.50 m; sau khi đạt mới tới 1.00 m.
2. Tiến theo đường cong: heading 0 -> +5° -> 0; rồi 0 -> -5° -> 0.
3. Sau khi đạt mới +10/-10 rồi +15/-15°. Dùng chuyển hướng mượt, không nhảy step omega; mục tiêu là heading có phản hồi IMU/EKF, không phải gửi omega theo thời gian rồi đoán góc.
4. Trở về heading 0 KHÔNG đồng nghĩa trở về cùng đường thẳng: ghi lateral displacement và so với đường thử dự kiến. Không tự chạy lùi để về vị trí đầu.
5. Nếu 30 cm không đủ đạt góc trong giới hạn bán kính/gia tốc thì bỏ case đó, tăng quãng trong tối đa 1 m nếu đủ chỗ. Không ép góc hoặc tăng tốc bất chấp clearance.
6. Preview full swept footprint gồm càng trước mỗi lượt; điểm cuối vẫn trước vùng camera mù/va chạm pallet. Người vận hành đặt lại xe giữa lượt.
7. Stop khi stale odom/safety scan, heartbeat/command lease mất, vượt corridor/heading/length/time hoặc operator stop. Khôi phục dữ liệu không tự chạy lại. Mất SSH heartbeat trong phiên thử dừng; watchdog thấp hơn vẫn độc lập.

## 7. Tiêu chí và chẩn đoán nguyên nhân

Ngưỡng sau là ĐỀ XUẤT khởi đầu; không tự nới để pass, không gắn nhãn chuẩn của nhà sản xuất:
- Đo RGB-D valid >=98%, >=10 Hz khi DIAGNOSTIC; result age p99 <=150 ms, max <=250 ms; sync <=50 ms theo yêu cầu repo. Ghi cả raw stream gaps để biết mất trước hay sau xử lý.
- Odom age <=200 ms; timing controller chọn 50 Hz rồi ghi deadline misses >20 ms, p99/max cycle. Không tuyên bố real-time chỉ vì timer cấu hình 50 Hz.
- Đứng yên: pose peak-to-peak tham khảo X<=30 mm,Y<=20 mm,yaw<=2° như test trước; thêm RMS/MAD/outlier count. Chạy động đánh giá residual trong odom và ground truth độc lập.
- Rung/blur chưa có ngưỡng px phổ quát: xây baseline A0 và so A/B, không dùng Laplacian giữa hai cảnh khác nhau để kết luận.
- Không thermal/undervoltage active; phân biệt bit hiện tại với historical sticky bits vcgencmd. Nếu lệnh không có trên OS thì ghi N/A, đọc sysfs/kernellog thay thế; không ghi 0.
- CPU tổng thấp không loại trừ một lõi 100%, IRQ/USB nghẽn, executor block hay I/O wait. Ghi per-core/thread + PSI/swap/frequency.

Bảng kết luận:
- Header rate giảm + kernel USB reset/error -> ưu tiên nguồn/cáp/hub/driver, chưa kết luận do CPU.
- Header đều, callback/result age tăng, queue dài -> executor/GIL/compute/CPU scheduling.
- Age tăng chỉ khi record -> disk/serialization/observer overhead.
- Depth ổn, ảnh blur/residual tăng cùng pose error -> rung/exposure cần A/B xác nhận.
- Sensor tốt, cmd không bám -> drivetrain/deadzone/PID/limiter.
- Wheel bám nhưng pallet odom trôi -> slip, calibration/extrinsic/timestamp/sign.
- Mọi dữ liệu đúng mà follower error tăng -> planner/controller/geometry.

Mỗi run xuất manifest.json, metrics.csv (tất cả mốc thời gian), events.jsonl, summary.json, report.md; ảnh/rosbag raw chỉ lưu cửa sổ ngắn có giới hạn dung lượng trên Pi. Ghi monitor dropped samples; không chỉ lưu summary đẹp.

## 8. Đề xuất tạo đường: đường cong quintic có ràng buộc + đoạn thẳng vào pallet

ĐÂY LÀ ỨNG VIÊN ƯU TIÊN, CHƯA CHỐT.
Đo pallet khi xe đứng yên; cần robust nhiều frame + độ bất định + đo lại tọa độ lỗ, chiều sâu đưa càng, chiều rộng từng càng/lỗ, cao độ và tải. Khóa T_odom_pallet cùng timestamp/epoch/calibration hash; dùng odom local liên tục cho đoạn cuối, không để AMCL map correction kéo đích. AMCL dùng tới vùng pre-dock; odom drift vẫn tồn tại và phải được chấm bằng ground truth.

Chọn S=current base pose, A=điểm đã thẳng hàng trước pallet, F=base pose tương ứng độ cắm càng cuối. Dựng curve S->A, rồi straight A->F cùng hướng trục lỗ. Không giả định khoảng cách 1.20 m tới gốc càng là khoảng cách tới mũi càng. Tính F từ hình học càng thật và insertion depth; kiểm tra mũi càng chạm vùng nào suốt đường.

Ứng viên đầu: parametric quintic Hermite x(u),y(u), u∈[0,1]. Ràng buộc vị trí, tiếp tuyến theo yaw, và đạo hàm bậc hai tại đầu/cuối bằng 0 để có curvature bằng 0 tại điểm nối thẳng. Tìm kiếm vài giá trị tangent scale/điểm A; không dựng một đa thức tùy ý rồi gọi là khả thi.
Chi phí: length + tích phân kappa² + tích phân (dkappa/ds)² + khoảng cách tới corridor; reject trước cost nếu vi phạm. Tính lại theo arc length; kiểm tra derivative không về 0, không vòng/cusp, forward-only, heading không quay đột ngột, đoạn thẳng đủ dài.
Hard constraints: kappa_max=1/Rmin đã đo; omega=v*kappa; lateral acceleration=v²*abs(kappa); wheel speed/accel; jerk; collision swept footprint bao càng/tải; unknown cells/chưa quan sát chặn. Sampling cần đủ mịn theo cell size/hình học, adaptive theo curvature; không chỉ check tâm hoặc các waypoint thưa.

Quỹ đạo theo thời gian thêm speed profile tăng/giảm tốc mượt, giới hạn v/omega/accel/jerk đo thực tế. Không dùng luật min_speed đẩy xe qua giới hạn curvature/collision. Nếu gentle curve không đủ chỗ sửa Y/yaw thì trả REPOSITION_REQUIRED; không bảo đảm mọi pose đều đi được bằng cong nhẹ. Yêu cầu xe tới pre-dock phù hợp thay vì planner uốn đường gắt.

| Ứng viên | Lợi ích | Hạn chế | Đề xuất |
|---|---|---|---|
| Quintic Hermite có tìm tangent + straight | ít biến, hợp đoạn ngắn, kiểm soát đầu/cuối, nhẹ Pi | cần tự kiểm curvature/collision và search vùng khả thi | ưu tiên thử trước |
| Clothoid nối straight | curvature tuyến tính theo arc length, chuyển hướng tốt | solver và tích hợp phức tạp hơn | ứng viên đối chứng |
| Smac Lattice/Hybrid + smoother ràng buộc | tận dụng Nav2, xử lý không gian nhiều vật cản | có thể ra đường vòng/gear change, cần lọc forward-only/gentle | giữ cho tiếp cận/pre-dock |

Không dùng cubic Bézier đơn giản mà bỏ kiểm curvature ở nối thẳng. Một đường đẹp trên đồ thị không chứng minh càng lọt lỗ.

## 9. Đề xuất bám: RPP profile docking riêng trước, MPC chỉ khi có bằng chứng cần

RPP là ứng viên ưu tiên vì dự án đang dùng RPP. FollowPath nhận path cố định trong odom; cấu hình docking riêng, không chạy lại planner theo pallet. Rà lại SafeRotationRPP tùy biến vì có logic quay tại chỗ/recovery; tắt rotate-to-heading trong profile docking và reject đường không tiếp tuyến với pose đầu, không bỏ các collision guard. Không copy tên tham số từ rolling sang Jazzy không kiểm tra.
- Follow curve rồi straight; encoder/IMU/EKF feedback liên tục, chiếu vào path monotonic không nhảy tiến độ khi gần giao nhau. Đo cross-track/yaw error, progress, stop distance.
- Lookahead phù hợp chiều dài đoạn cuối; preview không vượt path làm mất điều khiển ở endpoint. Tiêu chí kết thúc dùng cả heading/lateral/longitudinal, không chỉ XY Nav2 mặc định.
- Hạn chế omega khi càng bắt đầu vào lỗ; nếu sai số vượt corridor tại insertion gate thì dừng, không xoay để sửa khi càng đã cắm.
- RPP là follower hình học, không bảo đảm bám đúng lịch t của trajectory. Speed envelope/acceleration giới hạn riêng; nếu cần bám thời gian/ràng buộc actuator phức tạp thì đánh giá MPC sau.
- MPC giải ràng buộc tốt hơn nhưng tăng chi phí xây mô hình/tuning và không sửa được deadzone, slip, target sai hoặc odom sai.

Các ứng viên follower để người dùng chốt: (1) RPP docking riêng, (2) nonlinear pose/path tracking có feed-forward v*kappa, (3) MPC sau benchmark. Không đổi sang MPC chỉ vì camera trễ.

## 10. Sai số cuối, obstacle-only và nâng hàng

Đường đúng CHƯA đủ đưa càng đúng. Với D từ pose chốt tới vùng cần lọt lỗ, e_y cuối xấp xỉ e_y_chốt + D*tan(e_yaw) + drift + tracking error. D=1.2 m, yaw 1° đã thêm ~21 mm; 2° ~42 mm, chưa tính sai số khác. Đây là ví dụ hình học, không phải D thật từ tâm xe tới mũi càng. Mức ±2 cm/2° cũ có thể không đủ cho pallet thật. Tính clearance từng càng/lỗ, dung sai cao độ và phân bổ uncertainty + margin; không dùng độ lặp lại đứng yên thay độ chính xác.

Sau khóa: follower không subscribe pallet target; supervisor khóa epoch, dừng detector/tracker và kiểm tra không còn inference/measurement trong production. Obstacle pipeline chỉ tạo stop/slow, không sửa path/đích, không tự né/replan, không tự resume sau stop.
Camera/LiDAR không được tắt cùng pallet tracker nếu còn nhiệm vụ bảo vệ; nếu một process gộp hai việc phải tách lifecycle. Depth obstacle detection vẫn cần compute, phải benchmark.
Pallet là vật thể đích: không xóa toàn bộ bbox/pallet khỏi obstacle map. Chỉ áp mô hình hành lang insertion được kiểm chứng theo state, geometry và confidence; thân pallet/đố gỗ, ngoài corridor hoặc vật mới vẫn chặn. Không tắt monitor toàn cục để càng vào được. Camera mù <~1.1 m theo người dùng và LiDAR cao có thể không quan sát vùng càng: phải xác nhận coverage; nếu không đủ, dừng hoặc bổ sung cảm biến gần/công tắc/bumper đo lực phù hợp, không giả định camera bảo vệ được vùng mù.

Nâng chỉ sau: robot dừng xác nhận, insertion depth đạt với feedback độc lập hoặc phương án đo đã nghiệm thu, càng đúng độ cao, homed/endstop hợp lệ, tải/độ ổn định cho phép. Base lock khi nâng; nâng có giới hạn hành trình/thời gian/lỗi. Không tự nâng chỉ vì odom báo tới goal. Nếu chưa có xác nhận đưa càng vào đủ thì triển khai tới state WAIT_INSERTION_VERIFICATION, chưa bật AUTO_LIFT.

State dự kiến: IDLE -> ACQUIRE_STATIONARY -> LOCK_TARGET -> PLAN_VALIDATE -> READY -> FOLLOW_CURVE -> FOLLOW_STRAIGHT -> INSERTION_GATE -> INSERT -> VERIFY_INSERTION -> LIFT -> DONE. STOP/FAULT tại mọi state; không tự tiếp tục sau lỗi.

## 11. Đầu ra cần Codex bàn giao và tiêu chí nghiệm thu

A. Monitor: chạy terminal SSH, không egress, báo rõ nguyên nhân invalid, mất dữ liệu và overhead.
B. Báo cáo A/B đứng yên + 30cm/50cm/1m, heading ±5/±10/±15° có quãng phù hợp; N/A/skip có lý do.
C. Planner preview: ít nhất grid lateral ±2/5/10 cm, yaw ±2/5/10°, nhiều khoảng pre-dock; trả feasible/infeasible, kappa max, length, swept clearance, estimated time và expected uncertainty. Không phải mọi case bắt buộc feasible.
D. Shadow follower: không publish motor, so proposed cmd với constraints và dữ liệu replay; fake sensor tests cho stale/reset/obstacle/competing source.
E. Chạy sàn: ground truth đo độc lập, tối thiểu 5 lượt mỗi điều kiện chẩn đoán, ghi error đầu/cuối, không hứa độ tin cậy sản phẩm từ vài lượt.
F. Chỉ sau người dùng chốt planner/follower và cơ chế insertion verification mới tích hợp đường tới pallet + auto lift.

## 12. Nguồn và điểm cần xác minh

- Repo docs: camera/detect_astra/ALIGNMENT.md, TRACKING.md, PERCEPTION_HANDOFF.md; scripts/pallet_alignment_core.py; config/pallet_alignment.yaml; ros2_ws/src/safe_rotation_rpp.cpp.
- Nav2 Jazzy RPP: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/
- ROS discovery reference: https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Configuring-ROS2-Environment.rst (đối chiếu bản Jazzy/RMW trước dùng).
- Pi vcgencmd: https://www.raspberrypi.com/documentation/computers/os.html
Các lựa chọn quintic/RPP và ngưỡng thử trong kế hoạch là đề xuất kỹ thuật, chưa phải kết quả benchmark trên Pi của người dùng.
