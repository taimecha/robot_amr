# Kết quả mô phỏng docking trên laptop

- Commit: `9056b5f`; chạy trên `tai-Nitro-AN515-57` (x86_64); Pi 5 runtime: **chưa kiểm tra**.
- Nhánh acquisition danh định **giả định hiệu chuẩn độc lập đạt**: giữ `ACQUIRING` sau 5 s, READY ở 10.2 s sau khi tắt YOLO và nhận RGB-D mới; cần thao tác bắt đầu riêng.
- Hai mode được kiểm tra riêng: DIAGNOSTIC giữ đo pallet khi chuyển động nhưng không dùng phép đo đó để lái; FROZEN_TARGET tắt đo/tracking pallet trước khi bám đường cố định.
- Kịch bản nối các bước: dữ liệu có bias 10.2 mm bị chặn tại `ACQUIRING`; dữ liệu giả đã sửa bias còn 0.2 mm đi qua planner và shadow follower, dừng tại `WAIT_INSERTION_VERIFICATION`. Không phát lệnh nâng.
- Replay frame: 36 mẫu được nhận. Lý do loại: {'accepted': 36, 'blur': 1, 'sync': 1, 'depth_samples': 1, 'camera_motion': 1, 'duplicate_or_out_of_order': 1}. Sai số X tuyệt đối sau lọc 10.2 mm so với truth giả, vượt budget 5 mm. **Nhánh dữ liệu này bị chặn ở ACQUIRING**; dữ liệu ổn định không sửa được bias 10 mm.
- Planner quintic + straight: 70/73 trường hợp khả thi **trong mô hình 2D giả**; 3 trường hợp REPOSITION_REQUIRED. Clearance danh định lỗ/càng 17.5 mm mỗi bên, budget giả 11.6 mm.
- Ma trận chuyển động ngắn 30/50/100 cm với hướng 0, ±5/10/15°: 21 trường hợp offline, 2 bị SKIP vì độ cong vượt giới hạn giả định; có ghi độ lệch ngang cuối.
- Shadow follower: 15.3 s, dừng ở `WAIT_INSERTION_VERIFICATION`. Không phát lệnh robot, không xác nhận insertion và không nâng. Sáu ca lỗi đều latch STOP.
- Gazebo pre-dock: 253 mẫu trong 12.6 s, dừng ở `WAIT_INSERTION_VERIFICATION`, sai số XY 17.6 mm; max |v|=0.080 m/s, |ω|=0.179 rad/s, scan gần nhất 1.01 m. Sau lượt, odom velocity về gần 0. Lượt Gazebo dùng đích giả đã đặt sẵn và chỉ đi đến pre-dock; chưa nối nhận diện camera với planner.

Các số trên không phải phép đo camera, động cơ hoặc Pi thật. Đường đi chưa được xác nhận theo chiều cao càng, vật cản chưa quan sát, slip, độ võng tải, độ chính xác hiệu chuẩn hoặc ground truth thật. Cấu hình collision monitor hiện chỉ nhận `/scan_filtered`; chưa bảo vệ được vùng mù pallet. Vì vậy không dùng kết quả này để bật chế độ chạy thật.
