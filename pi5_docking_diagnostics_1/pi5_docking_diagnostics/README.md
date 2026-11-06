# Kế hoạch Pi 5 cập nhật theo robot_amr — 06/11/2026

Đọc `KE_HOACH_CODEX.md` hiện hành. Thứ tự mới: **G0 kiểm kê -> G1 đo pallet thật khi xe đứng yên -> G2 xác minh/khóa đích tĩnh -> G3 chạy ngắn -> G4 preview -> G5 pre-dock -> G6 insertion/nâng**.

Hiện tại tập trung G0–G2, không phát lệnh động cơ hoặc càng. Nhận diện không giới hạn thời gian, ưu tiên sai số X/Y/yaw thật, không yêu cầu FPS cao. Repo tham chiếu: taimecha/robot_amr @9398733. Mã chạy thực tế trên Pi phải được Codex kiểm tra lại.

## Nội dung

- `KE_HOACH_CODEX.md`: kế hoạch thay thế bản cũ, có đối chiếu tiến độ hiện tại và gate nghiệm thu.
- `NHAN_CODEX_TREN_PI.md`: lời nhắn và bước bắt đầu cho Codex.
- `scaffolds/stationary_pallet_probe.py`, `qualify_static_target.py`: code giả mới cho G1/G2.
- `scaffolds/host_health.py`, `metrics.py`: tiện ích chạy được; chưa thay monitor đầy đủ trong kế hoạch.
- Các file Python còn lại là hợp đồng TODO, chỉ in mô tả khi chạy, không subscribe ROS hoặc điều khiển xe.
- `templates/stationary_cases.csv`: header nhập số đo thật; chưa có hàng dữ liệu giả.
- `templates/acceptance_policy.json`: budget để null khi chưa đo, không thể tự PASS.
- `reference/KE_HOACH_TRUOC_2026_11_05.md`: lịch sử, không phải chỉ dẫn hiện hành.

## Cập nhật thư mục trên máy của bạn

Gói này được sửa từ bản kế hoạch đã tạo trong cuộc trò chuyện; không có quyền truy cập thư mục `/home/tai/Downloads/pi5_docking_diagnostics` trên máy bạn. Nếu bạn đã sửa code ở đó, giữ lại bản đó và nhờ Codex so diff trước khi ghép. Không giải nén ghi đè mù quáng.

Giải nén bản mới vào thư mục tạm khác, so sánh và ghép kế hoạch/scaffold vào thư mục cũ, giữ thay đổi cục bộ. Xác nhận file KE_HOACH_CODEX.md có ngày 06/11/2026 và gate G1 đo thật đứng yên. Sau đó đưa `NHAN_CODEX_TREN_PI.md` cho Codex. Không cần tạo lại simulation đã có trong repo.

## Chạy phần read-only đã có

```bash
python3 scaffolds/host_health.py --seconds 120 --output host_health_static_run01.jsonl
python3 scaffolds/stationary_pallet_probe.py
python3 scaffolds/qualify_static_target.py
```

Hai lệnh cuối chỉ IN HỢP ĐỒNG, chưa đo camera; phải để Codex triển khai ROS trước. host_health chỉ ghi tài nguyên Linux cơ bản, không mở actuator/ROS/network; file output phải là tên mới để tránh ghi đè run.

Chưa kết nối Pi, camera hoặc robot trong lần sửa gói này. Không dùng kết quả unit test/mô phỏng làm bằng chứng robot thật đạt.
