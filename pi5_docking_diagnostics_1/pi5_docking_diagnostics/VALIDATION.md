# Kiểm tra gói kế hoạch — 06/11/2026

- Đã kiểm tra cú pháp 11 file Python.
- Xác nhận các scaffold ngoài host_health/metrics chỉ là docstring, chưa có ROS/motor implementation.
- Kiểm tra acceptance policy: acquisition không deadline/FPS tối thiểu; budget chưa đo là null; motor/lift bị khóa.
- CSV ground truth chỉ có header, không có số liệu giả.
- host_health smoke test 1 giây trên môi trường tạo gói: đạt; đây không phải Pi 5.
- Đã kiểm tra thứ tự G1 trước G3 và ref robot_amr @9398733.
- Không kết nối Pi, camera, serial, không phát lệnh robot.
- Chưa chạy thuật toán nhận diện/freshness/accuracy gate trên ROS thật.
- Thư mục /home/tai/Downloads/pi5_docking_diagnostics của người dùng không truy cập được từ môi trường này. Gói mới cần được ghép vào đó, giữ các thay đổi cục bộ nếu có.
