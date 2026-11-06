# Nội dung gửi Codex

Hãy đọc `/home/tai/Downloads/pi5_docking_diagnostics/KE_HOACH_CODEX.md` bản cập nhật 06/11/2026 và README, rồi đối chiếu repo https://github.com/taimecha/robot_amr với source/runtime thực tế trên Pi. Xác định host hiện tại và SSH target trước khi làm; Downloads có thể ở laptop. Nếu thư mục cũ có thay đổi của tôi, so diff và ghép, không ghi đè.

Tôi đổi thứ tự triển khai: **trước hết cho robot thật đứng yên, dùng Astra Pro đo X/Y/yaw pallet thật, có lọc nhiều frame như ý tưởng mô phỏng, đối chiếu thước/mốc độc lập. Chỉ sau khi phần này đạt mới tiếp tục kế hoạch kiểm tra chuyển động và docking.**

Ngay bây giờ làm G0–G2: triển khai observer chỉ đọc và báo cáo terminal SSH, lưu dữ liệu tại Pi. Tận dụng pipeline RGB-D đã hiệu chỉnh và bộ lọc góc 5 frame mới; không copy cao độ/kích thước pallet hoặc ground truth Gazebo vào chương trình thật. Ưu tiên chính xác, không yêu cầu FPS cao, không giới hạn tổng thời gian nhận diện. Giữ freshness/timestamp, reset và quyền hủy.

Đo được candidate pose và error kể cả khi acquisition chưa READY do `accuracy_validated=False`. Không hardcode True để vượt gate: dùng báo cáo kiểm chứng độc lập và record khớp model/calibration/miền làm việc. Thiếu GT/budget thì ghi chưa nghiệm thu, không tự PASS.

Không khởi động chế độ truyền động, ARM/ENABLE/HOME, không publish vận tốc/nâng, không chạy script `--execute` hoặc `--gazebo-execute` để kiểm tra camera đứng yên. Chưa có odom vẫn đo base-relative được, nhưng không giả TF odom để khóa đích.

Chuẩn bị code online trước, tách phiên đo Pi cô lập mạng chỉ giữ SSH. Không stream ảnh hoặc dữ liệu ra laptop/mạng ngoài. Đừng cắt kết nối Codex đang cần để triển khai; kiểm tra rollback và SSH trước cấu hình firewall phiên đo.

Bàn giao lệnh chạy chính xác cho môi trường đã kiểm tra, bảng raw/filtered X/Y/yaw vs số đo chuẩn, thống kê bias/nhiễu/age/sync/CPU/USB và kết luận từng gate. Cho tôi biết cần đo kích thước/mốc nào để điền templates; tiếp tục những việc độc lập trong khi chờ số đo đó. Các bài 30 cm–1 m, đổi hướng ±5/±10/±15°, planner và follower thuộc giai đoạn sau. Giữ các lựa chọn quỹ đạo ở mức đề xuất để tôi chốt; robot chỉ chuyển động khi tôi chủ động bắt đầu lượt thử.
