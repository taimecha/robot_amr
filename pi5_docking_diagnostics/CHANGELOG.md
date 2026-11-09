# Thay đổi bản08/11/2026

- Thay kế hoạch06/11 bằng phạm vi trajectory/mission; bỏ diagnostics và các scaffold monitor liên quan khỏi gói mới.
- Đọc lại robot_amr @98d4d4d; phân biệt planner/follower đang có với đề xuất chờ duyệt.
- Đồng bộ camera optical/housing, fork_root, base, odom; không đoán dấu20mm.
- Cập nhật pallet355mm/4chân, càng385mm/155mm/75mm, rear300mm, stop20mm, lift80mm, requestedhome-settle1mm.
- Yawline<=0.5°, lateral<=20mm preferred10mm, maxbasev0.15m/s; v_min/tolerancegap còn chờ xác nhận.
- Thứ tự mô phỏng -> camera thật/preview -> người dùng bắt đầu -> chạy/nâng -> planned/odom.
- Không sửa source GitHub, không chạy solver/Gazebo/Pi/robot, không flash firmware.

## Đính chính r2 theo người dùng

4chân thay5; yaw được tự hiệu chỉnh; cameraoffset nhìn từtrước đã chốt dấu-20mm theoYxe; HOME1mm thay2mm, cơ khí đã xác nhận; P2Nav2 thêm như lựa chọn chưa duyệt. Chưa thay thuật toán/source/firmware.

## r3 — người dùng chốt phương án

SmacStateLattice primitivesvi sai/skid-steer + ConstrainedSmootherapproach-only + exactterminalstraight; giữcontroller hiệncó. Loại P1/P2/F1 chờchọn khỏi kếhoạch hiệnhành. Pure rotations táchphase, khôngsmoothing xuyênqua. Không tựfallback/relax. Chỉ sửa gói kếhoạch; chưa triểnkhai/robot execution.
