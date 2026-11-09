> Cập nhật triển khai08/11/2026: xem [TRIEN_KHAI_R3.md](TRIEN_KHAI_R3.md). Nội dung kiểm tra gói bên dưới là mốc trước triển khai, không phải kết quả chạy mới.

# Kế hoạch docking đã chốt —08/11/2026 r3

**Smac State Lattice (vi sai/skid-steer) -> Constrained Smoother đoạn tiếp cận -> đoạn thẳng cuối theo trục càng. Giữ bộbám hiện tại.**

Đọc KE_HOACH_CODEX.md, QUYET_DINH_DA_CHOT.md, templates/planning_requirements.json và NHAN_CODEX_TREN_PI.md. CAN_DUYET.md chỉ là chỉdẫn tươngthích, không còn gatechờchọn thuậttoán.

Giữ:4chân; y_F-y_H=-20mm theoYxe; HOMEthêm1mm đãxácnhận; lift80mm; rootgap20mm; v<=0.15m/s; lineyaw<=0.5°; lateral<=20mm ưu tiên10mm. Không diagnostics tổngquát hoặc PALLETtracking khi chạy.

Smoother chỉ làm đoạn tịnh tiến tiếpcận, giữ phasequay và không bẻ linecuối. Môphỏng trước, xácnhận camera thật và hìnhplanned, sauđó người dùng cho chạy thật. Vẽ planned/odom cảbase vàfork_root.

Scaffolds chỉ là codegiả/hợpđồng, in mô tả khi chạy; chưa có ROS/solver/controller/firmware thực tronggói. Không chạy môphỏng hoặc robot trong lần sửa kếhoạch này.

## Ghép vào thư mục máy bạn

Giữ tên thư mục pi5_docking_diagnostics để dùng với /home/tai/Downloads/pi5_docking_diagnostics. Giải nén bảnmới ở thưmục riêng, cho Codex so diff/ghép, không ghiđè code thực đã triểnkhai bằng scaffold. Bảnmới phải ghi r3-approved-lattice. Không cần duyệt lại planner/controller; chỉ làmrõ sốđo cònthiếu nếu runtime chưa có. Chạy robot thật vẫn chờ người dùng chủđộng bắtđầu sau xemhình.
