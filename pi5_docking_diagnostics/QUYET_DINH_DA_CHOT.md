# Quyết định đã chốt —08/11/2026 r3

> Yêu cầu giao diện trong phiên: `./simulation/launch_r3_sim.sh gui`; nhập tư thế theo tâm gốc càng, không min/max số hữu hạn, không thời hạn chạy tổng; hiển thị camera và kết quả nhận diện, hai đường planner/thực tế. Ca ngoài mẫu phải được chấm riêng; không suy rộng ca thẳng đạt thành mọi tư thế đạt. Xem [hướng dẫn](../simulation/r3/README.md).

> Cập nhật tiêu chí mô phỏng theo người dùng: chấp nhận phần tạo/bám quỹ đạo, sai số dừng dọc chỉ báo cáo và không chặn nghiệm thu phần bám. Giữ goal danh định gốc càng20mm, v≤0.15m/s, ngang≤20mm, góc≤0.5°. Nâng80mm chưa được miễn. Chạy xem: `./simulation/launch_r3_sim.sh world` rồi `./simulation/launch_r3_sim.sh run` ở terminal thứ hai.

Đính chính người dùng trong phiên triển khai: pallet trống100mm, mặt gần30mm, tổng khoảng130mm; cặp càng phủ bì155mm, dày3mm; mặt trên75mm tại trigger,74mm ở HOME cuối; gap20±5mm. Xem `TRIEN_KHAI_R3.md` để biết kết quả thực chạy.

Người dùng đã duyệt: **Smac State Lattice với primitives vi sai/skid-steer -> Constrained Smoother cho approach -> đoạn cuối thẳng theo trục càng; GIỮ bộbám hiện tại.** Không hỏi lại lựa chọn này.

- Không dùng SmacHybrid, CasADi/IPOPT, MPC/MPPI làm phương án thay tự động. Controller hiện có nhận path mới, giữ thuật toán, cập nhật giới hạn và điều kiện dừng theo spec.
- Smoother chỉ làm đoạn tịnh tiến tiếp cận; giữ line cuối và ranhgiới phasequay. Goal trong base quy từ fork_root, không từ tâmcamera.
-4chân50×50mm; pallet355×355×130mm (trống100mm, mặt gần30mm); càng385mm/155mm và cao75mm theo mốc cần đúng; phía sau trống300mm.
- y_F-y_H=-20mm theoYxe, người dùng nhìn từ trước đối diện đầu càng; không coi H là optical/base nếu chưa đúng.
- Tự chỉnhyaw khi cần; lineyaw<=0.5°, lateral<=20mm ưu tiên10mm, v<=0.15m/s; rootgap20mm sau dừng.
- HOMEtrigger dưới rồi đi xuống1mm, đặtzero cuối, nâng80mm từzero đó; cơ khí đủ1mm đã xác nhận. Không hỏi lại hoặc tự flash/chạyngay.
- Không PALLET tracking khi chạy; obstacle-onlystop; không diagnostics tổngquát. Mô phỏng -> camera thật/preview -> người dùng bắt đầu lượtthật -> overlayplanned/odom -> nângxong kếtthúc.

## Dữ liệu còn thiếu nếu runtime chưa có

Giữ số đo đã xác nhận trong repo/runtime; cần xác định mốcoptical/housing/base, vị trí4chân/bềdày mặt, bề rộng/vị trí từng nhánh càng, v_min_running nếu drive cần; tolerancegap đã chốt±5mm và mốc cao đã chốt. Không điền0 cho chưa biết, không dựng quỹđạo chạy thật từ giảđịnh ẩn. Những số đo này không làm planner/controller quay về trạngthái chờduyệt.

Thay đổi thiết kế khác hoặc nới yêu cầu phải báo người dùng trước. Việc triển khai/môphỏng theo đúng lựa chọn hiện tại đã được duyệt; chạythật cần người dùng xemhình và chủđộng bắtđầu.
