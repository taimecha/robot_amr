# Nội dung giao Codex — phương án đã chốt

Đọc KE_HOACH_CODEX.md bản08/11/2026-r3, QUYET_DINH_DA_CHOT.md và planning_requirements.json. Đối chiếu source/runtime với robot_amr baseline98d4d4d, giữ thay đổi cục bộ.

Tôi đã CHỐT thay bộ tạoquỹđạo bằng SmacStateLattice dùng primitives vi sai/skid-steer, tiếp theo ConstrainedSmoother CHỈ làm mượt đoạn tiếp cận, nối đoạn thẳng cuối dựng chính xác theo trục càng. GIỮ thuật toán bộbám hiện tại. Không xin lại lựa chọn đã duyệt; không dùngHybrid/CasADi/MPC/MPPI thay thế hoặc fallback ngầm.

Đồng bộ cameraoptical-vỏcamera-fork_root-base-odom. Người dùng nhìn từtrước đối diện càng: y_F-y_H=-20mm theoYxe, không nhầm với y_BF/optical. Goalbase tính từ goalfork_root; planner/follower/plot dùng chungtransformhash. Không chấm chỉ đường tâmxe.

Pallet355×355×100mm,4chân50×50mm; càng dài385mm, width155mm và height75mm theo mốc cần xácđịnh; trốngsau300mm. Lattice forward-only, penaltyưu tiêncong; tựchỉnhyaw khi cần, phaquay riêng không smoothing xuyên qua. Không tự thêm lùi. Terminalstraight không đưa vào smoother.

Bộbám giữ nguyên, nhận path mới và giới hạnv<=0.15m/s; lineyaw<=0.5°, lateral<=20mm ưu tiên10mm; nominalerror0; dừng thật rootgap20mm rồi nâng80mm từHOME mới. Không dùng floor0.012m/s cũ; dùng minđã xácnhận hoặc báo thiếu, không tự chỉnhPID/PWM. Giữ phasequay và feedbackyaw, không nearest-point skip quay. Nếu bộbám đang dùng chưa xửlý phaquay, báo chỗ tíchhợp thiếu thay vì tự đổi thuậttoán.

HOME mới: triggerdưới -> xuốngthêm1mm -> đặtzero; cơkhí đủ đã xácnhận, giữ ngoại lệ boundedHOME-only và endstop/fault/watchdog khác. Không hỏi lại xácnhận1mm, không tự flash/chạyngay khi đang viết/môphỏng.

Luồng: YOLOkhôngdeadline nhận đúngloại -> tắtYOLO -> RGB-D lọcổnđịnh -> khóapose -> lậpđườngmộtlần -> hìnhplanned -> tôicho bắtđầu -> bámfixedpath bằngodom/IMU -> dừngrootgap2cm -> lift80mm -> kếtthúc. Saulậpđường không PALLETmeasurement/tracking; RGB-D/LiDARchỉstop, khôngreplan. Không benchmarkCPU/USB/rung/PID hoặc mởlại gatekiểmtra cũ.

Triểnkhai môphỏng trước; truthGazebo chỉ chấm, không cấp đápán insertion. Xuấthìnhplanned/odom cho basevàfork_root, kèm yaw/lateral/gap/v/lift. Môphỏngđạt mới đo camera thật đứngyên và vẽhìnhcho tôixem; chưa tự phátlệnh thật. Tôichủđộng bắtđầulượt chạy thật saupreview. Muốn thay thiết kế hoặc nới0.5°/2cm phải đềxuất trước.
