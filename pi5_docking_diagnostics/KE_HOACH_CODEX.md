# Kế hoạch docking: đồng bộ tọa độ, tối ưu đường và bám đến khi nâng hàng

> Yêu cầu giao diện trong phiên: `./simulation/launch_r3_sim.sh gui`; nhập tư thế theo tâm gốc càng, không min/max số hữu hạn, không thời hạn chạy tổng; hiển thị camera và kết quả nhận diện, hai đường planner/thực tế. Ca ngoài mẫu phải được chấm riêng; không suy rộng ca thẳng đạt thành mọi tư thế đạt. Xem [hướng dẫn](../simulation/r3/README.md).

> Cập nhật tiêu chí mô phỏng theo người dùng: chấp nhận phần tạo/bám quỹ đạo, sai số dừng dọc chỉ báo cáo và không chặn nghiệm thu phần bám. Giữ goal danh định gốc càng20mm, v≤0.15m/s, ngang≤20mm, góc≤0.5°. Nâng80mm chưa được miễn. Chạy xem: `./simulation/launch_r3_sim.sh world` rồi `./simulation/launch_r3_sim.sh run` ở terminal thứ hai.

> Đính chính người dùng trong phiên triển khai 08/11/2026: pallet trống cao100mm, mặt dày gần30mm, tổng cao khoảng130mm; 155mm là phủ bì CẶP càng, càng dày3mm; mặt trên càng75mm tại trigger,74mm sau HOME_SETTLE1mm. Gap dừng20±5mm. Các số này thay thế cách hiểu/ô còn thiếu tương ứng bên dưới. Báo cáo triển khai: `TRIEN_KHAI_R3.md`.

Cập nhật 08/11/2026, bản r3 ĐÃ CHỐT. Repo: https://github.com/taimecha/robot_amr
Mốc đã đọc: `98d4d4d67ae0808226544373797dba66ecf6797a` (commit mới nhất tại thời điểm soạn).
Gói thay thế kế hoạch chẩn đoán cũ ở `/home/tai/Downloads/pi5_docking_diagnostics`. Không giả định máy đang chạy Codex chính là Pi. Không ghi đè code người dùng đã sửa khi ghép gói.

## 1. Phạm vi và quyền quyết định

Bỏ kế hoạch chẩn đoán tổng quát CPU/USB/mạng/rung camera/PID và ma trận chạy ngắn 30 cm–1 m. Không bắt người dùng làm lại các gate kiểm tra cũ. Chỉ giữ các bước người dùng yêu cầu: mô phỏng planner/follower mới, xác nhận phép đo camera và hình quỹ đạo trên robot đứng yên, sau đó chạy thật cả chu trình và vẽ planned vs odom.

Đây là kế hoạch và hợp đồng code giả, chưa sửa source repo, chưa triển khai thuật toán mới hoặc firmware, chưa chạy/nâng robot. Người dùng đã CHỐT: Smac State Lattice dùng primitives vi sai/skid-steer -> Constrained Smoother chỉ cho đoạn tiếp cận -> đoạn thẳng cuối theo trục càng; GIỮ bộ bám hiện tại. Codex được triển khai thiết kế này và mô phỏng, không cần xin lại lựa chọn planner/controller đã chốt. Các số đo còn thiếu xử lý theo QUYET_DINH_DA_CHOT.md, không tự điền. Không tự thay bằng Hybrid-A*, CasADi/IPOPT, MPC/MPPI hoặc bộ khác, không tự nới dung sai. Chạy robot thật vẫn cần người dùng xem hình và chủ động bắt đầu như kế hoạch; thay đổi thiết kế mới phải nêu trước.

## 2. Các yêu cầu đã chốt từ người dùng

| Nội dung | Giá trị/hành vi |
|---|---|
| Acquisition | YOLO nhận đúng loại pallet, không deadline tổng; RGB-D đo pose khi xe đứng yên |
| Sau nhận diện | Tắt YOLO, tiếp tục RGB-D qua bộ lọc ổn định, khóa pose rồi lập đường |
| Sau khi có đường | Tắt PALLET RGB-D measurement/tracking; RGB-D và LiDAR chỉ dùng dừng vật cản trên hành lang chạy |
| Tạo đường | Dùng Smac State Lattice vi sai + Constrained Smoother + đoạn thẳng cuối thay planner quintic hiện tại; đi chéo theo động học hoặc cong ngắn, rồi đoạn thẳng tới pallet; ưu tiên cong ngắn; nếu còn lệch thì tự điều chỉnh góc, kể cả quay tại chỗ khi cần và vùng quét hợp lệ |
| Sai lệch ở đoạn thẳng và điểm dừng | Mục tiêu càng sát 0 càng tốt; lệch ngang tối đa 0.02 m (ưu tiên ≤0.01 m); sai góc tuyệt đối ≤0.5° |
| Tốc độ base từ follower | 0 ≤ linear.x ≤0.15 m/s; linear.y=0; không tự lùi |
| Dừng cuối | Gốc càng cách mặt trước pallet0.020±0.005m SAU KHI XE ĐÃ DỪNG |
| Nâng | Khớp nâng đi tới +0.080 m tính từ HOME mới, sau khi xe dừng tại điểm đưa càng |
| HOME người dùng yêu cầu | Tiếp xúc công tắc dưới, sau đó đi xuống thêm 0.001 m để giữ công tắc ổn định; chỉ định nghĩa zero sau pha đó |
| Pallet | Dài 0.355 m, rộng 0.355 m, cao tổng thể khoảng0.130 m (trống0.100 m + mặt gần0.030 m); 4 chân, tiết diện mỗi chân 0.050×0.050 m |
| Phía sau pallet | Khoảng trống 0.300 m |
| Càng | Dài0.385m, phủ bì cặp0.155m, dày0.003m; mặt trên0.075m tại trigger,0.074m sau HOME cuối |
| Lệch gá | Người dùng đứng trước đối diện đầu càng: gốc càng lệch PHẢI của xe so tâm vỏ camera; y_F-y_H=-0.020 m theo trục Y của xe |
| Quy trình triển khai | Mô phỏng → xác nhận camera thật + hình planned → người dùng cho chạy thật tới nâng hàng → planned/odom overlay |

Không lấy 0.15 m/s làm tốc độ nâng; đó là vận tốc tiến của base. Không tự đặt tốc độ thấp nhất vì chưa có số người dùng chốt. Không dùng hằng 0.012 m/s của demo cho đoạn cuối theo quán tính.

## 3. Tiến độ source hiện tại để sửa đúng chỗ

- `simulation/docking_preview.py`: candidate quintic + straight và ShadowFollower Pure Pursuit tối giản, mặc định max 0.10 m/s. Đây là baseline để so sánh, không là planner được chọn cho bản mới.
- `simulation/one_pallet_visualizer.py` và `pallet_rgbd_filter.py`: đo X/Y/yaw từ Depth, lọc pose trước khóa, dùng heading đo để lập đường; vẫn có điều kiện ground truth kiểm ca trong mô phỏng.
- `simulation/gazebo_dock_trial.py`: insertion còn dùng pose Gazebo để tính khoảng còn lại, max insertion 0.04 m/s, có floor 0.012 m/s, goal lift khác 80 mm. Các giá trị/hành vi này phải được thay theo kế hoạch sau khi thiết kế được duyệt.
- `one_pallet_case.py`: vẫn có fallback heading sang yaw ca thử; runtime mới không được dùng fallback này khi thiếu góc đo.
- Firmware càng: `esp32_firmware/Projects/cang_nang_ha/src/main.cpp` hiện debounce 15 ms, `SEEK_LOWER` gặp lower-active thì completeHoming ngay, và chặn hướng đi xuống khi lower active. Yêu cầu ép thêm 1 mm chưa được hỗ trợ. Không thể chỉ gửi SET âm ở ROS rồi cho rằng đã có HOME mới.
- Gói diagnostics đã được copy vào repo ở `pi5_docking_diagnostics_1/`; bản kế hoạch này thay thế chỉ dẫn cũ, không tự xóa source người dùng.

Chỉ đọc git status/HEAD/AGENTS.md và diff để không mất thay đổi cục bộ. Không phát sinh dự án kiểm tra/benchmark tổng quát mới.

## 4. Đồng bộ camera – gốc càng – tâm xe

### 4.1 Các hệ tọa độ phải khai báo

B=base_link/base_footprint (X tiến, Y trái của XE, Z lên); O=odom local; C=camera optical; H=tâm vỏ/gá camera; F=gốc càng, tại mặt phẳng gốc và tâm ngang CẶP càng; P=frame pallet với origin tại tâm hành lang vào lỗ trên mặt trước, +X hướng đưa càng vào.

cx/cy của CameraInfo là tọa độ pixel principal point, không phải tâm vỏ camera, cũng không phải gốc càng. Không cộng “2 cm” vào pixel ảnh. Cần T_BC và T_BF có số đo thật; nếu dùng H làm mốc thì phải có T_HC và T_HF. Độ lệch H→F một trục không xác định đủ toàn bộ transform.

Người dùng đã xác nhận đứng trước xe đối diện đầu càng. Trái của người nhìn là phải của XE, nên đã chốt y_F-y_H=-0.020 m theo Y của xe. Đây là hiệu độ ngang giữa gốc càng và tâm VỎ camera, KHÔNG có nghĩa y_BF=-0.020 m nếu H không trùng tâm base. Không tự gán bằng đó cho optical center. Chiều dọc x_BF/x_BC, yaw gá, độ lệch tâm vỏ–optical vẫn dùng số đo đã xác nhận; thiếu thì không dùng 0 thay thế.

### 4.2 Transform và mục tiêu cuối

Với pose RGB-D thật trong optical: T_OP = T_OB(t_capture) * T_BC * T_CP.
Nếu estimator repo trả scalar đã hiệu chỉnh trong hệ base/chassis, dùng đúng adapter cho scalar đó, không transform optical lần hai. Mọi kết quả phải gắn frame_id, feature (front opening center/top center), applied_bias và calibration hash.

Đường base và đường gốc càng liên hệ tại MỌI sample:
 p_F = p_B + R(theta_B) * r_BF,
 theta_F = theta_B + psi_BF.

C là origin mặt trước/hành lang pallet trong O; n=(cos(theta_P),sin(theta_P)), t=(-sin(theta_P),cos(theta_P)). Goal gốc càng danh định:
 p_F_goal = C - 0.020*n,
 theta_F_goal = theta_P.
Goal base:
 theta_B_goal = theta_P - psi_BF,
 p_B_goal = p_F_goal - R(theta_B_goal)*r_BF.

Planner và follower cùng dùng MỘT r_BF/psi_BF/hash; không cho planner căn tâm base còn follower căn tâm camera. Chấm e_y tại F: dot(p_F-C,t); sai góc wrap(theta_F-theta_P); gap gốc càng = -dot(p_F-C,n). Bề rộng và tip sweep phải tính từ vị trí thật của từng nhánh càng, không chỉ line F.

Sai lệch ngang ≤2 cm là giới hạn chấp nhận, không là yêu cầu cố tình tạo đường lệch 2 cm. Solver đặt e_y=0 và e_yaw=0 danh định; tối ưu càng sát 0 càng tốt.

## 5. Hình học pallet/càng và điểm dừng

Dùng đúng 4 chân 50×50 mm theo đính chính của người dùng; không có chân thứ năm ở tâm. Vị trí các chân dùng hình học thực/đã xác nhận, không tự thêm chân giữa. Người dùng đã xác nhận chân/trống cao100mm, mặt dày gần30mm, phủ bì cặp càng155mm, dày3mm, mặt trên75mm tại trigger và74mm ở HOME cuối. Còn cần vị trí chân thật và bề rộng/khoảng cách từng nhánh càng; không coi các mốc ước lượng của URDF là số đo đã xác nhận.

Không dùng cao tổng thể 10 cm để suy lỗ thông thủy cao 10 cm. Không hạ/đẩy xuyên qua chân giữa vì chưa mô hình đủ.

Nếu càng dài 38.5 cm tính từ mặt gốc càng, goal gap=2 cm, pallet sâu35.5 cm và song song: độ cắm dưới pallet=38.5-2=36.5 cm, mũi càng nhô sau khoảng 1 cm. Khoảng trống sau30 cm đủ cho phần nhô danh định đó, nhưng không thay kiểm hình học 4 chân/toàn bộ sweep. Tính lại nếu mốc đo chiều dài càng khác.

Xây đường cong tới A sao cho trục càng đã thẳng trước khi mũi càng đi qua mặt trước. Chọn A dựa envelope tip của cả hai nhánh; không cho “còn chỉnh ngang” khi càng đã nằm giữa các chân. Đoạn thẳng A→goal là thành phần bắt buộc, chiều dài/điểm nối được solver tìm trong miền hợp lệ, không tự rút về zero để giảm path length.

## 6. Bộ tạo đường đã chốt: State Lattice vi sai + Constrained Smoother + đoạn thẳng cuối

Người dùng đã duyệt phương án này. Dùng `SmacPlannerLattice` của `nav2_smac_planner`, primitives cho differential/skid-steer (vy=0), hỗ trợ tiến/cung quay và pha quay tại chỗ cần thiết. Không dùng primitives Ackermann/omni; không chọn `SmacPlannerHybrid` chỉ vì tên gần giống.

Bốn motor độc lập không biến robot thành holonomic. Tận dụng động học/ước lượng effective track hiện có; không tự chỉnh PID hoặc mô hình trượt để làm đường đạt. Goal của planner là pose BASE được tính từ goal FORK_ROOT theo cùng transform ở mục4, không lấy camera center làm goal.

### 6.1 Luồng lập đường một lần

1. Nhận target pallet đã đo/lọc/khóa trong odom, geometry thật và pose đầu của xe. Không đọc ground truth/spawn yaw để tạo đường. Snapshot frame/calibration/config/plan ID trước khi solve.
2. Xác định miền điểm A nằm trên đường trục đưa càng, trước vùng mũi càng đi vào giữa chân pallet. A phải bảo đảm đoạn thẳng còn đủ dài, không bị rút về zero để giảm length. Tọa độ A/base được quy từ trục F và offset đã biết.
3. State Lattice tìm đường S->A theo mô hình vi sai, footprint gồm thân/càng. Tắt reverse expansion để không tự thêm chạy lùi; dùng rotation penalty ưu tiên đường cong liên tục, vẫn cho quay chỉnh hướng khi cần. Giá trị penalty và primitives lấy cấu hình hợp lệ rồi lưu vào plan, không tuyên bố shortestglobal từ đó.
4. Với các A hợp lệ, làm mượt phần TIẾP CẬN bằng Constrained Smoother. Giữ pose đầu/cuối và hướng nối với line; chỉ chấp nhận kết quả còn đúng động học, footprint và hướng goal. Không hiểu các trọng số smoother là bảo đảm hardconstraint.
5. Dựng line A->F_goal bằng hình học chính xác trong odom, theo theta_P, goal rootgap20mm. Không đưa line này vào smoother để tránh bẻ cong hoặc dịch điểm dừng. Kiểm tiếp tuyến và chuyển độ cong tại điểm nối; nếu không hợp lệ thì chọn A/nhánh tiếp cận hợp lệ khác theo cùng planner, không bóp méo line hoặc đổi thuật toán.
6. So các nghiệm khả thi: ưu tiên độ dài ngắn, ít quay dừng và mượt; báo length, maxcurvature, số pha quay, clearance, profilev, error mục tiêu. Không khẳng định shortesttoàncục; không dùng path ngắn hơn mà vượt clearance hoặc sai hướng.
7. Gắn speedprofile <=0.15m/s và giới hạn gia tốc/phanh/omega từ cấu hình đã xác nhận. Lưu path base, path fork_root, các pha và plan_id. Xuất hình trước khi cho chạy.

### 6.2 Pha quay, smoother và độ chính xác cuối

Nếu lattice có đoạn cùng XY nhưng đổi yaw (pure rotation), tách thành phase ALIGN_YAW, không để smoother biến các pose trùng XY thành cung cong hoặc để follower bỏ qua do nearest-point pruning. Chỉ làm mượt các đoạn tịnh tiến, giữ nguyên ranh giới/pose pha quay. Xử lý pha quay bằng khả năng quay/căn góc đang có; nếu bộ bám thực tế không hỗ trợ, báo rõ phần tích hợp còn thiếu thay vì tự thay controller.

Lattice là planner rời rạc; không dùng angular bins mặc định như bằng chứng đạt0.5°. Đoạn cuối phải dựng theo hướng pallet liên tục/exact, kiểm hướng nối thực tế và chấm follower riêng. Không đổi góc goal sang bin gần nhất, không giữ goal tolerance2° cũ hoặc ngầm dùng điểm gần đích là thành công.

Sau smoothing, kiểm lại full swept envelope2càng/thân,4chân,mặt pallet/sàn và các pose trung gian đủ mịn. Footprint 2D không tự thay kiểm khoảng hở theo cao độ khi đưa càng vào. Dùng hình học đã xác nhận, không thêm vật/cản giả để điều khiển bộ tối ưu.

Không áp smoother trên toàn đường nhiều lần khi xe đang chạy. Path cố định sau preview; obstacle chỉ stop. Planner/smoother lỗi hoặc infeasible thì giải thích và chờ, không tự fallback sang quintic/Hybrid/CasADi hoặc hạ0.5°/2cm. Thông số triển khai thường lệ có thể tinh chỉnh trong đúng thiết kế để mô phỏng đạt, phải ghi lại; thay thiết kế/đổi giới hạn mới cần duyệt.

## 7. Bộ bám: GIỮ HIỆN TẠI, không đổi thuật toán

Người dùng đã chốt giữ bộ bám hiện tại; không thêm LTV-MPC, MPPI hoặc đổi sang controller khác. Codex xác định đúng controller runtime/demo đang dùng trong nhánh được chạy; không gọi ShadowFollower tối giản là Nav2 RPP. Giữ thuật toán, sửa giao diện/giới hạn/tiêu chí hoàn tất theo yêu cầu để nhận path mới và dùng chung frame/offset càng.

Bám path cố định bằng odom + IMU/EKF. Không mở lại pallet vision/tracking. Có thể căn góc bằng chức năng hiện có; ưu tiên vừa tiến vừa chỉnh, quay tại chỗ khi cần trước khi càng vào với vùng quét hợp lệ. Đoạn thẳng không phải luônomega0: điều chỉnh nhỏ kéo yaw/Y về0, giữ |yaw_F-theta_P|<=0.5° và |ey_F|<=20mm, ưu tiên<=10mm. Càng đã vào thì chỉ chỉnh trong clearance đã tính, không quay mạnh quệt chân.

Kiểm tiến độ theo arc-length/phase, không shortcut xuyên qua chân hoặc bỏ qua phase quay. Chấm sai lệch tại FORK_ROOT bằng transform chung, không chỉ tâm base. Giữ endpoint/line/reference yaw của plan mới; không tự thay điểm dừng theo groundtruth.

Error theo target khóa chưa chứng minh error theo pallet thật: mô phỏng dùng truth vào scorer riêng. Nếu yawcamera ban đầu đã lệch hơn0.5° thì bám hoàn hảo vẫn sai; bước xác nhận camera phải thể hiện điều này. Median với inlier2.5° hiện tại không tự chứng minh accuracy0.5°.

Vận tốc: 0<=v<=0.15m/s; vy=0; không tự lùi. Không dùng floor0.012m/s của demo cũ. min_running_speed chưa có số thì dùng giá trị xe đã xác nhận trong runtime nếu tồn tại và báo nguồn; không bịa số mới. Profile chạy phải đủ thắng ma sát nhưng phanh để dừng thật tại rootgap2cm; không kẹp speedfloor lên lệnhzero/obstacle/E-stop. Không tự thêm PWMkick, sửa PID hoặc pulse driving.

Điểm dừng là khoảng cáchSAU hãm. Tolerancegap đã được người dùng chốt±5mm trong phiên triển khai; không lấy±4mm hoặc thay mục tiêu20mm. Giữ kết thúc là xe đứng yên đúng tư thế và sau đó nâng80mm từHOME mới. Mô phỏng chưa đạt thì báo lỗi/path/follow cụ thể, không đổi bộbám để làm bài thử thành công.

## 8. Luồng nhiệm vụ đã yêu cầu

IDLE → ACQUIRE_YOLO → YOLO_OFF → STABILIZE_RGBD → LOCK_TARGET → PLAN_ONCE → PREVIEW_WAIT → START_BY_OPERATOR → FOLLOW_APPROACH → ALIGN_YAW_IF_NEEDED → FOLLOW_STRAIGHT → STOP_AT_ROOT_GAP_2CM → LIFT_TO_HOME_PLUS_8CM → DONE.

- YOLO không deadline tổng, xác định loại pallet và chọn geometry profile đúng. RGB-D tính khoảng cách, lệch ngang, yaw khi đứng yên; không nạp ground truth/spawn values làm số đo.
- Sau khi nhận đúng: tắt YOLO/inference, RGB-D qua bộ lọc ổn định rồi khóa pose. Chưa đủ ổn định thì chờ dữ liệu mới; không cố hoàn tất theo đồng hồ.
- Sau PLAN_ONCE: PALLET detector/pose estimator/tracker ngừng. Camera driver có thể còn cho obstacle-only giống LiDAR; không sửa target hoặc vẽ lại path theo pallet. Lưu plan/target hash bất biến.
- PREVIEW_WAIT phải chờ người dùng xem hình rồi bắt đầu; không dùng “nhận diện xong” hoặc “có đường” làm lệnh chạy.
- Obstacle-only chặn/dừng, không tự né/replan hoặc bật pallet tracking. Gặp tình huống ngoài kế hoạch dừng và báo thay vì tự đổi phương án.
- Khi còn lệch góc, follower tự chỉnh omega theo yaw đo từ odom/IMU so target khóa, ưu tiên vừa tiến vừa chỉnh. Nếu cần xoay tại chỗ, thực hiện ở vùng càng còn ngoài pallet với sweep hợp lệ. Đoạn cuối vẫn giữ yaw<=0.5° và kéo về0; khi càng đã vào, chỉ chỉnh nhỏ trong envelope đủ khoảng hở. Dừng thật tại gap2cm, kiểm điều kiện yaw/lateral theo feedback và hình học, rồi nâng tới80mm so HOME. Không gọi quãng đang trôi sau zero là đã đứng yên.
- Kết thúc thử nghiệm khi xe đứng yên, lift target80mm đạt và quan sát hàng đã nâng (world truth chỉ chấm mô phỏng; robot thật dùng feedback lift hiện có và người vận hành xác nhận hàng). Không tự thêm nhiệm vụ rút càng/chở hàng/hạ hàng.

## 9. HOME thêm1mm và nâng80mm — cơ khí đã được người dùng xác nhận

Ghi nguyên yêu cầu: sau lần chạm dưới đi xuống thêm1mm rồi đặt HOME=0; sau đó nâng tới z_lift=+80mm. So với vị trí trigger đầu tiên, đích nâng là +79mm nếu zero cuối ở -1mm. Người dùng đã xác nhận7.5cm là mặt trên càng tại trigger; sau HOME_SETTLE1mm là7.4cm, sau nâng80mm là15.4cm.

Firmware hiện dừng ngay khi lower active; không thể dùng SET âm không sửa state machine. Phạm vi thực hiện yêu cầu đã được xác nhận: ngoại lệ có giới hạn CHỈ trong pha HOME_SETTLE sau tiếp xúc hợp lệ, tổng bước tiếp tục<=1mm theo steps/mm thật, có giới hạn thời gian và giữ upper/fault/watchdog. Không disable tất cả endstop hoặc bỏ chặn downward trong vận hành thường. Switch đã active từ đầu vẫn cần xử lý release/reapproach đúng logic đang có; dây đứt không được coi là HOME.

Người dùng đã xác nhận cơ khí/công tắc còn hành trình và đổi yêu cầu xuống thêm từ2mm thành1mm. Không hỏi lại xác nhận đó; khi triển khai sau mô phỏng phải làm bounded HOME_SETTLE đúng1mm, giữ upper/fault/watchdog và chặn endstop vận hành thường. Debounce hiện15ms vẫn giữ, không tự thay yêu cầu1mm bằng phương án khác. Xác nhận này không phải lệnh flash hoặc chuyển động ngay trong bước sửa kế hoạch.

Không tự flash firmware trong bước sửa kế hoạch. Mô phỏng home/lift phải dùng đúng zero mới và hành trình80mm, không dùng85mm demo cũ.

## 10. Thứ tự mô phỏng, xác nhận camera và chạy thật

### A. Mô phỏng trước — theo yêu cầu, không là dự án diagnostics

Dùng planner/controller đã chốt ở mục6–7; sau khi bổ sung geometry bắt buộc: dùng world pallet355×355×130mm (trống100mm + mặt gần30mm),4 chân, càng385mm/155mm và offset2cm đúng dấu. Những mốc chưa biết phải thể hiện chưa đủ spec, không thêm chân giữa hoặc lấp số đo chưa biết.
Đầu vào pose có thể có sai yaw/y để thấy planner chọn đường. Dựng đủ vài ca thẳng/lệch trái/phải/góc hai dấu trong vùng làm việc; người dùng duyệt tập ca, không áp lại ma trận kiểm tra chuyển động cũ. Tập trung shortest feasible vs baseline, straight alignment và hành trình lift.
Trong runtime simulation, planner/follower chỉ đọc target đo/khóa và odom/IMU. Truth Gazebo chỉ cung cấp chấm độc lập: e_yaw thật, e_y tạiF, gap, contact và lift/pallet rise; không được dùng truth để điều chỉnh điểm dừng insertion như code hiện tại. Đưa kịch bản “pose nhập chuẩn” và “pose RGB-D” vào báo cáo riêng, không trộn.
Hình trước chạy: S,A,goal, đường base, đường F, envelope2càng,4chân, mặt trước/sau, rear clear30cm, đoạn line, nhãn offset2cm/sign, chiều dài, profilev và yaw reference. Hình sau chạy: planned vs executed_odom cùng frame/aspect; thêm Gazebo truth chỉ cho scorer; v, yaw_error_F, ey_F,gap và lift_vs_time. Không chỉ vẽ đường tâm xe rồi kết luận càng đúng.
Điều kiện mô phỏng: maxv<=0.15; line yaw<=0.5° và lateral<=2cm; gap target2cm với tolerance±5mm đã chốt; lift80mm so HOME; không va chân/mặt/sàn. Báo INCOMPLETE nếu chưa có geometry/tolerance, không PASS giả. Solver feasibility không chứng minh follower đạt.

### B. Mô phỏng xong: xác nhận camera thật khi đứng yên và vẽ đường

Giữ phép đo Astra/hiệu chỉnh sẵn có, xác nhận độ lệch camera–F–B đúng với mốc thật; kiểm lại X/Y/yaw và dấu so pallet thực. Đây là bước xác nhận người dùng yêu cầu, không khôi phục plan CPU/USB/PID/rung dài hạn cũ.
Sau đo/lọc ổn định, xuất hình đường dự kiến dùng geometry thật, cùng model/transform/planner hashes và pose khóa. Không phát lệnh chuyển động/nâng chỉ để xem hình. Người dùng xem hình và chốt đường trước lượt thật. Xe/pallet/gá thay đổi sau preview thì hình cũ không còn đúng; phải đo/lập lại và cho xem lại, không tự chạy theo target cũ.

### C. Chạy thật đầy đủ sau khi người dùng cho bắt đầu

Dùng đúng plan đã duyệt/khóa và controller đã chọn. Odom/IMU feedback bám đường, obstacle-only dừng. Đi vào line rồi dừng khiF cách mặt trước2cm, nâng lên+80mm so HOME mới, kết thúc. Không tự mở lại tracking hoặc đổi planner.
Vẽ và lưu cả planned và odom thực chạy trong cùng hệO: path_base và path_fork_root. Muốn so với mặt pallet thật phải giữ target snapshot cùng frame. Odom trace là ước lượng, không tự coi nó là độ chính xác tuyệt đối của càng.

## 11. Delay cảm biến và các mục ngoài phạm vi

Theo người dùng: không triển khai benchmark/tối ưu latency, CPU, USB, network hoặc thay đổi planner chỉ vì FPS/delay camera/laser. Các gói trễ obstacle không được kéo đích đã khóa hoặc làm hệ tự nhận diện lại. Không để chương trình chờ camera mới trong loop follower; điều khiển dựa odom/IMU và plan cố định.

Tuy nhiên, bỏ qua độ trễ để điều khiển KHÁC với chứng minh hành lang không có vật cản: khi dữ liệu quá cũ, chức năng dừng vật bất ngờ có thể không còn bảo đảm. Chính sách stale obstacle hiện có cần được người dùng chốt riêng; chưa duyệt thì KHÔNG đổi code để bỏ fail-stop/watchdog. Không đưa điều kiện “tắt endstop/stop mọi nguồn” vào mặc định. Giữ E-stop, watchdog lệnh/drive và lỗi odom cơ bản; đây không phải khôi phục kế hoạch diagnostics. Mục này được ghi riêng ở đây; không làm thay đổi lựa chọn planner/controller đã được duyệt.

## 12. Dữ liệu, hình và handoff cho Codex

Đầu ra mỗi lượt chỉ phục vụ nhiệm vụ:
- target_snapshot.json (pose/feature/frame/calibration hash), geometry/transforms đã xác nhận, plan_id.
- path_planned_base.csv, path_planned_fork_root.csv; states/reference speed/curvature nếu có.
- executed_odom_base.csv, executed_odom_fork_root.csv; cmdv/cmdomega, yaw/lateral/gap và lift status.
- planned.png/svg trước chạy, overlay_planned_odom.png/svg sau chạy, heading_lateral_speed_lift.png; luôn có trục/đơn vị và scale bằng nhau ở plotXY.
- mission_result.json: tiêu chí đạt/chưa đạt theo simulation/camera_preview/real; chi tiết thiếu/vượt, không tự nới tiêu chí hoặc ghi số liệu mô phỏng thành số thực.

Gói giữ tên pi5_docking_diagnostics chỉ để tương thích thư mục Downloads; nội dung hiện là kế hoạch trajectory/docking đã chốt lựa chọn planner và giữ bộbám. Các scaffold chỉ in hợp đồng cho Codex triển khai, không có actuatorcode. Không tự commit/push hoặc sửa robot trong bước lập kế hoạch này.

## 13. Nguồn cho thiết kế đã chốt

- Repo baseline: https://github.com/taimecha/robot_amr/tree/98d4d4d67ae0808226544373797dba66ecf6797a
- Nav2 Jazzy State Lattice: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/
- Nav2 Jazzy Constrained Smoother: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/smoother_plugins/constrained_smoother/configuring_constrained_smoother/
- Smac drivetrain models: https://api.nav2.org/nav2-jazzy/html/md_nav2_smac_planner_README.html
Thiết kế đã được người dùng chọn, nhưng chưa triển khai hoặc chạy trong lần chỉnh kế hoạch này. Lựa chọn thuật toán không phải chứng nhận yaw<=0.5°/rootgap20mm hoặc shortestglobal; phải chứng minh bằng mô phỏng và bước camera/preview đã yêu cầu.
