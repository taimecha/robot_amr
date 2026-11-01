# Góc giữa mặt trước xe và mặt trước pallet

Góc chính trên web là độ lệch giữa hai mặt phẳng khi nhìn từ trên xuống:
`plane_alignment_deg = abs(face_yaw_deg)`. 0° nghĩa hai mặt song song. Góc
hướng tới điểm tâm lỗ vẫn là một đại lượng riêng. Góc hướng đưa càng vào
`entry_heading_rad = -face_yaw_rad` giữ nguyên để đối chiếu hướng trái/phải.

## Sửa riêng phép đo góc

Bộ đo cũ dùng hệ số lệch Depth trái–phải cố định `0.00015807 1/m/pixel` cho
cả vị trí và góc. Dữ liệu sàn cùng tư thế cho hệ số khoảng `0.00013431` trong
bản ghi 2026-10-31 nhưng khoảng `0.00010052` ở bản ghi 2026-11-01. Sự thay đổi
này phù hợp với việc góc bị lệch vài độ dù xe/pallet giữ nguyên. Chưa tách
riêng được nguyên nhân phần cứng, nhiệt hay đăng ký ảnh của sự thay đổi đó.

Tùy chọn `--heading-floor-reference` chỉ thay cách đo **góc**:

1. Dùng Depth hiện tại trong vùng sàn phẳng (cột 30–610, hàng 350–470), nội
   tại RGB và hồi quy bền vững để ước lượng sai lệch nghịch đảo Depth theo cột.
2. Dùng hệ số này riêng cho phép fit mặt phẳng trên phần trong của hai chân
   trước, trên nhiều hàng. Tính góc normal của mặt này so với hướng tiến xe.
3. Không cộng hệ số bù góc cũ 0,896° vốn đã fit theo công thức Depth cũ.
   Không lấy pallet đang thẳng làm điều kiện để tự đặt góc bằng 0.

**Điều kiện:** camera có roll 0° và không lệch yaw so với xe theo thiết lập
đã đo/mô hình hiện tại; vùng sàn tham chiếu phải thật sự phẳng, rõ Depth và
không bị vật cản chiếm đa số. Mã kiểm tra số mẫu, độ phủ và sai số mặt phẳng;
thiếu sàn/mất chân phải báo không hợp lệ. Chưa dùng chế độ này để điều khiển
xe chuyển động trên sàn nghiêng hoặc gồ ghề.

Khoảng cách, X/Y, độ rộng lỗ và bộ kiểm tra mẫu Depth vẫn dùng đúng công thức
và hệ số bù vị trí cũ. Kiểm thử và phát lại cùng bộ ảnh xác nhận các giá trị
này **giống hệt** khi bật/tắt tùy chọn góc. Chỉ normal/góc mặt phẳng và các
chẩn đoán mặt phẳng đã bù được cập nhật. Không chỉnh raw Depth, CameraInfo,
URDF, camera, FPS YOLO hoặc ngưỡng tuổi của phép đo.

## Chạy kiểm tra trên giao diện

Giữ camera ở terminal 1. Trong terminal 2:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml \
  --heading-floor-reference --start-paused
```

Trong terminal 3, mở lại web để nhận nhãn góc và API mới:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_web.sh
```

Mở http://localhost:8765 (chuyển tiếp cổng nếu dùng VS Code SSH), bấm
**Nhận diện lại (3–5 giây)**. Sau xác nhận, YOLO tự tắt và RGB-D tiếp tục đo.
Trong chế độ chờ goal, thêm `--recognize-after-goal` thay cho thao tác nhận
diện ngay; các giới hạn goal mới/thành công vẫn như PERCEPTION_HANDOFF.md.
Chưa chạy bộ căn xe thật.

## Xác minh

81 kiểm thử đạt, gồm mặt pallet thật sự xoay ±5° với các mức lệch Depth khác
nhau, vị trí/X/Y không thay đổi, sàn hoặc chân mất phải từ chối, và API góc
mặt phẳng không bị thay bằng góc tới tâm lỗ.

Phát lại dữ liệu thực (góc mặt có dấu, trước bộ lọc thời gian):

| Bộ dữ liệu | Cách cũ | Tham chiếu sàn + fit mặt phẳng |
| --- | --- | --- |
| 30 cặp ảnh hôm nay | −3,95 … −2,15° | −0,98 … +0,55° |
| Ảnh cùng tư thế hôm qua | −1,37 … −0,37° | −0,89 … −0,02° |
| Bộ ảnh ở khoảng cách khác | −5,04 … −4,44° | −1,43 … −1,13° |

Đây là dữ liệu đối chiếu, không chứng nhận độ chính xác tuyệt đối trên toàn
vùng nhìn. Chi tiết local: `data/diagnostics/floor_plane_heading_replay_2026_11_01.json`.
Trạng thái ROS ghi `heading_source=floor_referenced_plane`, hệ số góc đang
ước lượng ở `heading_depth_x_slope`, và góc trước bù ở `pre_bias_entry_deg`.

## Khôi phục khi một ảnh RGB lỗi

Một lỗi optical flow/tương quan không còn bỏ ngay tham chiếu RGB. Ảnh lỗi
vẫn bị xóa số đo, chỉ giữ tham chiếu tối đa 500 ms tính từ ảnh tốt cuối.
Khi ảnh rõ trở lại, phải xác nhận ba phép đo mới trước khi xuất hợp lệ.
Ảnh lỗi không cập nhật timestamp tốt. Mất chân kéo dài quá 500 ms, timestamp
đi lùi hoặc chuyển động ngoài giới hạn vẫn yêu cầu nhận diện lại bằng YOLO.
Bù chênh độ sáng chỉ hỗ trợ ghép RGB với ảnh/mask gốc, không tích lũy mask để
làm trôi góc và không che việc pallet thật sự biến mất.

## Hồ sơ bù hằng số đã loại

Hồ sơ thử `pallet_target_bias_2026_11_01_heading_reference.yaml` (bù mặt +4,0267°)
đã bị loại sau khi góc dịch khoảng 2° giữa hai lần nhận diện. Nó chỉ làm một
lượt nhìn gần 0°, không xử lý được sai lệch Depth. Bản lưu ở
`data/diagnostics/rejected_heading_reference_2026_11_01.yaml`; không dùng để
vận hành. Hướng dẫn chạy ở trên dùng lại hồ sơ vị trí 2026-10-31, chỉ thay
cách đo góc bằng tham chiếu sàn độc lập.

## Lượt kiểm chứng camera thật 60 giây

Bản sửa gồm thử lại RGB và góc tham chiếu sàn, có **1404/1404 phép đo hợp lệ
trong 60 giây (~23,40 Hz)**. Tuổi kết quả trung bình 32,4 ms, tối đa 82,5 ms;
khoảng ngắt hợp lệ lớn nhất 217 ms. Không có suy luận/seed YOLO mới trong cửa sổ.
Web có số đo **323/323 lần đọc**. Góc giữa hai mặt sau lọc 0,001 … 0,568°;
trước lọc có độ lớn tối đa 1,393°.

Tiêu chí kiểm chứng tổng thể vẫn có một mục chưa đạt: biên độ **góc có dấu
trước lọc** −0,835 … +1,393° = 2,228°, vượt ngưỡng biên độ 2°. Vì vậy không
coi lượt này là xác nhận đủ điều kiện căn xe thật, dù góc chính và độ liên
tục đã cải thiện. Báo cáo local: `floor_plane_rgbd_live_2026_11_01.json` và
`floor_plane_web_live_2026_11_01.json` trong `data/diagnostics/`.

Kiểm tra lại sau khi dừng và mở mới cả bộ đo/web, nhận diện một seed mới:
**99/99 lần đọc web có số đo trong 20 giây**, góc giữa hai mặt sau lọc
**0,279 … 0,669°**, trung bình 0,460°. YOLO giữ tắt trong cửa sổ. Chi tiết:
`data/diagnostics/floor_plane_restart_web_2026_11_01.json`.
Các tiến trình kiểm thử đều đã dừng; camera do người vận hành mở được giữ nguyên.
