# Đo mẫu để hiệu chỉnh khoảng cách và góc pallet

Mục tiêu là đo sai số trên nhiều tư thế, chọn mô hình bù đơn giản phù hợp, rồi kiểm tra ở các tư thế khác. Chưa có hệ số mới nào được áp dụng. `record_pallet_calibration.py` chỉ đọc `/pallet/target`; không chạy YOLO lần hai, không đọc Depth thô và không điều khiển xe.

## 1. Cố định thiết lập và đo lại mốc cơ khí

- Giữ xe và pallet đứng yên khi ghi từng mẫu. Không chạy nhiệm vụ Nav2 trong lúc đo. Cố định camera và độ cao càng; giữ nguyên model, độ phân giải, công thức Depth và ánh sáng giữa các tư thế.
- Kiểm tra khoảng cách từ mặt phẳng mép trước khung đến gốc càng (nơi càng bắt đầu nhô ra): mặc định phần mềm là **0,065 m**. Nếu thực tế khác, ghi giá trị đo được bằng `--fork-root-offset-m`. Sai mốc cơ khí cần sửa ở mốc cơ khí trước khi kết luận cảm biến Depth sai.
- Đo/ghi lại vị trí camera so với mép khung và góc chúc thực tế. Node đang mặc định mắt camera trước mép khung **0,0665 m**, góc chúc **20°**. Giá trị trong URDF là mốc cần kiểm tra bằng thực tế.
- Kẻ trục dọc xe trên sàn và đường ngang qua gốc càng, vuông góc với trục dọc. Gốc đo X là đường ngang này; gốc đo Y là trục giữa xe. Dùng thước vuông hoặc phương pháp dựng vuông góc, tránh chỉ nhìn bằng mắt.

## 2. Định nghĩa số đo thật

- **`--fork-root-m`**: khoảng cách theo trục tiến X từ mặt phẳng gốc càng đến tâm lỗ ở **mặt trước pallet**. Chiếu điểm cần đo xuống sàn nếu cần. Khi pallet xoay, vẫn đo theo X; không dùng khoảng cách ngắn nhất vuông góc mặt pallet hoặc kéo thước chéo.
- **`--lateral-m`**: độ lệch ngang của tâm lỗ so với trục giữa xe. Trái xe là dương, phải là âm. Chỉ nhập `0` nếu đã đo xác nhận tâm lỗ nằm trên trục giữa.
- **`--face-yaw-deg`**: góc mặt pallet; phía trái xa xe hơn phía phải thì dương. Đây là góc của mặt pallet, khác với góc hướng từ xe đến tâm lỗ. Pallet có thể thẳng mặt nhưng nằm lệch ngang.

### Xác nhận góc bằng hai khoảng cách

Chọn hai đường đo song song trục X của xe, cách nhau một khoảng ngang `B`. Từ cùng đường ngang gốc càng, đo dọc từng đường đến mặt trước phẳng của pallet: `D_trái`, `D_phải`. Hai điểm phải nằm trên cùng mặt phẳng trước pallet.

```text
góc mặt pallet = atan2(D_trái − D_phải, B) × 180 / π
```

`B` là khoảng cách theo ngang Y, không phải chiều dài chéo trên mặt pallet khi nó xoay. Để xác nhận 0°, hai khoảng cách phải bằng nhau trong sai số thước. Chọn hai đường xa nhau nhất có thể nhưng vẫn chạm đúng cùng mặt trước pallet. Ví dụ `B=0,30 m`, chênh 5 mm tương đương khoảng 0,95°; chất lượng số đo góc phụ thuộc trực tiếp độ chính xác đo này.

## 3. Khởi động hệ đo

Nếu camera và node đã chạy thì giữ nguyên, bật nhận diện trên web. Nếu cần khởi động node từ đầu, dùng terminal riêng:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_target.sh --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 --max-fps 2
```

Không chạy hai node `/pallet/target` cùng lúc. Giữ nguyên cấu hình trong cả đợt đo; `--note` có thể ghi model, góc lắp và cách đo thước.

## 4. Ghi từng tư thế

Mở terminal ghi mẫu, source môi trường một lần:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

Ví dụ **chỉ sau khi đã đo xác nhận** gốc càng đến mặt lỗ 1,40 m, lệch ngang 0 và góc mặt 0°:

```bash
.venv/bin/python scripts/record_pallet_calibration.py \
  --label fit_center_140 \
  --fork-root-m 1.400 --lateral-m 0.000 --face-yaw-deg 0.000
```

Thay các con số bằng số đo thực tế; không cần kê pallet đúng một con số tròn. Dùng dấu chấm cho phần thập phân và đơn vị mét/độ. Chương trình lấy **30 kết quả hợp lệ, khác timestamp, tuổi không quá 500 ms**; bỏ kết quả lỗi và không ghi tư thế nếu chưa đủ mẫu sau 90 giây. Mỗi tư thế thường cần ít nhất khoảng 15 giây nếu node đạt 2 FPS hợp lệ. Nó in trung vị, sai số, độ lệch chuẩn và P95 sai số tuyệt đối; giữ lại toàn bộ mẫu hợp lệ để thấy độ dao động.

Mỗi lần chạy thành công nối một bản ghi vào:

```text
camera/detect_astra/data/calibration/pallet_samples.jsonl
```

Trường `median_error` là **số đo node trừ số đo thật**. Nếu số đo là 1,366 m và thực là 1,400 m, sai số là −0,034 m. Các giá trị 1,40 m cũ không được tự dùng lại vì xe/pallet có thể đã dịch chuyển.

## 5. Bộ tư thế đề nghị

Khoảng cách dưới đây tính từ gốc càng; điều chỉnh theo vùng mà node còn nhìn đủ hai chân và có Depth hợp lệ. Đo lại X/Y/góc ở mọi tư thế.

| Mục đích | Bố trí gợi ý | Nhãn |
|---|---|---|
| Ước lượng sai số theo khoảng cách | Thẳng mặt, gần trục giữa, khoảng 1,0 / 1,4 / 1,8 m | `fit_center_100/140/180` |
| Kiểm tra sai số theo vùng ảnh | Khoảng 1,4 m, lệch trái rồi phải khoảng 0,15 m, giữ mặt thẳng | `fit_left/right` |
| Kiểm tra sai số góc | Khoảng 1,4 m, xoay mặt hai phía khoảng +5° / −5° | `fit_yaw_plus/minus` |
| Kiểm tra độc lập | Khoảng 1,2 / 1,6 m, rồi một tư thế vừa lệch ngang vừa xoay | `check_center_120/160`, `check_mixed` |

Các tư thế `check_*` được dành để kiểm tra sau khi chọn hệ số; không dùng để chọn lại hệ số rồi coi đó là kiểm tra độc lập. Ghi lặp một tư thế sau khi di chuyển pallet ra rồi kê lại để kiểm tra khả năng lặp lại của cả bố trí và phép đo.

## 6. Chọn cách hiệu chỉnh sau khi có dữ liệu

- Sai số ổn định ở các khoảng cách: kiểm tra mốc cơ khí và khả năng có độ lệch cố định.
- Sai số tăng theo khoảng cách: xem xét hệ số tỷ lệ hoặc mô hình Depth; cộng một hằng số không giải quyết được.
- Góc sai thay đổi theo khoảng cách hoặc trái/phải ảnh: cần đánh giá lại bù Depth trái–phải hiện tại. Không mặc định trừ cùng một góc cho mọi tư thế.
- Sai số góc lắp camera cần hiệu chỉnh nhất quán vị trí X/Y và hướng mặt pallet. Chỉ đổi số góc trên giao diện sẽ làm mục tiêu điều khiển không nhất quán.
- Nếu độ dao động lớn, xem lại mask chân, bề mặt Depth và cách gá camera trước khi fit hệ số. Chọn mô hình đơn giản nhất cải thiện rõ các tư thế kiểm tra; báo cả sai số trung vị, P95 và trường hợp tệ nhất. Chưa có dữ liệu để cam kết độ chính xác theo cm/độ.

Lần đầu chỉ cần ghi ba tư thế `fit_center_*`; dựa vào kết quả đó để quyết định đo tiếp và chọn cách bù. Công cụ thu mẫu không tự áp dụng hệ số mới vào node, web hay URDF.

## Hồ sơ bù thực nghiệm ngày 2026-10-31

Đã thu 30 mẫu hợp lệ ở mỗi khoảng cách gốc càng 1,195 / 1,400 / 1,800 m, cùng mốc lệch phải 4,5–5 cm. Người đo xác nhận dịch chuyển gần song song, giữ hướng mặt pallet. Tâm hai bánh thực đo 0,423 m; bên trái xe (bên phải người đứng đối diện xe) xa hơn 1–2 mm, suy ra góc mặt khoảng +0,135 đến +0,271°. Các giá trị giữa khoảng được dùng làm tham chiếu danh định; không coi thước đã xác định chính xác Y và góc tới mọi chữ số.

Hồ sơ `config/calibration/pallet_target_bias_2026_10_31.yaml` dùng trung bình sai số trung vị của ba tư thế, với trọng số bằng nhau:

- Khoảng tiến X: **+0,0384298849 m**.
- Ngang Y (trái xe dương): **+0,0205314774 m**.
- Góc mặt pallet: **+0,896271969°**; góc hướng đưa càng luôn bằng âm góc mặt đã bù.

Kiểm tra ở tư thế 1,590 m riêng, không dùng để chọn lại hệ số: sai số trung vị khoảng tiến còn −5,54 mm, P95 trị tuyệt đối 6,14 mm; sai số ngang danh định +1,24 mm nhưng mốc thước Y còn bất định ±2,5 mm; góc sai danh định +0,168°, P95 0,704°, chưa gồm bất định mốc góc khoảng ±0,068°. Đây là kết quả trên một tư thế kiểm tra với pallet gần thẳng, không chứng minh độ chính xác toàn vùng nhìn hoặc khi pallet xoay nhiều. Ở khoảng 1,0 m không có đủ Depth hợp lệ trên chân pallet để thu mẫu.

Ở tư thế kiểm tra, model nhận thêm một vùng chân thành pallet thứ hai. Lần kiểm tra trên dữ liệu đã dùng ngưỡng confidence 0,75 thay cho 0,60 để thu được một mục tiêu. Khi xác minh trực tiếp sau đó, mục tiêu chính có lúc chỉ đạt khoảng 0,69 nên ngưỡng 0,75 làm mất phát hiện. Cấu hình chạy hiện tại trở lại confidence 0,60 và dùng `--nms-iou 0.30` để loại các khung nhận diện chồng lên cùng pallet. Điều kiện chỉ chấp nhận đúng một pallet, các phép kiểm tra hình học, đăng ký và tuổi kết quả không thay đổi. Hai pallet tách rời vẫn tạo hai kết quả và bị từ chối đo; giảm ngưỡng IoU cũng có thể loại một đối tượng khi nhiều pallet chồng nhau trong ảnh, nên cần đánh giá riêng tình huống đó.

Xác minh trực tiếp sau khi bật hồ sơ, tại cùng tư thế 1,590 m: 30 mẫu hợp lệ có khoảng gốc càng trung vị 1,587492 m (chênh −2,51 mm so với số thước cung cấp; P95 3,31 mm), lệch phải 4,643 cm, góc mặt −0,021° (sai khác danh định −0,224°, P95 0,702°). Mọi mẫu đều có dấu `target_bias_applied`. Đây là kiểm tra thực thi trên cùng tư thế, không phải một tư thế độc lập bổ sung; số thước và giả thiết giữ hướng vẫn có bất định. Web đã kiểm tra nhận cùng timestamp ảnh/phép đo và hiển thị trạng thái bù. Chi tiết lưu ở `data/calibration/live_bias_verify_1590.json` và `live_bias_web_verified.json`.

Áp dụng hồ sơ vào node đo:

```bash
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 --max-fps 2 --conf 0.60 --nms-iou 0.30 \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml
```

Bỏ `--measurement-bias-yaml` để chạy số đo trước bù. Hồ sơ được nạp một lần khi khởi động. Đây là bù phần dư của vị trí/góc **sau** quy trình RGB + Depth; nó không thay thông số Depth thô, TF camera, URDF hay odometry. Hai giá trị forward chân trái/phải được cập nhật thành ước lượng mặt phẳng đã bù, giữ đúng trung điểm và góc mới. Sample count, độ rộng lỗ và các phép kiểm tra Depth vẫn từ dữ liệu gốc của quy trình hiện tại.

Web đọc trực tiếp số đã bù từ `/pallet/target`, tính lại khoảng cách và góc hướng tới tâm lỗ từ X/Y mới; dòng trạng thái ghi “Đã áp dụng bù thực nghiệm khoảng cách/ngang/góc”. Công cụ ghi mẫu lưu `source_status`, có dấu `target_bias_applied`; khi fit hệ số từ đầu phải dùng mẫu trước bù để tránh cộng bù hai lần.
