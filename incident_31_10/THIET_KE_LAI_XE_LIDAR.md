# Thiết kế "người lái" cho xe (bản 2, 31/10/2026 ~01:50, ở nhà)

Thay bản 1. Bản 2.3 (31/10 trưa) thêm mục 9 (bảng tuyến tiếp tuyến). Bản 2.2 (31/10 ~02:40) thêm mục 8 (mô phỏng cả hệ thống) và quyền phủ quyết; bản 2.1 (~02:10) thêm hướng A: phân biệt địa hình với vật lạ bằng bản đồ (mục 3b). Bản 1 tách "bị chặn phía trước" với "kẹt hông" theo nhãn vùng; bản 2 làm theo đúng chính sách 5 bước Thiên chốt lúc 01:10 và đã có mã thử được trong mô phỏng.
Loại: **Core** (Thiên nâng lên Core 31/10), phục vụ cổng **G3** (dừng trước vật, không đâm) và **G5** (chạy ổn định, không cần người can thiệp).
Trạng thái: mã đã viết (`lai_xe_lidar.py`, kèm hướng A), thử trong mô phỏng và với ROS giả. **Chưa chạy trên `Pi`, chưa thử trên xe.**

## 1. Chính sách 5 bước (Thiên chốt 31/10 01:10, thay quyết định 30/10 "bỏ lùi, chờ 60 giây")

| Bước | Xe làm gì | Do thành phần nào | Trạng thái |
|---|---|---|---|
| 1 | Đi theo đường thoáng nhất | Nav2 + bản đồ chi phí (như cũ) | có sẵn |
| 2 | Vàng/đỏ: không đi về phía vật | `lai_xe_lidar.py` (đỏ = cách mép thân xe không quá 8 cm [đề xuất]) | mã xong, chưa thử trên xe |
| 3 | Bị chặn phía trước (xe đứng yên, mũi cách vật ≤ 15 cm): nếu vật là vật lạ (người, hộp, tấm chắn không có trên bản đồ) thì đứng chờ: 30 giây là của cây `bt_cho.xml` (đồng hồ chính); nút người lái chờ 40 giây rồi mới xen vào (dự phòng), để hai đồng hồ không cộng dồn; nếu là địa hình (tường, hộp cố định có trên bản đồ) thì thoát ngay | `lai_xe_lidar.py` giữ lệnh 0; cây `bt_cho.xml` chờ ≈ 30 giây (8 lần thử) | mã xong; đổi 15 → 8 là một lệnh `sed` |
| 4 | Hết 30 giây, hoặc kẹt hông/góc: thoát bằng LiDAR (xoay về hướng thoáng; lùi/tiến thẳng; đi cung ra xa tường), rồi đi đường khác | `lai_xe_lidar.py` thoát; `chuyen.sh` (TRY=1) đổi tuyến | mã xong, chưa thử trên xe |
| 5 | Hết đường: dừng tại chỗ, chụp ảnh phía trước, báo người phụ trách, hiện nút lái tay | `chuyen.sh` (đã in `HET_DUONG`) + ảnh + trang `teleop_web` | **chưa làm** (mục 6) |

## 2. Nguyên tắc

1. Cảm biến → vùng → hành động, mỗi lần LiDAR quét xong (khoảng 12 lần/giây).
2. Mọi chuyển động gần tường phải được LiDAR kiểm ngay lúc đó, không dựa vào lệnh hay odom.
3. Hỏng thì về trạng thái cũ: nút mặc định CHỈ NHÌN (`phat_lenh` = false); tắt nút thì xe chạy như hôm nay.
4. Một lần đổi một thứ; có công tắc khi đang chạy (`ros2 param set`).
5. Không làm gì khi không chắc: không có lối thì "bỏ cuộc", nhả quyền cho Nav2, không nhúc nhích bừa.

## 3. Cách nhận biết "kẹt" (đã sửa sau khi mô phỏng bắt được lỗi bản đầu)

Không dùng nhãn vùng (tường chạy dọc hông làm hai góc tưởng đỏ). Dùng quãng đường thoáng theo hướng đi, đo từ MÉP thân xe (rộng bằng thân xe + 2 cm hai bên):
- **F** bị chặn phía trước: quãng thoáng khi tiến ≤ 8 cm (cần ít nhất 2 điểm LiDAR để loại nhiễu, và 2 lần quét liên tiếp).
- **B** bị chặn phía sau: quãng thoáng khi lùi ≤ 8 cm.
- **H** sát hông: khoảng cách mép hông tới vật ≤ 8 cm. Chỉ tính là KẸT khi xe không tiến được (Nav2 không gửi lệnh tiến hay xoay) liên tục 30 lần quét (khoảng 2,5 giây); xe đang đi lướt qua góc thì không.
- Chỉ **F** (người, hộp, tấm chắn, kể cả rộng hơn xe) ⇒ **chờ 30 giây**, rồi thoát. Có **B** hoặc **H** (kể cả F kèm H) ⇒ thoát ngay.

### 3b. Phân biệt địa hình với vật lạ (hướng A, Thiên chọn 31/10 01:29)

Khi chỉ phía trước bị chặn, nút lấy các điểm LiDAR đang chắn mũi (cách mép mũi ≤ 12 cm), đổi sang toạ độ bản đồ bằng vị trí xe (tf `map` → `base_footprint`), rồi hỏi bản đồ `/map` (ô 5 cm, cho lệch 1 ô):
- từ 70% điểm trở lên nằm trên ô bản đồ chiếm ⇒ **địa hình** ⇒ thoát ngay, không chờ 30 giây;
- ngược lại ⇒ **vật lạ** ⇒ chờ 30 giây.
- Không biết (chưa nhận bản đồ, không có tf, tf cũ hơn 1,0 giây) ⇒ coi là vật lạ (chờ). Cả hai kiểu nhầm đều lành: nhầm tường thành vật lạ chỉ tốn 30 giây; nhầm người thành tường thì xe quay đi chỗ khác, không lao vào người.
- Độ nhạy theo sai số vị trí xe (tường mỏng 5 cm, thử giả): lệch ≤ 7 cm vẫn nhận đúng là địa hình; từ 10 cm trở lên thì coi là vật lạ [thử giả, chưa thử trên xe]. Người dày hơn 5 cm, đứng cách tường hơn 5 cm: mặt trước của người không trùng tường ⇒ vẫn là vật lạ.
- Bật/tắt khi đang chạy: `ros2 param set /lai_xe_lidar dung_ban_do false`.
- Phụ thuộc vị trí đúng, nên giá trị của nó tăng khi việc 90 (vị trí sai) được chữa; ngày nào `tf` cũ thì nút ghi cảnh báo `tf map->base_footprint cu ... s`.

## 4. Cách thoát (bước 4), mỗi bước nhỏ, kiểm LiDAR sau mỗi lần quét

| Trường hợp | Hành động | Xong khi |
|---|---|---|
| F, xoay tại chỗ an toàn (mọi điểm LiDAR cách tâm xe hơn 0,1855 m) | xoay về hướng thoáng nhất (ưu tiên ít quay, xa vật gần nhất); khi đã hướng đúng thì tiến | quãng thoáng phía trước ≥ 0,25 m |
| F, không xoay được (hành lang hẹp) | lùi thẳng (v = −0,10 m/s) | quãng thoáng phía trước ≥ 0,12 m |
| B | tiến thẳng | quãng thoáng phía sau ≥ 0,12 m |
| H (một bên sát, bên kia còn ≥ 0,16 m) | đi cung ra xa tường (v = ±0,10 m/s, ω = 0,3 rad/s, hướng quay ngược phía tường) | mép hông ≥ 0,12 m |
| F và B cùng lúc; hai bên hông cùng sát; mép nào ≤ 2 cm; không đủ chỗ; quá 10 giây | **bỏ cuộc**: nhả quyền, nghỉ 20 giây | |

## 5. Kết quả thử (ở nhà, mô phỏng; không phải trên xe)

- 26/26 phép thử đạt (20 ca gốc + 6 ca hướng A): thoáng; chặn trước 6 cm (chờ đủ 30 giây rồi xoay 155°); sát hông trái 5 cm (cung ra xa, hông cách tường 13 cm); hành lang hẹp + chắn trước (lùi 7 cm); bít kín bốn phía (không nhúc nhích); vàng không can thiệp; không có đích không can thiệp; góc trong; hành lang hẹp hai bên; một điểm nhiễu.
- 40 ca ngẫu nhiên sát vật (khe 1,5–14 cm): 0 ca chạm; xe tự di chuyển ở 17 ca, bỏ cuộc 0 ca.
- Hướng A (ca 13a–13f): tường có trong bản đồ ⇒ thoát ngay (t = 0,1 s); người không có trong bản đồ ⇒ vẫn chờ ≥ 30 giây; vị trí lệch 15 cm hoặc không có bản đồ ⇒ coi là vật lạ, chờ ≥ 30 giây.
- Nút ROS thử với ROS giả: chỉ nhìn không gửi lệnh nào; `phat_lenh` bật mới gửi; hết đích hoặc `enabled` = false thì im; mất `/scan` khi có đích thì gửi lệnh dừng.
- Mô phỏng lý tưởng: không trễ, không trượt bánh, không duty tối thiểu. **Xe thật có thể thoát chậm hơn hoặc lệch hơn.**

## 6. Việc chưa làm và giới hạn đã biết

1. **Bước 5 (ảnh + thông báo + nút lái tay): chưa có mã.** Cách nhỏ nhất: `chuyen.sh` khi in `HET_DUONG` ghi tệp `~/trang_thai_xe.json` và chụp một ảnh; `teleop_web.py` đọc tệp đó và hiện thông báo kèm ảnh trên trang (trang lái tay có sẵn nên chỉ cần hiện khung báo). Cần sửa `teleop_web.py` và `index.html` (mã trong kho). Chưa kiểm `Pi` có lệnh chụp ảnh khi `camera_csi` đang tắt hay không.
2. Phân biệt địa hình với vật lạ (mục 3b) đã có mã nhưng chưa thử trên xe: cần `/map` và `tf` đúng. Vị trí lệch từ 10 cm trở lên thì nút coi tường là vật lạ và chờ 30 giây (lành, chỉ chậm).
3. Lùi dài ra khỏi hành lang hẹp: nút chỉ lùi tới khi mũi cách vật 12 cm rồi trả quyền. Không tự lùi hết hành lang.
4. Vật cách mép hông dưới khoảng 2 cm KHÔNG hiện trong `/scan` (`range_min` 0,10 m, LiDAR cách mép hông 8 cm) [tính].
5. Cung ω = 0,3 rad/s có thể không đạt trên xe thật (duty tối thiểu, trễ khởi động 0,5 s khi quay). Nút kiểm LiDAR mỗi lần quét nên không "đi mù", nhưng có thể chậm.
6. Chưa kiểm `/map` (kiểu QoS giữ lại) và `tf` trên `Pi` thật; chưa kiểm chủ đề `/navigate_to_pose/_action/status` và kiểu QoS trên `Pi` thật; nếu sai, nút thấy "không có đích" và không can thiệp (an toàn nhưng vô dụng).
7. Chưa đóng gói vào kho/launch: chạy tay `python3 ~/lai_xe_lidar.py` trong lúc thử.
8. Ngưỡng 8/12/25 cm, 30 giây, 10 giây là khởi điểm [đề xuất], chưa đo trên xe.

## 7. Kế hoạch (mỗi bước một thứ, có đường lui)

| Khi | Việc | Đường lui |
|---|---|---|
| 31/10, sau Bài 1 | **Bước thử 1: chỉ nhìn** (motor OFF): chép tệp, chạy, đẩy xe bằng tay sát tường, so số `TRUOC=`, `TRAI=`... với thước | tắt nút |
| 31/10 hoặc 1/11 | **Bước thử 2:** đổi chờ 60 → 30 giây (`sed`), bật `phat_lenh`, thử bài 3 (người/ống) và bài 2b, tay sát công tắc | `ros2 param set /lai_xe_lidar phat_lenh false`; `cp ~/bt_cho.xml.60s ~/bt_cho.xml` |
| 1/11 | Bước thử 3: ca kẹt thật (hông, góc), đếm x/10 | như trên |
| 1/11 ở nhà | Bước 5: thông báo + ảnh + nút lái tay (mã mới trong `teleop_web`, thử bằng dữ liệu giả) | không dùng |
| 2/11 | đóng băng; đếm x/10 | quay về bản không có nút |
| 3/11 | chấm trước thầy khoa | |

Rủi ro: thêm một thành phần khi hệ cũ chưa ổn; còn 3 ngày; mọi số đo trên xe thật còn thiếu. Nếu bước thử 1 cho thấy số LiDAR lệch thước hơn 1,5 cm hoặc nút bắt trạng thái sai, dừng ở đó và giữ "báo + lái tay".

## 8. Mô phỏng cả hệ thống đêm 31/10 và những gì nó tìm ra

Công cụ: `mo_phong_he_thong.py` (xưởng gần đúng ±5–10 cm từ bản đồ; xe vi sai lý tưởng; LiDAR 410 tia; tuyến và `TRY=1` của `chuyen.sh`; người lái thật; **Nav2 đồ chơi**: đi thẳng về điểm, dừng khi trước mặt không thoáng, bỏ cuộc sau 30 giây, có kiểm va chạm 1 giây như RPP, nhưng KHÔNG lập đường né vật, KHÔNG có SLAM, KHÔNG trượt bánh).
**Kết quả chỉ nói về cách các lớp phối hợp, không nói xe thật chạy ổn định hay không.**

Bốn điều mô phỏng tìm ra, và đã sửa:
1. **Hai đồng hồ chờ cộng dồn.** Nav2 chờ 34 giây rồi bỏ cuộc, người lái bắt đầu một lượt chờ 30 giây khác: xe đứng khoảng 64 giây. Sửa: một đồng hồ chờ (cây hành vi 30 giây; người lái chờ 40 giây dự phòng, không đặt lại đồng hồ giữa hai đích liên tiếp).
2. **Người lái giữ lệnh 0 khi Nav2 đang xoay để thoát.** Việc đó chặn đúng động tác cứu xe. Sửa: chỉ giữ lệnh 0 khi xe đang đứng yên (không có lệnh tiến hay xoay).
3. **Xe sát tường 4 cm cần quay đầu thì quay tại chỗ quét vào tường** (RPP không kiểm va chạm lúc quay [trí nhớ]). Sửa: quyền phủ quyết lệnh xoay khi có vật trong bán kính quét 17,5 cm; bị chặn quá 2,5 giây thì đi cung ra xa tường rồi nhả.
4. **Người lái can thiệp cả lúc xe chỉ đi sát góc tường.** Bản sửa đầu dùng mốc thời gian 20 lần quét (khoảng 1,7 giây; nay mặc định 30 lần, khoảng 2,5 giây, Thiên đồng ý 31/10), nhưng tính lại [tính]: thân xe dài 0,25 m lướt qua một góc tường ở 0,092–0,15 m/s mất 1,7–2,7 giây trong vùng sát hông, nên mốc 1,7 giây KHÔNG lọc được lần lướt qua đầy đủ. Bản sửa đúng: kẹt = sát hông/sau VÀ xe không tiến được (Nav2 không gửi lệnh tiến hay xoay) liên tục 30 lần quét (khoảng 2,5 giây); xe đang đi hoặc đang xoay thì không phải kẹt. Mô phỏng: tuyến 1 (cả bảng cũ và mới) người lái xen vào 0/20 chuyến.

Và một phát hiện về thiết kế tuyến (đề xuất sửa `chuyen.sh`, chưa áp dụng): Nav2 xem là "tới điểm" khi vào vòng dung sai 15 cm, nên xe dừng ở rìa vòng phía xe đi tới (số thật: dừng cách 13–15 cm [log]). Điểm `(1,45; 0,13)` làm xe vào miệng hành lang cao hơn tâm khoảng 10 cm, sát góc H0. [ĐÃ THAY ở mục 9, bản tiếp tuyến] Bảng điểm đề xuất đêm 30/10 (dời điểm lên tâm hành lang), kết quả mô phỏng (20 lần mỗi dòng): tuyến 1 khe nhỏ nhất 8,2 cm thay vì 5,9 cm; tuyến 2 tới nơi 20/20 thay vì 18/20, khe nhỏ nhất 5,1 cm thay vì 0,5 cm.

Còn lại trong mô phỏng: khe nhỏ nhất vẫn có lúc 4–6 cm ở vài ca (ví dụ vào hành lang sát K4, hoặc mũi sát tường 6 cm); không ca nào chạm.


## 9. Bảng tuyến tiếp tuyến (Thiên chốt 31/10 12:10; thay phần "Bảng điểm tuyến mới" của mục 8)

**Vì sao.** Thiên nghi xe kẹt một phần vì đường dẫn có quá ít chấm nên góc lệch giữa các chấm lớn. Đọc lại bảng tuyến đêm 30/10 thì phát hiện đúng điều đó, nhưng nguyên nhân nằm ở HƯỚNG gắn với mỗi chấm chứ không chỉ số chấm: mỗi chấm mang một hướng cố định (hướng cuối), không phải hướng đi tới chấm. Nav2 xoay tại chỗ khi lệch > 0,785 rad (45°) lúc xuất phát, hoặc lệch > 0,3 rad (17,2°) lúc tới điểm [cấu hình đọc được; cách bộ bám đường dùng hai ngưỡng này là hiểu biết chưa kiểm trên xe, ≈ 75%]. Bảng cũ buộc xoay ≈ 102°, 78°, 175° (tuyến 1) và ≈ 158°, 22°, 67°, 113° (tuyến 2). Khớp với quan sát 30/10: B→A 2/5, xe "xoay vô định, vào góc chết".

**Quy tắc thiết kế.** (1) Hướng mỗi chấm = hướng đi tới chấm đó (tiếp tuyến). (2) Mỗi chỗ rẽ giữa hai chặng ≤ 36° (thêm chấm); ngưỡng 45° của bộ bám đường trừ biên 9° vì đường thật do bộ lập đường vẽ có thể lệch vài chục độ so với đường thẳng. (3) Chấm cách nhau ≥ 30 cm (dung sai 15 cm: xe dừng ở rìa vòng, chấm kế gần hơn 15 cm sẽ "tới" ngay). (4) Chấm cách tường và hộp ≥ 20 cm theo mô hình hình học xưởng (H0, K4, K2, tường; đọc từ ảnh bản đồ, ± 5 cm). (5) Hướng cuối ở A không cần +x (Thiên: tạm không chấm). (6) `di_B` 1 giữ nguyên vì A→B đã 11/11 (29/10): "không đổi cái đang chạy ổn".

**Còn một lần xoay lớn ở B'** (≈ 120° tuyến 1, ≈ 167° tuyến 2): B' là ngõ cụt, xe tới hướng +x rồi phải quay đầu. Chỉ tránh được nếu cho xe lùi ra, mà đó là một thay đổi lớn hơn (xếp Stretch, chưa làm).

**Bảng (x y z w, z và w của quaternion, hướng = hướng đi tới điểm):**

| Tuyến | Điểm | Hướng (độ) | Chỗ rẽ so với chặng trước |
|---|---|---|---|
| `ve_A` 1 | (1,30; 0,30) | −120,0 | lần xoay ở B' |
| | (1,00; 0,10) | −146,3 | 26° |
| | A | −174,3 | 28° |
| `ve_A` 2 | (1,30; 0,75) | −166,9 | lần xoay ở B' |
| | (0,45; 0,76) | 179,3 | 14° |
| | (0,15; 0,60) | −151,9 | 29° |
| | (0,00; 0,30) | −116,6 | 35° |
| | A | −90,0 | 27° |

`di_B` 2 (dự phòng, A→B' qua hành lang trên) đảo ngược theo cùng quy tắc: 5 điểm, hướng cuối tại B' giữ 0°.

**Mô phỏng (Nav2 đồ chơi, KHÔNG phải Nav2 thật; `so_sanh_tuyen.py`, 20 lần mỗi dòng).** Tuyến 1: bảng cũ 20/20, khe nhỏ nhất 5,9 cm, 31 giây; bảng tiếp tuyến 20/20, 8,6 cm, 27 giây. Tuyến 2: bảng cũ 18/20, 0,5 cm; bảng tiếp tuyến 20/20, 4,9 cm. Ghi chú: đồ chơi xoay tại chỗ khi lệch > 45° và khi tới điểm, giống ngưỡng cấu hình, nhưng không có bộ lập đường né vật và không có bộ bám đường thật. Kết quả chỉ nói bảng mới không tệ hơn trong mô hình đó.

**Đánh đổi.** Nhiều điểm hơn = nhiều chuyến bắt đầu hơn (tuyến 1: 2 → 3 điểm; tuyến 2: 3 → 5). Lỗi `0 poses` và `SUCCEEDED` tức thì (việc 90) xảy ra lúc bắt đầu chuyến, nên số lần hỏng có thể tăng nếu xác suất mỗi chuyến không nhỏ. Cách khác không cần dừng ở mỗi chấm: `NavigateThroughPoses` -(một chuyến đi qua nhiều điểm)-, nhưng đổi hành động và cây hành vi. Xếp Stretch, chưa làm.

**Phép thử ở trường.** Lô 1: `BANG=cu bash ~/chuyen.sh ve_A` (3 vòng). Lô 2: `bash ~/chuyen.sh ve_A` (3 vòng). Quan sát duy nhất: ở B' xe xoay một lần rồi không xoay thêm ở các điểm sau. Đạt khi x/n của `ve_A_tt` không thấp hơn `ve_A_cu` và không có xoay thêm; ngược lại dừng, giữ dữ liệu.

**Loại và cổng:** Core, G2 (B→A ổn định). Chưa chạy trên xe.
