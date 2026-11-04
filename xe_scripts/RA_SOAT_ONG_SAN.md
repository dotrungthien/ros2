# Rà soát CHỈ ĐỌC: "ống đặt sẵn ⇒ HET_DUONG" (giai đoạn 70.8)

Bản kho 37268ff (= origin/dong-bo). KHÔNG sửa mã, KHÔNG commit. Không có dữ liệu 3/11 (bag, log, costmap) trong kho: các số [tinh] dùng bối cảnh của Thiên (xe dừng (0,39; 0,09), góc 29°, mũi cách ống ~10 cm) và GIẢ ĐỊNH ống nằm trên đường giữa khe dưới (y = 0,09).

Nhãn:
- **[doc code]**: đọc từ mã trong kho.
- **[tinh]**: tính tay hoặc tính bằng Python ngoài kho.
- **[Nav2]**: hành vi Nav2 Humble 1.1.20 theo trí nhớ mã nguồn. Hai chuỗi chữ "RegulatedPurePursuitController detected collision ahead!" và "Controller patience exceeded" có trong .so đã cài trên WSL. Chưa đối chiếu mã nguồn dòng-theo-dòng.

## 1) Lượt 1 của gui_duong tính đường trên costmap nào?

**[doc code]**
- gui_duong.py:277 nghe `/global_costmap/costmap` (OccupancyGrid). QoS: KeepLast 1, TRANSIENT_LOCAL, RELIABLE, nên nhận ngay bản Nav2 đang giữ.
- Mỗi tin mới thay `self.cm`. `ban()` (:335) đổi tin sang `TT.Ban.tu_luoi` (tinh_tuyen.py:88), chỉ đổi lúc cần và giữ đến khi có tin mới.
- Lượt 1: `cho_san_sang()` (:343) gọi `cho_tf_costmap`, chờ ≤ 15 s đến khi có costmap và tf map→base_footprint còn mới (≤ 1 s), rồi tính đường bằng `TT.tinh_duong(ban, …)`. Mỗi lượt sau cũng gọi lại `cho_san_sang()`, nên dùng costmap mới nhất lúc đó.

Ai xuất costmap và khi nào **[doc code]** (nav2_params_cho.yaml, `global_costmap`):
- Nút global_costmap của planner_server, `update_frequency 1.0`, `publish_frequency 1.0`, `always_send_full_costmap: true`. Tin dùng cho lượt 1 cũ nhất ≈ 1-2 s.
- Có 3 lớp: `static_layer` (bản đồ), **`obstacle_layer`** (nguồn `/scan`, `marking/clearing: true`, `obstacle_max_range 2.5`, `raytrace_max_range 3.0`), `inflation_layer` (`inflation_radius 0.20`, `cost_scaling_factor 3.0`). `track_unknown_space: true`, footprint 0,25 × 0,16 m.

Ống đặt trước khi bật có nằm trong costmap không:
- **CÓ** **[doc code + Nav2]**: từ lúc Nav2 active, mọi tia `/scan` chạm ống (A cách ống ~0,6 m < 2,5 m) đều đánh dấu ô LETHAL.
- Chỉ phần **cung mặt trước** mà LiDAR thấy mới được đánh dấu. Phía sau ống bị che, static_layer coi là trống.

gui_duong coi ô nào là "đỏ" **[doc code]**:
- tinh_tuyen.py:112-117 `pixel_tu_chi_phi`: ô có giá trị OccupancyGrid ≥ 98 là đỏ, Dijkstra không đi qua.
- **[Nav2 + tinh]** Bảng chuyển của costmap_2d: 254 → 100, 253 → 99, 1..252 → 1 + 97·(c−1)/251, nên ≥ 98 tương ứng chi phí ≥ 252. Đó là ô LETHAL cộng vòng inscribed: các ô cách ô vật ≤ 0,08 m, ở ô 5 cm là khoảng 1 ô kể cả đường chéo.
- Các ô ngoài vòng đó (chi phí ≤ 251) Dijkstra vẫn đi được, chỉ bị phạt K_GIA/d³.

Vì sao 3/11 lượt 1 lại có đường **[tinh]**:
- Khe dưới rộng ≈ 55 cm (mô phỏng, ±5-10 cm). Ống 18 cm ở giữa để lại mỗi bên ≈ 18-19 cm, trong khi thân xe rộng 16 cm, tức thực tế chỉ thừa 1-1,5 cm mỗi bên.
- Trên costmap 5 cm, bên tường và bên ống mỗi bên mất ~1 ô vì vòng inscribed. Còn 0 hay 1 dải ô trống là tuỳ ô lưới rơi vào đâu.
- Phần A (test_gui_duong, costmap_v3_020 + ống vẽ thêm vùng đỏ 8 cm): đường lách dưới ống ở y = −0,093, thân xe cách ống **1,3 cm**. Vùng đỏ 10 cm thì KhongCoDuong.
- Giả lập B1 (ống (0,73; 0,08)) thì KhongCoDuong, còn 3/11 thì có đường. Chênh nhau ±2 cm vị trí là đủ đảo kết quả.

## 2) RPP kiểm va chạm thế nào; phải cách ống bao nhiêu

Tham số **[doc code]** (nav2_params_cho.yaml, `FollowPath`):
- `use_collision_detection: true`, `max_allowed_time_to_collision_up_to_carrot: 1.0`.
- `desired_linear_vel 0.15`, `use_velocity_scaled_lookahead_dist: true`, `lookahead_time 2.0`, `min/max_lookahead_dist 0.18/0.6`.
- `use_rotate_to_heading: true`, `rotate_to_heading_min_angle 0.35`, `rotate_to_heading_angular_vel 1.0`, `max_angular_accel 3.2`, `allow_reversing: false`.
- `regulated_linear_scaling_min_speed 0.06`; controller_server `failure_tolerance 0.3`, `controller_frequency 10`.

Cách RPP kiểm **[Nav2]**:
- Kiểm trên **local_costmap**: khung odom, cửa sổ lăn 3 × 3 m, ô 0,05, cùng obstacle_layer `/scan` và inflation 0,20.
- `isCollisionImminent`:
  - (a) Kiểm **tư thế hiện tại** trước.
  - (b) Chiếu tư thế theo (v, ω) đã tính, bước = 1 ô / v (hoặc, khi xoay tại chỗ, bước góc 2·sin(0,025 / r_ngoại tiếp)/ω ≈ 0,30 rad / ω), đến khi tổng thời gian ≥ 1,0 s hoặc vượt khoảng carrot.
- Mỗi tư thế, `inCollision` lấy `footprintCostAtPose`, tức chi phí lớn nhất trên các ô mà **đường viền** footprint đi qua. Va chạm khi chi phí = **LETHAL (254)**; ô inscribed 253 không tính.
- Có va chạm thì báo "detected collision ahead!", controller_server phát 0. Sau 0,3 s vẫn còn thì báo "Controller patience exceeded" và FollowPath ra ABORTED. Ở 10 Hz điều này khớp với "collision ahead" ×4 trong bối cảnh của Thiên.

Tầm nhìn **[tinh]**:

| v (m/s) | tầm nhìn va chạm (v × 1,0 s) | carrot |
|---|---|---|
| 0,06 | 0,06 m | 0,18 m |
| 0,10 | 0,10 m | 0,20 m |
| 0,15 | 0,15 m | 0,30 m |

- Xoay tại chỗ: ω đầu tiên bị giới hạn gia tốc ≈ 0,32 rad/s (3,2 × 0,1), nên chỉ kiểm ~1 tư thế ở +0,3 rad. Khi ω = 1,0 rad/s thì kiểm tới ≈ 0,9 rad (~52°).
- Footprint quét khi xoay nằm trong hình tròn bán kính ngoại tiếp `hypot(0,145; 0,08)` = **0,166 m**.

Tư thế 3/11 **[tinh, giả định ống ở y = 0,09]**:
- Mũi giữa ở (0,517; 0,160), cách mặt ống 10,0 cm, suy ra tâm ống ≈ (0,693; 0,090).
- Góc **trước-phải** của thân xe ở (0,556; 0,090), cách mặt ống **4,8 cm**; góc trước-trái cách 16,7 cm. Tâm xe cách mặt ống 0,213 m.
- 4,8 cm < 1 đường chéo ô (7,1 cm), nên ô LETHAL của ống có thể trùng hoặc kề ô mà viền footprint đi qua.
- Hệ quả: (a) có thể báo va chạm **ngay tại chỗ**, đường mới hướng nào cũng bị huỷ. (b) Xoay tại chỗ về khe trên (hướng đường lệch ≈ 110°, > 0,35 rad, nên RPP xoay trước) thì góc trước-phải đi trên vòng 0,166 m, mà ống cách tâm 0,213 m (chỉ hơn 4,7 cm) nên cũng có thể bị huỷ.
- Kết luận: ăn khớp với "lượt 2-3 + tuyến 2 lượt 1-3 đều collision ahead ngay".

Cần cách bao nhiêu để đường mới (quay sang khe trên) không bị huỷ **[tinh]**:
- **Bảo đảm, bất kể ô lưới rơi đâu:** mọi điểm LiDAR chạm ống cách tâm xe ≥ 0,166 + 0,071 = **0,236 m**.
  - Ống thẳng trước mũi thì mũi cách mặt ống ≥ **9,1 cm**.
  - Tính theo điểm thân gần nhất (góc xe), khoảng hở ≥ **7,1 cm**.
- **Thường gặp** (ô rơi giữa): ≈ 0,166 + 0,035 = 0,20 m từ tâm, tức điểm thân gần nhất ≥ ~3,5-5 cm. 3/11 nằm đúng dải này: 4,8 cm.
- Phải đo **điểm thân gần nhất**, không chỉ mũi. Ở góc 29°, mũi 10 cm mà góc chỉ còn 4,8 cm.

## 3) Trong lúc chờ 30 s và giữa các lượt: ai thấy gì, ai ra lệnh gì

**gui_duong.py [doc code]**
- Lượt hỏng thì `cho_sau_hong` (:197) đứng yên ≥ CHO_HONG 30 s, kéo tới 60 s nếu phụ lái đang lái (`l1` ∈ RA_TIEN/RA_LUI/XOAY_P hoặc THOAT*, :36, :86) hoặc sếp đang giao đợt thoát (~/sep_thoat có mã mới).
- Trước mỗi lần gửi, `cho_truoc_khi_gui` (:175):
  - ~/sep_lenh DUNG thì dừng (mã 4);
  - CHO thì chờ 10 s, tối đa 2 lần;
  - phụ lái hoặc sếp đang làm thì chờ ≤ 30 s.
- Trong lúc chạy: phụ lái báo XOAY_XONG (/rosout, :296) thì huỷ đích, ghi TINH_LAI, tính lại (:227).
- gui_duong không phát tốc độ, không tự lùi.

**sep.py [doc code]**
- Đọc /scan, tf, costmap (`grid`), trạng thái đích (/follow_path/_action/status), /rosout của lai_xe_lidar, CSV.
- Ống ≤ 15 cm trước mũi, ≥ 2 điểm trong dải mũi và không có trên bản đồ thì `VAT_LA` (:193), TU_THE CHO 'VAT_LA ts'. Sau F4 (:701): ống xa B nên không phải TAI_B; sau 30 s thì DOI_HANH_LANG. Đây **chỉ là dòng ghi**.
- **LENH vẫn DI**: `di_hay_khong` (:346) không xét vật. Có VAT_LA thì **không xét `kep`** (:654), nên không giao thoát.
- Sếp chỉ giao RA khi phụ lái **vào CHO_SEP** trong lúc **đích còn chạy** và `ra_thoat = true` (:784). Lệnh đi qua tệp **~/sep_thoat** (:969-970, xoá ở :1122). Có thể theo ~/sep_lenh để DUNG gui_duong (khi het_luot, KHONG_TIN quá 15 s, Nav2 chết).

**lai_xe_lidar.py [doc code]**
- Chỉ "có đích" khi FollowPath đang chạy. 3/11 mỗi lượt chỉ chạy < 1 s vì bị huỷ ngay, nên gần như luôn ở `KHONG_CO_DICH` (:288) và không phát gì.
- Trong cửa sổ có đích, nếu mũi ≤ 15 cm và Nav2 phát 0 thì `CHO_n` (phát 0 trên /cmd_vel_avoid).
- Đồng hồ `t_truoc` **không đặt lại** giữa các đích khi vẫn bị chặn. Cộng dồn ≥ t_cho **40 s** (:335) thì ở cửa sổ có đích kế tiếp nó **tự THOAT** (`CHAN_TRUOC_QUA_40s`; `sep_ra_thoat` chỉ chặn KET_HINH_HOC và XOAY_BI_CHAN, không chặn ca này).
- THOAT vẫn chạy tiếp sau khi đích hết (:283 đứng trước :288), tối đa 10 s:
  - xoay về hướng thoáng nếu mọi điểm > R_XOAY 0,186 m từ tâm (:469). 3/11: ống ~0,21 m nên xoay được;
  - không xoay được thì THOAT_LUI (:491) chỉ khi LiDAR thấy sau thoáng ≥ 5 cm.
- **Lùi RA theo lệnh sếp:** phụ lái đọc ~/sep_thoat (`ra_bat_dau`, :337), làm RA_TIEN hoặc RA_LUI về phía thoáng (có kiểm LiDAR), rồi XOAY_P.
- **Ai báo "xong":** phụ lái, qua /rosout (tên trạng thái `RA_XONG`, `XOAY_XONG`, `XONG`, `BO_CUOC_*`). Sếp đọc để đếm lượt thoát. gui_duong chỉ đếm `XOAY_XONG`, rồi TINH_LAI.
- Ca 3/11 theo mã: không có CHO_SEP (vật chỉ ở phía trước, `red = {'F'}`, không KET_HINH_HOC), nên **sếp không giao RA**. Phụ lái có thể tự THOAT_XOAY sau 40 s cộng dồn; không có bag nên chưa biết 3/11 có xảy ra không.

**Ống để luôn khi xe đang chạy có ra cùng lỗi không [tinh + giả lập]**
- B1′ (đặt ống khi mũi còn 0,34-0,43 m): xe dừng **thẳng** (góc 2-3°), mũi 10,4-12,0 cm. Góc trước cách xa hơn mũi, tâm cách mặt ống ≈ 0,25 m > 0,236, nên lượt sau xoay hoặc đi đều không bị huỷ. Ở B1′ ống đã bị xoá, nên DEN_NOI.
- Nếu ống ở lại: lượt sau tính trên costmap có ống, rồi một trong hai:
  - (a) KhongCoDuong, sang tuyến 2, chỗ dừng đủ xa nên xoay được: dự đoán DEN_NOI (B1 cũ b1_1, b2 đi tuyến 2; tuyến 2 trong giả lập không ổn định, F2);
  - (b) còn khe 1 ô, gửi đường **lách sát ống**: RPP rẽ vào khe, dừng với một góc thân sát ống (như 3/11, góc 29°), rồi mọi lượt sau bị huỷ ngay, HET_DUONG.
- Vậy **cùng lỗi có thể xảy ra** khi ống đặt lúc xe đang chạy, miễn là có một lượt gửi đường lách khe. Đặt sẵn chỉ làm lượt 1 đã là đường lách ngay. Chưa thử ca "ống ở lại, đặt lúc chạy" trong giả lập.

## Gốc lỗi (tóm tắt)
- **G1:** tinh_tuyen chỉ tránh ô ≥ 98 (lethal + inscribed 8 cm), nên chấp nhận khe chỉ dư ~1 cm mỗi bên.
- **G2:** RPP dừng muộn khi đang rẽ vào khe; tầm nhìn 1 s × v nhỏ (0,06-0,15 m), nên góc thân dừng trong vòng 1 ô của ống.
- **G3:** khi đứng yên một góc thân trong vòng ~7 cm của ô ống, mọi đường mới đều bị huỷ ngay, kể cả xoay tại chỗ. gui_duong không có cách nào khác (không lùi), nên đi hết 6 lượt rồi HET_DUONG.
- **G4:** F1 làm nặng thêm: KhongCoDuong ở lượt 1 thì bỏ hẳn tuyến đó.

---

## 3a. Ngưỡng lớp 1: tâm xe không cách vật < 0,15 m (CHƯA SỬA MÃ, CHƯA COMMIT)

Script tính ngoài kho, trong scratchpad của phiên: `lop1/lop1.py`, `lop1/bien.py`, `lop1/ve.py`. Ảnh: `gia_lap/lop1_nguong.png` [tinh].

### 3a.1 Ngưỡng giá trị ô [tinh + mã Nav2 1.1.20]

Công thức:
- `InflationLayer::computeCost(distance)` (inflation_layer.hpp:148-162, tag 1.1.20):
  - d = 0 ⇒ 254;
  - d·res ≤ inscribed (0,08 = nửa bề ngang footprint) ⇒ 253;
  - còn lại ⇒ `(253−1)·exp(−3,0·(d·res − 0,08))`, cắt phần lẻ.
- Chỉ phồng tới `inflation_radius` 0,20 m (4 ô).
- `costmap_2d_publisher.cpp:91-99`: 253 → 99, 254 → 100, 1..252 → `1 + (97·(c−1))/251` (chia nguyên).

d là khoảng cách giữa **tâm ô xe và tâm ô vật** (lưới 5 cm), chỉ nhận các giá trị rời:

| d (m) | 0 | 0,05 | 0,0707 | 0,10 | 0,1118 | 0,1414 | **0,15** | 0,1581 | 0,1803 | 0,20 |
|---|---|---|---|---|---|---|---|---|---|---|
| chi phí | 254 | 253 | 253 | 237 | 229 | 209 | **204** | 199 | 186 | 175 |
| OccupancyGrid | 100 | 99 | 99 | 92 | 89 | 81 | **79** | 77 | 72 | 68 |

- "Không cho tâm cách vật **< 0,15 m**" ⇔ ô đỏ khi **OccupancyGrid ≥ 80**, tức chi phí ≥ 205. Ngưỡng 80 hay 81 cho cùng kết quả, vì không có ô nào bằng 80. Nếu muốn cấm cả đúng 0,15 m thì dùng ≥ 79.
- Ngưỡng cũ ≥ 98 tương đương tâm cách vật ≤ 0,0707 m (chỉ ô vật cộng vòng inscribed).
- Lưu ý: đây là khoảng cách **tâm ô–tâm ô**. Khoảng hình học thật có thể ngắn hơn tới ~3,5 cm (nửa đường chéo ô). Vậy ngưỡng 80 bảo đảm tâm xe cách mặt vật thật ≳ 0,115 m, tức thân bên cách ≳ 3,5 cm. Muốn chắc chắn 0,15 m hình học thì cần d ≥ 0,1854 m, tức ≥ 72 (cấm cả ô 0,1803).
- `costmap_v3_020.pgm` lưu **trinary** (0 / 205 / 254, ngưỡng 0,98 / 0,35), nên KHÔNG chứa được ngưỡng 80. gui_duong trên xe đọc OccupancyGrid đầy đủ (`Ban.tu_luoi` → `pixel_tu_chi_phi`, tinh_tuyen.py:112) nên dùng được. Còn test_gui_duong (dựa trên `T.Ban(T.YAML)`) sẽ cần nguồn dữ liệu khác.

### 3a.2 tinh_tuyen.tinh_duong với ngưỡng cũ và mới [tinh]

Costmap **dựng lại** từ `map_xuong_v3.png` (ô vật = điểm ảnh 0, chưa biết = 205) theo công thức trên, `inflate_unknown` false.
- Kiểm với `costmap_v3_020`: ô đỏ (≥ 98) khớp **1617/1664**, ô xanh khớp 1636/1664.
- 45 ô v3_020 đỏ hơn: 29 ô ở d = 0,10 (occ 92), 11 ô chưa biết, 5 ô occ 89. Đều ở mép tường, có lẽ do lớp vật cản /scan đánh dấu thêm. Vì vậy chạy thêm **ca biên**: coi 45 ô đó là ô vật (tường dày thêm ~1 ô).
- Ống: mọi ô có phần nằm trong hình tròn (0,73; 0,08) r 0,09 là ô vật (9 ô).
- "Cách vật" đo từ tâm xe (các điểm của đường) tới **mép ô vật** (hình vuông 5 cm).

| ca | tuyến | ngưỡng CŨ ≥ 98 | ngưỡng MỚI ≥ 80 | ca biên CŨ / MỚI |
|---|---|---|---|---|
| (a) không ống | di_B t1 (khe dưới) | có; 24,3 cm tại (0,54; 0,09) | có; 24,3 cm tại (0,54; 0,09) | 22,1 / 19,0 cm |
| (a) không ống | di_B t2 (khe trên) | có; 22,5 cm tại (0,56; 0,76) | có; 19,0 cm tại (0,74; 0,79) | 17,5 / 17,5 cm |
| (a) không ống | ve_A t1 | có; 24,3 cm tại (0,54; 0,09) | có; 24,3 cm | 22,1 / 19,0 cm |
| (a) không ống | ve_A t2 | có; 22,5 cm tại (1,16; 0,76) | có; 19,0 cm tại (0,74; 0,79) | 17,5 / 17,5 cm |
| (b) có ống | di_B t1 | **có, lách dưới ống**: tâm cách tường K4 7,5 cm tại (0,56; −0,09); tâm cách mặt ống 8,3 cm tại (0,72; −0,09), tức **thân cách ống 0,3 cm** | **KHÔNG có đường** ⇒ đổi tuyến | 6,7 cm / KHÔNG |
| (b) có ống | di_B t2 | có; 22,5 cm | có; 19,0 cm | 17,5 / 17,5 cm |
| (b) có ống | ve_A t1 | **có, lách**: 7,5 cm tại (0,72; −0,09) (thân cách ống 0,3 cm) | **KHÔNG** | 6,4 cm / KHÔNG |
| (b) có ống | ve_A t2 | có; 22,5 cm | có; 19,0 cm | 17,5 / 17,5 cm |

Kết luận [tinh]:
- Ngưỡng mới **giữ nguyên** mọi đường khi không có ống. Mọi đường mới đều cách vật ≥ 19,0 cm (ca biên ≥ 17,5 cm), tức ≥ 15 cm.
- Có ống: ngưỡng mới **đóng khe dưới** (không còn đường lách 0,3 cm), nên tinh_duong ra KhongCoDuong và gui_duong đổi sang khe trên ngay ở lượt 1, xe không tới gần ống.
- Lưu ý F1: KhongCoDuong ở lượt 1 thì bỏ hẳn tuyến 1.

### 3a.3 RPP Nav2 Humble có kiểm va chạm khi xoay tại chỗ không? **CÓ** [doc mã 1.1.20]

Nguồn: `nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp` ở tag 1.1.20 (= bản `ros-humble-nav2-*` 1.1.20 đã cài), tải từ raw.githubusercontent.com, 914 dòng.
- **:340-345**: `if (shouldRotateToGoalHeading(...)) rotateToHeading(...); else if (shouldRotateToPath(carrot_pose, angle_to_heading)) rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);`
- **:357-359**, chạy SAU nhánh xoay, cho mọi nhánh: `if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) { throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!"); }`
- **:386-397** `rotateToHeading`: `linear_vel = 0.0; angular_vel = sign * rotate_to_heading_angular_vel_;` rồi bị kẹp bởi `curr_speed.angular.z ± max_angular_accel_ * dt`.
- **:484-489** `isCollisionImminent`: kiểm **tư thế hiện tại** trước: `if (inCollision(robot_pose...x, ...y, tf2::getYaw(...))) return true;`
- **:500-509** nhánh xoay: `if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) { // rotating to heading ... double max_radius = ...getCircumscribedRadius(); projection_time = 2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel); }`
- **:523-546**: `while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_) { ... curr_pose.theta += projection_time * angular_vel; ... if (hypot(...) > carrot_dist) break; ... if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) { ... return true; } }`
- **:570-579** `inCollision`: `footprint_cost = collision_checker_->footprintCostAtPose(x, y, theta, getRobotFootprint()); ... return footprint_cost >= LETHAL_OBSTACLE;` Chỉ ô LETHAL 254 mới tính là va chạm; ô inscribed 253 thì không.
- controller_server.cpp **:488-502** (1.1.20): bắt PlannerException; `failure_tolerance_ > 0` ⇒ `RCLCPP_WARN(e.what())`, phát 0; quá `failure_tolerance_` (0,3 s) ⇒ `throw PlannerException("Controller patience exceeded")` ⇒ FollowPath ABORTED.

Số cho xe này [tinh]:
- r ngoại tiếp 0,166 m, ô 0,05, nên bước góc = 2·sin(0,025/0,166) = 0,300 rad.
- Chu kỳ đầu ω bị kẹp bởi gia tốc: 0 + 3,2·0,1 = 0,32 rad/s, nên projection_time = 0,94 s và chỉ kiểm hiện tại + 1 tư thế ở +0,30 rad (17°).
- Khi ω = 1,0 rad/s: kiểm hiện tại + 0,30 / 0,60 / 0,90 rad (17° / 34° / 52°).
- Xoay không dời tâm nên điều kiện `carrot_dist` không cắt vòng lặp.
- Vậy: xoay tại chỗ **bị huỷ** nếu viền footprint ở tư thế hiện tại hoặc các góc trên chạm ô LETHAL. Khớp với 3/11: góc trước-phải cách ống 4,8 cm (mục 2).

---

## 3a-2. So sánh ngưỡng 80 và 72 (CHƯA SỬA MÃ, CHƯA COMMIT)

Script trong scratchpad của phiên: `lop1/lop1.py` (8 ca), `lop1/bien.py` (ca biên), `lop1/quet.py` (quét ống, scipy `distance_transform_edt` cho khoảng cách ô), `lop1/ve2.py` (tổng hợp và vẽ). Ảnh `gia_lap/lop1_nguong.png` vẽ lại thành 2 hàng × 3 cột (98 | 80 | 72) [tinh].
Nhắc lại ý nghĩa: ≥ 72 nghĩa là cấm cả ô tâm cách vật 0,1803 m, tức tâm cách vật (tâm ô–tâm ô) ≥ 0,20 m; đây là mức bảo đảm ≥ 0,15 m hình học theo mục 3a.1.

### 3a-2.1 Tám ca với ngưỡng ≥ 72 [tinh]

| ca | tuyến | ≥ 98 | ≥ 80 | **≥ 72** | ca biên ≥ 72 |
|---|---|---|---|---|---|
| (a) không ống | di_B t1 / ve_A t1 | 24,3 cm | 24,3 cm | **24,3 cm** tại (0,54; 0,09) | 19,0 / 19,1 cm |
| (a) không ống | di_B t2 / ve_A t2 | 22,5 cm | 19,0 cm | **19,0 cm** tại (0,74; 0,79) | 17,5 cm |
| (b) ống (0,73; 0,08) | di_B t1 / ve_A t1 | lách, thân cách ống 0,3 cm | KHÔNG | **KHÔNG** | KHÔNG |
| (b) ống | di_B t2 / ve_A t2 | 22,5 cm | 19,0 cm | **19,0 cm** | 17,5 cm |

- **Không ca nào mất đường khi không có ống**, ở cả mô hình dựng lại lẫn ca biên tường dày thêm 1 ô.
- Có ống thì ≥ 72 giống ≥ 80: mất khe dưới, còn khe trên.
- Đường của 72 trùng đường của 80 trong cả 8 ca.

### 3a-2.2 Vì sao khe trên ngưỡng 80 cách vật 19,0 cm < ngưỡng cũ 22,5 cm [tinh + doc code]

- Tại x = 0,74, khe trên trống từ mép ô vật H0 (tâm ô (0,759; 0,507), mép 0,532) tới mép ô vật tường trên (tâm ô (0,759; 1,007), mép 0,982). Khe rộng 45 cm, giữa khe ở y = 0,757.
- **Ngưỡng 98:** đường đi đúng y = 0,757, cách đều hai bên 22,5 cm. Điểm gần nhất (0,559; 0,757), cách H0 22,5 cm.
- **Ngưỡng 80/72:** đỉnh rẽ ở đầu khe là **(0,41; 0,81)** thay vì (0,41; 0,76).
  - Đỉnh này do chính Dijkstra chọn, không phải do bị dời: vùng đỏ quanh góc trên-trái H0 (ô đỏ gần nhất (0,459; 0,607) ở ngưỡng 80) phình ra, phạt `K_GIA/d³` đẩy đường lên cao.
  - `doan_thang` (tinh_tuyen.py:237) vẽ **một đoạn thẳng** từ (0,41; 0,81) tới (1,56; 0,76), nên tại x = 0,74 đoạn thẳng ở y = 0,792, cao hơn giữa khe 3,5 cm, chỉ còn **19,0 cm tới tường trên**.
  - Tức là: không phải ngưỡng cho đi sát hơn, mà là đoạn thẳng nối hai đỉnh lệch nhau 5 cm. Vẫn ≥ 15 cm.
- **Phụ (doc code, tinh_tuyen.py:259-277):** luật "đỉnh rẽ cách ô đỏ > R_XOAY_CAN 0,2356 m" đo tới **ô đỏ**. Khi ô đỏ đã phình ra 0,14-0,20 m, luật thành "cách vật ≳ 0,38-0,44 m", điều mà khe 45 cm không thể đáp ứng.
  - Đỉnh (0,41; 0,81) chỉ cách ô đỏ 0,209 m (ngưỡng 80) / 0,161 m (ngưỡng 72), nên **không dời được**, `canh_bao` và đỉnh giữ nguyên.
  - Ở ngưỡng 98 đỉnh (0,41; 0,76) cũng đã < R_XOAY_CAN (0,226 m); đây chính là "CANH BAO: dinh re … khong doi duoc" thấy trong giả lập.
  - Nếu dùng ngưỡng mới, nên đo R_XOAY_CAN tới ô **vật** (≥ 99), không tới ô đỏ. Chưa sửa.

### 3a-2.3 Quét ống r 0,09 trên lưới 5 cm [tinh]

Cách quét:
- Tâm ống đặt ở tâm mỗi ô costmap có x 0,30-1,30 (phủ khe dưới, khe trên, hai đầu nối); đĩa ống không được chạm ô vật tĩnh. Tổng **300 vị trí**: khe dưới 113, khe trên 91, đầu/nối 96 (khe = 0,45 ≤ x ≤ 1,20, y < 0,40 hoặc > 0,50).
- Ô vật của ống = mọi ô có phần nằm trong đĩa. Costmap dựng lại như 3a (công thức Nav2 1.1.20).
- Mỗi vị trí, mỗi ngưỡng: `tinh_duong` di_B và ve_A × tuyến 1, 2. Đo khoảng cách nhỏ nhất từ **thân xe** (chữ nhật 0,145 / −0,105 / ±0,08 xoay theo hướng đoạn, lấy mẫu 1 cm dọc đường) tới **mặt ống**.

| ngưỡng | vị trí còn đường lách, thân cách mặt ống < 7,1 cm | trong đó thân chồng lên ống (< 0) | cặp (vị trí, đường) < 7,1 cm | nhỏ nhất | mất hết đường cả 2 khe (di_B / ve_A) |
|---|---|---|---|---|---|
| ≥ 98 | **166 / 300** (khe dưới 66, khe trên 60, đầu/nối 40) | 4 | 300 | **−4,1 cm** tại ống (0,509; 0,807), di_B t2 | 0 / 0 |
| ≥ 80 | **34 / 300** (khe dưới 14, khe trên 3, đầu/nối 17) | 0 | 57 | 0,8 cm tại ống (1,259; −0,043), ve_A t1 | 0 / 0 |
| ≥ 72 | **1 / 300** (đầu/nối) | 0 | 1 | 6,9 cm tại ống (0,309; 0,057), ve_A t2 | 0 / 0 |

- Ngưỡng 80, các vị trí còn lách tập trung ở **hai đầu khe và mép khe**:
  - dải y = 0,207 (x 0,31-0,76: ống sát mép trên khe dưới, ngay dưới H0);
  - y −0,04 đến −0,09 ở x 0,41-0,51 và 0,96-1,26 (sát mép dưới khe dưới, chỗ K4 nhô);
  - x 0,36-0,61 ở y 0,61-0,96 (đầu trái khe trên);
  - x 1,26 (đầu phải).
- **Không vị trí nào mất hết đường** ở cả 3 ngưỡng: ống chỉ chặn một khe, khe kia còn.

Giới hạn [tinh]:
- (1) Ống vẽ thành mọi ô chạm đĩa (9-12 ô). Trên xe chỉ cung mặt trước được đánh dấu, nên vùng đỏ nhỏ hơn và số "lách" thực tế có thể **cao hơn** bảng.
- (2) Chỉ là đường kế hoạch: chưa tính sai số bám của RPP và định vị (±4 cm giả lập).
- (3) "Lách" tính mọi đường đi qua gần ống, kể cả khi ống nằm cạnh góc rẽ.
- (4) Tường có thể dày hơn bản dựng lại ~1 ô (45 ô, mục 3a.2).

Tóm tắt [tinh]: ≥ 80 giảm vị trí lách từ 166 xuống 34; ≥ 72 xuống 1. Không ngưỡng nào làm mất đường khi không có ống, cũng không làm mất cả hai khe ở bất kỳ vị trí ống nào trong lưới.

---

## 3b. Viết lớp 1 (ngưỡng ≥ 72): CHƯA COMMIT (điều kiện 3 không đạt), chưa push, CHƯA CHẠY TRÊN PI

### 3b.1 Sửa (git diff: tinh_tuyen.py +33/−15, gui_duong.py 1 dòng, test_gui_duong.py +2 bài)

tinh_tuyen.py:
- `NGUONG_CAM, NGUONG_VAT = 72, 98`.
- `Ban.tu_luoi` thêm `self.cam` = các ô OccupancyGrid ≥ 72. `Ban(yaml)` (PGM trinary) có `cam = set()`, nên **BANG=xs và mọi bài thử dùng PGM không đổi**. Đã kiểm: `main()` mới và cũ in ra giống hệt 11 dòng.
- `Ban.do(k)` = ô vật **hoặc** ô cấm (dùng khi tìm đường). `Ban.vat(k)` mới = ô ≥ 98 (nghĩa cũ của `do`).
- `_khoang_cach`: `d` = khoảng cách tới ô cấm (giá Dijkstra); `d_vat` = tới ô vật.
- `cach_do` dùng `d_vat`, nên **luật dời đỉnh rẽ R_XOAY_CAN 0,2356 m, `san`, `cach_do_doan`, `canh_bao` đo tới ô CÓ VẬT** (yêu cầu 1).
- `dijkstra(..., tha)`: `chan = vat or (do and not tha)`; giá dùng `max(d, 0.5)` để ô `tha` (d = 0) không chia 0. Các ô thường có d ≥ 1 nên không đổi.
- `tinh_duong`: `tha` = ô trong R_THA_CAM 0,25 m quanh chỗ xe đứng.
  - **Thêm ngoài lệnh, báo rõ:** xe đứng trong dải cấm (ví dụ dừng 10-15 cm trước ống, hay sát tường) vẫn tính được đường ra. Không có `tha` thì Dijkstra kẹt ngay ở ô đầu, ra KhongCoDuong ở cả hai tuyến, rồi HET_DUONG. Ô có vật thì không bao giờ được đi.
- Kiểm `dich nam trong o do` vẫn dùng `do`, nên đích (A/B) bị cấm nếu cách vật < ~0,20 m (lớp 1 áp cho cả đích).

gui_duong.py:99:
- Kiểm SAI_VI_TRI "xe nằm trong ô đỏ" đổi `ban.do` thành **`ban.vat`**, giữ nghĩa cũ "định vị đặt xe vào vật". Không đổi thì xe đứng trong dải cấm lúc bắt đầu chuyến sẽ bị SAI_VI_TRI (mã 3).

test_gui_duong.py:
- Thêm `luoi_nav2` (OccupancyGrid giả theo công thức Nav2 1.1.20).
- `test_25`: occ 99 / 92 / 79 / 68 ở 0,05 / 0,10 / 0,15 / 0,20 m; `do`/`vat` đúng; `cach_do` đo tới ô ≥ 98; xe trong dải cấm vẫn ra được đường, ngoài 0,25 m quanh xe không ô nào ≥ 72; khe tâm 0,15 m: PGM-như-cũ có đường, ≥ 72 KhongCoDuong.
- `test_26`: xe ở ô 79 thì không SAI_VI_TRI, DEN_NOI; xe ở ô 99 thì SAI_VI_TRI (mã 3) như cũ.
- Hai câu kiểm của mình sai lúc đầu (occ ô 0,10 m là 92, không phải 99; ô vật ≥ 98 gồm vòng inscribed nên `cach_do` = 0,25, không phải 0,30), đã sửa câu kiểm.

Kết quả: test_gui_duong **26/26**, test_sep **44/44**, test_lai_xe_lidar **68/68**; py_compile OK.

### 3b.2 Kiểm tích hợp (chỉ đọc)

| bên ghi | bên đọc | chữ/tên phải khớp | khớp? |
|---|---|---|---|
| Nav2 costmap_2d_publisher (bảng chuyển 1.1.20) qua `/global_costmap/costmap` | gui_duong.py:340 `TT.Ban.tu_luoi(w, h, res, ox, oy, data)` | chữ ký hàm; data 0..100, −1 | khớp (chữ ký không đổi) |
| nav2_params_cho.yaml global_costmap `inflation_radius 0.20`, `cost_scaling_factor 3.0`, footprint ±0,08 | tinh_tuyen `NGUONG_CAM 72` (= chi phí 186 = tâm cách vật 0,1803 m) | tham số phồng | khớp; **đổi yaml (vd đường lui 0,25) thì phải tính lại 72** (ghi trong chú thích) |
| tinh_tuyen `Ban.do` (nghĩa mới: vật ∪ cấm) | dijkstra, doan_hop_le, tinh_duong (`dich nam trong o do`); test_gui_duong `khong_qua_ong` (PGM) | nghĩa | khớp (PGM: cam rỗng) |
| tinh_tuyen `Ban.vat` (mới) | gui_duong.py:99 SAI_VI_TRI | nghĩa cũ ≥ 98 | khớp (test_26) |
| `cach_do` (`d_vat`) → `canh_bao` (x, y, m) | gui_duong.py:125-126 in `CANH BAO: dinh re (..) chi cach o do %.2f m (< %.2f)`; VI_TRI_TREN_PI.md:214 | chữ log; số = khoảng tới ô ≥ 98 | khớp: chữ không đổi, nghĩa số như trước (giả lập: cùng cảnh báo `(0.36; 0.76) 0.22 m` như B1 03/11) |
| `R_XOAY_CAN = 0.1856 + 0.05` | lai_xe_lidar `R_XOAY = hypot(0.145, 0.08) + 0.02` = 0,1856 | hằng số | khớp (không đổi) |
| `KhongCoDuong` | gui_duong.py:121-124 `tuyen %d: %s => doi tuyen`, CSV `KHONG_DAT`, `break` (F1) | chữ, cột | khớp; lớp 1 làm KhongCoDuong xảy ra nhiều hơn (ống), nên F1 (bỏ hẳn tuyến) gặp nhiều hơn: B1 3/3 bỏ tuyến 1 ở lượt 1 |
| gui_duong CSV (`COT_CSV`, chữ kết quả), mã thoát 0/1/2/3/4/130 | sep.py `_csv`, chuyen.sh, cau_noi_xe.py `MA_CHUYEN`/`DONG_CHUYEN`, HUONG_DAN_THU lệnh x/n | không đổi | khớp |
| sep.py | (không import tinh_tuyen; dùng `/map` cho vat_truoc) | — | không ảnh hưởng |
| so_voi_pi.sh `tinh_tuyen.py\|~/tinh_tuyen.py`, `gui_duong.py\|~/gui_duong.py` | chép lên Pi | md5 đổi | phải chép **cả hai** tệp |

### 3b.3 Giả lập vòng kín (chưa hiệu chuẩn; HOME thật, domain 87, bat_gia mức B, như Q1 03/11)

Script ở scratchpad mới (`l1sim/`): ghi_vong.py, vong.sh, canh_ong.py, chay_het.sh, phan_tich.py. Các script và dữ liệu thô 03/11 đã mất khi /tmp bị dọn; số "trước" lấy từ HIEU_CHUAN_0311.md (Q1) và báo cáo phần B (B1).

**Q1 (3 vòng di_B + ve_A, không ống)**

| | trước (03/11) | sau lớp 1 |
|---|---|---|
| x/n DEN_NOI | 6/6 | **6/6** |
| tới ở lượt 1 | 6/6 | **6/6** (CSV KIEM_DAT, SUCCEEDED t1 l1, DEN_NOI) |
| mã thoát | 0 ×6 | 0 ×6 |
| cách đích (gui_duong) | 0,13-0,14 m | 0,13-0,14 m |
| thời gian di_B / ve_A | 25-26 s / 24-27 s | **32-33 s** / 25-29 s |
| CANH BAO | ve_A có `dinh re (1.26; 0.06) … 0.21 m` | ve_A 1 lần mỗi vòng `(1.26-1.31; 0.05-0.06) 0.22 m` |
| sếp KET_SAT_VAT / THOAT; phụ lái /cmd_vel_avoid | 0 / 0; 0 | 0 / 0; 0 |
| d_min thân → ô chiếm (xe_gia) | (thước đo cũ khác, script mất) | di_B 9,0-10,2 cm; ve_A 10,4-12,9 cm |

- di_B chậm hơn ~7 s: đứng 2 s + xoay 1 s ở A (như cũ), chạy 23-27 s, xoay 2 s ở B. Đường di_B nay 50 điểm (trước 48). Chưa tách được nguyên nhân (n = 3).

**B1 (ống đặt sẵn (0,73; 0,08), 3 lần; xoá ống 10 s sau khi xe dừng như 03/11)**

| lần | trước (03/11) | sau lớp 1 |
|---|---|---|
| 1 | DEN_NOI t2, 26 s | **HET_DUONG**, 83 s, mã 1: t1 KhongCoDuong, t2 l1 ABORTED, l2-l3 KHONG_DAT; phụ lái CHAN_XOAY → CHO_SEP → RA_TIEN; 38 tin avoid (7 khác 0); thân → ô chiếm 4,2 cm |
| 2 | DEN_NOI t2 (TU_CHOI, TINH_LAI), 68 s | **DEN_NOI t2**, 25 s, mã 0 |
| 3 | HET_DUONG, 82 s | **HET_DUONG**, 86 s, mã 1: TINH_LAI, CANCELED, ABORTED ×2; CHAN_XOAY → CHO_SEP → RA_LUI → XOAY_XONG …; 81 tin avoid (18 khác 0); thân → ô chiếm 3,5 cm |
| x/n | **2/3** DEN_NOI | **1/3** DEN_NOI |

- Đi khe trên từ lượt 1: 3/3 (đúng mong). Như trước: giả lập 03/11 cũng ra KhongCoDuong tuyến 1 ngay lượt 1.
- Chữ log, mã thoát: cùng bộ chữ (`tuyen 1: khong tim duoc duong … => doi tuyen`, `CANH BAO: dinh re (0.36; 0.76) chi cach o do 0.22 m`, `HET_DUONG`, mã 1 / 0). Không chữ mới.
- Hai lần hỏng là **khe trên**, cùng chuỗi với b1_3 trước khi sửa (F2): xoay bị phụ lái chặn (CHAN_XOAY, vật trong 0,176 m quanh tâm, xe dừng ở (0,90; 0,85), (0,76; 0,84), tức ~13 cm từ mép tường trên), CHO_SEP, RA, XOAY, ABORTED.
- Đường đầu khe trên lượt 1 có cùng cảnh báo và cùng đỉnh (0,36; 0,76) như trước, nên nhiều khả năng không đổi. Khác biệt 1/3 so với 2/3 với n = 3 chưa tách được khỏi độ dao động của F2. Mục 3a-2 cho thấy với ≥ 72 đoạn khe trên có thể bị đẩy lên 3,5 cm (đỉnh (0,41; 0,81)), nên chưa loại trừ lớp 1 làm F2 nặng thêm.

### 3b.4 Kết luận
- 2) khớp hết.
- 3) Q1 không kém (6/6, lượt 1), riêng di_B chậm ~7 s. **B1 kém hơn về x/n (1/3 so với 2/3)**.
- Theo điều kiện 4: **KHÔNG commit**. Thay đổi để nguyên trong cây làm việc.
- Đề xuất cho Thiên quyết:
  - (a) chạy B1 thêm n ≥ 10 cả bản cũ lẫn bản mới để tách nhiễu F2;
  - (b) sửa F2 trước (khe trên: xoay bị chặn sát tường trên);
  - (c) chấp nhận và commit.

---

## 3b-2. So đường cũ (37268ff) và mới (cây làm việc) trên costmap SỐNG của giả lập (chỉ đọc, không commit) [tinh]

Cách làm:
- Bật giả lập như Q1/B1 3b (bat_gia mức B, xe ở A), chụp `/global_costmap/costmap` (TRANSIENT_LOCAL, như gui_duong) sau 5 s: một lần không ống, một lần có ống (0,73; 0,08). Lưới 52 × 32, ô 0,05, gốc (−0,566; −0,417). Không chạy chuyến; tắt mã 0.
- Script ở scratchpad mới: `so_duong/chup.sh`, `chup.py`, `so.py`. `tt_cu.py` = `git show 37268ff:xe_scripts/tinh_tuyen.py`; `tt_moi.py` = cây làm việc.
- Điểm đầu như log giả lập: di_B từ tư thế xe lúc chụp (−0,04; −0,04); ve_A từ (1,63; 0,68) (xe dừng ở B trong Q1 3b).
- Đỉnh = điểm đổi hướng của đường dày. "Cách vật" = tâm xe tới **mép ô ≥ 98**, lấy mẫu 1 cm. `cd` = `cach_do` của từng bản (tâm ô–tâm ô, cùng số in trong CANH BAO).

### Đỉnh, độ dài, khoảng cách

| ca | đường | bản | dài (m) | đỉnh (x; y) [tâm → mép ô vật, cm] | min tâm → ô vật |
|---|---|---|---|---|---|
| không ống | di_B t1 | cũ | 2,249 | (−0,040;−0,040)[22,6] (0,059;0,058)[28,5] (1,209;0,008)[22,5] (1,559;0,358)[32,5] (1,600;0,820)[26,3] | 14,3 cm tại (0,787; 0,027), ô (0,759; −0,142) |
| | | mới | 2,289 | (−0,040;−0,040)[22,6] **(0,109;0,108)**[28,5] **(1,259;0,008)**[22,5] (1,559;0,358)[32,5] (1,600;0,820)[26,3] | 15,3 cm tại (0,425; 0,081), ô (0,459; 0,258) |
| | | lệch | +0,040 | 0 / **7,1** / **5,0** / 0 / 0 cm | |
| không ống | di_B t2 | cũ | 2,177 | (−0,040;−0,040) (0,059;0,308)[37,5] **(0,359;0,758)[19,0, cd 0,224]** (1,559;0,758)[32,5] (1,600;0,820) | 12,5 cm tại (0,589; 0,758), ô (0,609; 0,908) |
| | | mới | 2,187 | (−0,040;−0,040) **(0,009;0,258)**[37,6] **(0,359;0,758)[19,0, cd 0,224]** (1,559;0,758)[32,5] (1,600;0,820) | 12,5 cm, cùng chỗ |
| | | lệch | +0,010 | 0 / **7,1** (gần A) / 0 / 0 / 0 cm | |
| không ống | ve_A t1 | cũ / mới | 2,080 / 2,120 | như di_B t1 đảo chiều; lệch 0 / 0 / 0 / **5,0** / **7,1** / 0 cm | 14,3 / 15,3 cm |
| không ống | ve_A t2 | cũ / mới | 2,163 / 2,149 | (1,630;0,680) (1,509;0,758) (0,359;0,758)[19,0] (0,009;0,258) hoặc (0,059;0,308) (0;0); lệch 0 / 0 / 0 / **7,1** (gần A) / 0 | 12,5 / 12,5 cm tại (0,589; 0,758) |
| có ống | di_B t1, ve_A t1 | cũ VÀ mới | — | **KHÔNG có đường**, cả hai bản (`khong tim duoc duong (10, 7) -> (43, 24)`) | — |
| có ống | di_B t2, ve_A t2 | cũ / mới | như "không ống" t2 | đỉnh khe trên (0,359; 0,758) trùng; chỉ đỉnh gần A lệch 7,1 cm | 12,5 / 12,5 cm |

### Giải thích mâu thuẫn
- **3a-2** (đỉnh dời lên (0,41; 0,81), khe trên 19,0 cm so với 22,5 cm) tính trên costmap **dựng lại từ bản đồ tĩnh**. **3b** chạy trên costmap **sống**. Hai costmap khác nhau:
  - Costmap sống có **341 ô vật (100)** so với 252 ô của bản đồ tĩnh: lớp vật cản /scan đánh dấu thêm 89 ô, chủ yếu ở mép tường.
  - 694 ô ≥ 98 so với 606: 88 ô sống ≥ 98 mà bản dựng lại < 98, chiều ngược lại 0.
  - Ví dụ cột x = 0,609: sống có ô vật ở y 1,007 và ô 99 ở y 0,907 / 0,957, trong khi bản dựng lại chỉ 72-89. Vậy khe trên **sống hẹp hơn**: min 12,5 cm (cả cũ lẫn mới), không phải 19-22,5 cm.
- Trên costmap sống, ngưỡng 72 không đổi đường khe trên. Lý do: điểm sát nhất (0,589; 0,758) cách ô ≥ 98 (vòng inscribed) 12,5 cm mép (≈ 15 cm tâm), tức cách ô vật (100) ≈ 0,20 m, chưa vào dải ≥ 72. Cả hai bản cùng chọn đỉnh (0,359; 0,758) và cùng cảnh báo `cd 0,224 < 0,236`. Đó là lý do 3b thấy "cùng đỉnh, cùng CANH BAO".
- Phụ: costmap sống có giá trị cao hơn bản dựng lại 2 đơn vị ở cùng khoảng cách (0,10 m: 94 so với 92; 0,15 m: 81 so với 79; 0,20 m: 70 so với 68). Khớp với `footprint_padding` mặc định 0,01 m của costmap Nav2 (không đặt trong nav2_params_cho.yaml), làm bán kính inscribed 0,09 thay vì 0,08 [suy luận, chưa đọc mã padding]. Tập ô ≥ 72 không đổi (0,1803 m ra 74, 0,20 m ra 70). Bảng số ở 3a.1 lệch +1-2 so với xe.

### Kết luận (1 dòng)
**Đường khe trên GIỐNG HỆT** trong khe (đỉnh (0,359; 0,758) và đoạn (0,359; 0,758)→(1,559; 0,758) trùng, min 12,5 cm cả hai; chỉ đỉnh gần A lệch 7,1 cm). **Đường di_B không ống CÓ đổi nhẹ**: 2 đỉnh lệch 7,1 và 5,0 cm, dài +4 cm, min cách vật tăng 14,3 → 15,3 cm. Mức đổi này không đủ giải thích +7 s (4 cm ở 0,15 m/s ≈ 0,3 s); nguyên nhân +7 s chưa rõ, cần đo lại.

Hệ quả cho 3b: B1 1/3 so với 2/3 **không do lớp 1 đổi đường khe trên** (đường giống hệt). Nhiều khả năng là dao động của F2 (khe trên sống chỉ còn 12,5 cm tâm → mép ô vật).

---

## 3b-3. Commit lớp 1 (Thiên quyết theo luật đã ghi: đường khe trên giống hệt ⇒ commit). KHÔNG push. CHƯA CHẠY TRÊN PI.

Ghi lại khi commit:
- **Giả lập chưa hiệu chuẩn** (HIEU_CHUAN_0311.md: Q1 đạt, Q2 không đạt). Mọi số 3a-3b-2 là [tinh] hoặc giả lập, chưa đo trên xe.
- **di_B giả lập 25-26 s → 32-33 s: chưa rõ nguyên nhân.** Đường di_B không ống chỉ đổi nhẹ (2 đỉnh lệch 7,1 / 5,0 cm, dài +4 cm ≈ +0,3 s ở 0,15 m/s; mục 3b-2). Cần đo lại.
- **B1 1/3 DEN_NOI (sau) so với 2/3 (trước) nằm trong dao động F2.** Đường khe trên trên costmap sống giống hệt cũ và mới (mục 3b-2); hai lần hỏng đi cùng chuỗi F2 như b1_3 trước khi sửa (CHAN_XOAY → CHO_SEP → RA → XOAY → ABORTED). n = 3.
- **Bảng quét 3a-2** (166 / 34 / 1 vị trí lách, ảnh `gia_lap/lop1_nguong.png`) **tính trên costmap dựng lại từ bản đồ tĩnh: 252 ô vật**, trong khi costmap sống của giả lập có **341 ô vật**. Số thật có thể khác; vùng đỏ sống dày hơn nên số "lách" sống có thể thấp hơn, khe cũng hẹp hơn.
- **Khe trên trên costmap sống: tâm xe cách mép ô vật (≥ 98) 12,5 cm** tại (0,589; 0,758), cả bản cũ lẫn mới. Đây là chỗ hẹp nhất của tuyến 2, liên quan F2.
- Tệp trong commit: tinh_tuyen.py, gui_duong.py, test_gui_duong.py (test_25, test_26), RA_SOAT_ONG_SAN.md, gia_lap/lop1_nguong.png. Bộ thử: 26/26, 44/44, 68/68.
- Khi chép lên Pi: chép **cả** `tinh_tuyen.py` và `gui_duong.py` (so_voi_pi.sh). Đổi `inflation_radius` thì phải tính lại ngưỡng 72.

---

## 3c-1. Thiết kế lớp 2 (CHỈ ĐỌC, không sửa, không commit). CHƯA CHẠY TRÊN PI

Thiên chốt: trước mỗi lượt mới, nếu xe đứng sát vật thì SẾP ghi THOAT RA vào ~/sep_thoat. PHỤ LÁI lùi thẳng (LiDAR kiểm phía sau ≥ 5 cm) tới khi tâm xe cách mặt vật ≥ 0,30 m hoặc đã lùi tối đa 0,30 m, rồi báo RA_XONG. gui_duong chờ RA_XONG rồi mới gửi đường.

Script ở scratchpad mới: `lop2/vung.py` (LiDAR giả giống hệt xe_gia: `map_xuong_v3.png` ô chiếm p > 0,65, 500 tia, R_MIN 0,10, bước 0,01, LiDAR +0,025, cắt thêm hình tròn ống), `lop2/ve.py`. Ảnh: `gia_lap/lop2_vung.png` [tinh].

### 3c-1.1 Vùng kích hoạt [tinh]

Đề xuất: **hình chữ nhật khung thân xe, x từ 0,145 (mũi) tới X_XA = 0,25 m tính từ tâm, |y| ≤ W = 0,10 m (thân ±0,08 + dư 2 cm); ≥ 2 điểm LiDAR trong vùng (lọc nhiễu như `dem_diem_do` của lai_xe_lidar) thì LÙI.**

Bốn ca thử (vùng đề xuất):

| ca | tư thế | điểm LiDAR phía trước gần tâm nhất (khung thân) | điểm trong vùng | kết quả | mong |
|---|---|---|---|---|---|
| (1) 3/11 | (0,39; 0,09; 29°), ống (0,693; 0,090) r 0,09 (góc trước-phải cách ống 4,8 cm) | (0,187; −0,103) | 23 | **LÙI** | LÙI |
| (2) thẳng | (0,415; 0,08; 0°), mũi cách ống (0,73; 0,08) 8 cm | (0,225; 0,000) | 43 | **LÙI** | LÙI |
| (3) Q1 | 1057 tư thế dọc di_B / ve_A × khe dưới / khe trên. Đường từ tinh_tuyen MỚI trên **costmap sống** (`cm_khong_ong.json` của 3b-2), lấy mẫu 1 cm, cộng các hướng xoay tại chỗ ở mỗi đỉnh (bước 5°) và tư thế đích | gần nhất x = 0,308 tại A sau ve_A (0; 0; −101°) | 0 ở cả 1057 | **KHÔNG** | KHÔNG |
| (4) A, B | A (0;0;0°), A giả lập (−0,04;−0,04;0°), A hướng −135°, B (1,60;0,82;0°), B tới (1,63;0,68) 0° và 85° | (0,146-0,354; …): toàn ngoài dải \|y\| ≤ 0,10 hoặc xa hơn 0,25 | 0 | **KHÔNG** | KHÔNG |

Biên và độ bền:
- Quét X_XA 0,18-0,36 × W 0,08-0,16 (bước 1 cm): **50 cặp đạt cả 4 ca**, X_XA 0,23-0,31.
- Biên dưới do **ca 2** (điểm ở x 0,225 ⇒ X_XA ≥ 0,23). Biên trên do **ca 3**:
  - X_XA 0,32 hỏng ở đích di_B (1,60; 0,81; 85°) và khi xoay tại (0,359; 0,758) về 185-190°;
  - W 0,13 hỏng ở ve_A khe dưới (1,27; 0,02; −131°);
  - A sau ve_A có tường ở x 0,301-0,308.
- Độ bền ca 3 (mỗi tư thế lệch ngang ±4 cm × lệch hướng ±5°, 9513 tư thế), số lần kích hoạt nhầm:

| (X_XA; W) | nhầm |
|---|---|
| (0,30; 0,12) | 38 |
| (0,28; 0,12) | 26 |
| (0,26; 0,10) | 6 |
| **(0,25; 0,10)** | **4**, ví dụ ve_A khe dưới (1,30; −0,01; −126°), góc K4 |

- Chọn **(0,25; 0,10)**: dư 2,5 cm ở ca 2, 5,8 cm ở ca 3.
  - Xe dừng thẳng với mũi ≥ 10,5 cm thì KHÔNG lùi. B1′ (mũi 10,4-12,0 cm) nằm ở biên; ở các tư thế đó RPP vẫn xoay/đi được (3b-2: tâm–mặt ống ≥ 0,245 m > 0,166 + 1 ô).
  - Mục tiêu RA "tâm cách mặt vật ≥ 0,30 m" lớn hơn X_XA 0,25, tạo khoảng trễ 5 cm: sau RA vùng chắc chắn trống.
- **Có vùng đạt cả 4 ca.** Lưu ý: dư ở biên trên chỉ ~5 cm và vẫn có 4/9513 kích hoạt nhầm khi lệch. Kích hoạt nhầm chỉ làm xe lùi ≤ 0,30 m có kiểm LiDAR phía sau, không nguy hiểm, nhưng tốn ~3-5 s.

Giới hạn:
- Thế giới LiDAR là bản đồ tĩnh (không có lớp vật cản sống, không người).
- Ca 3 là đường kế hoạch, chưa có sai số bám thật ngoài phép lệch ±4 cm / ±5°.
- Chưa thử ống ở các vị trí khác ngoài 2 ca.

### 3c-1.2 Luồng tin (đọc mã; [CÓ] = đã có, [THÊM] = phải thêm mã, [SỬA] = phải sửa mã có sẵn)

**a) gui_duong báo "sắp gửi lượt mới" cho sếp.**
- [CÓ] Hiện **không có kênh riêng**. Gần nhất là dòng CSV `KHONG_DAT` / `TINH_LAI` gui_duong ghi vào `~/ket_qua_chuyen.csv` ngay sau một lượt hỏng (gui_duong.py:123, :136, :147). Sếp đọc tệp này mỗi vòng 0,1 s (`_tick`, sep.py:1003, đọc `csv_chuyen` :1046).
- Không đủ vì: (1) lượt 1 của **tuyến kia** sau KhongCoDuong (F1, `break`) được gửi **ngay**, không chờ 30 s; (2) gui_duong không biết sếp có định ra RA hay không, nên không biết phải chờ.
- [THÊM] Đề xuất tệp bắt tay mới `~/gui_duong_luot`: gui_duong ghi `<giay BOOTTIME> SAP_GUI <huong> <tuyen> <lan>` (ghi tệp tạm rồi đổi tên như `ghi_lenh`) trước MỖI lần gửi. Sếp trả lời trong ≤ 1 s:
  - **có** vật trong vùng thì ghi ~/sep_thoat dạng mới (xem c);
  - **không** thì ghi một câu "KHONG_CAN" (cùng tệp, hoặc dòng `LENH` riêng; cần chốt).
- Cách thay rẻ hơn (không tệp mới): sếp phản ứng với dòng CSV `KHONG_DAT`, còn gui_duong chờ cố định 1-2 s trước mỗi lần gửi. Có độ trễ đọc tệp và không phân biệt được "không cần" với "sếp chết". Không khuyên dùng.

**b) Sếp đọc LiDAR ở đâu.**
- [CÓ] sep.py:991 nghe `/scan` (qos sensor). `v['pts']` = `L.scan_to_xy` (khung thân xe, LIDAR_X 0,025, cùng khung với vùng ở 3c-1.1).
- [THÊM] hàm thuần `vung_lui(pts, x_xa=0.25, w=0.10, n=2)` trong sep.py.
- [THÊM] trạng thái "đợt RA lớp 2" tách khỏi `_dot_thoat`. `_dot_thoat` hiện chỉ mở khi phụ lái vào **CHO_SEP và đích đang chạy** (sep.py:784), và **đóng khi không có đích** (:794-795 `if not self.goal_active: self.thoat_ma = None`), mà node xoá ~/sep_thoat khi không còn đợt (:1122-1124). Giữa hai lượt không có đích, nên đường cũ không dùng được.

**c) Phụ lái RA hiện có làm gì** [CÓ] (lai_xe_lidar.py:337-387):
- `ra_bat_dau`: **từ chối khi không có đích** (`RA BO QUA: khong co dich`, :346).
- **Tự chọn hướng** về phía thoáng hơn (`huong = 1 if ff >= fb else -1`, :358), không bắt buộc lùi.
- **Không đi nếu mọi điểm đã cách tâm > R_XOAY 0,1856 m** (`RA XONG DU_CHO_XOAY 0 cm`, :353-355). Ca 3/11: mặt ống cách tâm 0,213 m > 0,186, nên **RA hiện có KHÔNG nhúc nhích**, dù RPP vẫn báo va chạm.
- Trong pha RA (`_ra`, :363-387) dừng khi, theo thứ tự:
  - `DO_PHIA_DI`: ≥ 1 điểm ≤ 0,08 m ở phía đang đi;
  - `DU_CHO_XOAY`;
  - `DA_DI_30CM` (quãng = max(ước theo lệnh v_esc 0,10 m/s, đo LiDAR));
  - `QUA_10S`;
  - `MAT_DICH` (mất đích thì dừng ngay).
- Xong thì log `RA XONG <lý do> <cm> cm` + chuyển trạng thái `RA_LUI -> RA_XONG` trên /rosout (:385, :692-702), rồi pha XOAY_P nếu còn đích.
- [THÊM] chế độ **"lùi thẳng lớp 2"**:
  - luôn lùi (huong = −1), chạy được **khi không có đích**;
  - dừng khi điểm gần nhất trong dải |y| ≤ W phía trước cách tâm ≥ **0,30 m**, hoặc đã lùi **0,30 m**, hoặc phía sau có ≥ 2 điểm ≤ **0,05 m** (Thiên chốt; RA cũ dùng 0,08 / ≥ 1 điểm), hoặc quá ~6 s (0,30 / 0,10 = 3 s + dư);
  - không vào XOAY_P;
  - log `RA XONG <lý do> <cm> cm` + `RA_XONG` như cũ để bên đọc không đổi.
- [SỬA] `doc_sep_thoat` / định dạng ~/sep_thoat (`<t> THOAT <xP> <yP> <luot> <ma> [yaw]`, lai_xe_lidar.py:512) cần một chữ phân biệt "RA lớp 2". Đề xuất cột 2 = `RA_LUI` thay `THOAT`. Luật G phải sửa cùng lúc: sep `ly_thoat` / `ghi_lenh`, lai_xe_lidar `doc_sep_thoat`, gui_duong `_thoat_moi` / `sep_dang_giao_thoat`, test_sep test_32.

**d) gui_duong chờ RA_XONG.**
- [CÓ] gui_duong đã nghe /rosout của lai_xe_lidar (gui_duong.py:278, :292-298, `self.l1`).
- `cho_truoc_khi_gui` (:175-194) đã chờ khi `phu_lai_ban()` (l1 ∈ RA_TIEN / RA_LUI / XOAY_P / THOAT*) hoặc `sep_dang_giao_thoat()`, tối đa `cho_hong` 30 s. Nhưng nó không chờ được lúc RA **chưa bắt đầu** (tranh chấp ~0,1-1 s).
- `sep_dang_giao_thoat` coi mã mới trong ~/sep_thoat là "đang giao" cho tới khi `da_xu_ly_thoat()`, mà hàm này chỉ được gọi trong `theo_doi` khi đếm XOAY_XONG (:227-229). RA lớp 2 không có XOAY, nên gui_duong sẽ chờ tới khi tệp cũ (> 3 s) hoặc hết 30 s.
- [THÊM] sau khi ghi `SAP_GUI`, chờ: (sếp trả "không cần") hoặc (l1 đã qua RA_LUI rồi `RA_XONG`). **Tối đa đề xuất 12 s** (RA lùi ≤ 6 s + trễ sếp 1 s + dư).
- [SỬA] gọi `da_xu_ly_thoat()` khi thấy RA_XONG lớp 2.
- **Hết giờ:** in `RA_HET_GIO` và gửi đường như hôm nay (không kém hơn hiện trạng). Nếu phụ lái vẫn đang RA_LUI thì `cho_truoc_khi_gui` sẵn có giữ lại tới khi xong. Không ghi CSV chữ mới (tránh luật G với sep / HUONG_DAN); cần Thiên chốt.

**e) Chỗ phải THÊM / SỬA (tóm tắt).**
- sep.py: `vung_lui`; đợt RA lớp 2 không cần đích; đọc `~/gui_duong_luot`; ghi chữ mới vào ~/sep_thoat; không xoá tệp khi chưa xong.
- lai_xe_lidar.py: chế độ lùi thẳng lớp 2 (không cần đích, ngưỡng 0,30 m / 0,30 m / sau 0,05 m / 6 s, không XOAY); `doc_sep_thoat` đọc chữ mới.
- gui_duong.py: ghi `~/gui_duong_luot` trước mỗi lần gửi; chờ trả lời / RA_XONG ≤ 12 s; `da_xu_ly_thoat` cho RA lớp 2.
- Bài thử: test_sep, test_lai_xe_lidar, test_gui_duong (bắt tay, hết giờ, ca 3/11). Luật G cho tên tệp mới, chữ mới, mã.
- Không cần sửa: cau_noi_xe.py (đọc kết quả chuyến, không đổi), chuyen.sh, Nav2 yaml.

---

## 3c-2. Viết lớp 2: lùi ra trước lượt mới. KHÔNG push. CHƯA CHẠY TRÊN PI. Giả lập CHƯA HIỆU CHUẨN

Thiên chốt:
- vùng: x 0,145-0,25 m, |y| ≤ 0,10 m, ≥ 2 điểm;
- lùi tới khi tâm cách mặt vật ≥ 0,30 m, hoặc đã lùi tối đa 0,30 m; dừng khi phía sau thoáng < 5 cm; tối đa 6 s;
- gui_duong chờ RA_XONG tối đa 12 s; hết giờ thì in RA_HET_GIO rồi gửi như cũ;
- bổ sung 05/11: "Dừng chuyến" (SIGINT gui_duong) thì phụ lái ngừng lùi trong ≤ 1 s.

Không sửa cau_noi_xe.py.

### 3c-2.1 Mã (git diff: lai_xe_lidar.py, sep.py, gui_duong.py + 3 tệp test)

Chuỗi: gui_duong ghi `~/gui_duong_luot` `<t> SAP_GUI <huong> <tuyen> <lan> <ma>` trước MỖI lần gửi → sếp, **chỉ khi thấy SAP_GUI mới**, xét vùng (`L.vung_lui`) và trả lời trong `~/sep_thoat`:
- `<t> RA_LUI <ma_luot> <ma>` khi có vật trong vùng và ra_lenh = true;
- `<t> KHONG_CAN <ma_luot> <lý do>` cho các trường hợp còn lại (`ra_lenh=false`, `mat_scan`, `vung_trong`).

Bên phụ lái và gui_duong:
- Phụ lái nhận mã RA_LUI mới thì vào trạng thái RA2: lùi thẳng `-v_esc`, tên đầu ra `RA_LUI`, không cần đích, không XOAY. Dừng theo thứ tự: `CHI_NHIN`, `HUY`, `SAU_5CM`, `CACH_30CM`, `DA_LUI_30CM`, `QUA_6S`. Xong thì log `RA XONG <lý do> <cm> cm` và chuyển `RA_LUI -> RA_XONG`.
- gui_duong (`cho_lop2`) xử lý câu trả lời:
  - KHONG_CAN thì gửi ngay;
  - RA_LUI thì chờ số lần RA_XONG tăng (≤ 12 s);
  - sếp không trả lời trong 2 s, hoặc hết 12 s, thì in `RA_HET_GIO` rồi gửi như cũ.

Bổ sung 05/11 (HUY):
- `chay_co_ngat`: khi bị ngắt, **trước tiên** ghi `<t> HUY` vào ~/gui_duong_luot (`huy_luot`), rồi mới hủy đích (chờ ≤ 2 s).
- Phụ lái `_doc_huy`: mỗi nhịp 0,1 s, nếu đang RA2 mà tệp là HUY hoặc tệp mất, thì bước kế tiếp phát **một lệnh 0** và log `RA XONG HUY <cm> cm`.
- Mã RA_LUI cũ đã được ghi nhớ (`ma_ra2_cu`), nên phụ lái không tự lùi lại.

Test: test_lai_xe_lidar 80/80, test_sep 45/45, test_gui_duong 29/29.

### 3c-2.2 Kiểm tích hợp

| bên ghi | bên đọc | chữ/tên phải khớp | khớp? |
|---|---|---|---|
| gui_duong `dong_sap_gui` → ~/gui_duong_luot (tệp tạm + đổi tên) | sep `doc_sap_gui` (tuổi ≤ 3 s), param `tep_luot` | `<t> SAP_GUI <huong> <tuyen> <lan> <ma>`, đường dẫn ~/gui_duong_luot | khớp (test_29 luật G) |
| gui_duong `dong_huy` → ~/gui_duong_luot | phụ lái `luot_huy` (param `tep_luot`); sep `doc_sap_gui` trả None | `<t> HUY`; mất tệp = HUY | khớp (test_29, bài HUY trong test_lai_xe_lidar) |
| sep `dong_ra2` + `ghi_lenh` → ~/sep_thoat | phụ lái `doc_sep_ra2`; gui_duong `tra_lop2` (= `L.doc_sep_ra2`) | `RA_LUI <ma_luot> <ma>` / `KHONG_CAN <ma_luot> <lý do>` | khớp (test_29) |
| cùng tệp ~/sep_thoat | phụ lái `doc_sep_thoat`; gui_duong `_thoat_moi` / `sep_dang_giao_thoat` | hai chữ mới phải bị **bỏ qua** | khớp: `doc_sep_thoat` trả None (test_29), nên không cần sửa `da_xu_ly_thoat` (3c-1 [SỬA] bỏ) |
| sep ghi ~/sep_thoat cả khi ra_thoat = false (câu KHONG_CAN) | phụ lái chỉ lùi khi RA_LUI (sếp chỉ ghi RA_LUI khi ra_lenh = true) | — | khớp; ra_lenh = false thì 0 lần lùi (test_45) |
| phụ lái /rosout `RA_LUI -> RA_XONG` | gui_duong `_on_rosout` (`n_ra_xong`); sep vòng rosout (`t_ra_xong`, LOP2 RA_XONG DONG) | tên trạng thái `RA_LUI`, `RA_XONG` (tên cũ) | khớp; `RA_LUI` ∈ gui_duong `PHU_LAI_BAN` và sep `DANG_LAM`; sep chỉ đếm lượt thoát khi `thoat_ma` mở (giữa hai lượt là None) |
| sep `ghi('LOP2', ...)` → ~/sep_nhat_ky.csv | cau_noi_xe `doc_sep` (chỉ TU_THE / LENH) | loại mới `LOP2` | khớp (cau_noi bỏ qua) |
| gui_duong in `   lop 2: ...`, `   RA_HET_GIO: ...` (thụt đầu dòng) | cau_noi `DONG_CHUYEN` `^(DEN_NOI\|HET_DUONG\|...\|HUY_DICH_KHI_NGAT\|...)` | không được khớp nhầm | khớp (thụt lề; HUY ghi tệp, không in) |
| cau_noi nút "Dừng" → SIGINT `pid_gui_duong` | gui_duong `chay_co_ngat` → HUY → phụ lái | luật tìm PID (python3 + gui_duong.py) | khớp: B3 dùng đúng luật này, 3/3 dừng ≤ 0,1 s |
| bat_xe.sh `tat`: `pkill -INT` mẫu TAY (sep, lai_xe_lidar, theo_doi, gui_duong) | — | tên tiến trình | khớp, không đổi; ghi chú: tắt cả bộ thì phụ lái cũng thoát cùng lúc (như mọi chế độ trước đây) |
| chuyen.sh `exec python3 ~/gui_duong.py` | — | chỉ đọc ~/sep_lenh | không đổi |
| so_voi_pi.sh | chép lên Pi | sep.py, lai_xe_lidar.py, gui_duong.py đã có trong danh sách | khớp; phải chép **cả ba** tệp cùng lúc (luật G) |

Rủi ro đã biết: nếu sếp không chạy, mỗi lần gửi chậm thêm 2 s (RA_HET_GIO). ~/gui_duong_luot không bị xoá lúc bật; điều đó an toàn vì sếp coi dòng cũ hơn 3 s là hết hạn.

### 3c-2.3 Giả lập (chưa hiệu chuẩn; HOME giả riêng mỗi lần, domain 87, bat_gia mức B)

Script trong scratchpad `l2sim/`: vong2.sh, ngat_ra2.py (B3), theo_lop2.py (B4), dat_ong.py, canh_ong.py, phan_tich2.py. Bản cũ là `git archive dc9b379`.

Sự cố trong lúc chạy:
- Mình sửa vong2.sh khi bash nền đang đọc nó, nên moi_b2_3 hỏng (không tắt) và kéo cu_b2_1 hỏng theo. Đã dừng, dọn sạch, chạy lại cả hai.
- Một lần moi_b2_3 khác hỏng vì Nav2 không lên trong 90 s, lỗi khởi động lẻ; đã chạy lại.

| ca | bản | x/n DEN_NOI | mã thoát | lùi lớp 2 (RA2) | RA_XONG lớp 2 / RA_HET_GIO | ghi chú |
|---|---|---|---|---|---|---|
| Q1 (3 vòng di_B + ve_A) | dc9b379 (3b) | 6/6, lượt 1 | 0 ×6 | — | — | di_B 32-33 s, ve_A 25-29 s |
| Q1 lần 1 (mã trước HUY) | mới | 6/6, lượt 1 | 0 ×6 | **0** | 0 / 0 | di_B 31-32 s, ve_A 27-30 s; sếp chỉ trả KHONG_CAN vung_trong; avoid 0 |
| Q1 lần 2 (mã cuối) | mới | 6/6, lượt 1 | 0 ×6 | **0** | 0 / 0 | di_B 32-33 s, ve_A 27-30 s; avoid 0 |
| B1 (ống đặt sẵn) | dc9b379 (3b) | 1/3 | 1, 0, 1 | — | — | |
| B1 | mới | 2/3 | 0, 1, 0 | 0 | 0 / 0 | xe không dừng sát ống (thân → ống ≥ 48 cm), nên khác biệt là nhiễu F2, không do lớp 2 |
| B2 (ống đặt lúc chạy, để luôn) | dc9b379 | 3/3 t2 | 0 ×3 | — | — | 85, 82, 84 s; mũi → ống min 9,9 / 10,5 / 9,0 cm |
| B2 | mới | 3/3 t2 | 0 ×3 | **2/3** (b2_2 vật 0,22 m, lùi 9 cm; b2_3 vật 0,24 m, lùi 7 cm) | 2 / 0 | 71, 83, 57 s; mũi min 9,6 / 8,7 / 10,0 cm |
| B3 (ống tâm (0,27; 0) lúc xuất phát, SIGINT 0,40-0,41 s sau RA2) | mới | — (ngắt) | 130 ×3, CSV NGAT | 3 → `RA XONG HUY 5 cm` ×3 | — | **SIGINT → /cmd_vel_avoid = 0: 0,089 / 0,099 / 0,099 s; trôi thêm 1,0 / 1,0 / 1,0 cm; lùi lại 0** (theo dõi ≥ 10 s); sếp LOP2 RA_XONG DONG ×3; tệp còn `HUY`. **3/3 đạt (≤ 1 s)** |
| B4 (cảnh B3, không ngắt) | mới | 2/3 t2 | 1, 0, 0 | 3 → `RA XONG CACH_30CM 13 cm` ×3 | 3 / 0 | chuỗi đủ 3/3 (xem dưới) |

B4: mốc thời gian tính từ SAP_GUI, theo /rosout, ~/sep_thoat, /follow_path status và /odom.

| lần | RA_LUI (sep_thoat) | phụ lái RA2 | RA_XONG | gửi /follow_path | lùi (odom) | thời gian lùi | gửi trong lúc RA2 | kết quả chuyến |
|---|---|---|---|---|---|---|---|---|
| 1 | 0,08 s | 0,18 s | 1,48 s | 1,76 s | 13,0 cm | 1,30 s | 0 | HET_DUONG, mã 1, 84 s (t1 KhongCoDuong; t2 ABORTED ×3 cách đích 0,89 m) |
| 2 | 0,08 s | 0,08 s | 1,38 s | 1,50 s | 13,0 cm | 1,30 s | 0 | DEN_NOI t2, mã 0, 31 s |
| 3 | 0,09 s | 0,09 s | 1,39 s | 1,66 s | 13,0 cm | 1,30 s | 0 | DEN_NOI t2, mã 0, 35 s |

- 3/3 có đủ chuỗi SAP_GUI → RA_LUI → lùi → RA_XONG → gửi đường, và RA_XONG trước khi gửi. Không có RA_HET_GIO. gui_duong in `lop 2: RA_XONG sau 1.4-1.5 s.`
- Lần 1 HET_DUONG ở lượt sau: ống vẫn để nguyên sát điểm A, tuyến 2 ABORTED ở đoạn sau. Đây không phải lỗi chuỗi lớp 2.

Câu Thiên yêu cầu ghi: "B2 gia lap khong kich lop 2 vi mui dung 10,4-12 cm, xe that ~8 cm se kich; chuoi lop 2 chi duoc thu bang canh dung san B3/B4".
- **Đính chính theo số liệu lần này:** B2 mới **có kích 2/3** (moi_b2_2, moi_b2_3).
  - Ở lượt sau ABORTED, sếp thấy vật 0,22 m và 0,24 m từ tâm, tức 7,5 và 9,5 cm từ mũi, vẫn nằm trong vùng.
  - Phụ lái lùi 9 và 7 cm, RA_XONG sau 1,0 và 0,9 s, rồi DEN_NOI tuyến 2.
- Mép vùng là 0,25 − 0,145 = **10,5 cm từ mũi**, nên mũi dừng ở 10,4 cm cũng kích. Kích hay không phụ thuộc chỗ xe dừng ở lượt đó (moi_b2_1 không kích).
- Vậy B2 có thử được chuỗi lớp 2, nhưng không ổn định. B3/B4 là cảnh dựng sẵn để chắc chắn có chuỗi.

### 3c-2.4 Kết luận
- 2) khớp hết.
- 3) Q1 không kém: 12/12 DEN_NOI lượt 1, 0 lần lùi, thời gian như dc9b379.
- B3 3/3 ≤ 0,1 s. B4 3/3 đủ chuỗi, RA_XONG trước khi gửi.
- Đủ điều kiện commit theo luật (một lần, "lop 2: lui ra truoc luot moi"). Không push.
