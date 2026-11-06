# RÀ SOÁT 118: scan_ket_5b2 chụp lúc nào (CHỈ ĐỌC; không sửa mã, không commit). CHƯA CHẠY GÌ TRÊN XE

> **Ghi chú 06/11 23:25:** nhánh luật khe (B6–B6d, mã ở tinh_tuyen.py) KHÔNG được gộp và KHÔNG chạy trên xe: trên costmap sống (dựng từ map_xuong_v3_pg) khe trên chỉ còn 2 ô nên luật khe không bật (mục B7b). Gốc lỗi là định vị lệch (mục B8–B9); gói đo xưởng ở do_xuong/ (mục B10).

Nhãn: [log] = đọc thẳng từ tệp dữ liệu; [tính] = tính từ số [log]; [đọc code] = đọc mã.

## B1. Giờ chụp scan_ket_5b2.txt so với lúc kẹt / lúc cứu

### 1) Kiểm đầu
- `git rev-parse --short HEAD` = **579cda5**; `git status -s` rỗng (cây sạch) [log].

### 2) Thư mục xe_scripts/du_lieu_0511_toi/
ket_qua_chuyen.csv, log_bat_xe_nav.txt, log_nut_chuyen.txt, log_phu_lai.txt, log_sep.txt, map_m0/m1/m3 (.pgm + .yaml), map_xuong_v3 (.png + .yaml), scan_ket_5b2.txt, scan_ket_5b3.txt, scan_ket_k1_2106.txt, sep_nhat_ky.csv, vet_5b2.txt, vet_5b3.txt (19 tệp) [log].

- `wc -l scan_ket_5b2.txt` = **855**, không phải 840 [log].
  - 840 là số dòng `- ` (`grep -c "^- "` = 840), tức 420 `ranges` + 420 `intensities`.
  - 15 dòng còn lại là header (stamp, frame_id, angle_*, time_increment, scan_time, range_*, hai nhãn `ranges:`/`intensities:`).
  - Scan đủ 420 tia, không bị cắt [log].

### 3) Giờ chụp và tư thế
- `header.stamp` = 1793879896.255262 [log] = **20:58:16.26 KST** (Pi chạy giờ KST, UTC+9, xem RA_SOAT_120.md mục 0) [tính].
- vet_5b2.txt (tf2_echo map → base_footprint, ~1 Hz; 148 mẫu 20:56:55.47 → 20:59:22.37) [log]:

| giờ KST | so với scan | x | y | góc (°) | ghi chú |
|---|---|---|---|---|---|
| 20:58:11.47 | −4,78 s | 0,432 | 0,812 | 7,4 | đứng yên từ ≤ 20:58:04 |
| 20:58:12.47 | −3,78 s | 0,432 | 0,812 | 7,5 | |
| 20:58:13.47 | −2,78 s | 0,353 | 0,802 | 7,6 | đang lùi |
| 20:58:14.47 | −1,78 s | 0,313 | 0,796 | 8,5 | |
| **20:58:15.27** | **−0,98 s** (sát trước) | **0,313** | **0,796** | **8,5** | |
| **20:58:16.45** | **+0,20 s** (sát sau) | **0,313** | **0,796** | **8,5** | |
| 20:58:17.47 | +1,22 s | 0,313 | 0,796 | 8,5 | |
| 20:58:18.47 | +2,22 s | 0,345 | 0,794 | −4,8 | bắt đầu chạy (quay phải + tiến) |
| 20:58:19.47 | +3,22 s | 0,418 | 0,786 | −12,5 | |
| 20:58:20.47 | +4,22 s | 0,584 | 0,808 | −13,6 | |
| 20:58:21.47 | +5,22 s | 0,673 | 0,786 | −15,3 | |

[log]

- Tư thế lúc chụp: **(0,313; 0,796; 8,5°)**. Hai mẫu sát nhất (−0,98 s / +0,20 s) **giống hệt tới mm và 0,1°**, nên **không cần nội suy** [log].
- Từ 20:58:12.47 tới 20:58:14.47 xe lùi theo −x: x 0,432 → 0,313, tức **11,9 cm** [tính]; góc 7,5° → 8,5°.

### 4) Đối chiếu log
**Độ phủ giờ của các log** [log]:
- log_phu_lai.txt (21:02:40 → 21:07:43), log_bat_xe_nav.txt (21:02:26 → 21:07:46), log_sep.txt (21:02:40 → 21:07:42) đều **không phủ 20:58**. Không có câu "collision ahead", không có `RA2 BAT DAU` / `RA XONG` của phụ lái cho lúc 20:58.
- log_nut_chuyen.txt chỉ có chuyến 21:04:19.
- Chỉ có **sep_nhat_ky.csv** và **ket_qua_chuyen.csv** phủ 20:58.

**Đổi giờ:**
- Cột `gio` của sep_nhat_ky chỉ tới giây [log]. Dòng `NAV2,ABORTED` lúc 21:04:44 có `gio_mono` 12981.23 [log] và đúng với epoch `Controller patience exceeded` 1793880284.35 trong log nav [log].
- Vậy epoch ≈ `gio_mono` + 1793867303.12 [tính]; giả định đồng hồ đơn điệu và epoch chạy cùng nhịp trong 7 phút.

| sự kiện (sep_nhat_ky.csv dòng) | gio_mono [log] | giờ KST [tính] | so với scan [tính] | tư thế sep ghi [log] |
|---|---|---|---|---|
| 4501-4502 `CSV KHONG_DAT di_B_day t2 d1`, `NAV2 ABORTED sau 15.2 s` (kẹt) | 12559.46 | 20:57:42.58 | −33,7 s | (0,43; 0,81; 7°) |
| 4506 `LOP2,RA_LUI,gui_duong SAP_GUI,di_B t2 l2,vat_cach_than_0.09m,phu lai lui thang` | 12589.46 | 20:58:12.58 | −3,7 s | (0,43; 0,81; 7°) |
| 4507 `TRUY_VET VAT_LA_TRUOC HET` | 12590.37 | 20:58:13.49 | −2,8 s | (0,35; 0,80; 8°) |
| 4508 `LOP2,RA_XONG,lai_xe_lidar,luot 125894,DONG` | 12590.46 | 20:58:13.58 | −2,7 s | (0,34; 0,80; 8°) |
| 4509 `TU_THE DI_THUONG` | 12594.36 | 20:58:17.48 | +1,2 s | (0,31; 0,80; 8°) |
| 4510 `TU_THE THAN_TRONG` (đã quay) | 12596.17 | 20:58:19.29 | +3,0 s | (0,41; 0,79; −10°) |
| 4521 `NAV2 SUCCEEDED sau 13.10 s` | 12607.46 | 20:58:30.58 | +14,3 s | (1,48; 0,75; 10°) |

- ket_qua_chuyen.csv [log]:
  - 348 `20:57:42 di_B_day 2 1 1 KHONG_DAT 15 1.17`;
  - 349 `20:58:30 ... 2 1 2 SUCCEEDED 13 0.14`;
  - 350 `DEN_NOI 69`.
- SAP_GUI lượt t2 l2 lúc 20:58:12.58 cách ABORTED 30,0 s, đúng `cho_hong` 30 s của gui_duong sau một lượt hỏng [tính] [đọc code: gui_duong `cho_sau_hong`].

**Sự kiện gần giờ chụp nhất:**
- `LOP2 RA_XONG` lúc 20:58:13.58, trước scan **2,7 s** [tính];
- `LOP2 RA_LUI vat_cach_than_0.09m` (SAP_GUI), trước scan 3,7 s [tính].

**Trong ±5 s quanh lúc chụp** (20:58:11.26 → 20:58:21.26):
- đứng yên tới 20:58:12.5;
- **lùi** 20:58:12.5 → 14.5 (11,9 cm);
- **đứng yên** 20:58:14.47 → 20:58:17.47, **gồm lúc chụp**;
- **chạy** từ ~20:58:17.5 (quay phải về −13°, tiến) [log vet + tính].

### 5) Kết luận
**scan_ket_5b2 KHÔNG chụp lúc kẹt và cũng không chụp lúc đang cứu.** Nó chụp 2,7 s **sau** RA_XONG của lần cứu RA_LUI 20:58:12-13: xe đã lùi 11,9 cm, đang đứng yên ở (0,313; 0,796; 8,5°) và 1-2 s trước khi chạy lượt t2 l2.
- Vật lúc kẹt đã cách xa hơn khoảng 12 cm so với lúc kẹt.
- Lúc kẹt (20:57:42.6 → 20:58:12.5, đứng yên ở (0,432; 0,812; 7,4°)), trong thư mục **không có scan nào**.
- scan_ket_5b3 (21:05:07) và scan_ket_k1_2106 (21:06:30) chụp đúng lúc kẹt, nhưng là **lần kẹt K1 khác** (21:04:44 ở (0,155; 0,756; 42°), xe đứng yên tới 21:06:34; RA_SOAT_120.md mục 1).

## B2. Khớp scan với map_xuong_v3 (CHỈ ĐỌC). DỪNG ở a): đối chứng lệch khác mong đợi quá 2 cm; **chưa làm b)**, chưa vẽ RA_SOAT_118_b2.png

### Cách khớp
- Bản đồ: du_lieu_0511_toi/map_xuong_v3.png + .yaml, `origin [-0.566, -0.415, 0]`, `resolution 0.05`; ô vật = occupied > 0,65 [log].
- Scan → khung thân: `lai_xe_lidar.scan_to_xy`, LiDAR (0,025; 0), không xoay, không lật [đọc code: `LIDAR_X = 0.025` ở **lai_xe_lidar.py:28** (dòng 29 là chú thích); `scan_to_xy` :81-89].
- Tư thế chỉnh = (x + dx, y + dy, θ + dθ), xoay quanh tâm xe. Dò dx, dy ±12 cm bước 0,5 cm; dθ ±6° bước 0,5° (49 × 49 × 25 ô) [tính].
- Điểm = tỉ lệ tia hợp lệ rơi vào ô vật v3, **cho sai 1 ô** (ô đó hoặc 8 ô kề có vật, không quấn qua mép). Kèm "ô đúng" (sai 0 ô) làm tiêu chí phụ khi hoà điểm [tính].
- Đỉnh thứ hai = cực đại địa phương (26 ô kề) cao nhất cách đỉnh nhất > 2 cm hoặc > 1° [tính].
- Script scratchpad `r120/khop118.py` (không vào kho).

### a) Đối chứng: scan_ket_5b3.txt
- Stamp 1793880307.10 = 21:05:07.10 KST [log/tính].
- Tư thế tin từ vet_5b3: 21:05:06.69 (−0,41 s) và 21:05:07.68 (+0,58 s) đều là (0,155; 0,756; 42,2°), không cần nội suy [log].
- 390/420 tia hợp lệ [log].

| | dx | dy | dθ | điểm (sai 1 ô) | điểm (ô đúng) | \|lệch\| |
|---|---|---|---|---|---|---|
| (0,0,0) | 0 | 0 | 0 | 0,764 | 0,105 | 0 |
| **cao nhất** | **+7,0 cm** | **+5,5 cm** | **0,0°** | **0,962** | 0,877 | **8,9 cm** |
| đỉnh thứ hai | +7,0 cm | +8,0 cm | 0,0° | 0,962 | 0,844 | 10,6 cm |

[tính]

- **Cao nguyên:** 91 ô cùng đạt 0,962 (sai 1 ô), trải dx +7,0..+8,0 cm, **dy +1,0..+9,5 cm**, dθ −2,0..0,0° [tính].
  - dx xác định rõ (khoảng 1 cm).
  - dy **không rõ** khi cho sai 1 ô: tường hai bên song song với hướng y của khúc này nên trượt dọc theo nhau. Phải dùng "ô đúng" để tách: +5,5 cm (0,877) hơn +8,0 cm (0,844).
- **So với mong đợi:** lệch ≈ 5 cm (RA_SOAT_120.md 3.2 ghi "khoảng 5 cm" theo y, ước bằng mắt trên hình). Kết quả **8,9 cm** khác **3,9 cm > 2 cm**, nên **DỪNG theo lệnh, chưa làm b)**.
  - Ghi chú: RA_SOAT_120.md mục 4.3 (tịnh tiến, trung vị khoảng cách, scan 21:06:30, tư thế (0,16; 0,76; 42°)) đã ra **(+5; +7) cm, |lệch| 8,6 cm** [tính]. Kết quả a) gần với số này (dx, dy đổi chỗ nhau do tiêu chí khác và cao nguyên dy rộng), không gần 5 cm.
- Tư thế chỉnh theo đỉnh: (0,225; 0,811; 42,2°) [tính].

### Chờ Thiên quyết
1. Chấp nhận mốc đối chứng ≈ 9 cm (theo 4.3 và a)) thay cho 5 cm rồi làm b).
2. Đổi tiêu chí điểm (ví dụ "ô đúng", hoặc trung vị khoảng cách tới ô) để dy rõ hơn rồi làm lại a).
3. Dừng việc khớp.

## B2b. Khớp tiếp (Thiên sửa mốc đối chứng: 8,6 cm theo RA_SOAT_120 mục 4.3, đo bằng số; a) 8,9 cm ⇒ **đạt**). CHỈ ĐỌC

Cách chấm giữ nguyên như B2 (tỉ lệ tia rơi vào ô vật v3, cho sai 1 ô; "đúng ô" làm tiêu chí phụ). Script scratchpad `r120/khop118b.py`.
- **Khung thân:** đổi theo góc xe **tin** θ: dọc = dx·cos θ + dy·sin θ (+ = về phía mũi), ngang = −dx·sin θ + dy·cos θ (+ = trái) [tính].
- **Tập hoà điểm** = mọi ô lưới đạt đúng điểm cao nhất (cho sai 1 ô) [tính].

### 1) Đối chứng a) và scan 21:06:30
| scan | tư thế tin [log] | (0,0,0) | cao nhất (bản đồ) | cao nhất (thân) | điểm / đúng ô | đỉnh thứ hai | tập hoà điểm: bản đồ | tập hoà điểm: thân |
|---|---|---|---|---|---|---|---|---|
| a) scan_ket_5b3 21:05:07 | (0,155; 0,756; 42,2°) | 0,764 / 0,105 | dx +7,0, dy +5,5 cm, dθ 0,0°; \|lệch\| **8,9 cm** | dọc **+8,9**, ngang **−0,6** cm | 0,962 / 0,877 | dx +7,0, dy +8,0, 0,0° (dọc +10,6, ngang +1,2) 0,962 / 0,844 | 91 nghiệm: dx +7,0..+8,0, dy +1,0..+9,5 cm, dθ −2,0..0,0° | **dọc +6,2..+12,3, ngang −4,3..+1,7 cm** |
| scan_ket_k1_2106 21:06:30 | (0,155; 0,756; 42,7°) (mẫu tf +0,11 s) | 0,786 / 0,121 | dx +8,0, dy +5,5 cm, dθ −1,0°; \|lệch\| 9,7 cm | dọc **+9,6**, ngang **−1,4** cm | 0,966 / 0,734 | dx +8,0, dy +8,0, −1,0° (dọc +11,3, ngang +0,5) 0,966 / 0,700 | 46 nghiệm: dx +8,0..+8,5, dy +1,0..+9,0 cm, dθ −1,5..−0,5° | **dọc +6,6..+12,4, ngang −4,7..+0,8 cm** |

[tính]

- Hai scan cùng chỗ đứng cho cùng kết quả trong khoảng 1 cm [tính]: xe thật ở **phía trước** tư thế tin khoảng **9-10 cm theo dọc thân**, còn ngang gần 0 (−1,4..−0,6 cm ở đỉnh).
- Theo khung bản đồ thì dy bất định (+1..+9,5 cm). Đổi sang khung thân, phần bất định chủ yếu nằm theo **dọc** (6-12 cm), vì khúc này tường chạy chéo 42° so với trục xe.

### 2) b) scan_ket_5b2 20:58:16, tư thế tin (0,313; 0,796; 8,5°) [log]
| | dx | dy | dθ | thân dọc | thân ngang | điểm (sai 1 ô) | đúng ô | \|lệch\| |
|---|---|---|---|---|---|---|---|---|
| (0,0,0) | 0 | 0 | 0 | 0 | 0 | 0,535 | 0,057 | 0 |
| **cao nhất** | **+8,5** | **+7,5** cm | **0,0°** | **+9,5** | **+6,2** cm | **0,992** | 0,848 | **11,3 cm** |
| đỉnh thứ hai | +8,5 | +9,5 cm | 0,0° | +9,8 | +8,1 cm | 0,992 | 0,846 | 12,8 cm |
| tập hoà điểm (75 nghiệm) | +8,5..+9,5 | +2,5..+10,5 cm | −1,5..0,0° | **+8,8..+10,9** | **+1,2..+9,1** cm | 0,992 | 0,848..(thấp hơn) | |

[tính]

- 389/420 tia hợp lệ [log]. Điểm tại (0,0,0) chỉ 0,535: scan ở tư thế tin lệch rõ khỏi v3 (hình: tường trên và mặt dưới K2 theo scan nằm thấp hơn v3 khoảng 8 cm) [tính].
- Dọc thân xác định khá rõ (**+8,8..+10,9 cm**), cùng chiều và cùng cỡ với a) [tính].
- Ngang thân bất định (+1,2..+9,1 cm), luôn **dương (về bên trái)** [tính]. Đỉnh hơn đỉnh thứ hai chỉ 0,002 điểm "đúng ô", nên **không rõ ràng** theo ngang.

### 3) Footprint 25 × 16 cm (x −0,105..0,145, y ±0,08) tới ô vật v3 phía K2 (b)
- Ô K2 = ô vật v3 có tâm 0,70 ≤ x ≤ 1,20 và 0,95 ≤ y ≤ 1,20, tức khối K2 nối tường trái. Khoảng cách = cạnh footprint → cạnh ô, lấy mẫu 0,5 cm [tính].

| tư thế | footprint → ô K2 gần nhất | ô gần nhất (tâm) |
|---|---|---|
| tin (0,313; 0,796; 8,5°) | **27,7 cm** | (0,709; 1,060) |
| chỉnh theo đỉnh (0,398; 0,871; 8,5°) | **16,7 cm** | (0,709; 1,060) |
| cả 75 nghiệm hoà điểm | **14,8..19,2 cm** | |

[tính]

- **Lệch đẩy xe về phía K2:** cả 75/75 nghiệm đều gần K2 hơn tư thế tin, gần hơn 8,5-12,9 cm [tính]. Vì xe thật ở phía trước và lệch trái so với tư thế tin, ô K2 gần nhất (góc dưới-trái của khối K2) gần hơn tư thế mà Nav2/sếp tin.

### 4) Hình RA_SOAT_118_b2.png (1 cm = 4 px, lưới 5 cm, khung bản đồ x 0..1,35, y 0,45..1,22)
- Ô vật v3 tô hồng, ô K2 viền cam.
- Scan 5b2 và footprint ở tư thế tin (xanh dương) và tư thế chỉnh theo đỉnh (xanh lá); đường từ tâm tới mũi chỉ hướng.
- Ở tư thế chỉnh, scan nằm khít tường trên, tường dưới, mặt trái và mặt dưới K2.

### 5) tf tĩnh base_footprint → laser của Pi
- **du_lieu_0511_toi: không có** [log]. Chỉ có `frame_id: laser_frame` trong 3 scan và dòng "Message Filter dropping message: frame 'laser_frame'" trong log_bat_xe_nav.txt. Không có bag (kho không có *.db3 / *.mcap / *.bag).
- **Có ở du_lieu_0211/chup_loi_204624_loi.txt:18** (danh sách tiến trình trên Pi, 02/11 20:46:24 +0900) [log]:
  `/opt/ros/humble/lib/tf2_ros/static_transform_publisher 0.025 0 0.1675 0 0 0 1 base_link laser_frame --ros-args -r __node:=static_tf_pub_laser`
  - tức base_link → laser_frame = x 0,025, y 0, z 0,1675 m; quaternion (0, 0, 0, 1) = **không xoay** [log/tính].
  - Không có trong kho my_bot (URDF không có link LiDAR; RA_SOAT_120 mục 2.1). BAO_CAO_SU_CO_31_10_CHIEU.md:55 liệt kê `static_tf_pub_laser` là tiến trình con của `ros2 launch` [log]. mapping_my_bot.launch.py:152-156 include `ydlidar_ros2_driver/launch/ydlidar_launch.py` (gói nằm trên Pi) [đọc code]. Suy ra nút này nhiều khả năng do launch driver YDLidar tạo; chưa đọc được tệp đó.
  - Khung cha là base_link. base_footprint → base_link = (0, 0, wheel_radius 0,0325), không xoay [đọc code: dev_ws/src/my_bot/description/robot.urdf.xacro:8, :19-22].
  - Vậy base_footprint → laser_frame = **(0,025; 0; 0,200) m, yaw 0** [tính], **khớp** `LIDAR_X = 0.025`, không xoay (lai_xe_lidar.py:28) [đọc code].
  - Đây là số **ngày 02/11**; chưa có bằng chứng ngày 05/11 giữ nguyên.

## B3. Odom, SLAM và bước nhảy (CHỈ ĐỌC)

### 1) odom_tu_lenh cộng dồn tốc độ nào
**Tham số trên xe** (bản chụp tiến trình Pi 02/11):
- du_lieu_0211/chup_loi_204624_loi.txt:10 là lệnh `ros2 launch my_bot mapping_my_bot.launch.py` với `bat_dong_co:=true bat_lidar:=true bat_camera:=false bat_gyro:=true bat_mo_hinh_san:=true` [log] (chỉ trích các cờ `bat_*`; dòng này còn tham số `access_code` đã bị xoá thành `XOA`, không chép).
- Launch truyền cho odom_tu_lenh **chỉ** `yaw_nguon` (= `gyro` khi `bat_gyro`) và `mo_hinh_san` (= true) [đọc code: dev_ws/src/my_bot/launch/mapping_my_bot.launch.py:126-140]. Truyền cho dieu_khien **chỉ** `max_wheel_speed` (mặc định launch 0,437) [:87-91, :157-164].
- Các tham số còn lại dùng mặc định trong mã [đọc code]:
  - odom_tu_lenh.py:74-83: `max_wheel_speed 0.437`, `wheel_separation 0.155`, `he_so_trai 0.92`, `duty_min_trai/phai 0.21`, `duty_ngung 0.05`, `he_so_xoay 0.46`, `tre_khoi_dong_xoay 0.5`; :57-60 `scale_linear 1.0`, `min_linear_effective 0.0`;
  - dieu_khien.py:36-52: `wheel_separation 0.155`, `he_so_trai 0.92`, `duty_min_trai/phai 0.21`, `duty_ngung 0.05`.
- Chưa có bằng chứng ngày 05/11 dùng cùng cờ.

**odom_tu_lenh** [đọc code, dev_ws/src/my_bot/scripts/odom_tu_lenh.py]:
- Nghe `/cmd_vel`, tức đầu ra twist_mux = **lệnh gốc** đã qua mux, không phải tốc độ bánh đo được (:51 `cmd_topic`, :163, :174-177).
- `effective_velocity` (:254-293):
  - watchdog `cmd_timeout` 0,5 s (:257-259);
  - `min_linear_effective` (:263-266, = 0 nên không tác dụng);
  - khi `mo_hinh_san` (:267-291): chia lệnh thành bánh phải/trái. Duty mỗi bánh **chỉ dùng để phân loại** (:273, bánh trái nhân `he_so_trai`):
    - duty ≥ `duty_min` ⇒ cộng **tốc độ bánh theo lệnh** `s` (:275), **không** nhân `he_so_trai`, **không** kẹp `max_wheel_speed` (duty ≥ 1 vẫn cộng `s`);
    - `duty_ngung` ≤ duty < `duty_min` ⇒ cộng `duty_min × max_wheel_speed` = 0,0918 m/s (:278-280), **cả bánh trái cũng không chia `he_so_trai`**;
    - duty < `duty_ngung` ⇒ 0;
    - bánh bị nâng thì `w *= he_so_xoay` (:283-284).
- `tick` (:295-315): cộng `v` theo hướng; **góc lấy từ gyro** khi có (:301, :303-309), nên ω tính ở trên không vào góc.

**dieu_khien** [đọc code, dev_ws/src/my_bot/scripts/dieu_khien.py]:
- bánh trái nhân `he_so_trai` (:109-110);
- duty = |s| / `max_wheel_speed`, kẹp [0, 1] (:114);
- duty < `duty_ngung` ⇒ 0; duty < `duty_min` ⇒ nâng lên `duty_min` (:115-121).

**Sai lệch odom** [tính]. Giả định mô hình vật lý (không có encoder để kiểm): bánh phải = duty × 0,437; bánh trái = duty × 0,437 / 0,92 (`he_so_trai` bù bánh trái nhanh hơn); gyro đúng.

| lệnh thẳng | duty trái / phải | thật (giả định) | odom (`mo_hinh_san` true, như xe 02/11) | sai mỗi giây | nếu `mo_hinh_san` false |
|---|---|---|---|---|---|
| 0,05 m/s | 0,210 / 0,210 (cả hai bị nâng lên duty_min) | v 0,0958 m/s, ω −0,052 rad/s (−2,95°/s, quay phải vì bánh trái nhanh hơn) | 0,0918 m/s | **−0,40 cm/s** (odom hụt); góc theo gyro nên không sai góc | odom 0,050: **−4,58 cm/s** |
| 0,15 m/s | 0,316 / 0,343 | v 0,150 | 0,150 | **0** (theo giả định) | 0 |

- Ở 0,15 m/s, odom chỉ đúng nếu 0,437 m/s ở duty 1,0 và `he_so_trai` 0,92 đúng ngoài thực tế. Sai số theo các hệ số này **chưa đo được** (không có encoder).

### 2) Tham số slam_toolbox
- Xe dùng `SLAM=/home/project1/slam_dinh_vi_xuong.yaml` [đọc code: xe_scripts/bat_xe.sh:30]. Bản trong kho là xe_scripts/du_lieu_0111/slam_dinh_vi_xuong.yaml (commit e2fdb9e, 01/11), có trong danh sách so_voi_pi.sh:36. **Chưa so md5 với Pi** cho ngày 05/11.

| tham số | du_lieu_0111/slam_dinh_vi_xuong.yaml | dev_ws/src/my_bot/config/slam_toolbox_params.yaml (launch mặc định, không dùng khi chạy bat_xe) |
|---|---|---|
| mode | **localization** (:26) | mapping (:26) |
| transform_publish_period | **0.05** (:38) | 0.05 (:38) |
| minimum_travel_distance | **0.25** (:52) | 0.25 (:52) |
| minimum_travel_heading | **0.25** rad (:53) | 0.25 (:53) |
| khác | throttle_scans 2 (:37), map_update_interval 5.0 (:39), use_scan_matching true (:50), scan_buffer_size 10 (:54), do_loop_closing true (:59) | |

[đọc code]

- du_lieu_* không có bản tham số nào khác.
- Ghi chú [đọc code + tính]: SLAM chỉ khớp scan khi xe đã đi ≥ 0,25 m hoặc quay ≥ 0,25 rad (14,3°). Vì vậy lúc đứng hoặc nhích ngắn (lùi 12 cm, tiến 10 cm), map → odom không được sửa, và vị trí tin chỉ theo odom.

### 3) Bước nhảy trong vet_5b2 / vet_5b3
**Tiêu chí** (theo lệnh) [tính]:
- tốc độ ngầm giữa hai mẫu liên tiếp > 0,437 m/s, hoặc
- dời > 2 cm lúc xe đứng (bước trước **và** bước sau đều dời < 0,5 cm).
- Hướng tính theo góc của mẫu đầu: dọc + = về mũi, ngang + = trái.

| tệp | số mẫu, khoảng [log] | khoảng cách mẫu [tính] | tốc độ ngầm lớn nhất [tính] | **số bước nhảy theo tiêu chí** |
|---|---|---|---|---|
| vet_5b2.txt | 148, 20:56:55.47 → 20:59:22.37 | 0,77-1,18 s | 0,167 m/s | **0** |
| vet_5b3.txt | 149, 21:04:06.58 → 21:06:34.68 | 0,87-1,13 s | 0,148 m/s | **0** |

Các bước nhanh nhất để tham khảo (≥ 0,12 m/s; RPP `desired_linear_vel` 0,15) [tính]:

| tệp | bắt đầu | dời / dt | tốc độ | dọc | ngang | đổi góc | đối chiếu sep_nhat_ky [log] |
|---|---|---|---|---|---|---|---|
| 5b2 | 20:58:19.47 | 16,7 cm / 1,00 s | 0,167 | +15,7 | +5,8 cm | −1,1° | sau lần cứu, đang chạy lượt t2 l2 |
| 5b2 | 20:58:24.47 | 16,3 cm / 1,00 s | 0,163 | +15,2 | +6,0 cm | +8,8° | **`NHAY_VI_TRI` 20:58:25 "0.10 m 1.2 do trong 1.0 s"** |
| 5b2 | 20:57:38.47 | 12,7 cm / 1,00 s | 0,127 | +12,7 | +1,2 cm | −5,6° | ngay trước ABORTED 20:57:42.6 |
| 5b2 | 20:57:35.47 | 12,2 cm / 1,00 s | 0,122 | +12,1 | +0,5 cm | −12,9° | |
| 5b3 | 21:04:43.68 | 14,9 cm / 1,01 s | 0,148 | +13,2 | +6,9 cm | −14,3° | **`NHAY_VI_TRI` 21:04:43 "0.14 m 1.8 do trong 1.0 s"**, ngay trước ABORTED 21:04:44.35 |

- `NHAY_VI_TRI` của sếp đo **thay đổi map → odom** (phần SLAM tự sửa), ngưỡng ≥ 0,10 m hoặc ≥ 5° trong < 1 s [đọc code: sep.py:58, :170-181, :630-641].
- Hai lần SLAM sửa 10-14 cm này [log] **không lộ ra** theo tiêu chí trên vết 1 Hz: chúng cộng vào chuyển động thật nên tốc độ ngầm chỉ 0,15-0,16 m/s [tính].
- Cả hai có **thành phần ngang +6-7 cm (về bên trái)** [tính]. Lần 21:04:43 rơi đúng 0,7 s trước khi RPP báo va chạm và huỷ.

## B4. Các lần SLAM sửa map → odom (CHỈ ĐỌC; không in dòng nào chứa ACCESS)

### 0) THIET_KE_109_111.md:69 (chỉ đếm, không in)
Dòng 69 có **1** chữ `ACCESS`, **0** lần chữ đó đi liền dấu bằng, nên **không** có giá trị nào theo sau [tính, đếm bằng `grep -o … | wc -l`]. Dòng này là tài liệu nhắc tới dòng ACCESS của bat_xe.sh, không chứa giá trị.

### 1) Mọi lần map → odom đổi trong sep_nhat_ky.csv đêm 05/11
- sep_nhat_ky **không ghi map → odom liên tục**. Sếp chỉ ghi `TRUY_VET NHAY_VI_TRI BAT` khi map → odom (tính tại vị trí xe) đổi **≥ 0,10 m hoặc ≥ 5° trong < 1 s** [đọc code: sep.py:58 `NHAY_M, NHAY_DEG, NHAY_DT = 0.10, 5.0, 1.0`; :170-181; :630-641]. Cột x, y, góc là tư thế xe trong map, không phải map → odom.
  - **Các lần đổi 2-10 cm không được ghi ở đâu** [log]: du_lieu_* không có bag và không có vết map → odom.
- sep_nhat_ky (du_lieu_0511_toi, bản đủ nhất) ngày 05/11: 961 dòng, 18:01:15 → 21:07:42 [log].
- Đổi giờ: epoch = gio_mono + 1793867303.12; `gio_mono` tăng liên tục cả tối, không khởi động lại máy [tính, mốc 21:04:44.35 ở B1].

| # | giờ (sep / chính xác [tính]) | chuyến / tuyến [log] | sep ghi [log] | vector ước [tính, ước] khung map | khung thân (góc xe đầu bước) | quãng từ mốc trước [tính] |
|---|---|---|---|---|---|---|
| 1 | 18:57:58 / 18:57:58.24 | di_B, tuyến 2 lượt 2: sau ABORTED 18:57:17 ở (0,46; 0,81) và RA_LUI 18:57:47-49 (lùi 0,46 → 0,34), đang chạy; SUCCEEDED 18:58:02 | 0,13 m, 0,6° | (+9,0; −0,1) cm, \|9,0\| | góc −8,7°: dọc **+8,9**, ngang +1,3 | 2,61 m từ lúc sep bật 18:54:06 |
| 2 | 20:44:57 / 20:44:57.87 | ve_A, tuyến 1 lượt 1 (bắt đầu 20:44:35), đang chạy, gần A; SUCCEEDED 20:45:01 | 0,11 m, 0,4° | **không ước được** (không có vết phủ giờ này) | — | 3,59 m từ lúc sep bật 20:41:49 |
| 3 | 20:58:25 / 20:58:25.08 | di_B, tuyến 2 lượt 2: sau ABORTED 20:57:42.6 và RA_LUI 20:58:12.6-13.6 (lùi 11,9 cm), đang chạy; SUCCEEDED 20:58:30 | 0,10 m, 1,2° | (+8,8; +2,4) cm, \|9,1\| | góc −19,3°: dọc **+7,5**, ngang +5,2 | 1,88 m từ lúc sep bật 20:55:39 |
| 4 | 21:04:43 / 21:04:43.75 | di_B, tuyến 2 lượt 1 (nhận đích 21:04:34.15), đang đi cung; RPP huỷ 21:04:44.35 | 0,14 m, 1,8° | (−1,8; +9,6) cm, \|9,8\| | góc 56,4°: dọc **+7,0**, ngang +6,8 | 0,75 m từ lúc sep bật 21:02:40 |

**Cách ước vector** [tính, ước]:
- lấy bước vết (tf2_echo map → base_footprint, ~1 Hz) chứa giờ chính xác, trừ chuyển động dự kiến:
  - lần 1, 3: trung bình hai bước kề;
  - lần 4: bước trước × 0,67, vì xe dừng lúc RPP huỷ 21:04:44.35.
- Vết dùng: du_lieu_0511/vet_t3.txt (18:55:37-19:00:35) cho lần 1; du_lieu_0511_toi/vet_5b2.txt cho lần 3; vet_5b3.txt cho lần 4.
- Độ lớn ước (9,0-9,8 cm) khớp cỡ sep ghi (0,10-0,14 m; số của sep gồm cả phần do xoay).

**Quãng không có lần sửa nào (≥ 10 cm)** [tính]:
- Tính theo tư thế sep ghi (mọi dòng có x, y, làm tròn 0,01 m), giữa các mốc "sep vừa bật" (`MAT_SCAN tuoi None`) và các lần `NHAY_VI_TRI`. Vết chỉ phủ 3 khoảng ngắn nên không dùng cho cả đêm được.
- Giả định: mỗi lần sep bật là một lần bat_xe.sh bật lại cả launch (SLAM khởi động lại, map → odom đặt lại).

| đoạn | quãng |
|---|---|
| **20:47:24 → 20:55:39** (sep bật → sep bật lại, không lần sửa nào) | **4,02 m** (dài nhất) |
| 18:45:25 → 18:54:06 | 4,01 m |
| 18:18:27 → 18:45:25 | 1,96 m (+ nghỉ) |
| 18:09:07 → 18:18:27 | 3,92 m |
| 18:01:15 → 18:09:07 | 3,88 m |
| 20:41:49 → 20:44:57 (tới lần 2) | 3,59 m |

- Bảy lần sep bật trong tối, thì có 3 lần chạy hết ≈ 4 m mà không có lần sửa ≥ 10 cm nào [tính]. Có sửa < 10 cm hay không thì không biết.

### 2) So vector sửa với lệch đo bằng khớp scan (khung map; "lệch" = tư thế thật − tư thế tin)
| | lệch đo bằng khớp scan [tính, B2b] | lần sửa [tính, ước] | còn lại sau sửa [tính] |
|---|---|---|---|
| **20:58:25** (lần 3) vs b) 5b2 chụp **trước** 9 s (20:58:16, xe đứng sau lùi) | đỉnh (+8,5; +7,5) cm; tập hoà điểm dx +8,5..+9,5, dy +2,5..+10,5 | (+8,8; +2,4) | theo đỉnh: (−0,3; +5,1); trong tập hoà điểm: dx −0,3..+0,7, dy +0,1..+8,1 |
| **21:04:43** (lần 4) vs a) 5b3 chụp **sau** 23 s (21:05:07, xe đứng) | sau sửa: đỉnh (+7,0; +5,5) cm, dọc +8,9 / ngang −0,6 | (−1,8; +9,6) | trước sửa (suy ngược) ≈ (+5,2; +15,1), \|16,0\| cm |

- **Lần 3 kéo về tư thế chỉnh:** theo x gần như đủ (+8,8 so với +8,5..+9,5). Theo y được 2,4 cm trong khoảng cần +2,5..+10,5, tức nằm ở mép dưới tập hoà điểm, nên có thể còn thiếu tới 8 cm theo y [tính]. Cùng chiều, cùng cỡ: lần sửa này là SLAM tự sửa đúng phần lệch mà khớp scan thấy lúc 20:58:16.
- **Lần 4 kéo về một phần:** lệch ước trước sửa ≈ 16 cm, sau sửa còn 8,9 cm [tính, ước].
- **Vì sao sau 21:04:43 vẫn lệch 8,9 cm:**
  - (a) Sau 21:04:44.69 xe **đứng yên** 110 s, x, y không đổi tới mm [log, B1]. Mà slam_toolbox chỉ khớp scan mới khi xe đã đi ≥ 0,25 m hoặc quay ≥ 0,25 rad [đọc code: du_lieu_0111/slam_dinh_vi_xuong.yaml:52-53]. Vậy không có lần khớp nào sau đó để sửa tiếp: map → odom đứng yên, và lệch 8,9 cm giữ nguyên tới lúc chụp 5b3 (21:05:07) và k1 (21:06:30, lệch 9,7 cm).
  - (b) Lần sửa 21:04:43 là kết quả khớp **một** scan lúc đang chạy. Vì sao SLAM chỉ sửa khoảng một nửa thì dữ liệu không cho biết (không có log slam_toolbox, không có bag) [log].
  - Khớp scan của mình cũng **không rõ theo dọc thân** (tập hoà điểm dọc +6,2..+12,3 cm, B2b), nên phần "còn lại" có sai số cỡ ±3 cm [tính].
- **Ba lần ước được vector đều có thành phần dọc thân dương** +7,0..+8,9 cm [tính, ước]: SLAM luôn kéo xe **về phía trước** so với odom, cùng chiều với lệch đo bằng khớp scan (dọc +8,8..+10,9 ở b), +8,9 ở a)). Tức là odom ghi **hụt** quãng đi tới, lặp lại mỗi khoảng 2-4 m.
  - Phù hợp với B3: odom hụt khi lệnh nhỏ (giả định duty). Nhưng −0,4 cm/s theo giả định thì quá nhỏ để giải thích 9 cm sau 2-4 m [tính], nên **chưa giải thích được** bằng mô hình duty.

### 3) Scan chụp lúc xe đứng ở vạch A ngay sau SAN SANG, hoặc ngay sau một lần sửa map → odom lúc xe đứng
- Mọi du_lieu_*: chỉ có 4 tệp scan [log]: du_lieu_0511/scan_ket_k2_1933.txt (19:33:58, kẹt K2, bị cắt), du_lieu_0511_toi/scan_ket_5b2.txt (20:58:16), scan_ket_5b3.txt (21:05:07), scan_ket_k1_2106.txt (21:06:30). Các tệp `du_lieu_0211/chup_*_ss_*.txt` là kết quả lệnh `ss` (socket), không phải scan.
- **Ở vạch A ngay sau SAN SANG: không có.**
- **Ngay sau một lần sửa map → odom khi xe đứng: không có.**
  - Gần nhất là 5b3 và k1, chụp lúc xe đứng 23 s / 106 s **sau** lần sửa 21:04:43. Nhưng lần sửa đó xảy ra **lúc xe đang chạy** (0,9 s trước khi đứng), và đã khớp ở B2b: dọc +8,9 / +9,6 cm, ngang −0,6 / −1,4 cm.
  - 5b2 chụp trước lần sửa 20:58:25, không phải sau.
- Đề xuất cho lần chạy sau (CHƯA làm):
  - chụp `ros2 topic echo --once --full-length /scan` và `tf2_echo map base_footprint` ngay sau SAN SANG ở A, rồi một lần nữa sau khi xe quay tại chỗ ≥ 15° (đủ ngưỡng 0,25 rad để SLAM khớp);
  - ghi kèm `tf2_echo map odom` 1 Hz cả chuyến.

## B5. Đường di_B tuyến 2 qua khe trên: hiện tại và "căn giữa" (CHỈ ĐỌC; không sửa mã / tham số)
Script scratchpad `r120/duong118.py`, `r120/b5118.py` (không vào kho). Hình RA_SOAT_118_b5.png.

### 1) Đường hiện tại
- **Log:** không có danh sách điểm của lượt 20:57. log_nut_chuyen.txt chỉ có chuyến 21:04 ("tuyen 2, lan 1/3: 48 diem tu (-0.00; 0.00) toi (1.60; 0.82)") [log].
- **Dựng lại như trên xe:**
  - gui_duong gọi `TT.Ban.tu_luoi` trên `/global_costmap/costmap` rồi `TT.tinh_duong(ban, pose, B, 2, yaw_cuoi=0.0)` [đọc code: gui_duong.py:37, :124; tinh_tuyen.py:92-101, :303-327, @579cda5].
  - Costmap toàn cục dựng lại từ map_xuong_v3 [tính]: ô vật = 254; còn lại theo InflationLayer 1.1.20 (`inflation_radius` 0,20, `cost_scaling_factor` 3,0 theo nav2_params_cho.yaml:192-193; bán kính nội tiếp 0,09 do footprint ±0,08 + padding mặc định 0,01), đổi sang OccupancyGrid bằng bảng 1.1.20 (254 → 100, 253 → 99, còn lại 1 + 97(c−1)/251).
  - Lớp obstacle (scan sống) **không** có trong bản dựng.
  - Điểm đầu (0,01; 0,00) = tư thế sep ghi lúc gửi t2 l1, 20:57:27 [log].
- **Kết quả:** 47 điểm, đúng từng điểm với `TT.tinh_duong` [tính] (trên xe 21:04 là 48 điểm từ (−0,00; 0,00) [log]). Không có cảnh báo đỉnh rẽ.

| đỉnh | (x; y) | rẽ | tới tâm ô vật v3 gần nhất | `cach_do` tinh_tuyen (tới ô ≥ 98) |
|---|---|---|---|---|
| 0 | (0,010; 0,000) | — | 0,340 m | 0,300 m |
| 1 | (0,059; 0,310) | 26,0° | 0,502 m | 0,450 m |
| **2** | **(0,409; 0,810)** | **57,5°** | 0,335 m (ô (0,559; 0,510), góc trên-trái H0) | 0,269 m |
| 3 | (1,559; 0,760) | 58,1° | 0,400 m | 0,350 m |
| 4 = B | (1,600; 0,820) | — (góc cuối 0°) | 0,340 m | 0,300 m |

[tính]

- Đỉnh 2 trùng chỗ kẹt 20:57:42, tư thế tin (0,43; 0,81; 7°) [log]. Xe đã xoay một phần ở đỉnh 2 (từ 55° về 7°) thì RPP huỷ.

**Khe trên đo trên v3** [tính]:
- x 0,60: mép dưới (đỉnh H0) 0,535, mép trên (tường) 1,135, rộng **0,600**.
- x 0,75-1,05: mép trên là đáy K2 0,985, rộng **0,450**, giữa khe **y 0,760**.
- x ≤ 0,45 và x ≥ 1,20: rộng ≥ 1,30 m.
- Đoạn 2-3 hiện tại chạy từ y 0,810 xuống 0,760, nên trong khe hẹp (x 0,73 → 1,18) nó cao hơn giữa khe 3,6 → 1,6 cm, tức **lệch về phía K2** [tính].

### 2) Phương án "căn giữa" (trên giấy, không sửa tinh_tuyen)
- Đỉnh (0,01; 0,00) → (0,059; 0,310) → **(0,34; 0,76)** → **(1,53; 0,76)** → B (1,60; 0,82) [tính]:
  - đỉnh 2 = điểm xa ô vật nhất trên y 0,76 trong vùng rẽ (x 0,05-0,60): tới tâm ô vật 0,332 m. Điểm xa nhất cả vùng rẽ là (0,36; 0,80), 0,351 m; đỉnh 2 hiện tại đã gần chỗ này;
  - đoạn 2-3 nằm **đúng giữa khe**, y 0,760;
  - đỉnh 3 = điểm xa nhất trên y 0,76 sau khe (x 1,18-1,55): 0,401 m.
- Góc rẽ: đỉnh 1 23,0°, đỉnh 2 58,0°, đỉnh 3 40,6° (hiện tại 58,1°) [tính].
- Đỉnh 2 có `cach_do` 0,250 m, **lớn hơn** `R_XOAY_CAN` 0,2356, nên đạt luật đỉnh rẽ của tinh_tuyen [tính; đọc code tinh_tuyen.py:40].

### 3) Footprint 25 × 16 cm dọc hai đường
**Cách đặt** [tính]:
- Tư thế tin: mỗi đoạn bước 2 cm, hướng theo đoạn; ở mỗi đỉnh trong và ở B xoay tại chỗ từng 5° (B xoay về 0°).
- Tư thế thật = tin + R(θ)·(dọc, ngang), dọc 0 / +9 / +14 cm × ngang −6 / 0 / +6 cm (+ = trái).
- Khoảng cách = thân (x −0,105..0,145, y ±0,08, không padding) tới ô vật v3, mỗi ô lấy mẫu lưới 0,5 cm; âm = đè.
- Số tư thế: 147 (hiện tại), 139 (căn giữa).

**Cả đường** (nhỏ nhất, chỗ xảy ra, số tư thế < 6 cm; không tư thế nào đè) [tính]:

| lệch dọc / ngang | hiện tại | căn giữa |
|---|---|---|
| 0 / −6 | +10,1 cm, đoạn 2-3 (1,26; 0,77) → H0 | 0 tư thế < 6 | +8,5, đoạn 2-3 (0,40; 0,76) → góc H0 | 0 |
| 0 / 0 | +10,9, đoạn 2-3 (0,59; 0,80) → góc K2 (0,759; 1,010) | 0 | +14,5, đoạn 2-3 | 0 |
| 0 / +6 | **+4,9**, đoạn 2-3 (0,59; 0,80) → góc K2 | **27** | +8,5, đoạn 2-3 (0,60; 0,76) → góc K2 | 0 |
| +9 / −6 | +9,8, đỉnh 2 (55°) → góc K2 | 0 | +8,5, xoay đỉnh 2 → góc H0 | 0 |
| +9 / 0 | +7,6, cuối đường ở B (56°) → tường trên | 0 | +10,1, ở B (41°) → tường trên | 0 |
| +9 / +6 | **+4,2**, ở B (56°) → tường trên | **33** | +5,6, ở B (41°) | 1 |
| +14 / −6 | +5,2, đỉnh 2 (55°) → góc K2 | 2 | +6,7, xoay ở B | 0 |
| +14 / 0 | +3,5, ở B (56°) | 7 | +6,9, ở B (41°) | 0 |
| +14 / +6 | **+0,1**, ở B (56°) → tường trên (1,66; 1,16) | **48** | **+2,3**, ở B (41°) | **6** |

**Chỉ khe trên + chỗ rẽ** (tư thế tin có x 0,30-1,35; 68 / 66 tư thế) [tính]:

| lệch dọc / ngang | hiện tại: nhỏ nhất (chỗ) | < 6 cm | căn giữa: nhỏ nhất (chỗ) | < 6 cm |
|---|---|---|---|---|
| 0 / −6, 0 / 0, 0 / +6 | +10,1 / +10,9 / **+4,9** (đoạn 2-3 dưới góc K2) | 0 / 0 / **27** | +8,5 / +14,5 / +8,5 | 0 / 0 / 0 |
| +9 / −6, 0, +6 | +9,8 / +8,7 / **+4,9** (đỉnh 2 → tường trên; đoạn 2-3 → góc K2) | 0 / 0 / **29** | +8,5 / +13,3 / +8,5 | 0 / 0 / 0 |
| +14 / −6, 0, +6 | +5,2 / +4,6 / **+1,1** (đỉnh 2, hướng 55°: mũi → tường trên (0,46; 1,16)) | 2 / 3 / **40** | +8,5 / +9,1 / **+5,9** (đỉnh 2, 58°) | 0 / 0 / **1** |

- Trong khe và chỗ rẽ, "căn giữa" giữ thân cách ô vật **≥ 5,9 cm** ở cả 9 mức lệch. Đường hiện tại xuống **+1,1 cm**, và có 27-40 tư thế < 6 cm khi lệch ngang +6 cm (về phía K2 / tường trên) [tính].
- Điểm xấu ở **cuối đường** (cả hai đường) là đoạn 3 → B rẽ lên 56° / 41° rồi xoay về 0° sát tường trên (y 1,135). Với lệch dọc +14 / ngang +6 thì còn +0,1 / +2,3 cm. Phần này nằm ngoài khe trên và ngoài phạm vi phương án căn giữa [tính].
- Không tư thế nào đè (âm) [tính]. Chưa tính padding / inflation / RPP: đây là thân hình học tới ô vật v3, không phải phép kiểm va chạm của RPP (B4 của RA_SOAT_120).

### 4) Hình RA_SOAT_118_b5.png
- 1 cm = 4 px, lưới 5 cm, khung bản đồ x −0,30..1,95, y −0,15..1,22. Hai đường cạnh nhau (trái: hiện tại, phải: căn giữa).
- Ô vật v3 hồng, đường tin đen, các đỉnh tròn.
- Với mỗi mức lệch dọc (xanh 0 / cam +9 / đỏ +14), vẽ footprint xấu nhất trong 3 mức lệch ngang và ô vật gần nhất (vuông), kèm số cm.
- Dưới mỗi ô là bảng 3 × 3 cho cả đường và cho riêng khe + chỗ rẽ.

## B6. Viết mã: đường qua khe hẹp đi giữa khe, đỉnh rẽ ở chỗ rộng (CHƯA commit). HEAD = 579cda5 [log]

### Sửa (git diff: tinh_tuyen.py +112, test_gui_duong.py +65; không sửa tệp nào khác)
**Luật chung theo khoảng trống, không gắn toạ độ** [đọc code: tinh_tuyen.py, hằng mới cạnh `DOI_MAX`]:

1. **Dò khe** (`tia_trong`, `tim_khe`):
   - Ở mỗi điểm đường gốc (Dijkstra, tâm ô), bắn hai tia vuông góc với tiếp tuyến (`pl[i-1] → pl[i+1]`) tới ô `do` (vật ∪ dải cấm lớp 1 ≥ 72), bước 2 mm.
   - Tổng ≤ `W_KHE = 0.175` m thì điểm đó thuộc khe; tâm khe là trung điểm hai biên. Bỏ qua các điểm cách đầu ≤ `R_THA_CAM`.
   - Chuỗi ≥ 3 điểm khe liền nhau có các tâm thẳng hàng (lệch ≤ `TOL_KHE = 0.01` m) thì thành **một khe**, trục khe là đường bình phương nhỏ nhất.
2. **Qua khe** (`qua_khe`, gọi sau `doan_thang` trong `tinh_duong`):
   - Các đỉnh cũ trong vùng khe (± `DOI_MAX`) được thay bằng **2 đỉnh nằm trên trục khe**:
     - đỉnh vào cách đầu khe `L_KHE_VAO = 0.145` (tâm → mũi) tới + `DOI_MAX`;
     - đỉnh ra cách cuối khe `L_KHE_RA = 0.105` (tâm → đuôi) tới + `DOI_MAX`.
   - Trong khoảng đó chọn chỗ `cach_do` lớn nhất (**chỗ rộng**); hoà thì chọn chỗ gần khe nhất.
   - Đỉnh ra mà ngay sau là đích có `yaw_cuoi` thì ưu tiên chỗ làm đoạn cuối lệch `yaw_cuoi` < `GOC_XOAY` (khỏi xoay ở đích sát tường).
   - Ràng buộc như `doan_thang`: đoạn hợp lệ, và không gần vật hơn đường gốc đoạn đó (trừ hao 3 cm).
   - Đỉnh mới rẽ ≥ `GOC_XOAY` mà `cach_do` ≤ `R_XOAY_CAN` thì **CANH BAO** (giữ luật); CANH BAO của đỉnh bị bỏ thì xoá theo.
   - Không có chỗ hợp lệ thì giữ nguyên các đỉnh cũ của khe đó.
3. **Giữ nguyên:**
   - lớp 1 (Dijkstra cấm ô ≥ 72, `tha` quanh điểm đầu);
   - luật dời đỉnh và CANH BAO của `doan_thang` (không sửa dòng nào);
   - đầu ra `tinh_duong` vẫn là danh sách (x, y, yaw) bước ≤ 5 cm qua `lam_day_doan` mà gui_duong.py:124 đọc;
   - `main()` (bảng xs cho chuyen.sh) in **giống hệt** bản 579cda5 (diff rỗng) [tính].

**Ngưỡng `W_KHE`:**
- Nằm giữa 3 ô (0,15 m) và 4 ô (0,20 m) của dải đi được. Với dải cấm ≈ 0,155 m mỗi bên tính từ mặt vật, tương đương **khe vật lý dưới khoảng 0,48 m** [tính].
- Trên costmap dựng lại: khe trên = 3 ô (0,152 m) là khe; khe dưới = 4 ô (0,202 m) **không** là khe nên không đổi [tính].
- Trên `BAN` của test cũ (PGM, không có dải cấm) khe trên rộng hơn nên không là khe; test cũ không đổi.
- 10/11 quét phòng khác thì luật vẫn chạy theo độ rộng, không theo toạ độ. Khe rộng đúng 4 ô sẽ **không** được căn giữa.

**Test:**
- test_gui_duong **32/32** (cũ 30 + 2 mới), test_lai_xe_lidar 97/97, test_sep 46/46 [tính].
- Mới:
  - `test_31`: lưới 40 × 30. Khe 3 ô: 1 khe, trục đúng; đỉnh vào/ra trên trục, cách đầu/cuối khe ≥ 0,145 / 0,105; mọi điểm trong khe lệch trục ≤ 1 cm; đoạn hợp lệ; luật đỉnh rẽ. Khe 4 ô: không là khe, `qua_khe` trả nguyên.
  - `test_32`: map_xuong_v3 + `luoi_nav2`. di_B / ve_A tuyến 2 trong x 0,73-1,18 lệch y 0,760 ≤ 1 cm, không CANH BAO, đoạn cuối vào B lệch hướng đích **19,9°** < `GOC_XOAY` 20,05° (sát ngưỡng). Tuyến 1 **y hệt** cách cũ (`doan_thang` + `lam_day_doan`).
- md5: tinh_tuyen.py `f81e056df9e368e48b21855c08657068`, test_gui_duong.py `8bb20b5b8d5a4cbe4f2ff35e06bf292c` [tính].

### Đo theo cách B5 (script scratchpad `r120/b6118.py`; costmap dựng lại từ map_xuong_v3 như B5)
**Cách đo** [tính]:
- Bản cũ = `git show 579cda5:xe_scripts/tinh_tuyen.py`.
- Nhiễu xuất phát: dx, dy ∈ {−5, 0, +5} cm × hướng ∈ {−5°, 0, +5°}, tức 27 tổ hợp. di_B từ (0,01; 0,00) hướng 0; ve_A từ B hướng 0.
- Tư thế: xoay tại chỗ từ hướng xuất phát, đoạn thẳng bước 2 cm, xoay tại mỗi đỉnh và ở đích (di_B về 0°), bước 5°.
- Thân 25 × 16 cm tới ô vật v3 (cạnh ô lấy mẫu 0,5 cm); âm = đè.

**a) Lệch giữa khe trên** (|y − 0,760|, tâm xe trên các đoạn thẳng có 0,73 ≤ x ≤ 1,18; lớn nhất qua 27 nhiễu) [tính]:

| | cũ 579cda5 | mới |
|---|---|---|
| di_B tuyến 2 | 3,6 cm | **0,0 cm** |
| ve_A tuyến 2 | 3,9 cm | **0,0 cm** |

⇒ **đạt** (≤ 1 cm).

**b) di_B tuyến 2, lệch dọc +9 cm** (thân → ô vật nhỏ nhất qua 27 nhiễu; vùng theo x tin: rẽ vào 0,25-0,73, khe 0,73-1,18, cuối ≥ 1,18) [tính]:

| ngang | cũ: rẽ vào / khe / cuối | **mới: rẽ vào / khe / cuối** | ≥ 7,1? |
|---|---|---|---|
| −6 cm | 9,8 / 10,1 / 10,1 | **8,5 / 8,5 / 8,5** | đạt |
| 0 | 8,7 / 10,9 / 7,6 | **13,2 / 14,5 / 13,6** | đạt |
| +6 cm | 4,9 / 4,9 / 4,2 | **8,5 / 8,5 / 8,5** | đạt |

⇒ **đạt**.
- Ngoài ba vùng, đoạn xuất phát quanh A (x < 0,25) còn 7,1 cm ở ngang −6, **bằng** bản cũ (do điểm xuất phát nhiễu, không do khe).
- Tham khảo ve_A tuyến 2 (+9): mới −6 / 0 / +6 nhỏ nhất cả đường 6,0 / 6,9 / 5,8 cm, cũ 4,5 / 6,9 / 5,8. Điểm xấu ở đoạn xuất phát quanh B và đoạn xuống A, không ở khe.

**c) Mọi tuyến, cũ / mới** (số đỉnh ở xuất phát danh nghĩa; khoảng cách nhỏ nhất cả đường qua 27 nhiễu) [tính]:

| tuyến | số đỉnh cũ → mới | đỉnh mới (khác cũ in đậm) | nhỏ nhất, lệch 0: cũ → mới | nhỏ nhất qua 9 mức lệch: cũ → mới |
|---|---|---|---|---|
| di_B 1 | 5 → 5 | y hệt | 13,3 → 13,3 | −0,1 → −0,1 (y hệt; đè 0,1 cm ở dọc +14 / ngang +6, có từ trước) |
| di_B 2 | 5 → 5 | (0,059; 0,310), **(0,374; 0,760)**, **(1,434; 0,760)** | 10,9 → **13,3** | 0,1 → **2,8** |
| ve_A 1 | 5 → 5 | y hệt | 13,3 → 13,3 | 1,6 → 1,6 |
| ve_A 2 | 5 → 5 | **(1,484; 0,760)**, **(0,374; 0,760)**, (0,059; 0,310) | 10,5 → **13,3** | 0,9 → 0,9 |

⇒ theo số đo của bảng (số đỉnh, khoảng cách nhỏ nhất): **không tuyến nào xấu hơn**.

**Báo thêm (theo từng mức lệch)** [tính]: có 2 ô mới thấp hơn cũ **0,5 cm**:
- di_B 2 (dọc 0, ngang −6): 9,0 → 8,5;
- ve_A 2 (dọc 0, ngang +6): 9,0 → 8,5.
- Lý do: đường cũ lệch về một phía (3,6-3,9 cm) nên phía kia rộng hơn; đi giữa thì hai phía bằng nhau (8,5 / 8,5). Cũng hai tuyến đó, mức đối xứng tăng 4,9 → 8,5 và 4,5 → 8,5.
- Các ô còn lại mới ≥ cũ. Chi tiết 9 mức × 4 tuyến ở `r120/b6118.txt`.

**Chưa chạy trên Pi; chưa giả lập cả hệ thống** (mới đo hình học tĩnh).

## B6b. Nâng ngưỡng khe, in stderr, góc vào đích ≤ 17° — **DỪNG: nhiều điều kiện không đạt, không vá tạm**. CHƯA commit

Bản cũ đọc bằng `git show 579cda5:...` (không dùng stash).

### Đã sửa (cây làm việc; bản lưu `r120/tinh_tuyen_b6b.py`)
- `W_KHE` 0,175 → **0,375**: giữa 7 ô (0,35) và 8 ô (0,40) của dải đi được [tính].
  - Dải cấm lớp 1 (≥ 72 ⇔ cách tâm ô vật ≤ 0,183 m) chiếm đúng **3 ô mỗi bên**. Vậy dải đi được = (số ô trống giữa hai mặt vật − 6) × 0,05 m:
    - khe 0,60 m → 0,30;
    - khe 0,56 m → 0,25 hoặc 0,30 tuỳ lưới rơi, nên cần ngưỡng ≥ 0,35 để lề ≥ 1 ô trong trường hợp xấu nhất;
    - khe 0,65 m → 0,35 (cũng thành khe).
- Khe nhiều bậc (`_gop_khe`):
  - tách theo đoạn thẳng, lấy trục của đoạn hẹp nhất;
  - nới ra hai phía chừng nào pháp tuyến còn vuông trục (≤ 10°) và trục cách hai biên ≥ nửa bề rộng đoạn hẹp.
  - Khe dưới v3 (x 0,53-0,73 rộng 0,50 m, tâm 0,085; sau đó 0,65 m) thành một khe theo trục 0,085.
- `GOC_VAO_DICH = 17°` thay `GOC_XOAY` trong ưu tiên đỉnh ra trước đích.
- Mỗi khe được căn giữa in 1 dòng **stderr**, ví dụ `khe can giua: (0.66; 0.76) -> (1.31; 0.76), rong 0.15 m (dai di duoc), dinh vao (0.37; 0.76) ra (1.48; 0.76)`. stdout không đổi.
- **Chưa** thêm test khe dựng tay 52/56/60 cm (dừng trước).

### Kết quả (đo như B6, script `r120/b6118.py`, 27 nhiễu) [tính]
| điều kiện | kết quả | đạt? |
|---|---|---|
| (a) lệch giữa khe (trên: x 0,73-1,18 trục 0,760; dưới: x 0,53-0,73 trục 0,085) | 0,0 cm cả 4 tuyến (cũ: t1 0,8 / 0,7; t2 3,6 / 3,9) | **đạt** |
| (4) góc đoạn cuối vào đích, lớn nhất | di_B t2 **27,3°** (cũ 55,7°); di_B t1 **84,9°** (cũ 84,9°, không đổi: đoạn cuối t1 đi từ dưới lên) | **không đạt** |
| (b) dọc +9, ngang −6 / 0 / +6, ≥ 7,1 cm ở khe / rẽ / gần đích | di_B t2: khe 8,5 / 14,5 / 8,5; rẽ 8,5 / 13,7 / 9,6; gần đích 10,6 / 13,6 / **8,3** ⇒ đạt. Ba tuyến kia **không**: di_B t1 rẽ 6,0, gần đích 6,7 / **4,1**; ve_A t1 gần đích **6,0**; ve_A t2 rẽ **6,8**, gần đích **5,8** | **không đạt** (3/4 tuyến) |
| (c) không tuyến nào xấu hơn | ve_A t1 **xấu hơn**: lệch 0 nhỏ nhất 13,3 → 11,9; (0; +6) 9,0 → 7,6; thêm **CANH BAO 27/27 nhiễu** (đỉnh vào khe dưới (1,404; 0,085), `cach_do` 0,206 ≤ 0,2356). Ba tuyến kia ≥ cũ, trừ hai ô 9,0 → 8,5 đã chấp nhận | **không đạt** |
| test cũ | **test_15, test_16 hỏng** (đã có trong 579cda5) | **không đạt** |
| test mới B6 | test_31 (khe 4 ô không là khe), test_32 (tuyến 1 y hệt cũ) hỏng: đúng dự kiến vì ngưỡng đổi; chưa viết lại | — |

### Vì sao không đạt (để Thiên quyết)
1. **(4) 17° với di_B t2.**
   - Khe theo dải cấm kéo tới x 1,31, tức **13 cm sau mặt tường thật** (K2 hết 1,184), vì dải cấm rộng 3 ô [tính].
   - Đỉnh ra phải ≥ 1,31 + 0,105 (đuôi xe) = 1,414. Từ (1,414; 0,76) tới B (1,60; 0,82) lệch **17,9°** ⇒ không thể ≤ 17° với luật này.
   - Ở B6 (ngưỡng 0,175) khe kết thúc 1,26 nên đạt 19,9°. Muốn ≤ 17° phải: rút lề đuôi (đo theo mặt vật thay vì biên dải cấm), hoặc dời B / góc cuối, hoặc cho đoạn cuối cong.
   - di_B t1 84,9° thì không liên quan khe: tuyến 1 vào B từ dưới lên, cả bản cũ.
2. **test_15 / test_16 (cũ).**
   - Trên `BAN` của test (PGM, không có dải cấm) khe trên rộng 0,25 m dải đi được, giờ ≤ 0,375 nên thành khe.
   - Đỉnh (0,41; 0,76) mà hai test đòi phải là CANH BAO bị thay bằng đỉnh (0,37; 0,76) đủ chỗ xoay. Hai test **ghim** hành vi cũ.
3. **ve_A t1 xấu hơn + CANH BAO mới.**
   - Đi giữa khe dưới (0,085) thì đỉnh vào từ phía B phải ≥ 0,145 m trước đầu khe (x 1,26) ⇒ x ≥ 1,405, sát tường phải, chỉ còn `cach_do` 0,206.
   - Đỉnh cũ (1,309; 0,06) nằm lệch 2,5 cm nhưng rộng hơn.
4. **(b) ở các tuyến khác:** các chỗ < 7,1 nằm ở **chỗ rẽ / gần đích không thuộc khe**: di_B t1 gần B 4,1 (bản cũ cũng 4,1), ve_A gần A 5,8-6,0 (cũ 5,8-7,0). Luật khe không chạm tới.

**Cây làm việc để nguyên ở trạng thái B6b** để Thiên xem: tinh_tuyen.py, test_gui_duong.py đã sửa, test_gui_duong 28/32. md5 ghi dưới.
e8d24e3f43f86927d56fd546db3e3479  tinh_tuyen.py
8bb20b5b8d5a4cbe4f2ff35e06bf292c  test_gui_duong.py

## B6c. Bỏ B6b, quay về hành vi B6, chỉ giữ dòng stderr (CHƯA commit)

### 1) Lưu và khôi phục
- Bản B6b lưu ở scratchpad `r120/tinh_tuyen_b6b.py` (md5 e8d24e3f43f86927d56fd546db3e3479) và `r120/test_gui_duong_b6b.py` (8bb20b5b…) [tính].
- **tinh_tuyen.py:**
  - không có bản lưu B6, nên mình **dựng lại** bằng cách áp đúng đoạn sửa của B6 lên `git show 579cda5:xe_scripts/tinh_tuyen.py`;
  - kết quả có md5 **f81e056df9e368e48b21855c08657068**, đúng md5 B6 đã ghi ở mục B6 [tính]. Lưu tại `r120/tinh_tuyen_b6.py` rồi chép vào kho.
- **test_gui_duong.py:** B6b không sửa tệp này; md5 vẫn **8bb20b5b8d5a4cbe4f2ff35e06bf292c** = B6 [tính].
- **Chỉ thêm** so với B6 (diff đúng 2 chỗ) [đọc code]:
  - `import sys`;
  - cuối mỗi lần `qua_khe` căn giữa một khe: 1 dòng **stderr** `khe can giua: (x0; y0) -> (x1; y1), rong W m (dai di duoc), dinh vao (..) ra (..)`. W = tổng hai tia vuông góc tại giữa trục, nên `tim_khe` giữ nguyên.
  - Ngưỡng 0,175, không gộp khe nhiều bậc, ưu tiên `< GOC_XOAY` như B6.
- md5 mới: tinh_tuyen.py **50ed279dab74d48baa4cbcce2f1a0b15**, test_gui_duong.py 8bb20b5b8d5a4cbe4f2ff35e06bf292c [tính].

### 2) Kiểm
- Test: test_gui_duong **32/32**, test_lai_xe_lidar 97/97, test_sep 46/46 [tính].
- **stdout 4 tuyến giống B6 từng byte:**
  - in `repr(tinh_duong(...))` và CANH BAO của di_B / ve_A × tuyến 1 / 2 trên hai bản đồ (costmap dựng lại từ v3 và `BAN` PGM), tức 8 dòng, 21 751 byte;
  - bản B6 dựng lại và bản B6c cùng md5 **64e25d995ed9211b3cf7f5a7a2a056cb** [tính] (script `r120/in4.py`).
  - stderr: B6 0 dòng; B6c 2 dòng, chỉ trên costmap dựng lại [tính]:
    - `khe can giua: (0.61; 0.76) -> (1.26; 0.76), rong 0.15 m (dai di duoc), dinh vao (0.37; 0.76) ra (1.43; 0.76)` (di_B 2)
    - `khe can giua: (1.26; 0.76) -> (0.61; 0.76), rong 0.15 m (dai di duoc), dinh vao (1.48; 0.76) ra (0.37; 0.76)` (ve_A 2)
  - Trên `BAN` không khe nào được căn giữa (như B6).
- `python3 tinh_tuyen.py` (`main()`, bảng xs cho chuyen.sh): stdout **giống 579cda5** (cmp rỗng) [tính].
- **(a)** khe trên lệch giữa **0,0 cm** (di_B 2, ve_A 2; cũ 3,6 / 3,9) [tính].
- **(b)** di_B 2, dọc +9 [tính]:
  - theo vùng của B6 (x): ≥ **8,5 cm** như B6 (stdout giống byte nên số giống);
  - theo vùng mới của script (rẽ = trong 10 cm quanh đỉnh hoặc xoay; gần đích ≤ 0,35 m): khe 8,5 / 14,5 / 8,5, rẽ 8,5 / 13,2 / 9,6, gần đích 11,3 / 13,6 / 10,3 (ngang −6 / 0 / +6).
- **(c)** bảng (`r120/b6c118.txt`) ra **đúng từng số của B6** [tính]:
  - di_B 2 theo 9 mức cũ/mới: 9,0/8,5 · 10,9/13,3 · 4,9/8,5 · 7,1/7,1 · 7,6/9,6 · 4,2/8,5 · 2,8/2,8 · 3,5/5,2 · 0,1/5,8;
  - ve_A 2: 4,5/8,5 · 10,5/13,3 · 9,0/8,5 · 4,5/6,0 · 6,9/6,9 · 5,8/5,8 · 1,7/1,7 · 2,0/2,0 · 0,9/0,9;
  - tuyến 1 cả hai chiều y hệt cũ; số đỉnh 5/5 cả 4 tuyến (xuất phát danh nghĩa).

### 3) di_B 2: RPP xoay tại chỗ ở đỉnh ra cuối
- Đỉnh ra (1,434; 0,760) giống nhau qua cả 9 nhiễu vị trí xuất phát [tính]. Góc rẽ ở đó **19,87°**, dưới `GOC_XOAY` 20,05° 0,18° [tính]. RPP có thể xoay tại chỗ nếu góc tới carrot thực tế vượt ngưỡng.
- Giả sử xoay tại chỗ từ 0° tới 19,87° (201 bước), thân 25 × 16 cm tới ô vật v3, lệch dọc +9 cm [tính]:

| lệch ngang | nhỏ nhất khi xoay | ở góc | đầu (0°) / cuối (19,9°) |
|---|---|---|---|
| −6 cm | **25,0 cm** | 0,0° | 25,0 / 27,6 |
| 0 | **22,0 cm** | 19,9° | 27,6 / 22,0 |
| +6 cm | **16,3 cm** | 19,9° | 23,5 / 16,3 |

- Xoay tại chỗ ở đỉnh ra không làm thân gần vật hơn 16,3 cm, tức chỗ này rộng [tính]. Chỗ sát nhất của tuyến vẫn ở trong khe (8,5 cm).

**Chưa chạy trên Pi; chưa giả lập cả hệ thống.**

## B6d-giấy. Lưới lệch dọc {0, +7, +14} × ngang {−7, 0, +7} cm trên B6c (CHỈ ĐỌC + TÍNH; không sửa tinh_tuyen.py / test, không commit)

Nhánh `viec-118`, HEAD **16e5269** [log]; tinh_tuyen.py md5 50ed279d (= B6c) [tính]. Script scratchpad `r120/b6d118.py` (đo), `r120/b6d118_gh.py` + `r120/b6d118_gh2.py` (giới hạn), `r120/b6d118_ve.py` (hình). Chưa thử trên xe.

### 1) Cách đo (như B5 / B6c) [tính]
- Costmap dựng lại từ map_xuong_v3 (InflationLayer 1.1.20: 0,20 / 3,0 / nội tiếp 0,09), `T.Ban.tu_luoi` rồi `T.tinh_duong` của B6c.
- Xuất phát như B6c: 27 nhiễu (dx, dy ∈ {−5, 0, +5} cm × hướng ∈ {−5°, 0, +5°}); di_B từ (0,01; 0,00) hướng 0, ve_A từ B (1,60; 0,82) hướng 0.
- Tư thế tin: xoay tại chỗ từ hướng xuất phát, đoạn thẳng bước 2 cm, xoay ở mỗi đỉnh và ở đích (di_B về 0°), bước 5°.
- Lệch tính **trong khung thân**: thật = tin + R(θ)·(dọc, ngang), + ngang = trái.
- Thân 25 × 16 cm (x −0,105..0,145, y ±0,08) tới ô vật v3 (cạnh ô lấy mẫu 0,5 cm); âm = đè. Ngưỡng **7,1 cm**.
- Vùng (theo thứ tự ưu tiên, theo tư thế tin): **gần B** (≤ 0,35 m từ B, hoặc xoay ở đích); **gần A** (≤ 0,35 m từ A); **chỗ rẽ** (≤ 10 cm quanh đỉnh trong, hoặc xoay ở đỉnh trong); **khe trên** (x 0,73–1,18, |y − 0,76| < 0,15); còn lại "khác".

Đỉnh (xuất phát danh nghĩa) giống B6c; 0/27 CANH BAO cả 4 tuyến [tính].

### 2) Góc đoạn cuối vào B so với `GOC_XOAY` 20,05° [tính]
| tuyến | góc (cả 27 nhiễu) | so ngưỡng |
|---|---|---|
| di_B t1 | **84,91°** (đỉnh trước B (1,559; 0,360), vào từ dưới) | vượt 64,9° ⇒ chắc chắn xoay tại chỗ ở B |
| di_B t2 | **19,87°** (đỉnh trước B (1,434; 0,760)) | dưới ngưỡng **0,18°** |

### 3) Nhỏ nhất mỗi vùng (qua 27 nhiễu × 9 mức), ca tệ nhất [tính]
| tuyến | vùng | nhỏ nhất | mức dọc/ngang | nhiễu (dx, dy, °) | tin (x; y; hướng) | thân thật tâm | điểm vật |
|---|---|---|---|---|---|---|---|
| di_B t1 | khe trên | — (không qua) | | | | | |
| di_B t1 | rẽ | **1,3** | +14/−7 | (−5, +5, −5) | (1,309; 0,060; 14°) xoay | (1,461; 0,025) | (1,634; −0,020) |
| di_B t1 | gần B | **−0,5** (đè) | +14/+7 | (−5, −5, −5) | (1,600; 0,820; 64°) xoay ở đích | (1,599; 0,977) | (1,594; 1,135) tường trên |
| di_B t1 | gần A | **2,5** | +14/−7 | (+5, −5, +5) | (0,060; −0,050; 10°) xoay đầu | (0,210; −0,094) | (0,384; −0,165) |
| di_B t2 | khe trên | 7,5 | 0/−7 (mọi mức ngang ±7) | | (0,734; 0,760; 0°) | (0,734; 0,690) | (0,684; 0,535) đỉnh H0 |
| di_B t2 | rẽ | **5,3** | +14/+7 | (−5, +5, −5) | (0,374; 0,760; 60°) đoạn vào đỉnh 2 | (0,384; 0,916) | (0,389; 1,135) tường trên |
| di_B t2 | gần B | **6,5** | +14/−7 | (−5, −5, −5) | (1,600; 0,820; 20°) | (1,755; 0,802) | (1,984; 0,775) tường phải |
| di_B t2 | gần A | **2,4** | +14/−7 | (+5, −5, +5) | (0,060; −0,050; 10°) xoay đầu | (0,210; −0,095) | (0,384; −0,165) |
| ve_A t1 | khe trên | — (không qua) | | | | | |
| ve_A t1 | rẽ | **1,0** | +14/+7 | (−5, −5, −5) | (1,309; 0,060; −130°) đoạn vào đỉnh 2 | (1,273; −0,092) | (1,244; −0,265) tường dưới |
| ve_A t1 | gần B | **1,2** | +14/+7 | (+5, −5, +5) | (1,650; 0,770; −26°) xoay đầu | (1,807; 0,771) | (1,984; 0,780) tường phải |
| ve_A t1 | gần A | **5,7** | +14/+7 | (−5, −5, −5) | (0,000; 0,000; −135°) | (−0,049; −0,149) | (−0,096; −0,365) |
| ve_A t2 | khe trên | 7,5 | 0/+7 (mọi mức ngang ±7) | | (1,161; 0,760; 180°) | (1,161; 0,690) | (1,049; 0,535) |
| ve_A t2 | rẽ | **2,1** | +14/−7 | (−5, −5, −5) | (0,374; 0,760; 205°) xoay ở đỉnh 2 | (0,218; 0,764) | (0,034; 0,785) tường trái |
| ve_A t2 | gần B | **1,2** | +14/+7 | (+5, −5, −5) | (1,650; 0,770; −25°) xoay đầu | (1,806; 0,774) | (1,984; 0,785) tường phải |
| ve_A t2 | gần A | **0,7** | +14/+7 | (−5, −5, −5) | (0,000; 0,000; −101°) | (0,043; −0,151) | (0,094; −0,315) |

"Khác" (ngoài 4 vùng): ≥ 7,5 cả 4 tuyến (di_B t1 9,1; ve_A t1 8,9; t2 7,5 ở đoạn khe rộng trước/sau) [tính].

Theo 9 mức (dọc/ngang: −7, 0, +7) [tính]:

| tuyến · vùng | dọc 0 | dọc +7 | dọc +14 |
|---|---|---|---|
| di_B t1 · rẽ | 12,5 / 16,8 / 11,7 | 7,2 / 10,4 / 12,1 | **1,3 / 3,8 / 6,7** |
| di_B t1 · gần B | 17,0 / 15,0 / 10,6 | 10,0 / 8,6 / **5,3** | **3,0 / 1,9 / −0,5** |
| di_B t1 · gần A | 8,2 / 13,3 / 10,5 | 8,2 / 11,4 / 9,1 | **2,5 / 5,3** / 9,1 |
| di_B t2 · khe trên | 7,5 / 14,5 / 7,5 | 7,5 / 14,5 / 7,5 | 7,5 / 14,5 / 7,5 |
| di_B t2 · rẽ | 7,5 / 14,5 / 14,3 | 7,5 / 14,5 / 9,9 | 7,5 / 8,9 / **5,3** |
| di_B t2 · gần B | 7,5 / 14,5 / 7,5 | 9,3 / 15,5 / 9,3 | **6,5** / 8,8 / 7,7 |
| di_B t2 · gần A | 8,2 / 13,3 / 16,0 | 8,2 / 11,4 / 15,1 | **2,4 / 5,2** / 11,2 |
| ve_A t1 · rẽ | 11,6 / 16,2 / 11,6 | 11,3 / 10,9 / **6,4** | 10,8 / **5,5 / 1,0** |
| ve_A t1 · gần B | 10,0 / 13,3 / 8,2 | 11,9 / 10,5 / 7,2 | **4,9 / 3,8 / 1,2** |
| ve_A t1 · gần A | 12,9 / 15,8 / 10,6 | 15,8 / 13,2 / **6,4** | 10,9 / 10,6 / **5,7** |
| ve_A t2 · khe trên | 7,5 / 14,5 / 7,5 | 7,5 / 14,5 / 7,5 | 7,5 / 14,5 / 7,5 |
| ve_A t2 · rẽ | 13,3 / 14,5 / 7,5 | 7,9 / 11,2 / 8,7 | **2,1 / 4,5 / 5,7** |
| ve_A t2 · gần B | 7,5 / 13,3 / 7,5 | 7,2 / 10,5 / 7,2 | **1,2 / 3,8 / 1,2** |
| ve_A t2 · gần A | 17,1 / 15,8 / 14,5 | 10,3 / 8,9 / 7,6 | **4,3 / 2,0 / 0,7** |

- Mọi chỗ < 7,1 đều ở **dọc +14**, trừ 3 ô ở dọc +7: di_B t1 gần B (+7/+7) 5,3; ve_A t1 rẽ (+7/+7) 6,4; ve_A t1 gần A (+7/+7) 6,4 [tính].
- Khe trên: **7,5 cm ở mọi mức ngang ±7**, không phụ thuộc lệch dọc (đoạn thẳng theo trục) [tính].

### 4) Giới hạn vật lý của từng vùng hỏng [tính]
Cách tìm (cùng thân, cùng ô vật, cùng cách đặt lệch):
- **rẽ**: dời **một** đỉnh trong (lưới 2 cm, trong 20 cm quanh đỉnh cũ), hai đỉnh kề giữ nguyên; xét ≤ 10 cm quanh đỉnh + xoay tại đỉnh (chiều ngắn).
- **rời A/B** (di_B gần A, ve_A gần B): xuất phát cố định (nhiễu của ca tệ nhất); **một** đoạn thẳng tới P (cách xuất phát 0,35–0,47 m), xoay hai chiều. Giới hạn cứng = chính tư thế xuất phát lúc **đứng yên**.
- **tới A/B**: **một** đoạn thẳng từ P (cách đích 0,35–0,47 m) vào đích; di_B xoay về 0° ở B (hai chiều). Giới hạn cứng: di_B = tư thế đích (B; 0°); ve_A = tư thế tốt nhất tại A theo hướng (A không có yaw cuối).
- Ghi hai số: "mức tệ nhất" = tốt nhất ở đúng mức đang hỏng; "chung 9 mức" = một lựa chọn duy nhất, lấy nhỏ nhất qua 9 mức (vì xe không biết mình lệch kiểu nào).

| tuyến · vùng | hiện tại | tốt nhất ở mức tệ nhất | chung 9 mức (lựa chọn) | kết luận |
|---|---|---|---|---|
| di_B t1 · rẽ | 1,3 | 17,4 (đỉnh 2 → (1,129; 0,08)) | **9,3** (đỉnh 2 → (1,209; 0,02)) | **làm được** bằng dời đỉnh 2 |
| di_B t1 · gần B | −0,5 | 9,9 / chung 7,2 khi **vào B từ trái** (P (1,14–1,18; 0,76–0,82), như t2) | vào **từ dưới như t1** (P y ≤ 0,50): dọc +14 tốt nhất **5,7–6,2**, chung **0,8** | **không làm được với kiểu vào của t1**; làm được (sát, 7,2) nếu đoạn cuối vào B theo hướng gần 0°. Tư thế đích (B; 0°) ở dọc +14 = 9,9 (giới hạn cứng đạt) |
| di_B t1 / t2 · gần A | 2,5 / 2,4 | 2,7 (+14/−7), 6,9 (+14/0) | 2,7 | **không làm được**: tư thế xuất phát **đứng yên** (0,060; −0,050; 5°) đã 2,7 / 6,9 ở dọc +14. 7/27 nhiễu (toàn bộ ở dx +5 cm) cho tư thế đứng yên < 7,1; với dx 0 thì đứng yên ≥ 7,7 |
| di_B t2 · rẽ | 5,3 | 13,7 (đỉnh 2 → (0,334; 0,68)) | **5,9** (đỉnh 2 → (0,274; 0,74)) | ở từng mức làm được; **không có một đỉnh 2 chung** cho cả 9 mức khi giữ đỉnh 1, 3 (cần đổi thêm đỉnh kề hoặc nhiều đỉnh) |
| di_B t2 · gần B | 6,5 | 9,9 (P (1,14; 0,82)) | **7,2** (P (1,18; 0,76)) | **làm được, sát** (7,2): đoạn cuối gần 0° như vào từ (1,18; 0,76) |
| ve_A t1 · rẽ | 1,0 | 17,2 (đỉnh 2 → (1,209; 0,14)) | **7,5** (đỉnh 2 → (1,209; 0,06)) | **làm được** bằng dời đỉnh 2 |
| ve_A t1 / t2 · gần B | 1,2 / 1,2 | 3,7–4,9 | 1,2 | **không làm được**: tư thế xuất phát **đứng yên** (1,650; 0,770; ±5°) đã 3,7–4,9 ở dọc +14 (mũi cách tường phải x 1,984). 9/27 nhiễu (toàn bộ dx +5 cm) cho đứng yên < 7,1; dx 0: ≥ 8,7 |
| ve_A t1 · gần A | 5,7 | 13,3 | **10,3** (P (0,40; 0,10)) | **làm được** bằng đổi đoạn cuối vào A |
| ve_A t2 · rẽ | 2,1 | 15,9 (đỉnh 2 → (0,514; 0,76)) | **8,4** (đỉnh 2 → (0,454; 0,82)) | **làm được** bằng dời đỉnh 2 |
| ve_A t2 · gần A | 0,7 | 13,3 | **10,3** (P (0,40; 0,10)) | **làm được** bằng đổi đoạn cuối vào A |

Lưu ý về giới hạn:
- Khe trên 7,5 cm **là giới hạn vật lý**: khe 0,45 m, nửa khe 22,5 cm − nửa thân 8 − lệch ngang 7 = 7,5 cm [tính]. Đường B6c đã đi đúng trục nên không tăng thêm được.
- Hai ca "không làm được" ở **xuất phát** (gần A của di_B, gần B của ve_A) đến từ tổ hợp nhiễu **dx +5 cm** với lệch dọc **+14**: tâm thân thật dịch 19 cm về phía trước ngay lúc đứng yên. Không đường nào sửa được; chỉ giảm được bằng vạch xuất phát / lệch thật nhỏ hơn.
- B cố định (1,60; 0,82), yaw cuối 0°: tư thế đích ở dọc +14 còn 9,9 cm. Phần hỏng ở gần B của di_B là **cách vào** (t1 vào từ dưới, xoay 85° ở B), không phải vị trí B.
- Tìm kiếm chỉ đổi **một** đỉnh hoặc **một** đoạn; "không làm được" ở di_B t2 rẽ (chung 9 mức) chỉ đúng với ràng buộc đó.
- Chưa tính padding / inflation / phép kiểm va chạm của RPP; hình học tĩnh.

### 5) Hình RA_SOAT_118_b6d.png
- 1 cm = 4 px (400 px/m), lưới 5 cm, khung x −0,30..2,05, y −0,40..1,25; 4 ô (di_B t1, di_B t2, ve_A t1, ve_A t2), 1910 × 1690 px.
- Ô vật v3 hồng, đường tin B6c đen (xuất phát danh nghĩa). Mỗi vùng vẽ thân thật ở ca tệ nhất (xanh = khe trên, cam = rẽ, đỏ = gần B, tím = gần A), vạch = mũi, vuông = điểm vật gần nhất, số = cm. Dưới mỗi ô: số đo + giới hạn (mức tệ nhất / chung 9 mức).
- md5 a80ab43f [tính].

**Chưa chạy trên Pi; chưa giả lập cả hệ thống.** Mọi số ở mục này là [tính] trên hình học tĩnh, trừ HEAD [log].

## B6d-phương án (trên giấy; CHỈ ĐỌC + TÍNH; không sửa tinh_tuyen.py / test, không commit)

HEAD **16e5269** [log]. Script scratchpad `r120/pa_lib.py` (đánh giá), `pa_tim*.py` (tìm), `pa_luoi_t1.py`, `pa_luoi_veA1.py`, `pa_luoi_veA2.py` (tìm lưới), `pa_kiem27.py` (kiểm), `pa_ve.py` (hình). Chưa thử trên xe; **chưa viết vào tinh_tuyen**.

### 1) Cách đo [tính]
- Đúng cách B6d-giấy: thân 25 × 16 cm tới ô vật v3, 27 nhiễu xuất phát × 9 mức lệch (dọc 0/+7/+14 × ngang −7/0/+7, khung thân), cùng 5 vùng, ngưỡng 7,1 cm.
- Khác một chỗ: cột A lấy nguyên số B6d-giấy (đỉnh do tinh_tuyen tính lại theo từng nhiễu). Cột B, C dùng **đỉnh cố định**, chỉ điểm xuất phát mang nhiễu.
- Bộ đánh giá viết lại dạng vector. Kiểm chéo trên đỉnh B6c cố định: di_B t1 ra đúng số B6d-giấy; di_B t2 ra 5,6 so với 5,3 ở vùng rẽ (khác do đỉnh tính lại theo nhiễu) [tính].
- Tìm đường: dò từng toạ độ (bước 4 → 2 → 1 cm) với 5 nhiễu, mục tiêu = lớn nhất của (nhỏ nhất qua các vùng × 9 mức). Bỏ khỏi mục tiêu vùng giới hạn bởi xuất phát (gần A của di_B, gần B của ve_A, xem B6d-giấy mục 4). Sau đó kiểm lại đủ 27 nhiễu.
- Mọi đỉnh mới nằm ngoài vùng cấm `CAM` của tuyến mình (t1: x 0,30–1,20, y ≥ 0,50; t2: x 0,30–1,20, y ≤ 0,40) [đọc code tinh_tuyen.py:31-34].

### 2) Đường (đỉnh sau điểm xuất phát) [tính]
| tuyến | B6c (A) | mới (B / C) | góc vào B |
|---|---|---|---|
| di_B t2 | (0,059; 0,31) → (0,374; 0,76) → (1,434; 0,76) → B | (0,059; 0,31) → **(0,20; 0,59)** → **(0,40; 0,76)** → **(1,32; 0,76)** → B | 19,87° → **12,09°** |
| di_B t1 | (0,109; 0,11) → (1,309; 0,06) → (1,559; 0,36) → B | **(0,109; 0,085)** → **(1,26; 0,085)** → **(1,41; 0,20)** → **(1,40; 0,76)** → B | 84,91° → **16,70°** |
| ve_A t1 (C) | (1,559; 0,36) → (1,309; 0,06) → (0,109; 0,11) → A | **(1,39; 0,15)** → **(1,21; 0,05)** → **(0,40; 0,10)** → A | — |
| ve_A t2 (C) | (1,484; 0,76) → (0,374; 0,76) → (0,059; 0,31) → A | (1,484; 0,76) → **(0,44; 0,76)** → **(0,24; 0,59)** → **(0,18; 0,10)** → A | — |

- ve_A t2: thử đỉnh đầu (1,32; 0,76) thì gần B −2,5 (tệ hơn B6c 1,2), nên giữ đỉnh đầu B6c (1,484; 0,76) [tính].

### 3) Luật chung theo khoảng trống (không gắn toạ độ)
Ký hiệu: "thân phóng" = thân cộng lệch tối đa. Mũi phóng L = 0,145 + 0,14 = **0,285 m**, nửa ngang phóng W = 0,08 + 0,07 = **0,15 m**, lề c = **0,071 m**. Bán kính quét khi xoay √(L² + W²) = 0,322 m, cộng lề = **0,393 m** [tính].

1. **Luật K (vào / rời trục khe).** Đoạn tới đỉnh vào khe (đỉnh nằm trên trục khe) lệch trục **≤ θ_K**, với L·sin θ + W·cos θ ≤ h − c. h là khoảng trống vuông góc trục, về phía đang rẽ, đo từ trục tại đỉnh vào.
   - Khe trên: h = 0,375 nên θ_K = **42,96°** [tính].
   - Đoạn tới góc lớn hơn thì **chèn một đỉnh** ở chỗ rộng trên tia lệch ≤ θ_K tính từ đỉnh vào, chọn chỗ cách vật xa nhất.
   - Rời khe làm đối xứng.
   - Áp: di_B t2 chèn (0,20; 0,59), lệch 40,36°; ve_A t2 chèn (0,24; 0,59), lệch 40,36°.
2. **Luật D (vào đích có `yaw_cuoi`).** Đoạn cuối lệch `yaw_cuoi` **≤ θ_D = min(17°, θ thỏa L·cos θ + W·sin θ ≤ g − c)**. g là khoảng trống phía trước đích theo `yaw_cuoi`.
   - B: g = 0,384 nên θ_D = **14,13°** [tính].
   - Đỉnh trước đích đặt trên trục khe (nếu có) hoặc ở chỗ rộng, sao cho góc ≤ θ_D, chọn chỗ thân cách vật xa nhất.
   - Áp: di_B t2 đỉnh ra (1,32; 0,76), góc 12,09°.
3. **Luật R (đỉnh rẽ ở chỗ rộng).** Đỉnh có góc rẽ ≥ `GOC_XOAY` chỉ đặt ở chỗ mà thân phóng, quét trong cung quay [hướng vào, hướng ra], cách vật ≥ c. Hai đoạn kề cũng phải cách vật ≥ c với thân phóng (lệch ngang ±W).
   - Tìm trong 20 cm quanh đỉnh cũ, lấy chỗ cách vật xa nhất. Không có chỗ đạt thì báo, không giữ im.
   - Áp: ve_A t1 dời (1,559; 0,36) → (1,39; 0,15) và (1,309; 0,06) → (1,21; 0,05), để đoạn chéo không sát góc dưới-phải H0.
4. **Luật A (vào đích không yaw).** Đoạn cuối chọn hướng sao cho phía trước đích (theo hướng đó) trống ≥ L + c = 0,356 m và hai bên ≥ W + c.
   - Áp: ve_A t1 vào A từ (0,40; 0,10), hướng −166°, phía trước trống 0,416 m; ve_A t2 vào A từ (0,18; 0,10), hướng −151°.
5. **Luật T (hành lang).** Đoạn trong hành lang/khe đi trên trục (như B6). Áp: di_B t1 đoạn hành lang dưới y 0,085.
   - Phiên bản đầu của di_B t1 lệch trục 5 cm, chỉ còn 5,1 cm ngay ở dọc 0, nên đã bỏ.

### 4) Bảng vùng | A | B | C (nhỏ nhất qua 27 nhiễu × 9 mức, cm; **đậm** = < 7,1) [tính]
(A) = B6c hiện tại; (B) = **khuyến nghị**: di_B mới + ve_A giữ B6c; (C) = di_B mới + ve_A mới.

| tuyến · vùng | A | B | C |
|---|---|---|---|
| di_B t1 · rẽ | **1,3** | **6,6** | **6,6** |
| di_B t1 · gần B | **−0,5** | **5,3** | **5,3** |
| di_B t1 · gần A (*) | **2,5** | **2,4** | **2,4** |
| di_B t1 · khác | 9,1 | **7,0** | **7,0** |
| di_B t1 · góc vào B | 84,9° | 16,7° | 16,7° |
| di_B t2 · khe trên | 7,5 | 7,5 | 7,5 |
| di_B t2 · rẽ | **5,3** | 7,4 | 7,4 |
| di_B t2 · gần B | **6,5** | 7,4 | 7,4 |
| di_B t2 · gần A (*) | **2,4** | **2,4** | **2,4** |
| di_B t2 · khác | 7,5 | 7,5 | 7,5 |
| di_B t2 · góc vào B | 19,9° | **12,1°** | 12,1° |
| ve_A t1 · rẽ | **1,0** | **1,0** | 8,8 |
| ve_A t1 · gần B (*) | **1,2** | **1,2** | **1,2** |
| ve_A t1 · gần A | **5,7** | **5,7** | 10,3 |
| ve_A t1 · khác | 8,9 | 8,9 | 9,1 |
| ve_A t2 · khe trên | 7,5 | 7,5 | 7,5 |
| ve_A t2 · rẽ | **2,1** | **2,1** | 7,5 |
| ve_A t2 · gần B (*) | **1,2** | **1,2** | **1,2** |
| ve_A t2 · gần A | **0,7** | **0,7** | 8,5 |
| ve_A t2 · khác | 7,5 | 7,5 | 7,5 |
| **số ô < 7,1** | **12** | **11** (4 ô (*) + 3 ô di_B t1 + 4 ô ve_A B6c) | **7** (4 ô (*) + 3 ô di_B t1) |

(*) Giới hạn bởi xuất phát: tư thế **đứng yên** lúc xuất phát đã < 7,1 khi nhiễu dx +5 cm và dọc +14. Không đường nào sửa được (B6d-giấy mục 4).

**di_B t1 theo mức** (mới; B6c trong ngoặc) [tính]:
- Dọc 0: nhỏ nhất **7,0** (B6c 8,2).
- Dọc +7: **6,3** ở gần B (+7/+7) (B6c 5,3).
- Dọc +14: **5,3** (B6c −0,5).
- Tức là t1 mới **thấp hơn B6c 1,2 cm ở dọc 0** nhưng cao hơn nhiều ở dọc +14. Đây là đánh đổi, chưa phải cải thiện thuần.

### 5) Vì sao di_B t1 không đạt 7,1 [tính]
- Luật D đòi đoạn cuối ≤ 14,13°, nên đỉnh trước B phải ở bên trái B, y ~0,74–0,82, x ≤ ~1,36. Đó là ngay cửa ra khe trên, sát góc K2 (1,184; 0,985) và tường trên (y 1,135).
- t1 không được đi qua khe (`CAM` t1). Muốn tới đó, xe phải từ vùng dưới đi lên rồi **xoay ≥ 70°** ở chỗ đó.
- Xoay ở đó với thân phóng: mép trên quét tới y + 0,322, nên cần y ≤ 0,742. Ở hướng 100–140° thì góc trước-trái chọc vào K2.
- **Tìm lưới đầy đủ** V3 (vùng phải H0, 5 cm) × V4 (trước B, 2 cm, góc ≤ 17°), 7 772 tổ hợp: cận trên **5,2 cm** (bỏ qua xoay ở V3 nên số thật ≤ 5,2) [tính]. Dò toạ độ với 3–4 đỉnh: tốt nhất 5,0–5,3.
- Vào từ dưới như B6c thì xoay 85° ở B: tốt nhất 5,7–6,2 ở dọc +14 cho từng mức, **0,8** khi phải chung cho cả 9 mức (B6d-giấy mục 4).
- ⇒ Với B cố định (1,60; 0,82), `yaw_cuoi` 0 và lệch dọc +14 cm, **tuyến 1 không có đường đạt 7,1** trong các dạng đã thử. Muốn đạt phải đổi một trong: lệch dọc tối đa, vị trí/hướng B, hoặc cho t1 đi chung đoạn cuối với t2 (qua khe).

### 6) Hình RA_SOAT_118_b6d_pa.png
- 1 cm = 4 px (400 px/m), lưới 5 cm, cùng khung x −0,30..2,05, y −0,40..1,25 cho 3 ô cạnh nhau (A | B | C), 2860 × 1014 px.
- Màu: xanh dương di_B t1, xanh lá di_B t2, cam ve_A t1, tím ve_A t2; nét liền = di_B, nét gạch = ve_A.
- Mỗi tuyến vẽ thân thật ở ca tệ nhất của tuyến (khung + vạch mũi + ô vuông = điểm vật gần nhất). Số cạnh đường = cm nhỏ nhất từng vùng (đậm = < 7,1). Dưới mỗi ô: bảng vùng và góc vào B.
- md5 56fc766f [tính].

### 7) Lưu ý
- Hình học tĩnh: chưa tính padding / inflation / RPP (RPP có thể không xoay tại chỗ khi góc < 20,05°, ở đây vẫn tính xoay tại mọi đỉnh). Chưa giả lập, chưa thử trên xe.
- Đỉnh mới là **toạ độ tìm được** để kiểm luật. Luật 1–5 mới là thứ sẽ viết vào tinh_tuyen; khi viết, số đỉnh ra phải được kiểm lại bằng chính cách đo này.

## B6d-mã. di_B t2 theo luật khoảng trống (CHƯA commit) — **1 ô chưa đạt 7,1 (6,9), cần Thiên quyết**

Nhánh `viec-118`, HEAD **16e5269** [log]. Chỉ sửa tinh_tuyen.py, test_gui_duong.py. Bản B6c đọc bằng `git show HEAD:...` (lưu ở `r120/tinh_tuyen_b6c.py`). Script: `r120/in4.py` (stdout), `r120/ma_do.py` (đo + so 27 nhiễu), `r120/ma_luoi_V.py`, `r120/ma_luoi_EV.py` (tìm lưới), `r120/ma_ve.py` (hình). Chưa thử trên xe.

### 1) Sửa [đọc code]
- **Hằng mới** (cạnh `L_KHE_RA`):
  - `THAN_PHONG = (-0.105, 0.285, 0.15)`: thân 25 × 16 cộng lệch tối đa (dọc +0,14, ngang ±0,07), theo khung thân;
  - `LE_THAN = 0.071`;
  - `GOC_VAO_DICH = 17°`.
- **Hàm mới:**
  - `cach_than(ban, tu_the)`: thân phóng tới tâm ô CÓ VẬT trừ nửa ô (ô vật của costmap gồm cả dải nội tiếp, nên số này thấp hơn khoảng cách tới vật thật ~5 cm). Chỉ dùng để **so sánh / chọn**, không làm ngưỡng.
  - `_xoay`, `_doan`: dãy tư thế để chấm.
- `tia_trong(..., chan=None)`: thêm tham số tùy chọn để tia dừng ở ô vật; các lời gọi cũ không đổi.
- **`qua_khe`, hai luật chỉ chạy khi có khe được căn giữa:**
  - **(D) Đỉnh ra ngay trước đích có `yaw_cuoi`:**
    - ứng viên trên trục, từ 0,15 m trước cuối khe (cuối khe đo theo dải cấm, dài hơn vật thật) tới hết khoảng cũ;
    - chỉ nhận ứng viên có đoạn cuối lệch `yaw_cuoi` ≤ 17° và đoạn hợp lệ như cũ;
    - chọn chỗ thân phóng (xoay ở đỉnh ra + đoạn cuối + xoay ở đích) cách ô vật xa nhất. Không có ứng viên thì giữ cách cũ.
  - **(K) Đỉnh vào khe:**
    - chỉ xét khi góc đoạn tới so với trục ≥ `GOC_XOAY`, và khoảng trống phía mũi (tia vuông góc trục, tới ô vật) < bán kính quét thân phóng 0,322 + `LE_THAN`;
    - khi đó thử chèn 1 đỉnh V trước đỉnh vào: lệch trục 10° … góc cũ (bước 2°), cách 0,15–0,45 m (bước 3 cm);
    - V phải: cách ô vật > `R_XOAY_CAN` (luật đỉnh rẽ cũ), hai đoạn hợp lệ, không gần vật hơn đường gốc (trừ hao 3 cm);
    - chọn V làm thân phóng (xoay ở V + đoạn V→E + xoay ở E) xa ô vật nhất; chỉ chèn khi xa hơn không chèn ≥ 1 cm.
  - CANH BAO xét thêm V. Dòng stderr `khe can giua` thêm `, chen (x; y) truoc dinh vao` khi có chèn.
- `main()`: bảng xs giống B6c (chỉ khác dòng đường dẫn ảnh do chạy bản B6c ở thư mục khác) [tính]; tệp du_lieu_0111/tuyen_xs.png không đổi (git diff rỗng) [log].
- md5: tinh_tuyen.py **bbec2ee6**, test_gui_duong.py **6e0664fa** [tính]. `git diff --stat`: tinh_tuyen.py +90/−7, test_gui_duong.py +28 [log].

### 2) Tuyến khác không đổi [tính]
- `in4.py` (4 tuyến × costmap v3 dựng lại + `BAN` PGM của test): **7/8 dòng giống B6c từng byte**; dòng khác duy nhất là di_B t2 trên costmap v3.
- Đầu ra `tinh_duong` + CANH BAO ở 9 điểm đầu (dx, dy ±5) × 2 bản đồ: di_B t1, ve_A t1, ve_A t2 khác **0/9** ở cả hai bản đồ; di_B t2 trên PGM 0/9 (không có khe nên đi như B6c), trên v3 9/9.
- Lý do ve_A t2 không bị luật K: đoạn tới đỉnh vào (1,484; 0,76) lệch trục 27°, nhưng phía mũi (xuống dưới, bên phải H0) trống ≈ 0,7 m > 0,393, nên không xét chèn. Luật D chỉ chạy với `yaw_cuoi` (ve_A không có); di_B t1 không có khe.

### 3) di_B t2: B6c → B6d (đỉnh từ chính tinh_tuyen, 27 nhiễu × 9 mức, cách đo B6d-giấy) [tính]
Đỉnh danh nghĩa:
- B6c: (0,059; 0,310) → (0,374; 0,760) → (1,434; 0,760) → B.
- B6d: (0,059; 0,310) → **(0,245; 0,635)** chèn → (0,374; 0,760) → **(1,339; 0,760)** → B.
- Qua 27 nhiễu có 2 bộ đỉnh (do điểm đầu); CANH BAO 0/27.
- Góc vào B: 19,87° → **12,95°** (≤ 17°).
- Thời gian `tinh_duong` di_B t2: 0,01 → **0,21 s** (WSL), t1 không đổi [tính]. Trên Pi chưa đo.

| vùng | B6c | B6d | ≥ 7,1? |
|---|---|---|---|
| khe trên | 7,5 | 7,5 | đạt |
| rẽ | 5,3 | **6,9** (+14/+7: cuối đoạn V→E, hướng 44°, mũi → tường trên (0,474; 1,135)) | **KHÔNG** (thiếu 0,2) |
| gần B | 6,5 | 7,3 (+14/−7, ở B hướng 13°, tường phải) | đạt |
| gần A | 2,4 | 2,4 | (giới hạn xuất phát, như B6c) |
| khác | 7,5 | 7,5 | đạt |

**Vùng thấp hơn B6c:** không vùng nào (theo nhỏ nhất của vùng). Theo từng mức thì có 5 ô thấp hơn, đều vẫn ≥ 8,5 [tính]:

| vùng · mức | B6c | B6d |
|---|---|---|
| rẽ · 0/+7 | 14,3 | **10,4** |
| gần B · 0/0 | 14,5 | 14,2 |
| gần B · +7/−7 | 9,3 | 8,5 |
| gần B · +7/0 | 15,5 | 15,0 |
| gần B · +7/+7 | 9,3 | 8,5 |

Các ô tăng: rẽ +14/0 8,9 → 11,9 và +14/+7 5,3 → 6,9; gần B +14/−7 6,5 → 7,3 và +14/+7 7,7 → 9,9.

### 4) Vì sao 6,9, và các lựa chọn (cần Thiên quyết) [tính]
- Chỗ 6,9 là cuối đoạn V → E (đỉnh vào (0,374; 0,760)), hướng 44°, chỉ quá ngưỡng giấy θ_K = 42,96° (mục B6d-phương án, luật K).
- Muốn hướng ≤ 43° với đoạn V→E ≥ 0,15 m thì V rơi vào ô cách ô vật ≤ 0,2236 m < `R_XOAY_CAN` 0,2356 (luật đỉnh rẽ cũ, gốc từ bán kính xoay của phụ lái 18,56 + 5 cm). Mã giữ luật cũ nên dừng ở 44°.
- Tìm lưới bằng **thước đo thật** (`ma_luoi_V.py`, 156 V hợp lệ, 5 nhiễu): với E giữ (0,374; 0,760), tốt nhất theo khoảng cách V→E:

  | V→E | tốt nhất |
  |---|---|
  | 5–9 cm | 7,3 |
  | 10–14 cm | 7,3 |
  | 15–19 cm | 6,9 |
  | 20–24 cm | 6,3 |
  | 25–29 cm | 6,8 |
  | 30–34 cm | 6,4 |

- Tìm lưới cho phép dời E (`ma_luoi_EV.py`, V→E ≥ 0,15): tốt nhất ở các vị trí E:

  | E (x; 0,76) | tốt nhất |
  |---|---|
  | 0,39 | 7,1 |
  | **0,40** | **7,3** (V (0,24; 0,62), V→E 0,21 m) |
  | 0,42 | 6,1 |
  | 0,44 | 4,8 |
  | 0,46 | 4,2 |

  Nhưng ô của E (0,40; 0,76) cách ô vật **0,2236 m < `R_XOAY_CAN`** nên sinh CANH BAO; điểm chèn của phương án giấy (0,20; 0,59) cũng chỉ 0,212.
- **Ba lựa chọn:**

  | | cách làm | rẽ | được | mất |
  |---|---|---|---|---|
  | (a) | giữ như mã hiện tại | 6,9 | không CANH BAO, giữ luật đỉnh rẽ | thiếu 0,2 cm ở 1 mức |
  | (b) | cho V gần E (V→E ≈ 0,10 m) | 7,3 | không CANH BAO | đoạn 0,10 m < `min_lookahead_dist` 0,18 của RPP [đọc cấu hình nav2_params_cho.yaml:111], RPP có thể không xoay riêng ở V nên mô hình không còn đúng |
  | (c) | dời E sang (0,40; 0,76), V (0,24; 0,62) | 7,3 | đoạn 0,21 m | E phạm `R_XOAY_CAN` (CANH BAO 1 đỉnh), tức là nới luật đỉnh rẽ cho riêng đỉnh vào khe |

- Mình **không vá tạm**; mã đang ở (a).

### 5) Test [log]
- `test_33_B6d_di_B_t2_chen_dinh_truoc_khe_va_goc_vao_B` (costmap v3 dựng lại như test_32):
  - di_B t2 có 6 đỉnh, không CANH BAO;
  - đỉnh chèn lệch trục nhỏ hơn đỉnh trước nó và < 45°, cách ô vật > `R_XOAY_CAN`, đoạn chèn ≥ 0,18 m;
  - góc đoạn cuối ≤ `GOC_VAO_DICH`;
  - thân phóng ở đoạn cuối + xoay ở B xa ô vật hơn đỉnh ra B6c;
  - ve_A t2 vẫn 5 đỉnh, không CANH BAO.
- Kết quả: test_gui_duong **33/33**, test_lai_xe_lidar **97/97**, test_sep **46/46**.
- Lần chạy đầu test_15 hỏng vì mình đặt trùng tên hàm `_dinh`; đổi tên thành `_dinh_xy` thì đạt.

### 6) Hình RA_SOAT_118_b6d_ma.png
- 1 cm = 4 px, lưới 5 cm, cùng khung x −0,30..2,05, y −0,40..1,25; 2 ô cạnh nhau: (TRƯỚC) B6c di_B t2, (SAU) đường thật từ tinh_tuyen.py bản làm việc.
- Mỗi vùng vẽ thân thật ở ca tệ nhất (màu theo vùng), điểm vật gần nhất, số cm; dưới ô: bảng vùng, góc vào B, đỉnh. 1910 × 914 px, md5 14c66444 [tính].

**Chưa chạy trên Pi; chưa giả lập cả hệ thống; hình học tĩnh (chưa padding / inflation / RPP).**

## B6d-RPP. Mô phỏng hình học pure pursuit cho di_B t2: B6c, (a), (b), (c) (CHỈ TÍNH + VẼ; mã giữ ở (a), không commit)

HEAD 16e5269 [log]. Script `r120/rpp_lib.py` (mô hình), `rpp_t3.py` (kiểm với xe), `rpp_do.py` (đo), `rpp_tom.py` (tóm tắt), `rpp_ve.py` (hình). **Mô hình chưa khớp xe (mục 3).** Chưa thử trên xe.

### 1) Tham số RPP thật [đọc cấu hình `nav2_params_cho.yaml`]
| tham số | giá trị | dòng |
|---|---|---|
| `controller_frequency` | 10,0 Hz | 82 |
| `xy_goal_tolerance` / `yaw_goal_tolerance` (SimpleGoalChecker, `stateful: true`) | 0,15 m / 0,3 rad | 101–103 |
| `desired_linear_vel` | 0,15 m/s | 108 |
| `use_velocity_scaled_lookahead_dist` | true | 109 |
| `lookahead_time` | 2,0 s | 110 |
| `min_lookahead_dist` / `max_lookahead_dist` | 0,18 / 0,6 m | 111 / 112 |
| `allow_reversing` | false | 113 |
| `use_rotate_to_heading` | true | 114 |
| `rotate_to_heading_angular_vel` | 1,0 rad/s | 115 |
| `rotate_to_heading_min_angle` | 0,35 rad (≈ 20,05°) | 116 |
| `max_angular_accel` | 3,2 rad/s² | 117 |
| `use_regulated_linear_velocity_scaling` / `use_cost_regulated_linear_velocity_scaling` | true / true | 118 / 119 |
| `min_approach_linear_velocity` / `regulated_linear_scaling_min_speed` | 0,06 / 0,06 m/s | 121 / 122 |
| `use_collision_detection`, `max_allowed_time_to_collision_up_to_carrot` | true, 1,0 s | 123 / 124 |
| velocity_smoother: OPEN_LOOP, `max_velocity` [0,2; 0; 1,0], `max_accel` [0,3; 0; 2,0], `max_decel` [−0,5; 0; −2,0] | | 238–242 |

Không đặt trong yaml nên lấy mặc định Humble [ước]: `regulated_linear_scaling_min_radius` 0,90; `cost_scaling_dist` 0,6; `cost_scaling_gain` 1,0; `approach_velocity_scaling_dist` 0,6.

### 2) Mô hình (theo hiểu biết về RPP Humble, chưa đối chiếu mã nguồn) [ước]
- Bước 0,1 s, xe một-trục (unicycle). Đường gửi = `lam_day_doan` (điểm cách 5 cm), cắt bỏ điểm phía sau điểm gần xe nhất.
- Tầm nhìn trước L = clamp(|v| · 2,0; 0,18; 0,6). Cà rốt = giao vòng L với đường (có nội suy).
- Còn cách đích ≤ 0,15 m: xoay về hướng đích, xong khi lệch hướng ≤ 0,3 rad.
- Ngược lại, nếu góc tới cà rốt > 0,35 rad: xoay tại chỗ 1,0 rad/s, gia tốc ≤ 3,2.
- Còn lại đi cung: v = 0,15, giảm theo bán kính < 0,90, theo khoảng cách tới ô vật < 0,6 m, và khi gần đích; v ≥ 0,06; ω = v·κ.
- Không mô hình: kiểm va chạm local costmap, SLAM / định vị nhảy, trượt bánh.
- Biến thể "mượt" có thêm velocity_smoother (gia tốc 0,3 / giảm tốc 0,5 m/s², góc 2,0 rad/s²).
- Khoảng cách đo như trước: thân 25 × 16 cm tới ô vật v3, tư thế thật = tư thế mô phỏng + lệch (dọc 0/+7/+14 × ngang −7/0/+7) trong khung thân, 27 nhiễu xuất phát.
- Vùng theo vị trí: gần B (≤ 0,35 m hoặc đang xoay về đích), gần A, rẽ (≤ 10 cm quanh đỉnh trong hoặc đang xoay tại chỗ), khe trên, khác.

### 3) Kiểm mô hình với xe: **KHÔNG KHỚP** [tính / log]
- Dữ liệu: lần T3 05/11 trong RA_SOAT_LOI_A.md việc 2: đường vạch 8 đỉnh trên `costmap_t3_1858`, vết `vet_t3.txt` 23 điểm (tf2_echo ~1 Hz, 18:56:54–18:57:16), controller nhận đích 18:56:56.41 [log].
- Mình **không tìm thấy** "T3 73.0" trong kho. Chỉ có `max_ms=73.0` ở sep_nhat_ky.csv (thời gian vòng của sếp, không liên quan), nên mình so với các số đã ghi: "lệch ≤ 1,1 cm phía trong ở K2" và "lệch lớn nhất +9,3 cm phía ngoài ở khúc rẽ đỉnh 2".

| biến thể | lệch đường lớn nhất | quanh K2 (x 0,30–0,50) | sai vị trí cùng giờ |
|---|---|---|---|
| xe thật | **9,3 cm** (0,064; 0,613) 18:57:12 | ≤ 1,1 cm phía trong | — |
| mô hình gốc | 3,8 cm | 3,0 cm | 0,04–0,28 m |
| không nội suy cà rốt | 4,8 | 3,5 | 0,04–0,28 |
| không giảm tốc theo vật | 5,8 | 4,3 | tới 0,60 |
| + velocity_smoother | 3,4 | 3,1 | 0,04–0,27 |
| + smoother + trễ 0,3 s | 5,5 | 5,5 | 0,07–0,27 |

- Xe thật xoay đầu 7° → 155° mất khoảng 4 s (mô hình khoảng 2 s), và đi vòng ra ngoài tới x −0,067 (đường vạch x 0,009). Mô hình không tái tạo được kiểu lệch này; có thể do định vị nhảy (B3–B4) hoặc động lực xe [suy luận].
- ⇒ Số ở mục 4 chỉ để **so sánh tương đối** các phương án dưới cùng một giả định. **Không** dùng làm khoảng cách tuyệt đối.

### 4) Kết quả trên đường xe đi theo mô hình (27 nhiễu × 9 mức; cm) [tính]
| | B6c | (a) | (b) | (c) |
|---|---|---|---|---|
| rẽ | **2,3** | **2,8** | **2,9** | **2,5** |
| khe trên | 6,6 | 6,7 | 6,7 | 6,7 |
| gần B | 7,3 | 7,1 | 7,1 | 7,1 |
| gần A (giới hạn xuất phát) | 2,3 | 2,3 | 2,3 | 2,3 |
| khác | 2,1 | 2,9 | 3,1 | 3,3 |
| cắt góc lớn nhất, cửa K2 (y > 0,55) | 3,0 | 2,1 | 1,9 | 1,9 |
| cắt góc lớn nhất, đoạn A → P1 (y ≤ 0,55) | 3,5 | 3,3 | 3,4 | 3,3 |
| hướng còn lệch khi đã trên trục (x 0,42–0,75) | 20,8° | 16,3° | 15,6° | 18,4° |
| xoay tại chỗ quanh đỉnh vào | 0/27 | 0/27 | 0/27 | 0/27 |
| dừng cách B / lệch hướng | 0,14–0,15 m / ≤ 12,1° | 0,14–0,15 / ≤ 11,1° | như (a) | như (a) |

- Biến thể "mượt" cho cùng các số vùng (lệch ±0,1 cm), cắt góc giảm 0,4–0,5 cm [tính].
- **Chỗ thấp nhất** của cả 4 phương án: tư thế (0,47–0,50; 0,76) hướng 8–12°, mức +14/+7, góc trước-trái tới góc dưới-trái K2 (0,734; 0,985) [tính].
  - Theo mô hình, RPP **không xoay tại chỗ ở đỉnh vào**: cà rốt đã nằm trên trục nên góc tới cà rốt < 20°. Xe đi cung vào trục, hướng còn lệch tới 16–21°.
  - Phép đo tĩnh B6d giả định xe xoay tại chỗ ở mọi đỉnh nên không thấy chỗ này.
- So các phương án: (a), (b), (c) hơn B6c 0,2–0,6 cm ở vùng rẽ, và hơn nhau ≤ 0,4 cm. Không phương án nào đạt 7,1 trên đường xe đi theo mô hình.
- Ưu thế 7,3 cm của (b) / (c) ở đo tĩnh **không còn** dưới mô hình RPP: (b) 2,9; (c) 2,5.
- Gần B dưới mô hình: xe dừng cách B 0,14–0,15 m (goal checker `xy_goal_tolerance` 0,15), hướng lệch ≤ 12°. Không tới đúng B như phép đo tĩnh giả định.
- Khe trên 6,6–6,7: xe lệch trục ≤ 0,8 cm (tĩnh 7,5).

### 5) Hình RA_SOAT_118_b6d_rpp.png
- 4 ô như `_abc` (1 cm = 8 px, cùng khung), thêm **đường xe đi theo mô hình** (nét đứt tím, nhiễu 0).
- Thân ở ca tệ nhất vùng rẽ trên đường xe đi (+14/+7, cam) và thân lệch 0 cùng tư thế (xanh).
- Dưới mỗi ô: rẽ / khe / gần B / khác, cắt góc, hướng còn lệch. Ô (b) ghi đoạn 0,103 m < 0,18; ô (c) ghi CANH BAO.
- 3730 × 1566 px, md5 dc9afc21 [tính].

**Mô hình chưa khớp xe; chưa thử trên xe; mã giữ ở (a).**

## B7a. Giả lập: cài "tật hụt quãng đi" (CHỈ ĐỌC + ĐỀ XUẤT; không sửa gia_lap/ hay mã xe, không chạy giả lập)

HEAD 16e5269 [log]. Đọc: gia_lap/xe_gia.py, gia_lap/bat_gia.sh, gia_lap/HIEU_CHUAN_0311.md, dev_ws/src/my_bot/scripts/odom_tu_lenh.py; số thật lấy từ B2b–B4 và nhật ký 05/11.

### 1) Giả lập hiện tạo odom và vị trí thật thế nào [đọc code]
- **Vị trí thật** (xe_gia.py):
  - Nghe `/cmd_vel` (đầu ra twist_mux), tích phân 50 Hz (`tich_phan`).
  - Tốc độ thật = `van_toc_san`: chia bánh như dieu_khien (bánh trái × 0,92 để ra duty); duty < 0,05 ⇒ 0; duty < 0,21 ⇒ nâng lên 0,21 (bánh bị nâng) và ω × 0,46.
  - Tốc độ bánh = duty × 0,437 × độ mạnh bánh (trái `banh_trai_that` = 1/0,92).
  - Xoay tại chỗ trễ 0,5 s; mất lệnh 0,5 s ⇒ dừng. Không trượt bánh, không nhiễu.
- **Con quay giả:** `/gyro_yaw` = góc thật, 50 Hz, không trôi, không nhiễu.
- **LiDAR giả:** `/scan` 10 Hz, 500 tia, bước 1 cm trên `map_xuong_v3.png` (ô chiếm p > 0,65). Tia dừng ở ô đầu tiên chạm nên thấy vật gần hơn thật 0–5 cm. Không nhiễu. TF tĩnh base_footprint → laser_frame x 0,025.
- **Odom** (odom_tu_lenh, bat_gia.sh bật lại với `yaw_nguon:=gyro`, `mo_hinh_san:=true`):
  - Tích phân **lệnh** `/cmd_vel` theo cùng mô hình sàn; góc lấy từ con quay giả (= góc thật).
  - Khác xe giả duy nhất ở bánh bị nâng: odom cộng 0,0918 m/s; xe giả bánh trái chạy 0,0918 / 0,92 = 0,0998.
  - Vậy odom gần như đúng tuyệt đối; hụt 0,4 cm/s chỉ khi cả hai bánh bị nâng (đúng giả định B3) [tính].
- **SLAM:** bản chép `slam_dinh_vi_xuong.yaml` (localization, `minimum_travel_distance` / `heading` 0,25), `map_start_pose` = `VI_TRI_DAU`. Scan giả không nhiễu nên khớp tốt và sửa đều.
  - HIEU_CHUAN_0311: 0 NHAY_VI_TRI; ở A lệch cố định 6–8 cm (ve_A) [log].
- **Nav2:** bản chép `nav2_params_cho.yaml`, ghi đè `inflation_radius` (0,20, như yaml) và tuỳ chọn `rotate_to_heading_min_angle`. Sếp và phụ lái chạy từ kho.
- **Ghi:** `/tmp/gia_lap/vi_tri_that.csv` 1 Hz (thật, tin = tf map → base_footprint, khoảng cách thân → ô chiếm). **Chưa ghi map → odom.**

### 2) Đề xuất cài "tật hụt quãng đi" [đề xuất, chưa làm]
**Số thật cần khớp** [B2b, B4]:
- Xe thật ở **phía trước** tư thế tin +8,8..+10,9 cm dọc thân ở cửa K2 (5b2, 5b3, k1); ngang bất định (−1,4..+9,1 cm); dθ 0..−1,0°.
- SLAM tự sửa `map → odom` **về phía trước** +7,0..+8,9 cm dọc thân, có thêm ngang +1,3..+6,8 cm.
- Các lần sửa ≥ 10 cm (sếp ghi) **thưa**: 4 lần cả tối. Quãng từ mốc trước tới lần sửa: 0,75 / 1,88 / 2,61 / 3,59 m. Có 3 quãng ≈ 3,9–4,0 m không lần sửa ≥ 10 cm nào.

**Nhận xét** [tính, suy luận]:
- Hụt đều theo quãng (ví dụ 10 % mọi tốc độ) cho +9–10 cm ở K2, vì đường di_B t2 từ A tới cửa K2 dài ≈ 0,94 m. Nhưng cùng tỉ lệ đó thì 4 m không sửa sẽ hụt ≈ 40 cm, trái với ve_A t2 4/4 DEN_NOI.
- Vậy, hoặc SLAM vẫn sửa phần nhỏ (< 10 cm, không ai ghi), hoặc hụt **dồn vào chỗ chạy chậm**. Cửa K2 sát tường nên RPP giảm tốc theo vật (`cost_scaling_dist` 0,6, mục B6d-RPP), và đi chậm thì bánh dễ rơi vào vùng bị nâng (duty 0,21).
- Mô hình B3 (0,4 cm/s) quá nhỏ: muốn 9 cm cần tốc độ thật ở duty 0,21 cỡ 0,13–0,15 m/s so với odom 0,0918. Chưa đo được (không có encoder).

**Cách cài (mỗi lần đổi một thứ, chạy D(2) như mục 3):**
1. **M1, hụt đều:** xe_gia thêm tham số `he_so_quang` (mặc định 1,0). Tốc độ thẳng thật = mô hình × `he_so_quang`, ω giữ nguyên (con quay vẫn đúng nên dθ ≈ 0, khớp số thật). Thử 1,06 / 1,08 / 1,10.
2. **M2, hụt khi chậm:** xe_gia thêm `v_nang_that` (mặc định 0,0918 = như odom). Bánh bị nâng chạy `v_nang_that` thay vì 0,0918; odom_tu_lenh giữ nguyên. Thử 0,12 / 0,14 m/s. Chỉ tác dụng khi lệnh nhỏ (RPP giảm tốc gần vật, gần đích, `min_approach_linear_velocity` 0,06).
3. **S, SLAM sửa thưa** (cần thêm, vì scan giả hoàn hảo sẽ sửa sau mỗi 0,25 m):
   - S-a (ưu tiên): LiDAR giả bớt hoàn hảo: nhiễu khoảng cách σ 1–2 cm, 12 Hz như xe, tia tới tâm ô thay vì mép.
   - S-b (nếu S-a chưa đủ): chỉ trong **bản chép** slam.yaml của giả lập, nâng `minimum_travel_distance` / `minimum_travel_heading`. Tham số này không còn giống xe nên ghi rõ là núm hiệu chuẩn.
   - Không đổi tham số Nav2 / SLAM của xe.
4. **Ghi thêm để chấm:** xe_gia ghi `map → odom` (tf) 1 Hz vào `vi_tri_that.csv`, và lệch thật − tin theo khung thân (dọc, ngang).
5. **Thứ tự đề xuất:** M2 + S-a trước (giải thích được cả "K2 +9–10 cm" lẫn "4 m không sửa"). Không khớp thì M1 + S-a, cuối cùng mới S-b.

### 3) Bảng hiệu chuẩn (chỉ dẫn D(2))
| thành phần | trên xe (số thật) | giả lập hiện tại | khớp? |
|---|---|---|---|
| lệch dọc thật − tin ở cửa K2 | +8,8..+10,9 cm (5b2), +8,9 / +9,6 (5b3, k1) [tính B2b] | ≈ 0 (Q1 03/11: khoảng cách tới B theo tin và theo thật chênh ≤ 0,3 cm cả 3 lần di_B; chưa đo lệch dọc ở K2) [log] | **không** |
| lệch ngang ở K2 | −1,4..+9,1 cm (bất định) [tính] | ≈ 0 | **không** (phụ) |
| dθ | 0..−1,0° [tính] | 0 (con quay = góc thật) | khớp |
| SLAM sửa map → odom ≥ 10 cm | 4 lần / tối, dọc +7,0..+8,9 cm; cách mốc trước 0,75–3,59 m; 3 quãng ≈ 4 m không sửa [log, tính B4] | 0 NHAY_VI_TRI (HIEU_CHUAN_0311) [log] | **không** |
| tham số SLAM | slam_dinh_vi_xuong.yaml (0,25 / 0,25) [đọc code] | cùng tệp (bản chép) | khớp (tham số) |
| odom ở 0,15 m/s | đúng nếu 0,437 / 0,92 đúng (chưa đo) [B3] | đúng tuyệt đối | chưa biết |
| odom khi bánh bị nâng | chưa đo; cần ≈ 0,13–0,15 m/s nếu M2 đúng [suy luận] | thật 0,0958–0,0998, odom 0,0918 | chưa biết |
| LiDAR | YDLidar ≈ 12 Hz, có nhiễu | 10 Hz, không nhiễu, gần hơn 0–5 cm | **không** |
| Nav2 / RPP | nav2_params_cho.yaml | bản chép cùng tệp | khớp |
| vật lạ trên local costmap | có (VAT_LA_TRUOC 18:21, 18:56 ở (0,44–0,48; 0,03); 18:57:16 ở K2) [log] | không có (trừ ống giả nếu đặt) | **không** (ngoài phạm vi tật này) |
| độ chậm Pi | có | không | không (phụ) |

**Ba kết quả thật phải dựng lại** (đếm lại từ nhật ký 05/11; ghi chỗ khác số Thiên giao):

| # | kết quả | số Thiên giao | nhật ký 05/11 [log] | đích cho giả lập |
|---|---|---|---|---|
| 1 | di_B t2 bản cũ (tinh_tuyen 579cda5, trên xe 05/11) kẹt ở cửa K2 | ≥ 2/3 | lượt 1 có chạy: **5/7** huỷ ở cửa K2: 18:22:14 (0,45; 0,81), 18:57:17 (0,46; 0,81), 19:28:26 (0,57; 0,85), 20:49:05 (0,71; 0,81), 20:57:42 (0,43; 0,81). 1/7 huỷ ở (0,16; 0,76) 21:04:44; 1/7 tới (18:47:49). Thời gian 10–24 s | ≥ 2/3 kẹt ở x 0,40–0,75, y ≥ 0,78 |
| 2 | di_B t1 tới B | 4/4 | **3/3** khi không gặp vật lạ (18:03:52, 18:12:32, 20:44:16 DEN_NOI). Thêm 2 lượt huỷ sau 4,4–4,5 s ở (0,44–0,48; 0,03) do VAT_LA_TRUOC, chưa tới vùng B. **Mình không ra được 4/4 từ nhật ký 05/11**: cần Thiên chỉ nguồn (có thể gồm 02/11 18:19 / 18:31) | 4/4 tới B |
| 3 | ve_A t2 qua khe trên | 2/2 | **4/4** DEN_NOI (18:50:34, 19:02:45, 20:52:20, 20:59:54), 26–30 s | 2/2 (hoặc 4/4) qua khe, DEN_NOI |

Thêm điều kiện đạt (đề xuất): lệch dọc thật − tin ở cửa K2 +8..+11 cm; ≥ 1 lần sửa map → odom 7–9 cm dọc trên di_B t2; trên một chuyến ve_A (≈ 2,3 m) không lần sửa ≥ 10 cm nào.

### 4) Ước thời gian [ước]
| việc | thời gian |
|---|---|
| cài M1/M2 + S-a + ghi map → odom (xe_gia, bat_gia; không đụng mã xe), kiểm khô | 45–60 phút |
| hiệu chuẩn: khoảng 4–6 cấu hình × 3 chuyến (di_B t2 cũ, di_B t1, ve_A t2), ≈ 4 phút / chuyến gồm bật / tắt | 50–75 phút |
| chạy chính: bản cũ di_B t2 × 3 + (a) di_B t2 × 10 (+ ve_A sau mỗi lần để về A) | 13 × ≈ 4–5 phút ≈ 55–65 phút |
| **tổng** | **≈ 2,5–3,5 giờ**, chưa kể lần giả lập treo (đã gặp ở việc 121) |

**Chưa làm gì ở bước này: không sửa gia_lap/, không chạy giả lập.**

## B7b. Giả lập có "tật hụt quãng đi": hiệu chuẩn rồi chạy (a) và (S) — **DỪNG: luật khe không chạy trên costmap thật, cần Thiên quyết**

Nhánh `viec-118`, HEAD 16e5269 [log]. Chỉ sửa gia_lap/ (không commit). Bản mã chạy trong giả lập là **bản chép** trong scratch, không sửa kho:
- `ver_cu`: tinh_tuyen @579cda5, md5 6d208304;
- `ver_a`: tinh_tuyen bản làm việc (a), md5 bbec2ee6;
- gui_duong / sep / lai_xe_lidar @HEAD (giống 579cda5).

Script `scratchpad/b7/` (`chay1.sh`, `loat.sh`, `cham.py`, `tong.py`). Giả lập chưa hiệu chuẩn đủ (bảng mục 2); chưa thử trên xe.

### 1) Sửa gia_lap/ (mặc định = hành vi cũ) [đọc code]
- **xe_gia.py** (e56eb05f → dba4b9aa, +81/−4):
  - tham số `v_nang_that` (M2: tốc độ thật của bánh bị nâng, < 0 = như cũ), `he_so_quang` (M1), `nhieu_lidar`, `tia_tam_o`, `chu_ky_lidar` (S-a);
  - 20 Hz đọc map → odom và odom → base_footprint; **ghi mọi lần map → odom đổi** (> 1 mm hoặc > 0,05°) vào `/tmp/gia_lap/map_odom.csv`: giờ, bước đổi, lệch thật − tin (khung thân tin) trước và sau;
  - ghi vết thật / tin 10 Hz vào `vet_that.csv`.
- **bat_gia.sh** (cad0fba2 → 4cd39a88, +7/−1): `XE_GIA_THAM` truyền tham số tật; `SLAM_MIN_TRAVEL` ghi đè `minimum_travel_distance` **chỉ trong bản chép** slam.yaml của giả lập.
- Giả lập chạy như xe 05/11: phụ lái `phat_lenh=true`, sếp chỉ nhìn (`ra_lenh` / `ra_thoat` false) [log: du_lieu_0511_toi/log_phu_lai.txt, log_sep.txt]. gui_duong gọi thẳng `di_B|ve_A <tuyen>`.

### 2) Hiệu chuẩn (tiêu chí ghi trước 19:41:01 trong `b7/tieu_chi.txt`): **M2 0,12 m/s đạt cả 3, nên không thử 0,14 / S-a / M1**
| kết quả thật phải dựng lại | xe 05/11 [log] | giả lập M2 0,12 (ver_cu) [log] | đạt? |
|---|---|---|---|
| di_B t2 (579cda5) kẹt ở cửa K2, lượt 1 (LUOT=1) | 5/7 | **2/3**: huỷ ở tin (0,45; 0,76) sau 13,0 s và (0,44; 0,76) sau 11,2 s; thật (0,50; 0,82) / (0,52; 0,82). Lần 1 tới B | đạt |
| di_B t1 tới B | 3/3 khi không có vật lạ | **2/2** DEN_NOI lượt 1 (26 s) | đạt |
| ve_A t2 qua khe trên | 4/4 | **2/2** (qua khe theo vết thật và DEN_NOI; lần 1 huỷ 1 lượt ở (0,91; 0,76) trong khe rồi lượt 2 tới) | đạt |

**Ghi chú cách chấm:**
- Vị trí huỷ lấy từ dòng `NAV2,ABORTED` của sếp, cùng nguồn với số xe.
- Bản chấm đầu của mình lấy vết 10 Hz lùi 0,5 s nên ra (0,40; 0,74), dưới ngưỡng y ≥ 0,75 1 cm. Mình đổi sang dòng của sếp **sau khi** đã thấy số này; ghi rõ ở đây để Thiên xét.

**Bảng hiệu chuẩn:**

| thành phần | trên xe (số thật) | giả lập (M2 0,12) | khớp? |
|---|---|---|---|
| lệch dọc thật − tin ở cửa K2 | +8,8..+10,9 cm (B2b) | +9,7 / +11,6 / +13,6 cm (3 lần di_B t2 ver_cu) | gần (cao hơn 1–3 cm) |
| lệch ngang ở K2 | −1,4..+9,1 (bất định) | +2,4..+4,6 | khớp trong khoảng |
| chỗ huỷ (tin) di_B t2 | x 0,43–0,71, y 0,81–0,85 | x 0,44–0,45, y 0,76 | x khớp, y thấp hơn 5 cm |
| thời gian tới lúc huỷ | 10–24 s | 11–13 s | khớp |
| SLAM sửa map → odom ≥ 10 cm | 4 lần / cả tối (≈ 15 chuyến) ≈ 0,3 / chuyến | 9 lần / 7 chuyến ≈ 1,3 / chuyến | **không** (giả lập sửa dày hơn) |
| SLAM sửa ≥ 1 cm | không ghi được trên xe | 6,7 / chuyến, bước trung vị 7,4 cm | chưa so được |
| mỗi lần sửa được bao nhiêu % lệch | lần 3 ≈ đủ theo x, thiếu theo y; lần 4 ≈ 44 % (16 → 8,9 cm) [ước B4] | trung vị 54 % (lần ≥ 5 cm: 58 %) | gần |
| không tật (đối chứng 1 lần, ver_cu) | — | lệch K2 +7,2 cm (SLAM lệch ngay lúc bật ≈ 4 cm); tới B lượt 1, hở K2 6,0 cm | — |
| odom / con quay / LiDAR | như B7a | M2 chỉ đổi bánh bị nâng; LiDAR vẫn hoàn hảo | một phần |

### 3) Chạy (a) và (S) cùng cài đặt (M2 0,12) [log, tính]
| | (a) tinh_tuyen bản làm việc × 10 | (S) 579cda5 + `minimum_travel_distance` 0,10 × 10 |
|---|---|---|
| tới B theo tuyến 2 | **8/10** (lượt 1: 5/10); thêm pa_9 tới bằng tuyến 1 sau khi t2 hỏng; pa_4 không tới | **8/10** (lượt 1: 7/10); ps_3 đổi tuyến, ps_6 HET_DUONG |
| chỗ kẹt (tin, sếp) | lượt 1: 5 lần ở cửa K2 (0,37–0,44; 0,74–0,76); pa_4 thêm 3 lần trong khe (0,93–1,04; 0,68–0,72) | lượt 1: 3 lần **trong khe** (0,74; 0,82), (0,91; 0,83), (0,91; 0,82) |
| khoảng hở nhỏ nhất ở cửa K2 (thân thật → ô chiếm) | 4,5–8,8 cm, trung vị 6,0 | 5,5–8,7 cm, trung vị 7,0 |
| lệch thật − tin lúc tới cửa K2 (dọc) | +7,3..+14,6 cm, trung vị 13,1 | +6,0..+9,9 cm, trung vị 7,7 |
| map → odom đổi ≥ 1 cm | 5,3 / chuyến, ≥ 10 cm: 11 / 10 chuyến | 8,4 / chuyến, bước trung vị 4,7 cm, ≥ 10 cm: 6 / 10 |
| mỗi lần sửa được (% lệch) | trung vị 49 % (−38..79 %) | trung vị 37 % (−17..76 %) |

- pa_2 bị **loại**: giả lập lỗi, không có máy chủ `/follow_path`, xe không chạy; đã chạy bù pa_11.

### 4) DỪNG: bộ (a) thực chất chạy **đường cũ** — luật khe không dò ra khe trên costmap thật [tính]
- Trong cả 10 lần (a), gui_duong in **cùng đường với bản cũ** (48 điểm, CANH BAO (0,36; 0,76)), **không có** dòng `khe can giua`. Nên (a) ≈ 579cda5 trong giả lập; so (a) với cũ / (S) ở mục 3 **không đo được tác dụng của luật khe**.
- **Lý do:** SLAM (xe và giả lập) nạp `map_xuong_v3_pg` (pose graph, slam_dinh_vi_xuong.yaml:32), không phải `map_xuong_v3.png` mà B5–B6d dùng để dựng costmap.
  - Trên costmap sống, khe trên chỉ còn **2 ô đi được** (giá trị 70) tại x 0,9 (bản dựng lại: 3 ô).
  - Tâm khe theo tia nhảy bậc 0,758 → 0,733 → 0,758 (lệch 2,5 cm > `TOL_KHE` 0,01), nên `tim_khe` loại cả chuỗi.
- **Kiểm trên costmap thật của xe 05/11** (du_lieu_0511, raw): `costmap_t3_1858`, `costmap_t4_1850`, `costmap_ket_k2_1933`: bản (a) đều **KHÔNG DÒ RA KHE**; `costmap_t2_1832`: không tìm được đường từ (0,01; 0). Cột x 0,9 có 2 ô giá trị 70 ở y 0,71–0,76 hoặc 0,76–0,81 tuỳ tấm.
- ⇒ Các phép đo B6–B6d (đo trên costmap dựng lại từ `map_xuong_v3.png`) **không đại diện** cho costmap mà gui_duong thấy trên xe. Trên xe, mã (a) nhiều khả năng đi đúng đường cũ [suy luận].
- **Cần Thiên quyết** (chưa làm gì thêm):
  1. đổi bản đồ đo B5–B6d sang `/map` thật (dựng từ pose graph, hoặc từ costmap 05/11) rồi đo lại;
  2. sửa luật dò khe cho khe nhiều bậc (gần ý B6b, đã bỏ);
  3. hoặc dừng nhánh luật khe.

### 5) Trạng thái
- Giả lập đã tắt (không tiến trình sót).
- gia_lap/xe_gia.py, gia_lap/bat_gia.sh **đã sửa, chưa commit**; tinh_tuyen.py / test_gui_duong.py vẫn ở (a), không đổi.
- Kết quả từng lần ở `scratchpad/b7/kq/`.

## B8. Tham số SLAM (localization) quyết định độ sửa: 4 tham số đổi riêng — **không cấu hình nào đạt TỐT; cả 4 tham số không đổi được % sửa (~40 %)**

Nhánh `viec-118`, HEAD 16e5269 [log]. Không sửa mã xe, không commit, không chạm Pi. Chỉ sửa:
- `gia_lap/bat_gia.sh` (+ biến `SLAM_THAM="ten=gia_tri ..."`, ghi đè **chỉ trong bản chép** slam.yaml của giả lập; tên phải có sẵn trong tệp, không thì dừng; md5 cad0fba2 → 53302613);
- `gia_lap/xe_gia.py` không đổi so với B7b (dba4b9aa).

Giả lập chưa hiệu chuẩn đủ (B7b mục 2: giả lập sửa ≥ 10 cm dày hơn xe); **chưa thử trên xe**. Tiêu chí ghi trước lúc 20:51:58 trong `scratchpad/b8/tieu_chi.txt`, trước mọi lần chạy.

### 1) Vì sao mỗi lần khớp chỉ sửa ~40 % lệch [đọc code slam_toolbox 2.6.10 Humble, tải nguồn về `scratchpad/b8/nguon/`]
- **Khi nào xử lý scan** (`slam_toolbox_common.cpp:502–547`, `shouldProcessScan`):
  - `scan_ctr % throttle_scans_` (throttle 2, :529);
  - `stamp − last_scan_time < minimum_time_interval_` thì bỏ (0,5 s, :534);
  - `dist2 < 0.8 * min_dist2` thì bỏ (:539);
  - **không có phép thử góc**.
- **`Mapper::HasMovedEnough`** (`Mapper.cpp:3142–3172`): `if (timeInterval >= m_pMinimumTimeInterval) return true;` (:3151) đứng **trước** phép thử `minimum_travel_heading` (:3160). Vì scan qua được cửa trên đã cách ≥ 0,5 s, phép thử góc không bao giờ được dùng. ⇒ đoán trước P4 không tác dụng.
- **`Mapper::ProcessLocalization`** (`Mapper.cpp:2863–2930`):
  1. Tư thế ban đầu = tư thế odom × phần sửa của scan trước (:2886–2891).
  2. `MatchScan(pScan, GetRunningScans(...))` (:2903–2906): khớp với **các scan định vị gần đây**. Các scan này đã mang lệch cũ (mới sửa được một phần), nên kết quả bị neo về lệch cũ.
  3. `AddEdges` (:2917).
- **`MapperGraph::AddEdges`** (`Mapper.cpp:1434–1497`):
  - `means.push_back(scanPose)` = kết quả khớp scan gần đây (:1484–1486);
  - `LinkNearChains` (:1493) thêm kết quả khớp với **chuỗi bản đồ nạp sẵn**, chỉ khi `iter->size() >= loop_match_minimum_chain_size` (:1646) và `response > link_match_minimum_response_fine` (:1655);
  - `SetSensorPose(ComputeWeightedMean(means, covariances))` (:1496): trung bình **theo nghịch đảo hiệp phương sai** (:1914–1950).
- ⇒ Tư thế mới = trung bình giữa "neo về lệch cũ" (scan gần đây) và "neo về bản đồ". Hai trọng số cỡ nhau thì sửa ≈ 50 % [suy luận]. Phạt khoảng cách (`DISTANCE_PENALTY_GAIN` 0,2, :674–684) không đáng kể ở ~10 cm [tính, B8 trước].
- `slam_toolbox_localization.cpp:185` gọi `ProcessLocalization`; `:199` đặt map → odom từ tư thế đã sửa.
- **4 tham số chọn** (mỗi cái đổi riêng, còn lại = nền):

| | tham số | nền → thử | lý do |
|---|---|---|---|
| P1 | `loop_match_minimum_chain_size` | 10 → 3 | nhiều chuỗi bản đồ được tính hơn |
| P2 | `scan_buffer_size` | 10 → 5 | chuỗi scan gần đây ngắn hơn, ít kéo về lệch cũ |
| P3 | `correlation_search_space_smear_deviation` | 0,1 → 0,03 | đỉnh khớp nhọn hơn, trọng số rõ hơn |
| P4 | `minimum_travel_heading` | 0,25 → 0,10 | đoán trước: **không tác dụng** |

### 2) Chạy: nền = M2 0,12 + tinh_tuyen @579cda5 (git show) + `minimum_travel_distance` 0,10 (bản chép); di_B t2, LUOT=3, × 10 mỗi bộ [log, tính]
| | n0 nền | P1 chain 3 | P2 buffer 5 | P3 smear 0,03 | P4 heading 0,10 |
|---|---|---|---|---|---|
| (a) lượt 1 tới B | 7/10 | 7/10 | 8/10 | 6/10 | **8/10** |
| tới B theo tuyến 2 (cả 3 lượt) | 10/10 | 8/10 (2 đổi tuyến) | 8/10 (2 đổi tuyến) | 6/10 | 9/10 |
| số lần huỷ (sếp) | 4 | 10 | 8 | 9 | 5 |
| (b) lệch dọc thật − tin ở cửa K2, trung vị | 7,8 cm (6,3..10,1) | 8,3 (−5,1..9,9) | 7,9 (6,4..9,1) | 7,3 (6,0..9,1) | **7,5** (6,0..10,3) |
| % lệch sửa mỗi lần (≥ 1 cm), trung vị | 41 % | 42 % (−90..86) | 38 % | 39 % | 37 % |
| khoảng hở nhỏ nhất ở cửa K2: min / trung vị | 4,7 / 6,9 cm | 5,5 / 6,5 | **3,8** / 6,3 | 4,2 / 5,6 | 5,6 / 7,8 |
| (c) map → odom nhảy ≥ 5 cm / 10 chuyến | 40 | 44 | 33 | 34 | 36 |
| nhảy ≥ 10 cm | 2 | 5 | 4 | 3 | 4 |
| CPU tiến trình SLAM, trung vị (% một nhân) | 1,9 | 1,9 | 1,9 | 1,8 | 1,9 |

- **Chỗ huỷ:**
  - n0 / P4: trong khe trên, x 0,90–0,94, y 0,82 (như (S) ở B7b);
  - P3: x 0,60–0,66, y 0,80–0,81 (3 lần chạy, mỗi lần huỷ cả 3 lượt);
  - P1: rải x 0,52–1,11.
- **Đối chiếu tái lập:** n0 cùng cài đặt với (S) ở B7b (7/10, 7,7 cm, 37 %, 39 lần nhảy). Hai đợt khớp nhau trong khoảng ±1 lượt và ±0,5 cm.
- **Loại 3 lần hỏng khởi động** (luật như pa_2 ở B7b), đã chạy bù:
  - n0_2, p3_2: gui_duong `SAI_VI_TRI (khong co costmap sau 15 s)`, chưa gửi lệnh nào; phụ lái lên sau 95 s → bù n0_11, p3_11;
  - p4_2: `BAT_GIA_HONG` (giả lập không lên) → bù p4_11.

**Xét tiêu chí ghi trước:**
- (a) lượt 1 ≥ 9/10: **không bộ nào**.
- (b) lệch trung vị ≤ 4 cm: **không bộ nào** (7,3–8,3 cm).
- (c) nhảy ≤ 2 × nền = 80: cả 5 đạt.
- ⇒ **không có TỐT**. Theo luật ghi trước, chọn "gần nhất" theo (a) rồi (b): P2 và P4 cùng 8/10, P4 lệch 7,5 < 7,9 ⇒ **P4**.
- **Lưu ý:** theo code, P4 = nền (mục 1). Nên P4 − n0 là **phép thử A/A**: nhiễu cỡ ±1/10 lượt, ±0,3 cm lệch, ±4 lần nhảy. Khác biệt của P1–P3 so với nền (trừ số huỷ) đều trong cỡ nhiễu này.

### 3) Mô hình lệch ổn định giải thích vì sao không bộ nào xuống ≤ 4 cm [tính từ `map_odom.csv`, `scratchpad/b8/mo_hinh.py`]
- Gọi d = lệch dọc tăng thêm giữa hai lần sửa, f = phần sửa mỗi lần. Ổn định khi L(1 − f) + d = L, tức **L = d / f** (L = lệch ngay trước lần sửa).

| bộ | d trung vị | f trung vị | d/f | L đo (trung vị) |
|---|---|---|---|---|
| n0 | 5,0 cm | 0,46 | 10,8 | 10,7 |
| P1 | 4,7 | 0,48 | 9,7 | 10,9 |
| P2 | 4,3 | 0,41 | 10,4 | 10,2 |
| P3 | 4,7 | 0,44 | 10,7 | 10,8 |
| P4 | 3,9 | 0,40 | 9,9 | 10,2 |

- Mô hình khớp số đo trong khoảng ±1,2 cm. Khoảng cách giữa hai lần sửa có lệch rõ là 1,9–2,0 s (trung vị) [log].
- Lệch lúc tới K2 ≈ trung bình trong chu kỳ ≈ L(1 − f/2) ≈ 8 cm, khớp 7,3–8,3 đo được [tính].
- Muốn ≤ 4 cm với d ≈ 4,5 cm thì cần **f ≥ ~0,75**, hoặc phải giảm d. Cả 4 tham số đều để f ở 0,40–0,48. ⇒ Độ sửa ~40 % **không do 4 tham số này quyết định** trong giả lập. Theo mục 1, nó do chính phép khớp với scan gần đây (đã lệch) [suy luận].

### 4) Cấu hình gần nhất (P4 = nền) thêm [log]
| phép thử | kết quả | đạt? |
|---|---|---|
| di_B t1 × 3 (M2 0,12) | lượt 1 tới B **3/3**, không huỷ, hở nhỏ nhất dọc đường 8,6–9,8 cm; nhảy ≥ 5 cm 11 / 3 chuyến, ≥ 10 cm 3 | đạt |
| ve_A t2 × 3 (M2 0,12) | DEN_NOI **3/3**, qua khe trên **3/3**, không huỷ, hở ở K2 7,6–9,4 cm; nhảy ≥ 5 cm 10 / 3 chuyến | đạt (không hỏng) |
| M2 **0,14** × 5 (di_B t2) | lượt 1 **5/5**, không huỷ; lệch dọc ở K2 trung vị **8,8** cm (7,0..13,0); hở 5,6–7,9 cm; sửa 48 %/lần; nhảy ≥ 5 cm 32 / 5 chuyến (6,4 / chuyến, nền 4,0), **≥ 10 cm 12** | chịu được ở 5 lần; lệch và nhảy lớn hơn |

- Loại q4t1_2 (`SAI_VI_TRI`, chưa gửi lệnh), bù q4t1_4.
- % sửa của ve_A (trung vị −30 %) không có nghĩa: lệch lúc đó chỉ ~2 cm, nên tỉ số nhiễu.
- 5/5 ở M2 0,14 so với 7/10 ở 0,12: mẫu nhỏ, không kết luận "tật nặng hơn lại tốt hơn".

### 5) Hình `RA_SOAT_118_b8.png`
- Nền n0 (trái) và gần nhất P4 (phải), mỗi bên 10 lần.
- Vết thật liền, vết tin đứt (cam), chấm đỏ = chỗ huỷ (tin, sếp).
- Nền là costmap sống; 1 cm = 4 px.

### 6) Đề xuất (chưa làm, cần Thiên quyết)
1. **Giảm d** thay vì tăng f, mỗi lần một thứ, trong bản chép:
   - `minimum_travel_distance` 0,10 → 0,05;
   - rồi `minimum_time_interval` 0,5 → 0,25.
   - Ở B7b, 0,25 → 0,10 đã hạ lệch từ ~13 xuống ~8 cm [log B7b].
   - Mô hình: d ≈ 2,5 cm, f 0,45 → L ≈ 5,5, lệch ở K2 ≈ 4,3 cm [ước]. CPU còn rộng (1,9 % một nhân).
2. **Sửa gốc** (mã / tham số xe): hiệu chỉnh odom bánh bị nâng (M2). Ngoài phạm vi B8.
3. Tăng f thật sự cần đổi cách trộn trong `AddEdges` (sửa mã slam_toolbox), hoặc dùng bộ định vị khác. Ngoài phạm vi.
- Nhắc lại B7b: luật khe không chạy trên costmap sống, nên B8 dùng 579cda5.

### 7) Trạng thái
- Giả lập đã tắt (không tiến trình sót).
- `gia_lap/bat_gia.sh`, `gia_lap/xe_gia.py` đã sửa, **chưa commit**; tinh_tuyen.py / test_gui_duong.py vẫn ở (a) (bbec2ee6 / 6e0664fa), không đổi.
- Kết quả từng lần ở `scratchpad/b7/kq/` (n0_*, p1_*..p4_*, q4t1_*, q4va_*, q4m14_*, loai_*); tổng hợp ở `scratchpad/b8/tong_10.txt`, `tong_q4*.txt`, `mo_hinh.txt`.

## B9. Giảm d: `minimum_travel_distance` 0,05 (Q1), thêm `minimum_time_interval` 0,25 (Q2) — **không bộ nào đạt; f tụt theo d, lệch chỉ giảm ~1 cm. DỪNG, cần Thiên quyết**

Luật như B8: chỉ bản chép slam.yaml của giả lập (qua `SLAM_MIN_TRAVEL`, `SLAM_THAM` của bat_gia.sh, **không sửa thêm tệp nào ở gia_lap/**). Không mã xe, không commit, không Pi. Giả lập chưa hiệu chuẩn đủ; **chưa thử trên xe**.
- Tiêu chí ghi trước lúc 22:21:32 trong `scratchpad/b9/tieu_chi.txt`. Luật: không bộ nào TỐT thì **không chạy thêm**, dừng.
- Nền = số liệu n0 của B8 (cùng cài đặt, không chạy lại).
- Đã kiểm trong bản chép lúc chạy: Q1 `minimum_travel_distance: 0.05`; Q2 thêm `minimum_time_interval: 0.25`; `throttle_scans` 2 giữ nguyên.

### 1) Kết quả (di_B t2, LUOT=3, × 10, M2 0,12, tinh_tuyen @579cda5) [log, tính]
| | nền n0 (B8) | Q1 travel 0,05 | Q2 Q1 + time 0,25 |
|---|---|---|---|
| (a) lượt 1 tới B | 7/10 | 6/10 | 6/10 |
| tới B theo tuyến 2 (cả 3 lượt) | 10/10 | 7/10 (2 đổi tuyến) | 9/10 (1 đổi tuyến) |
| số lần huỷ (sếp) | 4 | 10 | 9 |
| (b) lệch dọc thật − tin ở cửa K2, trung vị | 7,8 cm (6,3..10,1) | **6,9** (6,2..9,2) | 7,6 (6,0..9,5) |
| d trung vị (lệch thêm giữa 2 lần sửa) | 5,0 cm | **2,0** | **2,0** |
| f trung vị (phần sửa mỗi lần) | 0,46 | **0,25** | **0,26** |
| d / f ↔ L đo (lệch ngay trước lần sửa) | 10,8 ↔ 10,7 | 7,8 ↔ 7,6 | 7,7 ↔ 7,7 |
| khoảng cách giữa 2 lần sửa, trung vị | 2,0 s | 0,6 s | 0,6 s |
| khoảng hở nhỏ nhất ở cửa K2: min / trung vị | 4,7 / 6,9 cm | 5,9 / 6,5 | 4,4 / 6,7 |
| (c) map → odom nhảy ≥ 5 cm / 10 chuyến | 40 | **10** | **10** |
| nhảy ≥ 10 cm | 2 | 5 | 5 |
| map → odom đổi ≥ 1 cm / chuyến; bước trung vị | 8,3; 4,5 cm | 17,6; 2,2 cm | 18,9; 2,2 cm |
| % sửa mỗi lần (cách đo B8), trung vị | 41 % | 24 % | 24 % |
| CPU tiến trình SLAM, trung vị (% một nhân) | 1,9 | 2,9 (1,4..3,4) | 3,0 (1,5..3,2) |

- **Chỗ huỷ:**
  - Q1: 9/10 lần huỷ trong khe trên, x 0,92–0,96, y 0,83; 1 lần ở (0,29; 0,71).
  - Q2: 8/9 lần ở x 0,90–0,99, y 0,82–0,84; 1 lần ở (0,80; 0,81).
  - Như nền: chỗ kẹt nằm trong khe, không ở cửa K2.
- **q1_1:** huỷ ở tin (0,29; 0,71). Sau đó gui_duong in "khong tim duoc duong (24, 22) -> (51, 24)" cho cả 2 tuyến → `HET_DUONG`. Vết thật sau đó trôi xuống (0,57; 0,09) (hình, ô 2). Chưa xét nguyên nhân.
- Không có lần hỏng khởi động ở Q1, Q2.

**Xét tiêu chí ghi trước:**
- (a) ≥ 9/10: **không** (6/10, 6/10).
- (b) ≤ 4 cm: **không** (6,9; 7,6).
- (c) ≤ 80: đạt (10; 10).
- ⇒ **không bộ nào TỐT**, không chạy di_B t1 / ve_A / M2 0,14.

### 2) Đoán trước ở B8 sai: f không giữ ~0,45 mà tụt theo d [tính]
- Ở B8, tôi đoán giảm d thì f giữ nguyên 0,45, nên ước lệch ở K2 ≈ 4,3 cm. Thực tế d giảm 2,5 lần (5,0 → 2,0) thì f cũng giảm gần 2 lần (0,46 → 0,25). L chỉ giảm 10,7 → 7,6 cm; lệch ở K2 7,8 → 6,9 (Q1).
- Lệch ở K2 khớp mô hình L(1 − f/2): Q1 7,6 × 0,875 ≈ 6,7 so với 6,9 đo [tính].
- **Đọc theo B8 mục 1** [suy luận]:
  - Sửa dày hơn thì bộ đệm `scan_buffer_size` 10 chứa các scan **gần nhau hơn và cùng mang lệch cũ**. Kết quả khớp với chúng neo chặt hơn (hiệp phương sai nhỏ hơn), nên phần trọng số dành cho chuỗi bản đồ giảm.
  - ⇒ Trong giả lập, lệch ổn định có **sàn khoảng 7–8 cm** với tật M2 0,12, dù chỉnh nhịp xử lý (B8 + B9) thế nào.
  - Chưa chứng minh bằng log nội bộ slam_toolbox (hiệp phương sai từng `means`).
- Q2 ≈ Q1: nhịp 0,6 s do cửa quãng đường quyết định (0,05 m ở tốc độ xe), không do cửa thời gian. Đã đoán trước trong tieu_chi.
- Phần tốt: nhảy ≥ 5 cm giảm 40 → 10 (sửa nhỏ và đều hơn, bước 2,2 cm). Đổi lại, CPU tăng 1,9 → 3 % một nhân [log].

### 3) Hình `RA_SOAT_118_b9.png`
- Nền n0 | Q1 | Q2, mỗi ô 10 lần.
- Vết thật liền, vết tin đứt (cam), chấm đỏ = chỗ huỷ (tin, sếp).
- Nền là costmap sống; 1 cm = 4 px.

### 4) Cần Thiên quyết (chưa làm gì thêm)
- Chỉnh nhịp xử lý SLAM (B8: 4 tham số; B9: travel / time) **không** đưa lệch ở K2 xuống ≤ 4 cm trong giả lập. Còn lại:
  1. **Sửa gốc tật odom** (M2: bánh bị nâng, odom 0,0918 so với thật 0,12). Đụng mã / tham số xe, ngoài luật B8–B9.
  2. **Đổi cách trộn** trong `MapperGraph::AddEdges` (bỏ / giảm trọng số khớp scan gần đây khi định vị). Sửa mã slam_toolbox, ngoài phạm vi.
  3. **Chấp nhận lệch ~7–8 cm** và xử lý ở tầng đường (B6–B7: luật khe chưa chạy trên costmap sống).
  4. Đo trên xe: % sửa và nhịp sửa thật (giả lập chưa hiệu chuẩn phần này: B7b mục 2, xe sửa ≥ 10 cm thưa hơn).
- Q1 (travel 0,05) giảm nhảy ≥ 5 cm 4 lần, lệch −0,9 cm, nhưng lượt 1 không tốt hơn (6/10 so với 7/10, trong nhiễu ±1). Không đề xuất đưa lên xe.

### 5) Trạng thái
- Giả lập đã tắt.
- gia_lap/ không đổi so với B8 (bat_gia.sh 53302613, xe_gia.py dba4b9aa), chưa commit. tinh_tuyen.py / test_gui_duong.py vẫn ở (a).
- Kết quả từng lần ở `scratchpad/b7/kq/q1_*`, `q2_*`; tổng hợp `scratchpad/b9/tong.txt`, `mo_hinh.txt`.

## B10. q1_1 (HET_DUONG) + bộ công cụ đo ở xưởng `do_xuong/` — **công cụ chạy đúng; 2 tiêu chí thử của do_toc_do không khớp mô hình odom, cần Thiên quyết**

Luật như B8; thêm tệp MỚI trong `xe_scripts/do_xuong/` (không sửa tệp xe cũ). Không commit, không Pi. Các công cụ **chưa chạy trên xe**, chỉ thử trong giả lập (chưa hiệu chuẩn đủ). Script thử ở `scratchpad/b10/`.

### 1) q1_1 (B9): vì sao HET_DUONG, ai phát lệnh, vết thật đi đâu [log, đọc code, tính]
**Diễn biến** (vet_that.csv 10 Hz, sep_nhat_ky.csv, log_phu_lai.txt, map_odom.csv, gui_duong.txt của q1_1):

| giờ | sự kiện |
|---|---|
| 22:22:07.47 | Nav2 bắt đầu xoay tại chỗ (w 0,32 → 0,865 rad/s). Góc tin bám góc thật (lệch 0,7°) tới 08.37 |
| **22:22:08.47** | Góc tin **nhảy +62,6° trong 1 mẫu 0,1 s** (thật 42,6°, tin 104,5°). map → odom **không đổi** (lần đổi đầu tiên ở 22:22:10.215), nên cú nhảy nằm ở **odom → base_footprint (góc odom)** |
| 22:22:08 | Sếp ghi `TREO, sep treo 2.2 s` |
| 22:22:09–19 | Nav2 lái theo góc tin sai khoảng 63°: vết tin đi lên phía K2, vết thật đi sang phải / xuống dưới. SLAM sửa một phần (26,6 cm / 15° ở 10.215, rồi các bước 1–3 cm) |
| 22:22:19.5 | Nav2 `ABORTED` (sau 14,8 s): tin (0,29; 0,71; 38°), thật (0,62; 0,07; −38°), **lệch 71–76 cm** |
| 22:22:20 → hết | v = w = 0 suốt 50 s: **xe không chạy nữa** sau khi huỷ |
| 22:22:48 | gui_duong lập đường lại từ ô tin (24, 22) tới (51, 24): "khong tim duoc duong" cả t2, t1 → `HET_DUONG` |

**Vì sao góc odom nhảy:**
- **Đồng hồ monotonic của máy giả lập (WSL2) nhảy khoảng +2,2 s.**
  - Trong sep_nhat_ky.csv, hiệu `gio_mono − giây (wall)` là 80844,7 tới 22:22:07, rồi 80847,0 từ 22:22:08. Lần nhảy thứ hai, +2,1 s, ở quanh 22:22:38, trùng dòng `sep treo 2.3 s`.
  - Sếp đo "treo" bằng monotonic. Trong khi đó vet_that.csv (đồng hồ wall) đều 0,10 s/mẫu, không hở. Vậy tiến trình **không** treo thật; chính đồng hồ monotonic nhảy [log, tính].
- **Cơ chế trong `odom_tu_lenh.py`** [đọc code, dev_ws @e28c4c0]:
  1. `dt = now − last_tick_time` lấy theo monotonic (:296, :303). Sau cú nhảy 2 s, tin gyro bị coi là cũ (> `gyro_timeout` 0,5 s, :230).
  2. Nên tick đó tích phân góc **theo lệnh**: `theta += w·dt` (:315), với dt ≈ 2 s.
  3. Lệnh w 0,865 rơi vào vùng "bánh bị nâng" của mô hình sàn, cho 0,545 rad/s. 0,545 × 2,0 s = **1,09 rad = 62,6°**, khớp đúng cú nhảy [tính].
  4. Tin gyro kế tiếp được coi là "bắt lại": `gyro_offset = theta − gyro_yaw` (:240–242). Lệnh này **khoá luôn 62,6° sai** vào offset.
- **Ai phát lệnh:**
  - **Nav2 (RPP, `/cmd_vel_nav`)**, từ 22:22:07 tới lúc huỷ.
  - Phụ lái ở `XANH_VANG` từ 22:22:07.29 tới 22:22:19.99. Trạng thái này trả về None, tức **không** phát `/cmd_vel_avoid` (lai_xe_lidar.py:335). Trước và sau đó là `KHONG_CO_DICH`, cũng None (:314).
  - Sếp `ra_lenh=false` (mọi dòng "chi nhin").
  - Teleop: không ai.
  - Sau khi huỷ, không ai phát gì: v = w = 0.
- **"Vết thật trôi tới (0,57; 0,09)":** đó là vị trí thật lúc Nav2 huỷ (cham lấy theo dòng ABORTED của sếp). Xe đi tới đó **trong lúc Nav2 còn chạy**, không phải sau HET_DUONG. Xe dừng ở (0,66; 0,04).
- **HET_DUONG** [suy luận, chưa kiểm vì không lưu costmap]: costmap sống quanh tư thế tin (0,29; 0,71) được vẽ bằng scan chụp từ chỗ thật (0,66; 0,04), lệch góc khoảng 78°. Tường bị vẽ chắn quanh ô xuất phát, nên Dijkstra không ra đường. Dừng là đúng.
- **Tần suất trong giả lập** (`scratchpad/b10/nhay_odom.py`):
  - **3/110 lần chạy** có góc odom nhảy > 5° không do SLAM: p2_8 (−13,5°), q1_1 (+62,6°), q4m14_2 (−65,2°).
  - Cả 114/114 lần chạy đều có ≥ 1 dòng TREO, tức đồng hồ nhảy thường xuyên trên máy này. Chỉ hại khi trùng lúc xe xoay.
  - Số liệu B8/B9 **có tính** 3 lần này (không loại).
- **Xe thật có thể bị không:**
  - **Kiểu đồng hồ nhảy của WSL2: khó** [suy luận: Pi chạy Linux trực tiếp]. Log sếp trên xe 05/11 có 4175 dòng VONG, **0 dòng TREO** [log].
  - **Cơ chế khoá lỗi góc trong odom_tu_lenh: CÓ THỂ.** Mỗi khi `/gyro_yaw` không tới ≥ 0,5 s (nút gyro treo, I2C lỗi), hoặc odom_tu_lenh nghẽn ≥ 0,5 s **trong lúc xoay**, góc tích phân theo lệnh rồi bị khoá vào offset.
  - **Chưa có bằng chứng** trên xe theo cả hai chiều: du_lieu_0111..0511_toi không có log odom_tu_lenh (0 dòng `gyro yaw 로 전환`) [log].
  - Đề xuất (đụng mã xe, cần Thiên quyết): lưu log odom_tu_lenh trên xe; hoặc giới hạn dt / không bắt lại offset theo theta đã tích phân.

### 2) `do_xuong/do_toc_do.sh` [thử giả lập]
- Phát `/cmd_vel_teleop` 0,05 m/s trong 10 s, in và ghi odom đầu / cuối vào `do_toc_do.csv`.
- **HUỶ:**
  - Ctrl+C, mất ssh (SIGHUP), SIGTERM đều dẫn tới phát Twist 0 trong 1 s, mã thoát 130.
  - `rclpy.init(signal_handler_options=NO)` để rclpy không tắt context trước khi kịp phát Twist 0.
  - In PID để `kill -INT` từ tab khác.
- **Kết quả thử:**

| phép thử | thật (xe_gia) | odom | đạt? |
|---|---|---|---|
| M2 0,12, 10 s | quãng thẳng **1,255 m**, cong phải 17,9° | **0,919 m** | thật ≈ 1,2 ✔; odom ≈ 0,46 **✘** |
| không tật, 10 s | 0,959 m, cong phải 13,6° | 0,920 m | ±3 % **✘** (−4,1 %) |
| HUỶ SIGINT sau ~5,5 s | `/cmd_vel` về 0 ở mẫu kế (≤ 0,1 s), không chạy lại | 0,504 m | ✔ |
| HUỶ SIGHUP | như trên | 0,506 m | ✔ |

- **Vì sao 2 tiêu chí không khớp** [đọc code `odom_tu_lenh.py:267–283`, `xe_gia.py:80–98`]:
  - Lệnh 0,05 m/s cho duty 0,114 (phải) / 0,105 (trái), cả hai < `duty_min` 0,21, nên mô hình coi **bánh bị nâng** và odom tính **0,21 × 0,437 = 0,0918 m/s**, bất kể lệnh. 10 s ra 0,92 m chứ không phải 0,46 m. Số 0,46 m ứng với "odom = lệnh × 0,918", không phải mô hình đang chạy.
  - "Không tật" trong giả lập: bánh trái thật mạnh hơn 1/0,92 lần (`banh_trai_that` 1,087), nên thật 0,0958 m/s so với odom 0,0918: **−4,1 %** (đúng như tính), và xe đi cong.
  - Công cụ đo đúng như mô hình; **tiêu chí cần Thiên chỉnh** (ví dụ: M2 odom ≈ 0,92 m / thật ≈ 1,25 m; không tật ±5 %), hoặc đổi tốc độ thử.

### 3) `do_xuong/ghi_sua_vi_tri.py` [thử giả lập]
- Chỉ nghe `/tf`. Ghi mọi lần map → odom đổi (> 1e-6) gồm giờ, map → odom, bước (cm, °), vị trí tin sau khi đổi.
- Thử song song 1 lần di_B t2 cấu hình Q1 (B9): **23/23** lần đổi khớp `map_odom.csv` của xe_gia. Giá trị map → odom và độ lớn bước lệch 0,00 mm / 0,000°. Ghi sớm hơn xe_gia 7–48 ms (xe_gia đọc 20 Hz).
- 16 lần ≥ 1 cm, 0 lần ≥ 5 cm, cùng cỡ Q1 ở B9.

### 4) `do_xuong/khop_scan_A.py` [thử]
- Khớp như B2/B2b (lưới ±12 cm / 0,5 cm, ±6° / 0,5°; điểm "sai 1 ô", phụ "đúng ô"), viết lại dạng véc-tơ: 0,6 s một scan.
- Thêm `chup`: trên Pi lấy 1 `/scan` (QoS sensor data) và tư thế tin map → base_footprint **đúng lúc scan**, ghi tệp.

| scan | tư thế tin | kết quả | B2b |
|---|---|---|---|
| **scan_ket_5b2** | (0,313; 0,796; 8,5°) | dx +8,5, dy +7,5, 0,0°; **dọc +9,5**, ngang +6,2 cm; 0,992 / 0,848; 75 nghiệm hoà | **trùng** |
| scan_ket_5b3 | (0,155; 0,756; 42,2°) | dọc +8,9, ngang −0,6; 0,962 / 0,877; 91 nghiệm | trùng |
| scan_ket_k1_2106 | (0,155; 0,756; 42,7°) | dọc +9,6, ngang −1,4; 0,966 / 0,734; 46 nghiệm | trùng |
| giả lập, `chup` ở A (thật (0; 0; 0°)) | (−0,040; −0,040; 1,0°) | dọc +4,1, ngang +3,9, −1,0°; tư thế chỉnh **(0,000; 0,000; 0,0°)** = thật | — |

- Lần chụp đầu thất bại: giữ scan đầu tiên, cũ hơn bộ đệm tf. Đã sửa: giữ 20 scan gần nhất, lấy scan mới nhất đã có tf.
- LiDAR giả lập hoàn hảo nên tập hoà điểm rộng (1811 nghiệm); "đúng ô" vẫn tách đúng.

### 5) `do_xuong/HUONG_DAN.md`
- Lệnh từng bước theo tab (`aaa@` máy tính, `project1@` Pi): chép lên Pi; đo tốc độ 3 lần + thước; ghi sửa vị trí; chụp + khớp scan; danh sách gửi lại.

### 6) Trạng thái
- Tệp mới: `do_xuong/do_toc_do.sh`, `ghi_sua_vi_tri.py`, `khop_scan_A.py`, `HUONG_DAN.md`.
- Không sửa tệp cũ nào khác (gia_lap/ như B8). Giả lập đã tắt. **Chưa commit.**
