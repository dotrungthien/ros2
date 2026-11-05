# RÀ SOÁT 120: vật ở sau thân lúc cua (CHỈ ĐỌC, không sửa tệp có sẵn, không commit). CHƯA sửa mã

## 0. Kiểm trước
- `git rev-parse --short HEAD` ở máy = **50ae80e** (mong b296baf).
  - `git fetch origin` cho thấy origin/dong-bo = **b296baf**, tức máy thiếu 5 commit: 27d5523, d297dea, 35b6c1e, 1d893aa, b296baf. Máy không có commit nào chưa push.
  - **Không pull** (CHỈ ĐỌC). Mọi dữ liệu đọc thẳng từ git object `origin/dong-bo:xe_scripts/du_lieu_0511_toi/...` (`git show`). Cây làm việc không đổi.
- `grep -c "^- " scan_ket_k1_2106.txt` (bản origin/dong-bo) = **840** ✓.
- Đồng hồ: Pi chạy giờ **KST (UTC+9)**. Kiểm chéo: log_nut_chuyen.txt `== cau_noi_xe 21:04:19: di_B` ↔ vet_5b3 bắt đầu 1793880246 = 21:04:06 KST; `Controller patience exceeded` 1793880284.35 = **21:04:44.35** (mốc Thiên).
- tf2_echo (vet_5b3.txt) chỉ có mẫu **~1 Hz** (cách 0,87-1,13 s). Lưới 0,5 s ở 1.2 là **nội suy tuyến tính**; tốc độ lấy theo đoạn giữa hai mẫu gốc.
- `At time` của tf2_echo là dấu thời gian của tf, cùng đồng hồ epoch với log ROS.

## 1. B1: chuyển động 21:04:20 → 21:05:50 (vet_5b3.txt, map → base_footprint)

### 1.1 Mẫu gốc (chỉ in các mẫu tới khi xe đứng hẳn; sau 21:04:44.69 mọi mẫu giống nhau, xem 1.4)
| gio Pi | epoch | x (m) | y (m) | goc (do) | dich (cm/s) | xoay (do/s) | ban kinh cong (m) |
|---|---|---|---|---|---|---|---|
| 21:04:19.68 | 1793880259.69 | -0.000 | 0.000 | 0.8 | 0.0 | 0.0 | - |
| 21:04:20.68 | 1793880260.68 | -0.000 | 0.000 | 0.8 | 0.0 | 0.0 | - |
| 21:04:21.68 | 1793880261.68 | -0.000 | 0.000 | 0.8 | 0.0 | 0.0 | - |
| 21:04:22.68 | 1793880262.68 | -0.000 | 0.000 | 0.8 | 0.0 | -0.0 | - |
| 21:04:23.68 | 1793880263.69 | -0.000 | 0.000 | 0.8 | 0.0 | 0.0 | - |
| 21:04:24.63 | 1793880264.64 | -0.000 | 0.000 | 0.8 | 0.0 | 0.0 | - |
| 21:04:25.66 | 1793880265.66 | -0.000 | 0.000 | 0.8 | 0.0 | -0.0 | - |
| 21:04:26.63 | 1793880266.63 | -0.000 | 0.000 | 0.8 | 0.0 | 0.0 | - |
| 21:04:27.69 | 1793880267.70 | -0.000 | 0.000 | 0.9 | 0.0 | 0.1 | - |
| 21:04:28.68 | 1793880268.68 | -0.000 | 0.000 | 0.9 | 0.0 | 0.0 | - |
| 21:04:29.68 | 1793880269.68 | -0.000 | 0.000 | 0.9 | 0.0 | 0.0 | - |
| 21:04:30.68 | 1793880270.68 | -0.000 | 0.000 | 0.9 | 0.0 | 0.0 | - |
| 21:04:31.68 | 1793880271.68 | -0.000 | 0.000 | 1.0 | 0.0 | 0.0 | - |
| 21:04:32.68 | 1793880272.68 | -0.000 | 0.000 | 1.0 | 0.0 | 0.0 | - |
| 21:04:33.69 | 1793880273.69 | -0.000 | 0.000 | 1.0 | 0.0 | -0.0 | - |
| 21:04:34.68 | 1793880274.68 | -0.000 | 0.000 | 11.8 | 0.0 | 10.9 | 0.00 |
| 21:04:35.68 | 1793880275.68 | -0.000 | 0.000 | 51.1 | 0.0 | 39.4 | 0.00 |
| 21:04:36.68 | 1793880276.68 | 0.008 | 0.036 | 79.4 | 3.7 | 28.3 | 0.07 |
| 21:04:37.68 | 1793880277.69 | 0.017 | 0.153 | 91.4 | 11.7 | 11.9 | 0.56 |
| 21:04:38.68 | 1793880278.69 | 0.008 | 0.223 | 86.7 | 7.0 | -4.7 | 0.86 |
| 21:04:39.68 | 1793880279.68 | 0.016 | 0.326 | 84.5 | 10.4 | -2.1 | 2.77 |
| 21:04:40.68 | 1793880280.68 | 0.029 | 0.398 | 75.9 | 7.3 | -8.7 | 0.48 |
| 21:04:41.69 | 1793880281.69 | 0.055 | 0.468 | 67.6 | 7.4 | -8.2 | 0.52 |
| 21:04:42.68 | 1793880282.68 | 0.091 | 0.531 | 58.2 | 7.3 | -9.4 | 0.44 |
| 21:04:43.68 | 1793880283.68 | 0.140 | 0.608 | 56.4 | 9.1 | -1.8 | 2.85 |
| 21:04:44.69 | 1793880284.69 | 0.155 | 0.756 | 42.1 | 14.8 | -14.1 | 0.60 |
| 21:04:45.68 | 1793880285.68 | 0.155 | 0.756 | 42.1 | 0.0 | 0.0 | - |


### 1.2 Lưới 0,5 s (NỘI SUY)
| gio Pi | x (m) | y (m) | goc (do) | dich (cm/s) | xoay (do/s) |
|---|---|---|---|---|---|
| 21:04:20.00 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:20.50 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:21.00 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:21.50 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:22.00 | 0.000 | 0.000 | 0.8 | 0.0 | -0.0 |
| 21:04:22.50 | 0.000 | 0.000 | 0.8 | 0.0 | -0.0 |
| 21:04:23.00 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:23.50 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:24.00 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:24.50 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:25.00 | 0.000 | 0.000 | 0.8 | 0.0 | -0.0 |
| 21:04:25.50 | 0.000 | 0.000 | 0.8 | 0.0 | -0.0 |
| 21:04:26.00 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:26.50 | 0.000 | 0.000 | 0.8 | 0.0 | 0.0 |
| 21:04:27.00 | 0.000 | 0.000 | 0.9 | 0.0 | 0.1 |
| 21:04:27.50 | 0.000 | 0.000 | 0.9 | 0.0 | 0.1 |
| 21:04:28.00 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:28.50 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:29.00 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:29.50 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:30.00 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:30.50 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:31.00 | 0.000 | 0.000 | 0.9 | 0.0 | 0.0 |
| 21:04:31.50 | 0.000 | 0.000 | 1.0 | 0.0 | 0.0 |
| 21:04:32.00 | 0.000 | 0.000 | 1.0 | 0.0 | 0.0 |
| 21:04:32.50 | 0.000 | 0.000 | 1.0 | 0.0 | 0.0 |
| 21:04:33.00 | 0.000 | 0.000 | 1.0 | 0.0 | -0.0 |
| 21:04:33.50 | 0.000 | 0.000 | 1.0 | 0.0 | -0.0 |
| 21:04:34.00 | 0.000 | 0.000 | 4.3 | 0.0 | 10.9 |
| 21:04:34.50 | 0.000 | 0.000 | 9.8 | 0.0 | 10.9 |
| 21:04:35.00 | 0.000 | 0.000 | 24.2 | 0.0 | 39.4 |
| 21:04:35.50 | 0.000 | 0.000 | 43.9 | 0.0 | 39.4 |
| 21:04:36.00 | 0.003 | 0.011 | 60.1 | 3.7 | 28.3 |
| 21:04:36.50 | 0.007 | 0.029 | 74.2 | 3.7 | 28.3 |
| 21:04:37.00 | 0.011 | 0.073 | 83.2 | 11.7 | 11.9 |
| 21:04:37.50 | 0.015 | 0.131 | 89.1 | 11.7 | 11.9 |
| 21:04:38.00 | 0.014 | 0.175 | 89.9 | 7.0 | -4.7 |
| 21:04:38.50 | 0.010 | 0.210 | 87.6 | 7.0 | -4.7 |
| 21:04:39.00 | 0.011 | 0.255 | 86.0 | 10.4 | -2.1 |
| 21:04:39.50 | 0.015 | 0.307 | 84.9 | 10.4 | -2.1 |
| 21:04:40.00 | 0.020 | 0.349 | 81.8 | 7.3 | -8.7 |
| 21:04:40.50 | 0.027 | 0.385 | 77.5 | 7.3 | -8.7 |
| 21:04:41.00 | 0.037 | 0.420 | 73.2 | 7.4 | -8.2 |
| 21:04:41.50 | 0.050 | 0.455 | 69.1 | 7.4 | -8.2 |
| 21:04:42.00 | 0.066 | 0.488 | 64.7 | 7.3 | -9.4 |
| 21:04:42.50 | 0.084 | 0.519 | 59.9 | 7.3 | -9.4 |
| 21:04:43.00 | 0.106 | 0.555 | 57.6 | 9.1 | -1.8 |
| 21:04:43.50 | 0.131 | 0.594 | 56.7 | 9.1 | -1.8 |
| 21:04:44.00 | 0.145 | 0.654 | 51.9 | 14.8 | -14.1 |
| 21:04:44.50 | 0.152 | 0.728 | 44.8 | 14.8 | -14.1 |
| 21:04:45.00 | 0.155 | 0.756 | 42.1 | 0.0 | 0.0 |
| 21:04:45.50 | 0.155 | 0.756 | 42.1 | 0.0 | 0.0 |

| 21:04:45.00 → 21:05:50.00 | 0.155 | 0.756 | 42.1 → 42.3 | 0.0 | 0.0 |

### 1.3 Dòng collision | patience | rotat của log_bat_xe_nav.txt (21:04:15 → 21:06:05)
- dong 127, 21:04:44.15 (epoch 1793880284.151593968): RegulatedPurePursuitController detected collision ahead!
- dong 128, 21:04:44.25 (epoch 1793880284.251546265): RegulatedPurePursuitController detected collision ahead!
- dong 129, 21:04:44.35 (epoch 1793880284.351671172): RegulatedPurePursuitController detected collision ahead!
- dong 130, 21:04:44.35 (epoch 1793880284.352043913): Controller patience exceeded
- dong 133, 21:05:16.20 (epoch 1793880316.206051949): RegulatedPurePursuitController detected collision ahead!
- dong 134, 21:05:16.30 (epoch 1793880316.307060209): RegulatedPurePursuitController detected collision ahead!
- dong 135, 21:05:16.40 (epoch 1793880316.406201968): RegulatedPurePursuitController detected collision ahead!
- dong 136, 21:05:16.50 (epoch 1793880316.506161653): RegulatedPurePursuitController detected collision ahead!
- dong 137, 21:05:16.50 (epoch 1793880316.506577190): Controller patience exceeded
- dong 140, 21:05:48.07 (epoch 1793880348.077977616): RegulatedPurePursuitController detected collision ahead!
- dong 141, 21:05:48.17 (epoch 1793880348.175086838): RegulatedPurePursuitController detected collision ahead!
- dong 142, 21:05:48.27 (epoch 1793880348.275215264): RegulatedPurePursuitController detected collision ahead!
- dong 143, 21:05:48.37 (epoch 1793880348.374953504): RegulatedPurePursuitController detected collision ahead!
- dong 144, 21:05:48.37 (epoch 1793880348.375293578): Controller patience exceeded

- Không có dòng nào chứa "rotat" trong khoảng này.
- Mỗi lượt: `Received a goal` (21:04:34.15 / 21:05:16.20 / 21:05:48.07) → collision ahead → patience 0,20 / 0,30 / 0,30 s sau dòng collision đầu → `Aborting handle`.
- Phụ lái (log_phu_lai.txt) cùng lúc: `XANH_VANG -> KHONG_CO_DICH` 21:04:44.37, với vùng `SAU-TRAI=0.07 TRAI=0.09 TR-TRAI=0.17 TRUOC=0.27`. Lượt 2, 3 giữ nguyên `SAU-TRAI=0.06-0.07`.

### 1.4 Ghép theo giờ
- 21:04:34.15 nhận đích. 21:04:34.7 → 35.7: **xoay tại chỗ** (góc 1° → 51°, dịch 0 cm), tức RPP quay về hướng đường.
- 21:04:36 → 21:04:43.7: tiến theo cung, 7-12 cm/s, góc 79° → 56° (quay phải).
- Đoạn 21:04:43.68 → 21:04:44.69 (chứa mốc collision 44.15-44.35):
  - dịch 14,8 cm (x 0,140 → 0,155, y 0,608 → 0,756), góc 56,4° → 42,1° (−14,1°/s), tức **đi cung** bán kính ≈ 0,6 m;
  - tốc độ này cao hơn các đoạn trước (7-10 cm/s). Mẫu 1 Hz không tách được một phần bước nhảy có phải do SLAM chỉnh map→odom hay không.
- Từ 21:04:44.69 tới hết vết (21:06:34.68, 111 mẫu, gồm cả lúc chụp scan, stamp 1793880390.44 = **21:06:30.44**):
  - x = 0,155, y = 0,756 **không đổi tới mm**;
  - góc trôi 42,1° → 42,8° (≈ 0,35°/phút, cỡ trôi gyro), không phải lệnh xoay;
  - lượt 2 (21:05:16) và lượt 3 (21:05:48) bị hủy trong 0,3 s, xe không nhích.

**Kết luận B1:** lúc 21:04:44 xe **đi cung** (tiến + quay phải, ≈ 15 cm/s, −14°/s, r ≈ 0,6 m), không xoay tại chỗ, không đi thẳng. Từ 21:04:44.69 tới lúc chụp scan 21:06:30 xe **không nhích** (x, y đổi < 1 mm; góc chỉ trôi 0,7°).

## 2. B2: scan_ket_k1_2106.txt sang khung thân (đọc tệp trong máy sau `git merge --ff-only origin/dong-bo`, HEAD = b296baf)

### 2.0 Số tia
- `grep -c "^- "` = 840 là **420 `ranges` + 420 `intensities`**, tức scan có **420 tia** (−180..180°, bước 0,859°).
- 387 tia hợp lệ (0,10-12 m). Stamp 1793880390.44 = 21:06:30.44, `laser_frame`.

### 2.1 Vị trí, hướng, cờ inverted của LiDAR trong mã
| nội dung | tệp:dòng | giá trị |
|---|---|---|
| URDF my_bot | dev_ws/src/my_bot/description/robot.urdf.xacro:17-51 | chỉ có base_footprint, base_joint, base_link, bánh. **Không có link/joint LiDAR** (`grep -c "laser\|lidar"` = 0) |
| Driver LiDAR | dev_ws/src/my_bot/launch/mapping_my_bot.launch.py:152-156 | include `ydlidar_ros2_driver/launch/ydlidar_launch.py`. Gói này **không ở trong kho** (nằm trên Pi), nên TF base → `laser_frame` và các cờ driver không đọc được ở đây |
| Ghi chú đo tay | dev_ws/src/my_bot/config/nav2_params.yaml:138 (và :171) | "LiDAR는 base_link 기준 앞쪽 +2.5cm"; footprint :139 `[[0.145, 0.08], [0.145, -0.08], [-0.105, -0.08], [-0.105, 0.08]]` (cùng thân xe Thiên cho) |
| Phụ lái đổi scan | xe_scripts/lai_xe_lidar.py:29 `LIDAR_X = 0.025`; :81-89 `scan_to_xy` | x = 0,025 + r·cos θ, y = r·sin θ, θ = angle_min + i·inc + offset; **không xoay, không lật** |
| Góc lệch | xe_scripts/lai_xe_lidar.py:675 `lidar_angle_offset_deg` = 0.0; dev_ws/src/my_bot/scripts/logic_quyet_dinh.py:100 = 0.0 | 0° |
| `inverted` / `reversion` | — | **không có ở đâu trong kho** (grep toàn kho, trừ bat_xe.sh) |

Mình dùng đúng cách đổi của phụ lái: LiDAR ở (0,025; 0), không xoay, giữ nguyên chiều `/scan`.

### 2.2 Kiểm chiều trục (không tự lật)
- Đặt scan lên map_xuong_v3 ở tư thế lúc dừng (0,155; 0,756; 42,1°, mục 1.4):
  - **như /scan:** 387/387 điểm trong bản đồ, **86 %** cách ô chiếm ≤ 1 ô;
  - **lật y:** 241 điểm trong bản đồ, chỉ **10 %**.
- Vậy chiều `/scan` là đúng (driver đã áp chiều); không có dấu hiệu lật trục.

### 2.3 Mọi điểm cách mép thân ≤ 0,15 m
Thân: x −0,105 → 0,145, y ±0,08. Vùng: trước x ≥ 0,095 / giữa / sau x < −0,05; trái y > 0. Ngưỡng xoay 0,1855 m (`R_XOAY` = hypot(0,145; 0,08) + 0,02 = 0,1856, lai_xe_lidar.py:37).

| # | x (m) | y (m) | toi mep than (cm) | vung | toi tam (m) | < 0,1855? |
|---|---|---|---|---|---|---|
| 1 | -0.125 | 0.140 | 6.3 | sau-trai | 0.187 | khong |
| 2 | -0.124 | 0.143 | 6.5 | sau-trai | 0.189 | khong |
| 3 | -0.123 | 0.146 | 6.9 | sau-trai | 0.191 | khong |
| 4 | -0.120 | 0.148 | 6.9 | sau-trai | 0.190 | khong |
| 5 | -0.118 | 0.151 | 7.2 | sau-trai | 0.192 | khong |
| 6 | -0.117 | 0.153 | 7.4 | sau-trai | 0.193 | khong |
| 7 | -0.115 | 0.156 | 7.6 | sau-trai | 0.193 | khong |
| 8 | -0.112 | 0.158 | 7.8 | sau-trai | 0.194 | khong |
| 9 | -0.110 | 0.160 | 8.1 | sau-trai | 0.195 | khong |
| 10 | -0.109 | 0.163 | 8.3 | sau-trai | 0.196 | khong |
| 11 | -0.106 | 0.165 | 8.5 | sau-trai | 0.196 | khong |
| 12 | -0.104 | 0.168 | 8.8 | sau-trai | 0.198 | khong |
| 13 | -0.102 | 0.171 | 9.1 | sau-trai | 0.199 | khong |
| 14 | -0.100 | 0.173 | 9.3 | sau-trai | 0.200 | khong |
| 15 | -0.098 | 0.176 | 9.6 | sau-trai | 0.202 | khong |
| 16 | -0.096 | 0.179 | 9.9 | sau-trai | 0.203 | khong |
| 17 | -0.094 | 0.181 | 10.1 | sau-trai | 0.204 | khong |
| 18 | -0.092 | 0.184 | 10.4 | sau-trai | 0.206 | khong |
| 19 | -0.090 | 0.187 | 10.7 | sau-trai | 0.207 | khong |
| 20 | -0.087 | 0.189 | 10.9 | sau-trai | 0.208 | khong |
| 21 | -0.086 | 0.193 | 11.3 | sau-trai | 0.211 | khong |
| 22 | -0.083 | 0.195 | 11.5 | sau-trai | 0.212 | khong |
| 23 | -0.081 | 0.198 | 11.8 | sau-trai | 0.213 | khong |
| 24 | -0.079 | 0.201 | 12.1 | sau-trai | 0.216 | khong |
| 25 | -0.076 | 0.203 | 12.3 | sau-trai | 0.217 | khong |
| 26 | -0.074 | 0.207 | 12.7 | sau-trai | 0.219 | khong |
| 27 | -0.071 | 0.209 | 12.9 | sau-trai | 0.221 | khong |
| 28 | -0.069 | 0.212 | 13.2 | sau-trai | 0.223 | khong |
| 29 | -0.066 | 0.215 | 13.5 | sau-trai | 0.225 | khong |
| 30 | -0.064 | 0.219 | 13.9 | sau-trai | 0.228 | khong |
| 31 | -0.061 | 0.222 | 14.2 | sau-trai | 0.230 | khong |
| 32 | -0.059 | 0.225 | 14.5 | sau-trai | 0.232 | khong |
| 33 | -0.056 | 0.228 | 14.8 | sau-trai | 0.235 | khong |

- **33 điểm, tất cả ở sau-trái.** Trước, giữa, sau-phải, trái thuần: 0 điểm.
- Vùng lớp 2 hiện tại (cách thân ≤ 0,105 và x ≥ 0,095): **0 điểm**.
- **Không điểm nào < 0,1855 m tới tâm.** Gần nhất là 0,187 m (điểm #1), tức chỉ hơn ngưỡng xoay **1,5 mm**.

### 2.4 Kiểm chéo
- **Điểm gần nhất:** #1 (−0,125; 0,140), **sau-trái**, cách mép thân **6,3 cm**:
  - nằm sau đuôi 2,0 cm và ngoài hông trái 6,0 cm, tức chéo góc đuôi-trái (−0,105; 0,08);
  - các điểm #2-#9 (cùng mặt) cách 6,5-8,1 cm.
- **So với Thiên đo 7-8 cm:** cùng phía (sau-trái); số đo của Thiên rơi vào dải 6,3-8,1 cm của mặt vật. Điểm gần nhất ngắn hơn khoảng 1 cm.
- **So với phụ lái `SAU-TRAI`:** phụ lái đo cùng thước (`do_vung` = khoảng tới mép thân nhỏ nhất theo vùng, lai_xe_lidar.py:193-201).
  - Log lúc 21:04:44.37: **0.07**; lượt 2 (21:05:16): 0.06; lượt 3 (21:05:48): 0.07.
  - Scan này (21:06:30) qua `mo_ta_vung`: **0.06** (6,3 cm).
  - Xe đứng yên suốt (mục 1.4), nên chênh 0,06 ↔ 0,07 là dao động 1 cm giữa các vòng quét, **khớp**.
  - Ba vùng còn lại trong log (`TRAI=0.09 TR-TRAI=0.17 TRUOC=0.27`) cũng giống hệt scan này.
- **Vật là gì:** đặt lên map_xuong_v3, điểm #1 → map (−0,032; 0,776), cách ô chiếm 1 ô; #9 → (−0,034; 0,801), đúng ô chiếm. Tức là **vật đã có trên bản đồ v3**, không phải vật lạ: mặt dưới của khối nhô ra từ tường trái (ô chiếm x −0,16..0,03; y 0,79..1,16). (Sửa ở B3: lúc đầu ghi "tường trái, cột x ≈ −0,05..0"; xem lưới ô ở 3.2.)

## 3. B3: hình RA_SOAT_120.png (1 cm = 4 px, lưới 5 cm). Script: scratchpad `r120/b3.py` (không vào kho)

### 3.1 Hình 1: khung thân
- Vẽ: thân xe, LiDAR (0,025; 0), vòng 0,1855 m, vùng lớp 2 hiện tại (tô cam; 0 điểm), và 149 điểm cách **mép thân** ≤ 0,30 m (đỏ nếu ≤ 0,15 m).
- **Vệt quét (1):** xe đi tiếp cung R 0,6 m quay phải, 15 cm/s, 0,5 s, tức quay 7,2° và tiến 7,5 cm (đúng chuyển động lúc 21:04:44, mục 1.4). Tâm quay ở (0; −0,6) khung thân lúc dừng.
  - góc đuôi-trái (−0,105; 0,080) → (−0,019; 0,088): tiến 8,6 cm, sang trái 0,8 cm;
  - góc đuôi-phải (−0,105; −0,080) → (−0,039; −0,071): tiến 6,6 cm, sang trái 0,9 cm.
- Khoảng cách nhỏ nhất từ scan tới thân trong suốt 0,5 s đó (bước 1/50) = **6,3 cm, đạt ngay lúc φ = 0**. Khi đi tiếp, đuôi rời xa vật, nên vật sau-trái **không bị đuôi quét trúng** nếu xe được đi tiếp cung đó.

### 3.2 Hình 2: khung bản đồ
- Vẽ: ô chiếm map_xuong_v3, thân xe tại (0,16; 0,76) góc 42° (theo lệnh; đo được (0,155; 0,756; 42,1°)), scan 21:06:30, vệt hai góc đuôi.
- Lưới ô v3 quanh xe (`#` = chiếm):
  ```
   y\x  -0.54 ... -0.04 0.01 ...
   1.16 ...........################.......   (tường trái, x -0,02..0,78)
   1.11 .######....#....
   0.86 .#.........#....
   0.81 .#......####....                     (khối: x -0,16..0,03, y 0,79..1,16)
  ```
- **Tên vật:**
  - Kho **không có tọa độ K1**. `mo_phong_he_thong.py:22-26` chỉ có TUONG_NGOAI, H0, K4, K2; không md nào ghi vị trí K1.
  - Vật sát đuôi-trái là **khối nhô ra từ tường trái**, ghi trên hình "CÓ THỂ là K1" (commit 1d893aa / b296baf gọi lần kẹt 21:05 là "kẹt K1"). **Chưa khẳng định**; Thiên xác nhận theo bố trí xưởng.
  - Các hộp H0, K2, K4 trên hình lấy từ mo_phong_he_thong.py (xấp xỉ ±5-10 cm; trên v3 H0 lệch khoảng +12 cm theo x).
- Scan nằm lệch v3 khoảng 5 cm (tường trái theo scan ở y ≈ 1,09, v3 ở 1,14-1,19). Đây là sai số định vị/bản đồ, không đổi kết luận về phía của vật.

## 4. B4: dựng lại kiểm va chạm của RPP lúc 21:04:44 bằng tính tay (CHỈ ĐỌC, không chạy Nav2)

Script: scratchpad `r120/b4.py` (không vào kho). Hình: nửa dưới RA_SOAT_120.png (ghép dưới hình B3, để git status vẫn chỉ có 2 tệp).

### 4.1 Tham số (xe_scripts/nav2_params_cho.yaml, commit cuối a6cc2df; **bản trên Pi chưa đối chiếu**)
| tham số | tệp:dòng | giá trị |
|---|---|---|
| footprint (local) | nav2_params_cho.yaml:140 (global :173) | `[[0.145, 0.08], [0.145, -0.08], [-0.105, -0.08], [-0.105, 0.08]]` |
| footprint_padding | **không có** trong nav2_params_cho.yaml lẫn dev_ws/src/my_bot/config/nav2_params.yaml | mặc định costmap_2d Humble **0,01**, nên footprint thực dùng −0,115..0,155, ±0,09 |
| resolution (local) | :137 | 0,05 |
| local costmap | :132 global_frame odom; :134 rolling_window; :135-136 3 × 3 m | |
| **các lớp local costmap** | **:142 `["obstacle_layer", "inflation_layer"]`** | **KHÔNG có static_layer**. obstacle: `/scan`, :153 obstacle_max_range 2,5, :154 raytrace 3,0; inflation :157-158 (3,0; 0,20) |
| use_collision_detection | :123 | true |
| max_allowed_time_to_collision_up_to_carrot | :124 | 1,0 s |
| controller_frequency | :82 | 10 Hz (dt 0,1 s) |
| failure_tolerance | :86 | 0,3 s |
| RPP | :108 desired_linear_vel 0,15; :110 lookahead_time 2,0; :111-112 lookahead 0,18-0,6; :114 use_rotate_to_heading true; :115 rotate_to_heading_angular_vel 1,0; :116 rotate_to_heading_min_angle **0,35**; :117 max_angular_accel 3,2 | |

### 4.2 Mô hình RPP 1.1.20 dùng để tính tay
- `computeVelocityCommands`: nếu góc tới carrot > 0,35 rad thì `rotateToHeading` (v = 0, ω = ±1,0, kẹp theo gia tốc: ω_hiện ± 3,2 × 0,1). Ngược lại đi cung. Sau đó gọi `isCollisionImminent(v, ω)`; trả true thì ném "detected collision ahead!".
- `isCollisionImminent`:
  - kiểm tư thế hiện tại;
  - rồi chiếu tới với bước `projection_time` khi i·pt < 1,0 s (dừng nếu vượt quá carrot):
    - **đi cung:** pt = res / |v| = 0,333 s, tức kiểm thêm ở +5 và +10 cm;
    - **xoay tại chỗ:** pt = 2·sin(0,025 / R_ngoại) / |ω|, với R_ngoại = 0,179 (có padding). Mỗi bước là **15,9°**.
      - Lượt 1 (ω_hiện −0,245 nên ω = −0,565): kiểm −15,9° và −31,9°.
      - Lượt 2-3 (đứng yên nên ω = −0,32): kiểm −15,9°.
- `inCollision`: `footprintCostAtPose`, tức max cost trên **các cạnh** footprint (Bresenham ô → ô); va chạm khi cost ≥ 254.
- **Góc tới carrot:** đường tuyến 2 dựng lại bằng tinh_tuyen từ (0,16; 0,76) chạy thẳng +x dọc y 0,76 (31 điểm; Pi ghi 35 điểm, chênh vì Pi dùng costmap sống). Xe hướng 42°, nên góc tới carrot là **−42° > 20°**, và RPP **xoay phải tại chỗ** chứ không đi cung.
  - Điều này khớp lượt 2, 3: bị hủy ngay khi đứng yên.
  - Lượt 1 lúc 21:04:44.15: xe vừa tới đường y ≈ 0,76 với hướng 42°, cũng quá ngưỡng.

### 4.3 Ba nguồn ô 254, tư thế (0,16; 0,76; 42°)
- Lệch scan–v3 (dò tịnh tiến ±12 cm, bước 1 cm): scan cần dời **(+5; +7) cm** để sát ô v3. Trung vị khoảng cách tới tâm ô giảm 5,4 → 1,7 cm; tỉ lệ điểm nằm trong ô tăng 21 % → 94 %. Vậy (c) = v3 dời (−5; −7) cm.

| nguồn | số ô 254 | hiện tại | RPP đi cung (+5, +10 cm) | đè từ (trên cung R 0,6) | RPP xoay phải (−15,9°; −31,9°) | đè từ (xoay phải / trái) |
|---|---|---|---|---|---|---|
| (a) ô chiếm v3 (lớp tĩnh), lưới bản đồ | 252 | không | không | 46,0 cm | **ĐÈ 1 ô ; ĐÈ 1 ô** (cùng ô (0,009; 0,81), nằm trên 2 cạnh) | **14,0°** / 63,0° |
| (b) ô do scan 21:06:30 đánh dấu, lưới bản đồ | 109 | không | không | 41,5 cm | không ; không | > 90° / > 90° |
| (b) 4 pha lưới khác (dời 2,5 / 3,75 cm) | 105-108 | không | không | 37,5-40,5 cm | không | > 90° / 75,5-> 90° |
| (c) v3 dời (−5; −7) cm | 252 | không | không | 37,0 cm | không ; không | > 90° / > 90° |

- (a): ô đè là góc khối nhô từ tường trái (ô tâm ≈ (0,01; 0,81), cách tâm xe 0,16 m < R_ngoại 0,179). Khi xoay phải, **góc đuôi-trái quét vào khối này**.
- (b) hụt sát: chỉ cần padding ≥ **0,04** (thay vì 0,01) là (b) đè ngay ở tư thế hiện tại, và đè cả khi xoay lẫn khi đi cung. (a) với padding 0 vẫn đè khi xoay −15,9°.
- B3 nói đuôi rời xa vật khi đi tiếp cung; điều đó vẫn đúng. Nhưng RPP **không đi cung** ở đây mà **xoay phải tại chỗ**, và khi xoay thì đuôi-trái quét về phía khối.

### 4.4 Kết luận
**Tính tay chỉ nguồn (a) (ô tĩnh v3 ở tư thế tin) làm RPP từ chối, ngay ở bước xoay phải tại chỗ đầu tiên (−15,9°; đè từ 14°). Nhưng theo nav2_params_cho.yaml:142 trong kho, local costmap không có lớp tĩnh. Nguồn (b) (scan, thứ local costmap thật sự có) hụt khoảng 3 cm, (c) không đè. Vậy chưa khép được: cần đối chiếu nav2_params_cho.yaml trên Pi (so_voi_pi.sh) và chụp `/local_costmap/costmap` + `/local_costmap/published_footprint` lúc kẹt.**

Giả định ghi rõ:
- padding mặc định 0,01;
- pha lưới local costmap (khung odom) không biết; đã thử 5 pha;
- v, ω lấy từ vết; scan lúc 21:06:30 (xe đứng yên từ 21:04:44.7);
- vết ô cũ (đánh dấu khi odom trôi lúc xe còn chạy) **không dựng lại được** từ một scan. Đây là ứng viên thứ tư cho phần hụt 3 cm của (b).

## 5. B5: tám động tác từ tư thế lúc kẹt [tính] (CHỈ ĐỌC, tính tay; script scratchpad `r120/b5.py`)

Xe đứng yên tại (0,16; 0,76; 42°). Footprint có padding 0,01 (−0,115..0,155; ±0,09).
- Cách đo: khoảng cách nhỏ nhất **có dấu** (âm = đè, bằng độ sâu đè) từ footprint tới (b) 387 điểm scan 21:06:30 và (a) ô chiếm v3 (mỗi ô lấy mẫu lưới 0,5 cm cả cạnh).
- Xoay quanh base_footprint; lùi/tiến theo hướng 42°.
- Hình: hàng cuối RA_SOAT_120.png (8 ô, 1 cm = 4 px).

| động tác | (b) scan (cm) | đổi | (a) v3 (cm) | đổi | rời xa cả (a) và (b)? |
|---|---|---|---|---|---|
| hiện tại | +5,1 | | +1,3 | | |
| lùi 5 cm | +5,0 | −0,1 | +1,3 | 0,0 | không |
| lùi 10 cm | +5,0 | −0,1 | +1,3 | 0,0 | không |
| tiến 5 cm | +7,8 | +2,7 | +1,8 | +0,5 | **có** |
| tiến 10 cm | +9,3 | +4,2 | +5,5 | +4,2 | **có** |
| xoay trái 15° | +7,7 | +2,7 | +2,9 | +1,6 | **có** |
| xoay trái 30° | +9,3 | +4,3 | +3,8 | +2,5 | **có** |
| xoay phải 15° | +4,4 | −0,7 | **−1,1** | −2,3 | không (đè (a)) |
| xoay phải 30° | +5,9 | +0,9 | **−0,4** | −1,7 | không (đè (a)) |

- Điểm gần nhất:
  - (a) luôn là cạnh trái footprint với góc dưới-phải (0,034; 0,785) của ô (0,009; 0,81), tức góc khối nhô từ tường trái;
  - (b) là điểm scan (−0,026; 0,780). Riêng tiến 10 cm thì điểm gần nhất chuyển sang tường trái phía trước (0,277; 1,090), còn 9,3 cm.
- **Lùi không giúp:** xe lùi song song cạnh trái của chính nó, nên khoảng cách vuông góc từ cạnh trái tới góc khối không đổi (1,3 cm); điểm scan còn gần hơn 0,1 cm.
- **Xoay phải** (đúng động tác RPP chọn, mục 4) đè ô v3 1,1 cm ở 15°. Phù hợp kết luận B4 (a).
- Kiểm (b) với B4: B4 nói (b) chỉ đè khi padding ≥ 0,04. Ở đây (b) cách footprint padding 0,01 là 5,1 cm (đo liên tục, không lượng tử ô). Hai số khác nhau vì B4 tính theo ô 5 cm (ô chứa điểm có thể gần hơn điểm tới 3,5 cm).

**Kết luận B5:** động tác rời xa vật ở **cả (a) lẫn (b)** là **tiến 5 / 10 cm** và **xoay trái 15 / 30°**. Lùi thẳng không đổi; xoay phải làm gần hơn và đè (a).

## 6. Việc cần làm trên xe (CHƯA làm; chờ Thiên)
1. So `nav2_params_cho.yaml` trên Pi với kho bằng `so_voi_pi.sh`, để biết local costmap thật có lớp tĩnh, `footprint_padding`, footprint khác kho hay không (mục 4.1 giả định theo kho).
2. Lúc kẹt (ngay khi RPP báo "detected collision ahead!"), chụp:
   - `ros2 topic echo --once --full-length /local_costmap/costmap > costmap_local_<gio>.txt`;
   - `ros2 topic echo --once --full-length /local_costmap/published_footprint > footprint_<gio>.txt`;
   - kèm tf odom → base_footprint cùng lúc,
   để so trực tiếp ô 254 thật với ba nguồn (a)/(b)/(c) ở mục 4.3.

## 7. B6: thoát xong RPP có xoay về tuyến được không [tính] (CHỈ ĐỌC, tính tay; script scratchpad `r120/b6.py`)

Giả định:
- Tư thế lúc kẹt (0,16; 0,76; 42°); footprint padding 0,01.
- Sau thoát, RPP xoay phải từng bước 15,9° (bước chiếu của `isCollisionImminent`, mục 4.2) tới khi hướng lệch tuyến 2 (+x, mục 4.2) ≤ 0,35 rad. Chỉ xét hướng, không tính carrot theo vị trí mới.
- (a) ô chiếm v3, (b) điểm scan 21:06:30. Số là khoảng cách nhỏ nhất có dấu (cm, âm = đè).
- "quét" = nhỏ nhất khi xoay liên tục giữa hai bước (8 bước con).
- Thêm cột kiểm theo ô 5 cm (cách RPP kiểm thật, B4) ở mỗi tư thế.
- Hình: hàng cuối RA_SOAT_120.png.

| phương án | bước | hướng | (b) | (a) | quét (b) / (a) | ô 5 cm (a)/(b) |
|---|---|---|---|---|---|---|
| tiến 5 cm | thoát xong | 42° | +7,8 | +1,8 | | −/− |
| | xoay 1 | 26° | +8,5 | +2,0 | +7,8 / +1,7 | −/− |
| | xoay 2 | 10° | +10,1 | +3,4 | +8,6 / +2,1 | −/− |
| **tiến 10 cm** | thoát xong | 42° | +9,3 | +5,5 | | −/− |
| | xoay 1 | 26° | +11,6 | +5,8 | +9,6 / +5,4 | −/− |
| | xoay 2 | 10° | +13,9 | +7,1 | +12,0 / +5,9 | −/− |
| tiến 15 cm | thoát xong | 42° | +6,1 | +9,2 | | −/− |
| | xoay 1 | 26° | +8,3 | +9,5 | +6,3 / +9,1 | −/− |
| | xoay 2 | 10° | +11,8 | +10,8 | +8,6 / +9,6 | −/− |
| xoay trái 15° | thoát xong | 57° | +7,7 | +2,9 | | −/− |
| | xoay 1 | 41° | +4,9 | +1,2 | +5,3 / +1,4 | −/− |
| | xoay 2 | 25° | +4,6 | **−1,2** | +4,2 / **−1,0** | **ĐÈ**/− |
| | xoay 3 | 9° | +6,3 | **−0,1** | +4,8 / **−1,1** | **ĐÈ**/− |
| xoay trái 30° | thoát xong | 72° | +9,3 | +3,8 | | −/− |
| | xoay 1 | 56° | +7,6 | +2,8 | +7,9 / +3,0 | −/− |
| | xoay 2 | 40° | +4,8 | +1,0 | +5,1 / +1,3 | −/− |
| | xoay 3 | 24° | +4,7 | **−1,1** | +4,1 / **−1,2** | **ĐÈ**/− |
| | xoay 4 | 8° | +6,5 | **−0,0** | +4,8 / **−1,0** | **ĐÈ**/− |

- **Xoay trái rồi xoay phải** chỉ quay về chính chỗ cũ (xoay quanh cùng một tâm), nên qua hướng 25° vẫn đè (a) như B5 (xoay phải từ chỗ kẹt). Xoay trái **không** giải quyết được.
- **Tiến** dời tâm xoay xa góc khối, nên xoay phải không đè. Biên nhỏ nhất suốt quá trình:
  - tiến 5 cm: (a) **1,7 cm** (sát), (b) 7,8;
  - tiến 10 cm: (a) **5,4**, (b) **9,3**, rộng nhất cả hai;
  - tiến 15 cm: (a) 9,1, (b) **6,1** (đầu xe tới gần tường trái phía trước).
- (3) **Khoảng trống phía trước** lúc kẹt, theo LiDAR, dải |y| ≤ 0,10 m: điểm gần nhất ở thân (0,389; 0,098), **mũi → điểm 24,4 cm** (29 điểm trong dải). Điểm này nằm ở mép dải phía trái, thuộc tường trái phía trước. Tiến 10 cm còn 14,4 cm; tiến 15 cm còn 9,4 cm.

**Kết luận B6:** **tiến thẳng 10 cm rồi để RPP xoay phải** (2 bước) là phương án xoay xong mà không đè cả (a) lẫn (b), biên nhỏ nhất 5,4 cm (a) / 9,3 cm (b). Tiến 5 cm cũng không đè nhưng (a) chỉ còn 1,7 cm. Xoay trái 15/30° rồi xoay phải thì đè (a) khoảng 1 cm.

## 8. Việc 120: nhánh RA_TIEN của lớp 2 (Thiên duyệt 00:39). CHƯA commit, chưa push, CHƯA CHẠY TRÊN PI

### 8.1 B1: sửa và test
**sep.py**
- `P_MAC_DINH` thêm `lop2_sau=True`. Đây là tham số ROS của nút sep (khai báo qua vòng `P_MAC_DINH`); `false` thì tắt riêng nhánh sau.
- `_lop2`, lúc nhận SAP_GUI mới (thứ tự kiểm cũ giữ nguyên: `ra_lenh=false`, `mat_scan`):

  | vật trước (`L.vung_lui`, như cũ) | vật sau (`L.vung_sau`, chỉ khi `lop2_sau`) | trả lời | lý do |
  |---|---|---|---|
  | có | có | KHONG_CAN | `vat_truoc_va_sau` |
  | có | không | RA_LUI | `vat_cach_than_<m>` (như cũ) |
  | không | có | **RA_TIEN** | `vat_sau_than_<m>` (than → điểm sau gần nhất) |
  | không | không | KHONG_CAN | `vung_trong` |

- Vùng sau: ≥ 2 điểm cách thân ≤ 0,105 m với x ≤ −0,05.
- RA_TIEN được giữ và đóng như RA_LUI: thấy RA_XONG thì ghi `LOP2 RA_XONG DONG`; quá 12 s thì ghi `LOP2 HET_GIO`. Cột "se_lam" ghi `phu lai tien thang`.
- `dong_ra2` ghi `RA_TIEN <ma_luot> <ma>` vào ~/sep_thoat. Sếp vẫn không phát tốc độ.

**lai_xe_lidar.py**
- Hằng `LOP2_X_SAU = -0.05`.
- DEFAULT thêm `ra_tien_di_max 0,10`, `ra_tien_truoc 0,15`, `ra_tien_w 0,10`, `ra_tien_t_max 3,0`.
- Hàm thuần mới: `vung_sau`, `cach_sau`, `mui_truoc` (mũi → điểm gần nhất, x ≥ 0,145, |y| ≤ 0,10).
- `ra2_bat_dau(..., tien=True)` → log `RA2 BAT DAU TIEN (mui-truoc .. m, than-sau .. m)`; chạy trong cùng trạng thái `RA2`.
- `_ra_tien`:
  - tiến thẳng `v_esc` = 0,10 m/s, tên đầu ra `RA_TIEN` (tên cũ);
  - dừng theo thứ tự: `CHI_NHIN`, `HUY`, `TRUOC_15CM`, `DA_TIEN_10CM`, `QUA_3S`;
  - báo `RA XONG <lý do> <cm> cm` rồi `RA_TIEN -> RA_XONG`. HUY thì phát một lệnh 0 (dùng chung `_ra2_xong` với RA_LUI).
- `doc_sep_ra2` nhận `RA_TIEN`; `doc_sep_thoat` vẫn bỏ qua. Node `_doc_ra2` giao RA_TIEN như RA_LUI. `_doc_huy` dùng được cho cả hai vì chung trạng thái `RA2`.

**gui_duong.py** (bắt buộc sửa): `tra_lop2` đọc qua `L.doc_sep_ra2`. Nếu không sửa, RA_TIEN sẽ bị bỏ qua, gui_duong in `RA_HET_GIO` sau 2 s và **gửi đường trong lúc phụ lái đang tiến**.
- `cho_lop2` chờ RA_XONG cho cả RA_TIEN.
- Thêm câu log `   lop 2: sep bao RA_TIEN: cho phu lai tien ra, toi da 12 s.`; câu RA_LUI giữ nguyên.

**Test:** test_lai_xe_lidar **97/97** (cũ 86 + 11), test_sep **46/46** (+ test_46), test_gui_duong **30/30** (+ test_30). Các bài mới:
- scan K1 21:06: `vung_sau` có, `vung_lui` không; thân → sau 6,3 cm; mũi → trước 24,4 cm.
- RA_TIEN trên scan K1 (mỗi bước dời điểm −d):
  - nhịp 0,05 s: dừng `TRUOC_15CM` sau **9,5 cm**;
  - nhịp 0,1 s như trên xe: dừng sau **10,0 cm**, báo `RA XONG TRUOC_15CM 10 cm`, mũi còn 14,4 cm. Ngưỡng 9,4 cm rơi giữa hai nhịp 1 cm.
- Phía trước trống: `DA_TIEN_10CM` (xe đi 11 cm do cộng dồn số thực, như RA_LUI 30 cm; dung sai 9-11,5 cm). `QUA_3S`; `CHI_NHIN 0 cm`; HUY phát 0; mũi sẵn ≤ 0,15 thì xong ngay 0 cm.
- sep:
  - K1 → `RA_TIEN vat_sau_than_0.06m`, đóng khi RA_XONG, `HET_GIO` sau 12 s;
  - `lop2_sau=false` → `vung_trong`;
  - K1 + vật 5,5 cm trước mũi → `vat_truoc_va_sau`; cũng cảnh đó mà `lop2_sau=false` → RA_LUI;
  - `ra_lenh=false` → KHONG_CAN.
- gui_duong: RA_TIEN → gửi sau RA_XONG (1,5 s), không RA_HET_GIO. Luật G sep → phụ lái / gui_duong khớp.

### 8.2 B2: kiểm (CHƯA commit)

**(1) Đối chiếu tệp (mục D(1))**

| chữ / tệp / tên | bên ghi | bên đọc | phải khớp | khớp? |
|---|---|---|---|---|
| `RA_TIEN <ma_luot> <ma>` trong ~/sep_thoat | sep.py `dong_ra2` + `ghi_lenh` | lai_xe_lidar `doc_sep_ra2` → `_doc_ra2`; gui_duong `tra_lop2` (= `L.doc_sep_ra2`) | chữ, số cột, kiểu int | khớp (test_30 luật G; giả lập sau_1/2) |
| cùng tệp ~/sep_thoat | sep.py | lai_xe_lidar `doc_sep_thoat`, gui_duong `_thoat_moi` / `sep_dang_giao_thoat` | RA_TIEN phải bị **bỏ qua** | khớp: `o[1] != 'THOAT'` thì None (test 120 doc_sep_thoat) |
| ~/gui_duong_luot (`SAP_GUI` / `HUY`) | gui_duong (không đổi) | sep `doc_sap_gui`; phụ lái `luot_huy` → `_doc_huy` (dùng cho mọi `RA2`, cả tiến) | không đổi | khớp (giả lập ngat_1: HUY dừng RA_TIEN) |
| tên trạng thái `RA_TIEN` (/rosout `KHONG_CO_DICH -> RA_TIEN`, `RA_TIEN -> RA_XONG`) | lai_xe_lidar `_ra_tien` (tên cũ của pha RA) | sep `doc_rosout` (`_RE_L1_DOI`), `DANG_LAM`; gui_duong `PHU_LAI_BAN`, `_on_rosout` | tên có sẵn | khớp; sep dòng 921 chỉ đếm lượt thoát khi `thoat_ma` mở (giữa hai lượt không mở) |
| `RA_XONG` | lai_xe_lidar `_ra2_xong` → `_ket_thuc(..'RA_XONG'..)` | sep `t_ra_xong` (đóng `LOP2 RA_XONG DONG` cho RA_TIEN); gui_duong `n_ra_xong` (mọi chuyển sang RA_XONG) | tên | khớp (test_46, test_30; giả lập: `LOP2,RA_XONG,...,DONG`, gửi sau RA_XONG 0,07-0,10 s) |
| câu `RA2 BAT DAU TIEN (...)`, `RA XONG TRUOC_15CM / DA_TIEN_10CM / QUA_3S <cm> cm` | lai_xe_lidar | không ai so khớp (`doc_rosout` chỉ bắt câu đổi trạng thái / định kỳ) | — | khớp |
| sep_nhat_ky.csv dòng `LOP2` | sep `_lop2` `ghi('LOP2', chu, nguon, phep_kiem, ly, se_lam)` | cau_noi_xe `doc_sep` (chỉ TU_THE / LENH); người đọc | cột `COT_NHAT_KY` (11 cột) không đổi; giá trị mới `RA_TIEN`, `vat_sau_than_<m>`, `vat_truoc_va_sau`, `phu lai tien thang` | khớp (giả lập: dòng đủ cột) |
| ~/sep_lenh | sep (không đổi) | chuyen.sh | không đổi | khớp |
| ~/ket_qua_chuyen.csv, cột `COT_CSV` | gui_duong (không thêm dòng/chữ) | sep `_csv`, chuyen.sh, cau_noi | không đổi | khớp |
| câu gui_duong `   lop 2: sep bao RA_TIEN: cho phu lai tien ra, toi da 12 s.` | gui_duong | cau_noi `DONG_CHUYEN` `^(DEN_NOI\|HET_DUONG\|...)` | không được khớp nhầm | khớp (thụt lề) |
| mã thoát gui_duong 0/1/2/3/4/130 | gui_duong (không đổi) | cau_noi `MA_CHUYEN`, chuyen.sh | không đổi | khớp |
| tham số `lop2_sau` (sep, qua `P_MAC_DINH`, bool) | sep.py | `Sep.p` (vòng `for k in P_MAC_DINH`) | tên | khớp; bat_xe.sh / bat_gia.sh không truyền ⇒ mặc định true |
| tham số `ra_tien_di_max`, `ra_tien_truoc`, `ra_tien_w`, `ra_tien_t_max` | lai_xe_lidar `DEFAULT` (khai báo ROS qua vòng DEFAULT) | `NguoiLai.p` | tên | khớp; bat_xe.sh / bat_gia.sh chỉ truyền `phat_lenh`, `sep_ra_thoat` |
| kênh / action / frame | — | `/cmd_vel_avoid` (RA_TIEN phát v > 0, w = 0; giả lập: chỉ 0,1), `/rosout`, `/follow_path` | không đổi | khớp |
| tên tiến trình bat_xe.sh kiểm (`TAY`, `TAY_SEP`, `TAY_LAI`) | — | — | không đổi | khớp |
| đường dẫn trên Pi | so_voi_pi.sh dòng 31-33: `sep.py\|~/sep.py`, `lai_xe_lidar.py\|~/lai_xe_lidar.py`, `gui_duong.py\|~/gui_duong.py` | — | **chép cả ba cùng lúc** | khớp. sep.py gọi `L.vung_sau`, `L.cach_sau`: chỉ chép sep.py mà không chép lai_xe_lidar.py thì sep **chết (AttributeError)** ở SAP_GUI đầu tiên |

Không có dòng "không khớp".

**(2) Quét kích nhầm** (script scratchpad `r120/quet120.py`; quyết định như `sep._lop2`, ra_lenh = true, `lop2_sau` = true; "cũ" = 35b6c1e)
- du_lieu_0511/scan_ket_k2_1933.txt: 128 tia (121 hop le) [BI CAT: chi -180..-71 do] | moi: KHONG_CAN vung_trong | cu: KHONG_CAN vung_trong | than-sau 0.351, than-truoc 0.277, mui-truoc 9.990
- du_lieu_0511_toi/scan_ket_5b2.txt: 420 tia (389 hop le) | moi: KHONG_CAN vung_trong | cu: KHONG_CAN vung_trong | than-sau 0.203, than-truoc 0.178, mui-truoc 0.234
- du_lieu_0511_toi/scan_ket_5b3.txt: 420 tia (390 hop le) | moi: RA_TIEN | cu: KHONG_CAN vung_trong | than-sau 0.064, than-truoc 0.171, mui-truoc 0.244
- du_lieu_0511_toi/scan_ket_k1_2106.txt: 420 tia (387 hop le) | moi: RA_TIEN | cu: KHONG_CAN vung_trong | than-sau 0.063, than-truoc 0.171, mui-truoc 0.244

- Scan thật: RA_TIEN kích **2/4**, cả hai là cảnh kẹt K1 (5b3 lúc 21:05:07 và k1 lúc 21:06:30, cùng chỗ đứng). Scan K2 19:33 bị cắt (128 tia, chỉ −180..−71°), nên chỉ kết luận được cho phần có dữ liệu.
- Hành lang dựng tay (420 tia): rộng 45 / 52 / 56 cm × lệch về tường trái 0-9 cm (bước 1) × góc xe −10/−5/0/+5/+10° × (không vật / hộp 8 × 6 cm cách hông 4 cm ở sườn sau trái / phải). Mỗi nhóm 150 ca.

  | nhóm | RA_TIEN | RA_LUI | KHONG_CAN vat_truoc_va_sau | KHONG_CAN vung_trong |
  |---|---|---|---|---|
  | không vật | **14** | 14 | 28 | 94 |
  | vật sườn sau trái | 108 | 0 | 42 | 0 |
  | vật sườn sau phải | 108 | 0 | 42 | 0 |

- **14 lần RA_TIEN không có vật lạ** (tường hành lang):
  - chỉ khi xe nghiêng −5/−10° (mũi quay ra xa tường trái) và lệch về tường trái: 45 cm lệch 3-6 cm, 52 cm lệch 6-9 cm, 56 cm lệch 8-9 cm;
  - lúc đó góc đuôi-trái cách tường ≤ 10,5 cm còn nửa trước thì không;
  - RA_TIEN đi thẳng theo hướng mũi, tức ra xa tường; vẫn bị chặn bởi `TRUOC_15CM`.
- **Đổi so với cũ:** 28 ca hành lang trống trước ra RA_LUI (tường sát cả dọc hông) nay ra `KHONG_CAN vat_truoc_va_sau` (đúng thiết kế: trước lẫn sau đều có điểm). 14 ca RA_LUI còn lại là khi mũi nghiêng vào tường.
- Có vật sườn sau: 216/300 ra RA_TIEN. 84/300 ra `vat_truoc_va_sau` khi tường cũng sát nửa trước (hành lang hẹp, lệch lớn).
- Bảng chữ từng ca: `r120/quet120.txt`.

**(3) Giả lập** — **giả lập chưa hiệu chuẩn**, chỉ dùng bắt lỗi ghép tệp, không dùng để quyết.
- bat_gia mức B, HOME giả mỗi lần, domain 87. Script dựng lại ở scratchpad `r120/sim/` (vong.sh, theo_lop2.py; bản 05/11 đã mất khi /tmp bị dọn).
- Ống r 0,09 tâm (−0,225; 0), tức mặt ống cách đuôi xe 3 cm lúc xuất phát ở A.

| lần | chuỗi (mốc từ SAP_GUI) | kết quả |
|---|---|---|
| sau_1 | sep_thoat `RA_TIEN` 0,02 s → `RA2 BAT DAU TIEN (mui-truoc 1.49 m, than-sau 0.03 m)` 0,10 s → `RA XONG DA_TIEN_10CM 11 cm` 1,20 s (odom 11,0 cm, avoid chỉ +0,10) → /follow_path 1,30 s | DEN_NOI t1 lượt 1, mã 0, 31 s |
| sau_2 | 0,03 → 0,03 → `DA_TIEN_10CM 10 cm` 1,03 s (odom 10,0) → gửi 1,10 s | DEN_NOI t1 lượt 1, mã 0, 32 s |
| ngat_1 | RA_TIEN, SIGINT gui_duong 0,32 s sau RA2 → `RA XONG HUY 4 cm`; SIGINT → /cmd_vel_avoid 0 **0,080 s**, 0 tin khác 0 sau đó (≥ 10 s) | NGAT, mã 130; sep `LOP2 RA_XONG DONG` |
| q1_1 (di_B + ve_A, không ống) | sep `KHONG_CAN vung_trong` cả hai; 0 lần RA2 | DEN_NOI t1 lượt 1 ×2, mã 0, 38 / 31 s |

- Không có RA_HET_GIO và không có lần gửi đường nào trong lúc RA2.
- gui_duong in đúng `lop 2: sep bao RA_TIEN ...` và `lop 2: RA_XONG sau 1.3 / 1.1 s.`
- Sau mỗi lần, `bat_xe.sh tat` báo `KIEM_XONG: da tat sach`.

**py_compile** `sep.py lai_xe_lidar.py gui_duong.py`: đạt.

**md5** (cây làm việc; HEAD b296baf):
- sep.py `4fa9ffd7c8cbe8aa98eb9a0b1c082b78`
- lai_xe_lidar.py `0328ef9dde29132acb367f6ee1857779`
- gui_duong.py `ff496a60db58b90239bee4203e7fbbde` (HEAD: 6c937df6922ed236ca46f41f4b762877)
- test_sep.py `697f9656cf277ae75d042c6b0e308964`, test_lai_xe_lidar.py `76496979dad68934f6aaf837cb12d4dd`, test_gui_duong.py `b936ad38c4a613caae8b5bccdfb9fc3c`

Test: 97/97, 46/46, 30/30. **CHƯA CHẠY TRÊN PI.**

### 8.3 Kiểm trước commit (Thiên quyết: chấp nhận 14 ca RA_TIEN trong hành lang trống, không thêm điều kiện bản đồ)
Quyết định lớp 2 (ra_lenh = true, scan mới): bản 35b6c1e chạy đúng `lai_xe_lidar.py` của commit đó (`git show`) với luật sep 35b6c1e `vung_lui ⇒ RA_LUI, không ⇒ KHONG_CAN vung_trong`; bản mới như `sep._lop2` hiện tại.

| scan | 35b6c1e | bản mới |
|---|---|---|
| scan_ket_5b2.txt (420 tia) | KHONG_CAN vung_trong | KHONG_CAN vung_trong |
| scan_ket_5b3.txt (420 tia) | KHONG_CAN vung_trong | **RA_TIEN** |
| scan_ket_k2_1933.txt (128 tia, bị cắt: chỉ −180..−71°) | KHONG_CAN vung_trong | KHONG_CAN vung_trong |

Không scan nào trước ra RA_LUI mà nay ra `KHONG_CAN vat_truoc_va_sau`, nên commit theo lệnh.
