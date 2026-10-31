# Báo cáo sự cố 31/10/2026 chiều (ở trường): Nav2 / TF không ổn trên Pi sau khi dùng bộ tệp thử 31/10 (bản 2, đã đính chính 16:58)

**ĐÍNH CHÍNH so với bản 1 (đọc trước):** (1) `bat_xe.sh` và `kiem_xe.sh` CHƯA TỪNG chạy trên Pi trước 31/10 (`Trang thai`: việc 82, 88 chưa làm). Thứ đã chạy 28–30/10 là các lệnh thủ công của `Quy trinh chuan 71` (bước 5 launch, bước 7 Nav2, `ros2 action send_goal`). Vì vậy `bat_xe.sh` là biến số MỚI, đáng nghi ngang giả thuyết A. (2) PID 2532 là suy luận từ số PID, không phải log trực tiếp. (3) Quy tắc mới (Thiên chốt 31/10): thứ làm tăng độ ổn định của kết quả đồ án được ưu tiên hàng đầu; viết mã, mô phỏng, đánh giá mã và đồng bộ mã thành một khối là việc của Claude Code, mỗi lần đổi MỘT thứ.

**Cách dùng gói:** Claude Code mở ở thư mục repo (nhánh `pi-30-10`); gói này nằm ở `./incident_31_10` (KHÔNG `git add`).

Người viết: Claude (phiên chat trên claude.ai, máy chủ đám mây KHÔNG có ROS thật, KHÔNG thấy Pi). Mọi tệp bên dưới được viết ngoài đường và ở nhà trong ngày 30–31/10, chỉ thử bằng `ros2` giả và mô phỏng đồ chơi. Người chạy trên Pi thật: Thiên. Nhãn: [log] = kết quả Thiên dán, [suy luận] = chưa kiểm, [ước] = số đoán.

## 1. Việc nhờ (một việc)

Rà soát ĐỘ ĐỒNG BỘ giữa các tệp dưới đây và với `Pi`/repo (nhánh `pi-30-10`), rồi tìm vì sao hệ thống lên không ổn (mục 4). Chỉ ĐỌC và kiểm trước; mỗi thay đổi đề xuất phải là MỘT thứ, có đường lui. Hạn: 3/11 chấm trước thầy khoa, 10/11 chấm thật; tiêu chí duy nhất: "chạy ổn định" (ổn định trước, chính xác sau).

## 2. Các tệp và vai trò

| Tệp | Việc | Viết khi nào, thử ở đâu |
|---|---|---|
| `bat_xe.sh` (v3) | Bật cả hệ thống: `mapping_my_bot.launch.py` (chờ dòng `보정 완료`), rồi `navigation_my_bot.launch.py` với `NAV_PARAMS` (chờ `Managed nodes are active`), rồi gọi `kiem_xe.sh 0 0 0`; có nhánh `tat` (SIGINT, rồi `pkill`, rồi KILL -9) | 30/10, thử với `ros2`/`pkill` giả. Trên Pi: lần đầu 31/10 |
| `kiem_xe.sh` (v3) | Đọc `ros2 param get`, `tf2_echo map base_link`, đếm tiến trình, tuổi costmap | như trên |
| `chuyen.sh` (v4) | Gửi mục tiêu `/navigate_to_pose` theo bảng tuyến (`BANG=cu|tt`); đọc vị trí bằng `tf2_echo map base_footprint`, `compute_path_to_pose`, `published_footprint` | như trên |
| `ghi_bag.sh` (v3) | `ros2 bag record` nền | như trên |
| `bt_cho.xml`, `nav2_params_cho.yaml` | Cây hành vi chờ; tham số Nav2 | 30/10; chưa nạp thử vào Nav2 thật trước 31/10 |
| `cau_noi_xe.py`, `cai_cau_noi.sh` | Dịch vụ systemd trên Pi (cổng 5001): nút kết nối gọi `bat_xe.sh tat` rồi `bat_xe.sh`; đọc `/proc` để báo trạng thái; phục vụ trang `chon_b_v4.html` và bản đồ | 31/10 ngoài đường, thử với `bat_xe.sh` giả |
| `lai_xe_lidar.py` | Bộ lái LiDAR (mặc định chỉ nhìn). CHƯA bật ở lần thử nào | 31/10, thử với ROS giả |
| `THIET_KE_LAI_XE_LIDAR.md`, `HUONG_DAN_THU_31_10.md`, `HUONG_DAN_CAU_NOI.md` | Thiết kế và hướng dẫn | |

ĐÍNH CHÍNH: `bat_xe.sh` và `kiem_xe.sh` chưa từng chạy trên Pi trước 31/10. Cái đã chạy 28–30/10 (A→B 11/11 ngày 29/10) là các lệnh thủ công của `Quy trinh chuan 71` (xem `Lich su giai doan` 54–60 trong Project).

## 3. Mốc thời gian 31/10 (giờ Pi, KST)

- 15:45 cài cầu nối; 15:46 thêm quyền `sudo` không mật khẩu cho `timedatectl` và `systemctl stop apt-daily*`. `Pi` đang ngày 29/7 (không có pin giữ giờ).
- 15:46:27 bấm nút kết nối lần đầu: cầu nối chỉnh giờ Pi sang 31/10 (`timedatectl set-ntp false; set-time`: nhảy +94 ngày). Lệnh bật bị dừng giữa chừng do lỗi đồng hồ trong `cau_noi_xe.py` (đã sửa bằng `time.monotonic()`), chưa bật gì.
- **Lần chạy 1** (15:58:55, `shm` 0 tệp): con quay lấy số 0 15:59:09, Nav2 sẵn sàng 15:59:20, `kiem_xe.sh` ĐẠT (x 0,000; y 0,010; góc −0,7°; `bt_xml` đúng; không trùng node), `SAN SANG` lúc ≈ 15:59:50. [log]
- **16:00:57** `lifecycle_manager_navigation`: `CRITICAL FAILURE: SERVER velocity_smoother IS DOWN after not receiving a heartbeat for 4000 ms. Shutting down related nodes.` (97 giây sau `Managed nodes are active`). KHÔNG có dòng `process has died`. Tại 16:07:53 `velocity_smoother` (PID 2725) vẫn sống, các tiến trình Nav2 khác cũng sống (cầu nối cũ chỉ kiểm 9 tên và báo `on` sai). [log]
- 16:01:21–16:25 `ros2 bag record` chạy (`~/bag_1031_160121`, 64 MB, giữ lại; có tf, scan, costmap, plan, behavior_tree_log).
- 16:03 `chuyen.sh di_B`: `khong doc duoc vi tri xe (tf=' ')`. 16:04: `tf2_echo map base_footprint` và `map base_link` đều `Invalid frame ID "map" ... frame does not exist` (trong khi 9 tiến trình đều còn, `throttled=0x0`).
- **Lần chạy 2** (nút `재시작` ≈ 16:12): sống ≥ 145 giây, không `CRITICAL`. 16:17:06 `chuyen.sh di_B` (công tắc motor OFF, nên xe không đi): `KIEM` tf (0,000; 0,010), bộ lập đường 97 điểm, lệch 0,02 m, **tuổi tin costmap toàn cục 2,2 s**; goal hỏng sau ≈ 15 s (progress checker, do xe không nhúc nhích). Tuyến 2: **tuổi tin 21,3 s** (≥ 5 s là dấu hiệu tf cũ). [log]
- **Lần chạy 3** (16:18:21, bằng nút): `tat` phải leo tới KILL -9; `/dev/shm` có **57 tệp** trước khi bật; con quay 16:19:07; **sau 120 s Nav2 không bao giờ in `Managed nodes are active`**. `planner_server`: `Timed out waiting for transform from base_footprint to map ... Lookup would require extrapolation` (lặp mỗi 0,5 s). [log]
- 16:23:50 `top`: load 3,00/4,30/3,32 (4 nhân), CPU 27,7 us 15,7 sy 56,6 id, nhiệt 41,8 °C, `throttled=0x0`. `/dev/shm` 118 tệp trước `tat`, **84 sau `tat`** (`fastrtps_*`, `fastrtps_*_el`). Đang sống: `joint_state_pub` **PID 2532 (suy luận từ số PID: thuộc lần chạy 1, ≈ 85%; đã sống qua hai lần `tat`)**, `ros2` PID 3308 (bộ ghi bag, 28% CPU), hai `python3` ≈ 30% mỗi cái.
- 16:25 dừng bag, xóa `/dev/shm/fastrtps_*` khi không còn tiến trình ROS (0 tệp).
- **Lần chạy 4** (≈ 16:25:56, bằng nút, `shm` 4 tệp trước khi bật): con quay 16:26:15; Nav2 lại không `active` sau 120 s; lỗi `extrapolation into the past`; `shm` 85 tệp sau đó. [log]
- **Lần chạy 5** (16:31:26, bằng `ssh`, thủ công, sau `tat` + xóa `shm`; `shm` 24 tệp xuất hiện NGAY trước khi bật dù không có tiến trình ROS nào do ta bật): con quay 16:31:41; Nav2 `Managed nodes are active` 16:32:05; nhưng `kiem_xe.sh` cả 3 lần: không đọc được vị trí (`tf`), và `ros2 param get` lúc được lúc không (`yaw_nguon`, `dau_yaw`, `gyro_scale`, `bt_xml` mỗi lần hỏng một số khác nhau); `costmap` không đọc được. `KET QUA: KHONG DAT`. [log]

## 4. Quan sát then chốt

1. Hỏng ở cả cách bật bằng nút (systemd) và bằng `ssh`: không riêng cầu nối. [log]
2. Triệu chứng chung: các tiến trình sống nhưng TF `map` không tới / chậm / thiếu, và `ros2 param get` không ổn định (giống lỗi khám phá giữa các tiến trình DDS). [log]
3. Tệp `/dev/shm/fastrtps_*` tăng qua các lần và `tat` không dọn được; đã có tệp xuất hiện khi lẽ ra không có tiến trình ROS nào. [log]
4. Có tiến trình mồ côi sống qua `tat` (PID 2532 `joint_state_publisher`). [log]
5. Lần chạy 1 và 2 tốt, lần 3–5 hỏng; lần 1 hỏng muộn (97 s) dù `shm` = 0. [log]

## 5. Giả thuyết (sắp theo mức tin; mỗi cái kèm phép thử rẻ nhất)

| # | Giả thuyết | Tin | Phép thử rẻ |
|---|---|---|---|
| A | Tiến trình con của `ros2 launch` KHÔNG nằm trong danh sách `NODES` (`joint_state_publisher`, `robot_state_publisher`, `twist_mux`, `static_tf_pub_laser`, có thể thêm node camera) sống sót sau `tat` (vì `tat` leo lên KILL -9 giết `ros2 launch` cha) và cộng dồn: trùng `/tf`, `/tf_static`, `/cmd_vel` | ≈ 40% [suy luận] | `ps -eo pid,ppid,etimes,comm`; đếm bản sao; `ros2 node list` tìm cảnh báo trùng tên |
| B | Tệp `fastrtps_*` sót hoặc một tiến trình DDS mồ côi làm khám phá giữa các tiến trình hỏng | ≈ 25% | tắt TẤT CẢ tiến trình ROS (kể cả `ros2 daemon`, bag), xóa `shm`, bật bằng tay, theo dõi `ls /dev/shm` |
| C | Giờ hệ thống: nhảy +94 ngày lúc 15:46 (NTP tắt), dấu thời gian TF/bond lệch | ≈ 10% | `timedatectl`, so `date` với dấu thời gian trong log; xem `use_sim_time` mọi node |
| E | Chính `bat_xe.sh` v3 (mới, chưa từng chạy trên Pi): thứ tự bước, `ros2 daemon stop/start` ở B3, các `pkill` của `tat`, kiểm vị trí chạy ngay sau Nav2 | ≈ 20% | bật bằng đúng `Quy trinh chuan 71` (bước 5, 7), không dùng `bat_xe.sh`, rồi so |
| D | Lỗi riêng của cầu nối/systemd | ≈ 5% (lần 5 hỏng khi bật bằng `ssh`) | bật hoàn toàn bằng `ssh` sau khi gỡ dịch vụ |

## 6. Chỗ nghi "không đồng bộ" cần rà (danh sách kiểm)

- Tên tiến trình khác nhau giữa: `NODES` trong `bat_xe.sh` (12 tên), `CHINH` trong `cau_noi_xe.py` (12 tên), lệnh tắt trong `Quy trinh chuan 71` (7 tên), `Trang thai` (12 tên); không tên nào có 4 nhóm ở giả thuyết A.
- Khung gốc: `chuyen.sh` dùng `base_footprint`; `kiem_xe.sh` và `Quy trinh chuan 71` dùng `base_link`; Nav2 dùng `base_footprint`. Cả hai tồn tại trong URDF nhưng phải khớp thực tế trên Pi.
- `bat_xe.sh` B3: `ros2 daemon stop/start` trong lúc nhiều tiến trình ROS đang chạy; mọi lệnh `ros2` CLI (kể cả `tf2_echo`, `param get`) do `ssh` mới gọi nên phụ thuộc khám phá DDS.
- Hai nhóm bật nền (`setsid nohup`) và `KillMode=process` của dịch vụ: tiến trình sống ngoài cầu nối nhưng vẫn trong cgroup của dịch vụ.
- `NAV_PARAMS`/`params_file`, `MONG_BT` (`bat_xe.sh` đọc `default_nav_to_pose_bt_xml` từ tệp YAML bằng `sed`).
- Quaternion trong bảng tuyến của `chuyen.sh` (`z w`) và tên đối số `x y z w`.
- Thời gian chờ (`T1` 60 s, `T2` 120 s, `HAN` 480 s, 420 s của cầu nối) so với thực tế trên Pi.
- `lai_xe_lidar.py` và `chuyen.sh` cùng dùng `/cmd_vel_avoid` (`logic_quyet_dinh.py` cũ không nằm trong launch chính).

## 7. Điều chưa biết

- Vì sao lần chạy 1 tốt 97 giây rồi `velocity_smoother` mất nhịp tim 4 giây trong khi tiến trình vẫn sống.
- Vì sao có tệp `shm` xuất hiện khi không có tiến trình ROS nào do ta bật.
- `bag_1031_160121` trên Pi chứa dữ liệu quanh các lần hỏng; chưa phân tích.
- Không biết Wi-Fi trường có Internet không; NTP đang tắt do quy trình.
- Chưa từng chạy `lai_xe_lidar.py` hay `chuyen.sh v4` với motor ON trên xe.

## 8. Kế hoạch sau lượt đọc (mỗi bước dừng, chờ Thiên)

1. Lượt đọc: ba lệnh kiểm trong `LENH_CLAUDE_CODE.txt`; báo bảng lệch giữa các tệp; KHÔNG sửa gì.
2. Nếu kiểm 1 thấy bản sao hoặc tiến trình mồ côi (giả thuyết A): đề xuất MỘT thay đổi (bổ sung tên vào danh sách tắt, hoặc đổi cách `tat`), có đường lui (`git`), ghi Core hay Stretch và cổng (G2, G5).
3. Mốc đối chứng (Thiên làm): tắt Pi bằng lệnh có chặn, bật lại, bật hệ thống bằng đúng `Quy trinh chuan 71` bước 5 và 7 (không `bat_xe.sh`, không nút). Chạy ổn thì lỗi nằm ở bộ tệp mới; hỏng thì lỗi nằm ở môi trường Pi.
4. Phân tích `~/bag_1031_160121` (64 MB, quanh lần mất nhịp tim 16:00:57) chỉ khi bước 1–3 chưa đủ.
5. Đóng băng (Stretch): cầu nối, giao diện mới, bộ lái LiDAR, bảng tuyến tiếp tuyến, tới khi cổng Core ổn trở lại.
