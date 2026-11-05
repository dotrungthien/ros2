# Kế hoạch việc 109 (Core, G2 G3 G5): chuỗi chỉ huy + bám đường dày + lùi có mắt

Soạn 02/11/2026 ở trường, **chỉ đọc, chưa có mã, chưa commit**. Nguồn đọc: `THIET_KE_109_111.md` mục 0, 1, 4–7; `THIET_KE_SEP.md` mục 2b, 7, 10; `sep.py` (các hàm `lech_dau_plan`, `nhin_xa`, `_plan`, `_goals`, `chon_tu_the`, nút ROS); `lai_xe_lidar.py` (toàn bộ); `chuyen.sh`; `tinh_tuyen.py`; `bat_xe.sh` (đoạn tắt, `NAV_PARAMS`); `nav2_params_cho.yaml`; `bt_cho.xml`; `twist_mux.yaml`; launch `navigation_my_bot.launch.py`. Chưa đọc hết: thân `sep.py` ngoài các hàm trên, `test_lai_xe_lidar.py`, `mo_phong_he_thong.py`. Nhãn: [đọc code] [kiểm] [ước] [suy luận].

## 0b. ĐÃ LÀM 02/11 chiều: 109a (`c1b0d52`) và 109b (commit kế tiếp) — Thiên trả lời mục 6: (1) costmap đọc thẳng `/global_costmap/costmap`; (3) `gui_duong.py` là bên gửi/huỷ đích, đọc `~/sep_lenh`, `~/sep_thoat`; (4) 1.6 để sau

**Gốc lỗi đang sửa:** xe 17:03 bảng `xs` (6 điểm, `/navigate_to_pose`): T1.1, T1.2 nhận "đã tới" ở 14–15 cm (`xy_goal_tolerance` 0,15), sai số dồn qua từng điểm, xe trôi 14 cm về K4 rồi kẹt. Đường dày = **một** đích `/follow_path`, chỉ điểm cuối có dung sai.

**109a — `gui_duong.py` (một tiến trình rclpy, không gọi `ros2` CLI), `chuyen.sh BANG=day`:**
- `tinh_tuyen.py`: thêm `Ban.tu_luoi` (OccupancyGrid → đỏ ≥ 98, xanh < 30, còn lại/−1 vàng), `tinh_duong` (đường ~5 cm, cùng giá 1 + 60/d³ và vùng cấm tuyến 1/2; ô cấm cách điểm đầu < 0,25 m được đi), `lam_day`, `KhongCoDuong`. `main()` in **y từng dòng** như trước (đã so).
- Luồng: KIEM (tf ≤ 1 s, có costmap, xe không trong ô đỏ; sai ⇒ `SAI_VI_TRI` mã 3) → mỗi tuyến tối đa `LUOT`=3 lần: hỏi sếp (`DUNG` ⇒ mã 4; `CHO` ⇒ chờ `SEP_CHO_S` tối đa 2 lần), chờ khi phụ lái đang `RA_TIEN/RA_LUI/XOAY_P/THOAT_*` hoặc `~/sep_thoat` mới → tính đường từ vị trí **hiện tại** trên costmap **hiện tại** → `/duong_day` (2 lần/giây, phần còn lại) → `FollowPath` (`controller_id FollowPath`). Trong lúc chạy `DUNG` ⇒ huỷ đích. `SUCCEEDED` mà cách đích > `TOI_SAI_SO` 0,20 m ⇒ coi là hỏng. Hỏng ⇒ chờ ≥ `CHO_HONG` 30 s (tối đa gấp đôi khi phụ lái còn lái; thêm ≤ 30 s trước lần gửi) rồi tính lại. Hết lượt ⇒ tuyến kia; hai tuyến hỏng ⇒ `HET_DUONG` mã 1. CSV và mã thoát như `chuyen.sh`.
- Không làm: kiểm "bộ lập đường thấy xe ở đâu" của `chuyen.sh` (`compute_path_to_pose`) — thay bằng tf mới + xe không trong ô đỏ. Đổi tuyến giữa khe (đang ở khe trên mà tuyến 1) ra `KhongCoDuong` ⇒ sang tuyến kia.
- `bat_xe.sh tat` tắt cả `gui_duong.py` (mẫu `TAY`). `so_voi_pi.sh` có `gui_duong.py`; `tinh_tuyen.py` thành bắt buộc (bỏ `tuychon`).

**109b:** `sep.py`, `lai_xe_lidar.py` nghe thêm `/follow_path/_action/status` ("có đích" gộp theo từng kênh; tham số `nghe_follow_path` mặc định true); sếp lấy đường từ `/duong_day` (đích = cuối đường, P = điểm cách đầu đường 0,40 m khi đường mới ≤ 3 s); `ABORT_NHANH` chỉ còn cho `/navigate_to_pose`. `ghi_bag.sh` thêm 7 kênh.

**Đã kiểm:** `test_gui_duong.py` 13/13 (IO giả: tới nơi, `SUCCEEDED` lệch, 3 lần hỏng đổi tuyến, hết đường, `DUNG` giữa chừng, `CHO`, phụ lái đang lái, sai vị trí, đường dày khớp 6 điểm `xs` < 3 cm, `main()` in đúng bảng `xs` của `chuyen.sh`); `test_sep.py` 37/37; `test_lai_xe_lidar.py` 60/60. Chạy **nút rclpy thật** cùng tiến trình với thế giới giả: ca thẳng và ca hỏng lần đầu đều `DEN_NOI`; sếp + phụ lái thấy đích qua `/follow_path`.
**Chưa kiểm:** Nav2/RPP thật bám đường dày; DDS giữa các tiến trình (PC trường không chạy được: cả `ros2 topic pub` + `topic list` không thấy nhau); ngưỡng đổi costmap (suy luận theo `occupied_thresh`); `goal_checker_id` rỗng (đọc chuỗi trong thư viện). **Chưa giả lập cả hệ thống, chưa chạy trên Pi.**
**Thử trên xe (đề xuất):** chép 6 tệp (`gui_duong.py`, `tinh_tuyen.py`, `sep.py`, `lai_xe_lidar.py`, `chuyen.sh`, `ghi_bag.sh`) + `bat_xe.sh` (giữ dòng `ACCESS`), `so_voi_pi.sh` phải `GIONG`; motor OFF: `BANG=day bash ~/chuyen.sh di_B` xem `/duong_day` và đích; rồi motor ON, tay sát công tắc. Đường lui: `BANG=xs`.

## 0. CẬP NHẬT 02/11 15:57 — chỉ làm 109c, chạy trên `BANG=xs` (Thiên quyết; thay mục 1–6 cho hôm nay)

Thiên: 109a, 109b, 1.6 và câu hỏi (1) costmap **để sau**. 1.7 (ghi nhớ chỗ kẹt) **chỉ ghi nhật ký**. `~/sep_thoat` là **tệp**. Chưa có `gui_duong.py` ⇒ `chuyen.sh` vẫn là bên gửi và gửi lại đích như cũ. Hạn lên xe 18:00. Mức hiện tại 15:57 [kiểm `date`].

**Luồng (không cần đường dày):**
1. **Sếp** (`sep.py`, `ra_thoat=true`, mặc định false). Khi `KET_SAT_VAT` bật lần mới và `_luot_moi()` còn lượt (1–3): mỗi vòng ghi `~/sep_thoat` theo khuôn `ghi_lenh` (ghi tạm rồi đổi tên): `<giây BOOTTIME> THOAT <xP> <yP> <lượt> <mã>`. `mã` = giây BOOTTIME lúc dấu hiệu bật (duy nhất, vì `luot_thoat` đặt lại mỗi chuyến). **P = `self.dich`** = điểm cuối `/plan` = điểm xs kế tiếp Nav2 đang đi tới. Chưa có `dich` hoặc vị trí không tin ⇒ `P = -`. Xoá tệp khi dấu hiệu tắt, `ra_thoat=false` hoặc sếp tắt (như `xoa_lenh`). **Lượt 4** (`het_luot`): không ghi `~/sep_thoat`; `di_hay_khong` trả `DUNG` (`HET_3_LUOT`) ⇒ `~/sep_lenh` (cần `ra_lenh=true`). **1.7:** thêm phía kẹt và P vào cột `phep_kiem` của dòng `TRUY_VET,KET_SAT_VAT` (đã có x, y, góc), chỉ ghi.
2. **Phụ lái** (`lai_xe_lidar.py`), mỗi vòng đọc `~/sep_thoat`; nhận khi tuổi ≤ 3 s (BOOTTIME), `mã` chưa xử lý, đang có đích, trạng thái `BINH_THUONG`. Ra tay **chỉ khi `phat_lenh=true`**, ngược lại chỉ log "SE LAM". Hàm thuần `NguoiLai` nhận `thoat=dict(ma, goc_P)` (`goc_P` = hướng tới P trong khung thân xe, nút tính từ tf mỗi vòng), ba pha:
   - **RA:** so thoáng trước `ff` và sau `fb` theo bề rộng thân, chọn phía thoáng hơn (≥ `thoang_toi_thieu` 0,05 m); đi thẳng ±`v_esc` 0,10 m/s tới khi mọi điểm LiDAR cách tâm > **18,55 cm** (`R_XOAY`) hoặc đủ **30 cm** (đếm bằng đồng hồ ≈ 3 s: odom hở), hoặc mép ≤ `dung_neu_mep` 2 cm theo hướng đi ⇒ bỏ cuộc.
   - **XOAY:** chỉ khi đủ chỗ; xoay ±`w_esc` 1,0 rad/s về `goc_P` theo phía ngắn; **mỗi vòng** kiểm mọi điểm LiDAR > `R_XOAY` (không thì dừng, bỏ cuộc); chậm lại 0,6 rad/s khi còn < 45° (tf có thể cũ tới 1 s); xong khi lệch < 20° (`xoay_den_deg`). `P = -` hoặc tf không tin ⇒ bỏ qua pha này.
   - **XONG:** `_ket_thuc('XONG', 3 s)`: im, `twist_mux` trả bánh cho Nav2 (60 → 20, timeout 0,2 s). Quá `t_thoat_max` 10 s ⇒ `BO_CUOC_HET_GIO` (nghỉ 20 s). Phản xạ cũ giữ nguyên.
3. **`chuyen.sh` không đổi.** Nó đọc `DUNG` chỉ ở ranh giới (trước lần gửi, sau `KHONG_DAT`) ⇒ sau lượt 4 xe có thể đứng tới `TO` = 130 s trước khi `chuyen.sh` thấy `DUNG` [đọc code]. Huỷ đích giữa chuyến thuộc `gui_duong.py` (sau).
4. **Bật tay trên xe** (như hiện nay, không qua `bat_xe.sh`): sếp `ra_lenh:=true ra_thoat:=true`, phụ lái `phat_lenh:=true`. Lần đầu motor OFF để xem log "SE LAM".

**Ca thử:** `test_sep.py`: lượt 1 ghi tệp đủ trường, P = `dich`; `ra_thoat=false` không ghi; dấu hiệu tắt ⇒ tệp bị xoá; lượt 4 không ghi tệp và ra `DUNG`; vị trí không tin ⇒ `P=-`; `test_14` vẫn qua (không `Twist`, một publisher). `test_lai_xe_lidar.py`: đọc tệp (tuổi, mã lặp, tệp hỏng); RA chọn phía thoáng (K4 phía trước ⇒ lùi); dừng khi đủ 18,55 cm / 30 cm / mép 2 cm; XOAY về P hai dấu; vật xuất hiện giữa lúc xoay ⇒ dừng; `phat_lenh=false` không phát; mã đã xử lý không chạy lại. `mo_phong_he_thong.py`: ca xe (0,60; 0,02; −90°) sát K4 5,5 cm, P = (1,05; 0,06): không chạm, kết thúc `XONG` ≤ 10 s. Giả lập cả hệ thống: chưa làm được ở PC trường (thiếu `install`).

**Ước thời gian riêng 109c** [ước, chưa đo], chia ba commit, mỗi commit mặc định tắt (`ra_thoat` / `phat_lenh` = false):

| Commit | Làm | Viết + thử (PC) | Xong lúc (nếu bắt đầu 16:05) | Lên xe |
|---|---|---|---|---|
| **c1** | sếp ghi/xoá `~/sep_thoat`, 1.7 nhật ký, lượt 4 ⇒ `DUNG`; phụ lái chỉ **đọc và in "SE LAM"** (chưa ra tay) | ≈ 2 giờ | ≈ 18:00 | motor OFF, xem sếp ghi đúng P và phụ lái đọc đúng; **không** có động tác |
| **c2** | phụ lái pha RA (ra phía thoáng, kiểm thân) | + 1,5 giờ | ≈ 19:30 | motor ON sát K4, tay sát công tắc |
| **c3** | pha XOAY về P + ca mô phỏng K4 | + 1,5–2 giờ | ≈ 21:00 | như c2 |

Tức làm đủ 109c mất ≈ 5–5,5 giờ nên **không kịp 18:00**; kịp 18:00 chỉ c1 (không ra tay). Đề xuất: c1 trước 18:00, c2 và c3 tối nay hoặc 3/11.

**Rủi ro riêng của 109c:**
- **Hướng tới P ≠ đường đi.** Xe sát K4 mà P nằm sau một khúc cua: đường thẳng tới P có thể cắt qua tường, xoay đúng `goc_P` làm mũi đối mặt vật. Đề xuất (c3): chỉ xoay tới hướng thoáng nhất trong ±45° quanh `goc_P`, và dừng nếu `ff` < 0,12 m. Cần Thiên duyệt.
- **Vòng điều khiển xoay phụ thuộc tf** (SLAM, có thể trễ tới `tf_tuoi_toi_da` 1 s; 31/10 có lúc tf cũ hơn nhiều): xoay 1 rad/s với tf trễ 0,3 s lệch ≈ 17°. Đã chậm khi gần, nhưng chưa đo trên xe.
- **Ba bên cùng lái:** `progress_checker` 15 s có thể ABORT trong lúc phụ lái làm; `bt_cho` còn thử lại tới 15 lần nên chịu được, nhưng chưa thử.
- **Lượt 4 chậm có tác dụng** tới 130 s (mục 0.3) vì chưa có bên huỷ đích.

## 1. (Kế hoạch ban đầu 15:3x — 109a/109b/1.6 để sau) Ba mảnh, mỗi mảnh một commit, tự chạy được, tắt bằng một cờ

| Mảnh | Nội dung | Tệp đụng tới | Cờ bật/tắt | Đường lui |
|---|---|---|---|---|
| **109a** bộ gửi đường dày | (1) `tinh_tuyen.py`: tách `tinh_duong(ban, dau, cuoi, tuyen) -> [(x, y, yaw)]` và `Ban.tu_luoi(OccupancyGrid)` (đỏ khi giá trị ≥ 98, xanh khi ≤ 30, còn lại vàng = đúng ngưỡng `occupied_thresh 0.98 / free_thresh 0.3` của `costmap_v3_020.yaml`); `main()` in PC giữ nguyên từng dòng. (2) `gui_duong.py` mới: nút rclpy nghe `/global_costmap/costmap` (TRANSIENT_LOCAL), đọc tf `map→base_footprint`, tính đường dày từ xe tới đích theo tuyến 1/2, gửi `nav2_msgs/FollowPath` tới `/follow_path`, phát phần còn lại lên `/duong_day`, in `status: SUCCEEDED` khi tới (để `gui()` của `chuyen.sh` nhận). Tự gánh việc chờ/thử lại (mục 4). (3) `chuyen.sh`: nhánh `BANG=day` trong `lay_tuyen` và `gui`. (4) `so_voi_pi.sh`: thêm `gui_duong.py`. | `tinh_tuyen.py`, `gui_duong.py`, `chuyen.sh`, `so_voi_pi.sh`, ca thử mới | `BANG=day` (mặc định chưa đổi: vẫn `tt`/`xs` tới khi xe thử xong) | `BANG=xs` (6 điểm, `/navigate_to_pose`); `git revert` |
| **109b** nghe trạng thái `/follow_path` | `sep.py` và `lai_xe_lidar.py` nghe thêm `/follow_path/_action/status`, **giữ** `/navigate_to_pose/_action/status`; `sep.py` lấy tuyến từ `/duong_day` ngoài `/plan`. `ghi_bag.sh`: thêm `/sep/trang_thai`, `/cmd_vel_nav`, `/cmd_vel_avoid`, `/cmd_vel_teleop`, `/rosout`, `/follow_path/_action/status`, `/navigate_to_pose/_action/status`, `/duong_day`, `/received_global_plan`; sửa dòng đầu (đã chạy 31/10). | `sep.py`, `lai_xe_lidar.py`, `ghi_bag.sh`, ca thử | tham số `nghe_follow_path` (mặc định true; false = như cũ) ở cả hai nút | `ros2 param set … nghe_follow_path false`; `git revert` |
| **109c** sếp THOÁT + điểm P → phụ lái thi hành | Sếp ghi `~/sep_thoat` (kèm điểm nhập P trên tuyến) khi `KET_SAT_VAT`; `lai_xe_lidar` (`phat_lenh=true`) đo thoáng trước/sau, đi ra phía thoáng, xoay mũi về P, kiểm thân từng bước, báo xong; `twist_mux` trả bánh cho Nav2. Nối `het_luot` → `di_hay_khong` trả `DUNG` (Thiên 02/11: để việc 109) và `gui_duong.py` đọc `~/sep_lenh` giữa chuyến để huỷ đích + thoát mã 4 (`chuyen.sh` chỉ hỏi sếp giữa hai lần gửi, nên với một đích dài nó mù). | `sep.py`, `lai_xe_lidar.py`, `gui_duong.py`, ca thử | `ra_thoat` ở sếp (mặc định false) **và** `phat_lenh` ở phụ lái (mặc định false): hai khoá | tắt hai tham số = như 109b; `git revert` |

Thứ tự bắt buộc: a → b → c. b cần `/duong_day` của a; c cần cả hai. Cổng: 109a G2+G3+G5, 109b G2+G5, 109c G2+G3+G5 (xe thật, công tắc motor sát tay).

## 2. Ước thời gian (viết + thử) [ước, chưa đo]

| Mảnh | Viết | Thử | Ghi chú |
|---|---|---|---|
| 109a | 3–4 giờ | 2 giờ trên PC (hàm thuần so với đầu ra cũ của `tinh_tuyen.py`; nút với `controller_server` + costmap thật, không cần `my_bot`) + 1,5 giờ giả lập/Pi | PC trường có `rclpy`, `nav2_msgs`, `nav2_controller` [kiểm]; thiếu `~/ros2_ws/install`, `twist_mux`, `joint_state_publisher` nên **không chạy được `gia_lap/bat_gia.sh`** [kiểm]. Giả lập cả hệ thống phải làm ở máy nhà hoặc trên Pi |
| 109b | 1,5 giờ | 1 giờ ca thử + 1 chuyến xe chỉ nhìn | |
| 109c | 4–5 giờ | 3 giờ mô phỏng (`mo_phong_he_thong.py`) + 2 giờ xe motor OFF rồi ON sát K4 | phần lớn thời gian là động tác mới của phụ lái |
| **Tổng** | ≈ 8,5–10,5 giờ viết, ≈ 9,5 giờ thử | | tức ≈ 18–20 giờ; mốc: 3/11 chấm trước thầy, 10/11 chấm thật |

## 3. Bảng kênh

| Kênh | Ai phát | Ai nghe | Tên / kiểu / frame | Hiện có? |
|---|---|---|---|---|
| `/follow_path` | `gui_duong.py` (client) | `controller_server` (server) | action `nav2_msgs/action/FollowPath`: goal `path`, `controller_id`, `goal_checker_id`; result `Empty`; feedback `distance_to_goal`, `speed` [kiểm trên Humble]; path frame `map` | server **có** (launch Nav2, remap `cmd_vel→cmd_vel_nav_raw`); client **chưa** (109a) |
| `/follow_path/_action/status` | `controller_server` | `sep.py`, `lai_xe_lidar.py` | `action_msgs/GoalStatusArray`, QoS RELIABLE + TRANSIENT_LOCAL như kênh cũ | topic có sẵn; người nghe **chưa** (109b) |
| `/navigate_to_pose` | `chuyen.sh` (`ros2 action send_goal`) | `bt_navigator` | action `NavigateToPose`, frame `map`; cờ lui `BANG=xs` | **có** |
| `/navigate_to_pose/_action/status` | `bt_navigator` | `sep.py`, `lai_xe_lidar.py` | `GoalStatusArray` | **có**, đang nghe |
| `/plan` | `planner_server` khi có yêu cầu (BT; có thể cả `compute_path_to_pose` của `chuyen.sh` KIEM: `sep.py` mục 6 còn ghi là chưa biết) | `sep.py` | `nav_msgs/Path`, frame `map` | **có**; với `/follow_path` thì **im** (không BT) |
| `/duong_day` | `gui_duong.py` (phần còn lại của đường, cắt từ điểm gần xe nhất, 2 Hz) | `sep.py` (109b) | `nav_msgs/Path`, frame `map`; tên là đề xuất | **chưa** (109a phát, 109b nghe) |
| `/received_global_plan` | `controller_server` khi nhận goal [suy luận; `ghi_bag.sh` đã ghi tên này, chưa kiểm tên/QoS] | `ghi_bag.sh` | `nav_msgs/Path` | chưa kiểm |
| `/cmd_vel_nav` | `velocity_smoother` (`cmd_vel_smoothed→cmd_vel_nav`) | `twist_mux` (20, timeout 0,5), `lai_xe_lidar` (phủ quyết) | `Twist` | **có** |
| `/cmd_vel_avoid` | `lai_xe_lidar` (khi `phat_lenh=true`); `logic_quyet_dinh.py` cũng phát nhưng không có trong launch | `twist_mux` (60, timeout 0,2) | `Twist` | **có** |
| `~/sep_lenh` | `sep.py` (`ra_lenh`, mặc định false) | `chuyen.sh` `doc_sep`; `gui_duong.py` (109c) | tệp `"<giây CLOCK_BOOTTIME> DI\|CHO\|DUNG <lý do>"`, ghi tạm rồi đổi tên | sếp ghi **có**; `chuyen.sh` đọc **có**; `gui_duong.py` **chưa** |
| `~/sep_thoat` | `sep.py` (`ra_thoat`) | `lai_xe_lidar` | tệp `"<giây BOOTTIME> THOAT <x_P> <y_P> <mã lượt>"` (đề xuất, cùng khuôn `ghi_lenh`) | **chưa** (109c) |

## 4. Rủi ro lớn nhất mỗi mảnh

**109a.** `FollowPath` đi thẳng tới `controller_server`, bỏ qua `bt_navigator` và `bt_cho.xml`: không vạch lại, không xoá costmap, không chờ 3 s × 15 lần, không có `/navigate_to_pose`. RPP chỉ chịu lỗi `failure_tolerance 0,3 s` (`nav2_params_cho.yaml`), rồi `FollowPath` ABORT; `progress_checker` ABORT sau 15 s không dịch 0,2 m. Không có BT thì **xe dừng vĩnh viễn** sau lần ABORT đầu, trừ khi `gui_duong.py` tự chờ ≤ 30 s (#9) rồi gửi lại phần đường còn lại, tối đa cỡ 60 s như `bt_cho`. Rủi ro phụ: đường tính một lần lúc gửi từ costmap lúc đó, mà vị trí lúc bật có thể sai (sự cố 31/10: tf cũ) ⇒ phải giữ bước KIEM của `chuyen.sh` và từ chối nếu xe cách A/B > 0,30 m (vùng cấm của `tinh_tuyen.py` cho tuyến 1/2 chặn đường nếu điểm đầu nằm sai hành lang). Hướng cuối: `goal_checker` đòi lệch hướng ≤ 0,3 rad so với pose cuối của đường; `di_B` giữ 0° như bảng `tt`. Và `inflation_radius` 0,25 trong tệp, 0,20 đặt tay lúc hiệu chuẩn 1/11: đường theo costmap lúc chạy nên đổi theo (không phải việc 109, nhưng đừng để lệch im lặng).

**109b.** Hai lỗi âm thầm nếu làm cho qua: (1) `sep.py` `_goals` đặt `goal_active = any(...)` ghi đè mỗi bản tin; có hai kênh trạng thái thì kênh này ghi đè kênh kia ⇒ phải gộp theo từng topic. (2) `lech_dau_plan` và `nhin_xa` dùng **điểm đầu** của đường: với đường cố định từ lúc gửi, xe đi quá 0,30 m là báo `PLAN_LECH` sai (sếp bảo KHÔNG_ĐI) và `nhin_xa` nhìn sai đoạn ⇒ `gui_duong.py` phải phát phần còn lại đã cắt (đã tính ở bảng kênh), và ca thử phải có "xe đi dọc đường dày 1 m không báo `PLAN_LECH`". Phụ: `ABORT_NHANH` (< 2 s ⇒ nghi "0 poses") vốn là dấu hiệu của bộ lập đường; với `FollowPath` ABORT sớm có nghĩa khác.

**109c.** Động tác mới của phụ lái (đi ra phía thoáng, xoay mũi về P, kiểm thân từng bước) **chưa từng chạy trên xe** (`lai_xe_lidar` mới có mô phỏng 26/26), odom là tích phân lệnh, hành lang 50–56 cm, xoay tại chỗ cần ≥ 45,6 cm, nên lỗi = va chạm. Bàn giao ba người điều khiển (sếp, phụ lái, RPP) là chỗ dễ hỏng: sau THOÁT, `progress_checker` có thể đã ABORT, nên `gui_duong.py` phải biết phụ lái xong (dấu hiệu `XONG` trong `~/sep_thoat` hoặc log) rồi **cắt lại đường từ điểm P**, không cho RPP bám đường cũ từ vị trí lạ. Ràng buộc có sẵn: ca thử 14 của `test_sep.py` cấm action client và chỉ cho **một** publisher String ở sếp ⇒ dùng **tệp** `~/sep_thoat` (như `~/sep_lenh`) thay vì topic mới, và sếp không tự huỷ đích được (nên `gui_duong.py` huỷ).

## 5. Ngoài ba mảnh (cần Thiên quyết nơi làm)

- 109 mục 1.6 (vật lạ ⇒ sang khe kia, tối đa 3 lượt **đổi luồng**) và 1.7 (ghi nhớ chỗ kẹt): không thuộc a/b/c. Đề xuất: 1.7 nhập 109c (chỉ thêm cột nhật ký), 1.6 để mảnh sau (109d) vì cần `gui_duong.py` đổi tuyến giữa chuyến.
- "Hết 3 lượt: dừng, **ảnh**, báo": 109c làm dừng + ghi `DUNG_SEP` + mã 4; ảnh LiDAR và màn đỏ thuộc trang 111 (đã chốt mục 4a, 4b).
- Mặc định `BANG` sau khi 109a qua xe: đổi một dòng trong `chuyen.sh`, làm ở commit riêng.

## 6. Câu hỏi cho Thiên (trả lời "đồng ý" cũng được)

1. Đồng ý lấy costmap **trực tiếp từ `/global_costmap/costmap` lúc chạy** (không qua `map_saver` + tệp)? Bước xuất tệp trong `THIET_KE_109_111.md` mục 7 vẫn ghi cho `bat_xe.sh` (việc 110); tôi bỏ nó khỏi 109a để bớt một chỗ treo `ros2` trên Pi.
2. Đồng ý `~/sep_thoat` là **tệp** (khuôn `~/sep_lenh`) thay vì topic, vì ca thử 14?
3. Đồng ý `gui_duong.py` là bên huỷ đích khi sếp bảo `DUNG` (109c)?
4. 1.6 và 1.7 xếp như mục 5?
