# BÁO CÁO VIỆC 111: trang điều khiển `/v10` (chạy đêm 06/11/2026, 03:14 → 04:10)

**Giả lập chưa hiệu chuẩn, chỉ bắt lỗi ghép; chưa thử trên xe.** Không ssh/scp, không chạm `Pi`, không push.

Nhánh `trang-111` (worktree `~/ros2_ws/src/ros2_trang111`, gốc `579cda5`). Cây chính `~/ros2_ws/src/ros2` không bị đụng: vẫn `dong-bo` @ `579cda5`, `git status` vẫn đúng B6c (`tinh_tuyen.py` md5 `50ed279d`, `test_gui_duong.py`, `RA_SOAT_118*`).

## 0. Kiểm đầu và luật cứng

- `git rev-parse --short HEAD` = `579cda5` ✓; `git status` chỉ có B6c ✓ → tạo worktree `git worktree add ../ros2_trang111 -b trang-111 579cda5`.
- Tệp lái **không đổi**: `git diff 579cda5 -- sep.py lai_xe_lidar.py gui_duong.py tinh_tuyen.py chuyen.sh bat_xe.sh nav2_params_cho.yaml bt_cho.xml dev_ws/src/my_bot` rỗng.
- `/nut` giữ nguyên từng byte: md5 trang `/nut` = `9a96075b` ở cả cây chính lẫn nhánh; mọi đường `/api/nut/*`, `/api/b/*` trả như cũ (test hồi quy, mục 2).
- Luật 5 trong tệp giao việc chứa chữ không được in. Mình không in dòng đó, chỉ dò bằng cách đếm từ khoá, rồi áp dụng như sau:
  - Mã truy cập không ghi vào tệp, log hay câu trả lời nào.
  - Trước mỗi commit, đếm chuỗi cấm (chữ đó + dấu bằng) trong `git diff --cached` = 0 (đủ 8 lần).
- Tên biến môi trường của mã teleop:
  - Lấy giống `bat_xe.sh`.
  - Trong mã nó được ghép chữ (`'ACC' + 'ESS'`), để không tệp mới nào chứa chữ đó.
  - Test D(1) so sánh tên này với `bat_xe.sh` mà không in ra.

## 1. Mảnh đã làm, commit, test

Thứ tự đúng M1 → M4 → M2 → M3 → M5 → M7 → M6, mỗi mảnh 1 commit, cuối cùng có 1 commit D(1) và báo cáo.

| Mảnh | Commit | Nội dung | Test sau mảnh (trang / cầu nối cũ) | md5 sau mảnh (`trang_v10.py`, `.html`, test) |
|---|---|---|---|---|
| M1 khung | `ab0a355` | `GET /v10`, `/v10/trang_thai`. Trạng thái suy từ tiến trình, bật/tắt, chuyến, log sếp, phụ lái, gui_duong, `~/sep_thoat`. `han()` lấy nguyên từ `TRANG_NUT`. `cai_cau_noi.sh` chép 2 tệp mới. | 6/6 · 8/8 | `20a58729` `749f3a02` `8c72166e`; `cau_noi_xe.py` `e6f8f8d8`, `test_cau_noi.py` `d78d3627`, `cai_cau_noi.sh` `6922b960` |
| M4 đi/dừng | `45272c0` | `POST /v10/api/{di,dung,bat,tat}` → gọi thẳng `H._nut` (cùng PIN, 409, câu xác nhận) | 11/11 · 8/8 | `f5d896f6` `36ee77ad` `b0e7183a` |
| M2 bản đồ | `e540299` | `/v10/ban_do`: `slam_dinh_vi_xuong.yaml` → `map_file_name` → ảnh. Footprint, inflation lấy từ `NAV_PARAMS`. Costmap Nav2 dựng lại, vùng đỏ = `tinh_tuyen.Ban.do`. Đọc PNG bằng Python thuần. | 16/16 · 8/8 | `0721066f` `4caf1dd3` `690c5db3` |
| M3 경로 확인 | `dbb8be2` | `/v10/tuyen?dich=B\|A`: `tinh_tuyen.tinh_duong` từ vị trí sếp ghi, đích + góc cuối lấy từ `gui_duong.DICH`, tuyến 1 trước rồi 2. Chỉ chọn B, A (`CHON_B_TU_DO = False`). | 20/20 · 8/8 | `b44809ff` `e418c92c` `a8e9f47f` |
| M5 두뇌 + bảng | `ef302e3` | Dòng 두뇌, bảng 7 dòng, chỗ kẹt (X đỏ). Ưu tiên lấy từ `twist_mux.yaml`, `R_XOAY` từ `lai_xe_lidar`. Tiến trình: chỉ đọc 4 tham số true/false. | 24/24 · 8/8 | `269eb2b9` `5eb31c0e` `04e04793` |
| M7 도움이 필요해요 | `397a302` | Màn đỏ + rung/âm. Ảnh LiDAR nhìn từ trên, lấy từ `~/scan_*.txt` mới nhất qua `lai_xe_lidar.scan_to_xy`. `다시 시도` → `_nut(cùng hướng)`. | 28/28 · 8/8 | `a806d2af` `ba1224ab` `ba364322` |
| M6 수동 조종 | `9a1731b` | Pad giữ-để-chạy, gửi mỗi 0,1 s → `POST /v10/api/lai_tay` → teleop_web `/cmd` (đăng nhập bằng mã trong biến môi trường, giữ cookie, hỏng thì chờ 10 s). Thả nút → gửi 0. | 33/33 · 8/8 | `fb0d2396` `547e0fa7` `c1f441ee` |
| D(1) + sửa | commit cuối | Sửa các chỗ không khớp ở mục 3 + test D(1) + báo cáo này | **42/42 · 8/8**; tệp lái: gui_duong 30/30, sếp 46/46, phụ lái 97/97 | ghi trong commit cuối |

`test_cau_noi.py` là test hồi quy mới viết, vì kho không có test nào cho cầu nối. Mình chạy nó trên `cau_noi_xe.py` @ `579cda5` **trước** khi sửa: đạt 8/8.

Đường mới: `GET /v10`, `/v10/trang_thai`, `/v10/ban_do`, `/v10/tuyen`, `/v10/anh_lidar`; `POST /v10/api/{di,dung,bat,tat,thu_lai,lai_tay}`. Nếu `trang_v10.py` lỗi thì chỉ `/v10` trả 500, còn `/nut` vẫn chạy (đã có test).

## 2. Cách kiểm (b) mỗi mảnh

- **`env -i`:** chạy cầu nối với `env -i`, HOME giả trong scratchpad, cổng 5091, `bat_xe.sh`/`chuyen.sh` giả, rồi `curl` mọi API. `/nut` trả md5 `9a96075b` y như cây chính.
- **Edge headless:** Edge trên Windows chạy JS thật của trang và chụp màn hình (`BAO_CAO_111_m3.png`, `_m5.png`, `_m7.png`). Chạm B được giả bằng iframe cùng nguồn. Ở bề rộng 360 px không tràn ngang.
- **Log thật 05/11 (M5):** xe kẹt K1 ở (0,16; 0,76; 43°), tâm xe trong vùng đỏ. Trang ghi "제자리 회전 불가 (13 cm < 18,6 cm)" và lấy ưu tiên 100/60/20 từ tệp.
- **M6 với teleop_web THẬT** (ROS domain 87, mã thử, không phải mã thật):
  - 21 lệnh đều 200.
  - `/cmd_vel_teleop` nhận 24 tin, giá trị {0,15; 0,0}, tin cuối 0,0.
  - Không câu trả lời nào chứa mã.
- **Giả lập cả chuỗi:** `gia_lap/bat_gia.sh` (Nav2 + SLAM + xe giả + sếp + phụ lái), HOME giả, `TAM` = HOME. Lần 1 Nav2 kẹt cấu hình; dọn bằng `bat_xe.sh tat` rồi chạy lại thì lên.
  - Bấm `/v10` B rồi A: 준비 완료 → 이동 중 (`-- tuyen 1, lan 1/3`) → 도착, cả 2 chiều DEN_NOI (34,6 s và 28,8 s).
  - `정지` 7 lần: 6 lần đúng (130 NGAT, `/odom` đứng yên 3 s sau). 1 lần **hỏng nặng**, xem mục 4.

## 3. Kiểm tích hợp D(1): bên ghi | bên đọc | phải khớp | khớp?

Các dòng có "test" được kiểm tự động trong `test_trang_v10.py` lớp `D1KhopGhep` (đọc mã nguồn bên ghi, có dòng chạy ghi-đọc thật).

| Bên ghi | Bên đọc | Phải khớp | Khớp? |
|---|---|---|---|
| `sep.py` → `~/sep_nhat_ky.csv` (`COT_NHAT_KY`) | `trang_v10.sep_cuoi`, `nho_ket`; `cau_noi.doc_sep` | cột 1 gio, 2 loai, 3 dau_hieu, 6 ket_qua, 8–10 x, y, goc_do | khớp (test, log 05/11, giả lập) |
| `sep.py`: giá trị TU_THE, LENH, `CHUYEN BAT_DAU`, `L1 BAT_DAU_*`, `TRUY_VET KET_SAT_VAT`/`VAT_LA_TRUOC`, `DUNG_VA_BAO` | `trang_v10` | tên giá trị | **KHÔNG khớp → đã sửa.** Mình dùng `KET_SAT_VAT`, `VAT_LA_TRUOC` như TU_THE (thật ra là TRUY_VET). TU_THE `CHO` (`KHONG_CO_DICH`) làm đầu/cuối chuyến hiện 대기 중 sai (giả lập). Nay không suy trạng thái từ TU_THE nữa, vì sếp có thể chỉ nhìn; TU_THE chỉ hiện ở dòng 두뇌. Giả lập chạy lại: hết lỗi. |
| `bat_xe.sh`: `lai_xe_lidar` → `~/log_phu_lai.txt` | `phu_lai_cuoi`, `vung_phu_lai` | dòng ROS `[INFO] [epoch] [lai_xe_lidar]:`, câu `'%s -> %s (%s)%s \| %s'`, `'vung gan nhat: … trang_thai=%s'`, tên vùng `TEN_VUNG`, tên trạng thái | khớp (test, log 05/11, giả lập). Giả lập `bat_gia.sh` ghi vào `$TAM` (khi thử đặt `TAM`=HOME). |
| `gui_duong.py` stdout → `bat_dau_chuyen` → `~/log_nut_chuyen.txt` | `dong_chuyen_cuoi`, `lan_thu` | `-- tuyen %d, lan %d/%d:`, `lop 2: sep bao …`, `sep bao CHO`, `khong den duoc`, `RA_XONG`, `RA_HET_GIO`, `tinh lai duong` | khớp (test, giả lập) |
| `sep.ghi_lenh` → `~/sep_thoat` `<BOOTTIME> CHU …` | `thoat_moi` (`/proc/uptime`) | đồng hồ BOOTTIME; chữ THOAT, RA_LUI, RA_TIEN | khớp (test ghi-đọc thật) |
| `chuyen.sh`/`gui_duong.py` mã thoát | `MA_CHUYEN` (cũ) → `/v10` | 0 DEN_NOI, 1 HET_DUONG, 3 SAI_VI_TRI, 4 DUNG_SEP, 130 NGAT | khớp, **trừ một chỗ**: gui_duong chết vì `RuntimeError` cũng ra mã 1, nên `/nut` ghi HET_DUONG. `/v10` nay tách riêng thành `LOI_THOAT` (mục 4). |
| `bat_xe.sh` `SLAM="${SLAM:-/home/project1/slam_dinh_vi_xuong.yaml}"` | `tep_ban_do` | cùng tệp | **không khớp khi có đặt biến `SLAM` → đã sửa** (cầu nối đọc `SLAM` như `bat_xe.sh`) |
| `slam_dinh_vi_xuong.yaml` `map_file_name: /home/project1/map_xuong_v3_pg` | `tep_ban_do` | ảnh `~/map_xuong_v3_pg.yaml` hoặc `~/map_xuong_v3.yaml` | **chưa rõ trên Pi** (VI_TRI_TREN_PI chỉ nói có posegraph). Thiếu thì trang báo lỗi, không dùng số dự phòng. `cai_cau_noi.sh` thêm dòng `[2b]` cảnh báo. Cần quyết 2. |
| `cai_cau_noi.sh` `NAV_PARAMS=` trong `cau_noi.env` (`bat_xe.sh` cũng đọc) | `tep_nav2` | cùng `nav2_params_cho.yaml` | khớp (test) |
| `nav2_params_cho.yaml` `global_costmap`: footprint, `inflation_radius` 0,20, `cost_scaling_factor` 3,0 | `doc_nav2`, `luoi_chi_phi` | bán kính nội tiếp 0,09 | khớp: lưới giống hệt bản dựng lại B5/B6, 1066 ô cấm [tính]. `footprint_padding` tệp không ghi → lấy mặc định Nav2 0,01 (cần quyết 4) |
| `tinh_tuyen.py` (`~` trên Pi) | M2 vùng đỏ, M3 tuyến (import, không chép số) | `Ban.do`, `tinh_duong` | khớp (test: tuyến `/v10` = gọi thẳng) |
| `gui_duong.DICH`; `chuyen.sh` `TUYEN_DAU="${2:-1}"`, `/nut` không truyền đối số 2 | M3 | B (1,60; 0,82) góc 0; A (0; 0); tuyến 1 trước | khớp (test) |
| `twist_mux.yaml` (teleop 100, avoid 60, nav 20) | `uu_tien` | đường `~/ros2_ws/install/my_bot/share/my_bot/config/twist_mux.yaml` | số khớp (test); đường cài trên Pi chưa kiểm |
| `teleop_web.py`: cổng 5000 (launch không đổi), `/login` form `code` → 302 + cookie `teleop_token`, `/cmd` JSON `linear`/`angular`, `MAX_FAILS` 5 | `lai_tay` | | khớp (test + teleop_web thật) |
| `bat_xe.sh`: biến mã → `access_code` | `TEN_BIEN_MA` | cùng tên biến | khớp (test, không in). systemd không có biến này trừ khi thêm vào `cau_noi.env` (cần quyết 3) |
| teleop `cmd_topic` `/cmd_vel_teleop` | `twist_mux` teleop | | khớp (test) |
| `lai_xe_lidar.scan_to_xy`, `lidar_angle_offset_deg` = 0 (`bat_xe.sh` không đổi) | `anh_lidar` | khung thân | khớp. Số tia tệp thật = RA_SOAT_120: 420/389, 420/390, 128/121 (bị cắt) |
| ai ghi `~/scan_*.txt`? (người chụp tay bằng `ros2 topic echo --once /scan`) | `anh_lidar` | ảnh lúc HET_DUONG | **không ai tự ghi** → trang ghi "사진 없음" hoặc giờ của tệp cũ (cần quyết 5) |
| `lai_xe_lidar.R_XOAY` 0,1856 | bảng 제자리 회전 | | khớp (import) |
| argv của `sep.py`/`lai_xe_lidar.py`/`gui_duong.py` | `tien_trinh_lai` | chỉ `ra_lenh`, `ra_thoat`, `phat_lenh`, `sep_ra_thoat` | khớp (giả lập: false/false). Test: tham số lạ không lọt ra |
| Tên trạng thái: bảng HAN của `/nut` + `HAN_V10` `TT_*`; mã `TT_*` trong HTML; URL trang gọi | cầu nối | | khớp (test) |
| `cai_cau_noi.sh` chép `trang_v10.py`, `.html` vào `~/cau_noi` | `cau_noi_xe.py` `HERE` | | khớp (test) |
| cổng 5001 | `/v10` cùng `/nut` | | khớp |
| `~/gui_duong_luot`, `~/ket_qua_chuyen.csv` | `/v10` **không đọc trực tiếp** (dùng mã thoát + log chuyến) | | ghi rõ; không phải lỗi ghép |

## 4. Phát hiện nghiêm trọng (tệp lái, KHÔNG sửa, Thiên quyết)

`정지` có lúc **không dừng xe**. Lỗi này có từ trước và đi chung với `/nut` (`/v10` gọi đúng hàm đó).

**Đã xảy ra gì** [log, giả lập 06/11 03:56]:
- 03:56:50 bấm đi B. ~6 s sau bấm `정지`.
- SIGINT rơi đúng lúc `rclpy` lấy tin (`take_message`), nên `gui_duong.py` chết vì `RuntimeError: Unable to convert call argument to Python object` chứ không phải `KeyboardInterrupt`.
- `chay_co_ngat` chỉ bắt `KeyboardInterrupt`, nên không hủy đích FollowPath, không ghi `HUY`/`NGAT`, và thoát mã 1.
- `/nut` vẫn báo `DA_DUNG 201 ms`, ghi chuyến HET_DUONG.
- Nav2 log: không có lệnh hủy; "Reached the goal!" lúc 03:57:22, tức **xe chạy tiếp 26 s tới B**.

**Tần suất:** 1/7 lần `정지` trong giả lập. 6 lần còn lại đúng.

**`/v10` đã làm** (chỉ ở trang mới, `/nut` không đổi):
- Thoát mã không thuộc {0, 3, 4, 130}, hoặc mã 1 mà dòng cuối không phải `HET_DUONG` → 도움이 필요해요 với lý do `LOI_THOAT`.
- Dòng phụ ghi "Nav2 목표가 아직 살아 있을 수 있어요: 모터 스위치를 끄세요", và không cho bỏ qua bằng nút 정지.

**Đề xuất (cần sửa tệp lái hoặc đổi `/nut`):**
- (a) `gui_duong.py`: cờ "đã nhận tín hiệu" đặt trong handler; `chay_co_ngat` bắt mọi `Exception` khi cờ bật → `huy_khi_ngat()`.
- (b) `dung_chuyen` chỉ báo `DA_DUNG` khi thấy dòng `HUY_DICH_KHI_NGAT`; không thấy thì báo `KHONG CHAC - TAT MOTOR`. Việc này đổi kết quả của `/nut`.

Trong lúc chưa sửa: **dừng chắc chắn = công tắc motor**.

## 5. Cần quyết

1. Mục 4: sửa `gui_duong.py` và/hoặc `dung_chuyen`.
2. Ảnh bản đồ v3 trên Pi (`~/map_xuong_v3.png` + `.yaml`, có trong `du_lieu_0111/`) do Thiên chép. Thiếu thì `/v10` chỉ báo lỗi bản đồ, M2/M3 không chạy.
3. Biến mã teleop trong `~/cau_noi/cau_noi.env`: Thiên tự thêm, mình không ghi. Thiếu thì pad trả 503 nếu teleop_web có mã.
4. `footprint_padding` không có trong `nav2_params_cho.yaml`: trang dùng mặc định Nav2 0,01. Có ghi rõ vào tệp không? (tệp lái)
5. Ảnh LiDAR lúc HET_DUONG: không ai tự chụp `/scan`. Cần sếp, gui_duong hoặc `bat_xe.sh` ghi `~/scan_*.txt` (tệp lái), hoặc cho cầu nối gọi `ros2` (hiện cầu nối không cần ROS).
6. Vị trí xe chỉ lấy từ dòng nhật ký sếp có x, y (VONG 10 s/lần + sự kiện), nên biểu tượng xe nhảy tới 10 s. Thiết kế mục 3 ghi "sếp ghi JSON mỗi 1 s" → cần sửa `sep.py`.
7. `CHON_B_TU_DO` (tắt): chọn ô tuỳ ý cần `gui_duong` nhận đích tuỳ ý (tệp lái).
8. "Costmap xuất lúc bật": chưa có ai xuất. Tuyến xem trước tính trên bản đồ tĩnh nên có thể khác tuyến xe tính trên costmap sống. Trang có ghi "정적 지도 기준".
9. Vào 수동 조종 khi đang có chuyến thì không tự dừng chuyến: teleop 100 đè Nav2, nhưng thả nút 0,3 s sau Nav2 lái lại. Trang chỉ nhắc và hiện nút `주행 정지`. Có nên tự dừng chuyến không?
10. 방향 맞추는 중 chỉ suy từ phụ lái `XOAY_P`. RPP xoay tại chỗ không có log nào đọc được.
11. `so_voi_pi.sh` chưa thêm `trang_v10.py`/`.html` (tệp có chữ cấm in, mình không đụng).
12. Test cầu nối gọi `dung_chuyen()`, mà hàm này SIGINT **mọi** `gui_duong.py` trên máy: lúc thử nó đã ngắt chuyến của giả lập. Đừng chạy `test_cau_noi`/`test_trang_v10` khi xe hoặc giả lập đang chạy.
13. `SAI_VI_TRI`: `/v10` hiện 준비 완료 + "지난 주행: SAI_VI_TRI", không vào 도움이 필요해요. Đúng ý chưa?
14. Một test đọc log hơi sớm, hỏng 3/15 lần khi máy bận: do test (đua thời gian), đã sửa thành chờ dòng log; sau sửa đạt 10/10 + 3/3.

## 6. Chưa thử trên xe (tất cả)

- Toàn bộ `/v10` trên `Pi`, điện thoại thật, hotspot, systemd (`EnvironmentFile`, biến `NAV_PARAMS`/`SLAM`/mã).
- Đọc PNG thuần Python và thời gian dựng costmap trên Pi (PC < 0,1 s; `tinh_duong` lần đầu 0,5 s).
- Rung/âm trên điện thoại (trình duyệt có thể chặn âm khi chưa chạm).
- Pad ↔ teleop_web thật trên xe; trễ mạng với `cmd_timeout` 0,3 s.
- Trạng thái 탈출 중 / 대기 중 / 방향 맞추는 중 chỉ thử bằng log giả và log 05/11, chưa thấy trong giả lập (giả lập không có vật chắn).
- Giả lập không hiệu chuẩn: Nav2 thật nhưng xe và LiDAR là giả.
