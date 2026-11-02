# Thiết kế đã chốt cho việc 109, 110, 111 (chat việc 108, 02/11/2026 01:34 → 04:07, ở nhà; cập nhật 04:08)

> Bản trong kho của tệp Project `Thiet ke 109 111 (chot 02 11)` (chép nguyên văn 02/11 15:25). Tài liệu thiết kế do chat soạn; **chưa có mã**. Mã do Claude Code viết theo từng việc, mỗi việc một chat, kèm kiểm tích hợp (chỉ dẫn Project mục G). Số có nhãn nguồn: [đọc code] kho `dong-bo` `c311c3c`; [log] dữ liệu 1/11; [Thiên đo]; [tính]; [đề xuất].

## 0. Quyết định của Thiên trong chat này

| Giờ | Quyết định |
|---|---|
| 01:42 | Ngày 2/11 xe dùng `BANG=xs` (hiểu là cả `di_B` và `ve_A`). |
| 01:59 | Bỏ hẳn "lùi mù" (kể cả `BackUp` trong `bt_cho.xml`). Phải có "lùi có mắt" theo tư duy người lái ô tô, làm ngay 2/11. |
| 02:08 | Kẹt: so trước/sau, đưa xe ra phía thoáng (tiến hoặc lùi) rồi mới xoay; xoay về hướng thoáng gần nhất **trên tuyến đang đi**; bám tiếp tuyến đã vẽ; không chui lại đường cũ; ghi nhớ chỗ kẹt để không sai lại. |
| 02:13 | Xe bám **nguyên đường dày** của `tinh_tuyen.py` qua `/follow_path` (không để `NavFn` tự vạch giữa hai điểm), cả lúc đi thường lẫn sau khi thoát. Giữ đường lui bằng một cờ về 6 điểm `xs`. |
| 02:28–02:30 | **Chuỗi chỉ huy 3 người:** sếp quan sát, quyết, chỉ điểm nhập tuyến P; phụ lái (`lai_xe_lidar`) gỡ kẹt và xoay mũi về P; lái chính (Nav2/RPP) bám đường dày. Không ai tự vạch đường mới. Phản xạ an toàn của phụ lái không chờ sếp (#10). |
| 02:49–02:53 | Mục đích bật/tắt: một nút/lệnh bật **đủ đội**, một nút/lệnh tắt sạch; tắt `Pi` bằng tay. |
| 02:57 | Gộp lái tay và chọn B thành **một trang**; chọn B phải chạy được. |
| 03:43 | Vật chắn hành lang: khe dưới chắn ⇒ sang khe trên, khe trên chắn ⇒ về khe dưới; **tối đa 3 lượt** (một lượt = một lần vào một khe); lượt 3 vẫn chắn ⇒ chụp ảnh, báo. |
| 03:47 | Chốt thiết kế; ba điểm mở theo khuyến nghị của chat (mục 4). |
| 03:59 | Kênh "có đích" cho phụ lái và sếp khi đổi sang `/follow_path` (mục 1.9): làm ngày 2/11. |
| 04:06–04:07 | Công cụ (mục 7): `so_voi_pi.sh` thêm tệp thiếu + tự nhắc tệp mới; `ghi_bag.sh` ghi thêm "lời nói" của chuỗi chỉ huy; `tinh_tuyen.py` thành hàm chạy trên `Pi` với costmap của lần bật. |

## 1. Việc 109 (Core, G2, G3, G5): chuỗi chỉ huy + bám đường dày + lùi có mắt

1. **Bộ gửi đường dày:** tính tuyến (hàm của `tinh_tuyen.py`: Dijkstra 8 hướng trên costmap, cấm đỏ, không cắt góc đỏ, giá 1 + 60/d³) từ vị trí xe tới đích; gửi nguyên đường qua `/follow_path`. Cờ đường lui: 6 điểm `xs` qua `/navigate_to_pose` như cũ. Không còn khoảng đứng ≈ 7,5 s giữa các điểm [log 1/11].
2. **Sếp thấy kẹt** (định nghĩa đã chốt 65.4, có đích + dịch < 3 cm/10 s + LiDAR ≤ 10 cm; sửa lỗi lượt 20/3 = việc 105 trước) ⇒ ra lệnh `THOAT` kèm điểm nhập P (điểm trên tuyến phía trước) cho phụ lái. Sếp **không** phát tốc độ (luật H).
3. **Phụ lái thi hành** (`phat_lenh=true`, sau phép thử "chỉ nhìn" motor OFF sát K4): đo thoáng trước/sau theo bề rộng thân xe ⇒ đi ra phía thoáng hơn tới khi đủ chỗ xoay (mọi vật cách tâm > 18,55 cm [đọc code `R_XOAY`]) hoặc tối đa 30 cm [đề xuất]; xoay mũi về P, **kiểm thân xe từng bước**; báo xong ⇒ `twist_mux` trả bánh cho Nav2 (ưu tiên 60 > 20).
4. **Xoay tại chỗ:** RPP xoay khi lệch > 45° [đọc code] nhưng **không kiểm va chạm lúc xoay** ⇒ phụ lái chặn xoay (`CHAN_XOAY`); không đủ chỗ ⇒ vào thoát.
5. **Thứ tự xét mỗi vòng:** hướng (cần xoay?) trước, vật trước mũi sau.
6. **Vật lạ trước mũi** (người, hộp): Nav2 dừng ≈ 8 cm [Thiên đo 9/9 ≥ 6 cm]; sếp CHỜ ≤ 30 s (#9); vật đi ⇒ bám tiếp tuyến cũ. Vẫn chắn ⇒ sang khe kia, chỉ tránh vật đang chắn; tối đa 3 lượt; **đổi luồng xong mà vật còn ≤ 12 cm trước mũi ⇒ thoát trước, không chờ lại vật cũ**.
7. **Ghi nhớ:** sếp ghi mỗi chỗ kẹt/chắn (x, y, góc, loại) vào nhật ký; lần sau dùng để tránh hoặc sửa tuyến.
8. **Hết 3 lượt:** dừng, ảnh, báo người phụ trách, hiện nút lái tay (chính sách 5 bước, bước 5).
9. **Kênh "có đích" (phát hiện ở mảnh 6, bắt buộc vào bảng đối chiếu luật G):** `lai_xe_lidar.py` và `sep.py` hiện biết "có đích" qua `/navigate_to_pose/_action/status`; `sep.py` đọc đường từ `/plan`. Đổi sang `/follow_path` thì hai kênh này im ⇒ phụ lái và sếp mù. Phải cho cả hai nghe trạng thái `/follow_path` (và sếp lấy tuyến dày từ bộ gửi), giữ được cả đường lui `/navigate_to_pose`.
10. `logic_quyet_dinh.py` cũng phát `/cmd_vel_avoid` (không có trong launch): ghi chú để không ai chạy song song với `lai_xe_lidar` (Stretch: chuyển sang thư mục đóng băng).

## 2. Việc 110 (Core, G4, G5, luật H): một nút/lệnh bật đủ đội, một nút/lệnh tắt sạch

- `SAN SANG` = xe chạy được một chuyến như ngày chấm mà không cần người làm thêm bước nào: thân xe, Nav2 + `bt_cho` + đúng tệp tham số, xe ở A, **sếp chạy**, **phụ lái chạy** đúng mức, **vùng đệm đúng giá trị đã chốt (lấy từ tệp)**, tuyến của ngày có sẵn. Thiếu dòng nào ⇒ không báo `SAN SANG`, in tên dòng thiếu. Thêm "người" mới vào xe ⇒ thêm một dòng.
- `bat_xe.sh` tự đọc `~/cau_noi/cau_noi.env` ⇒ lệnh tay = nút bấm.
- Nút `연결 해제` gọi `bat_xe.sh tat`; cầu nối đọc mã thoát (`DA_DON` / `CON_ROS_SONG`) và hiện lên điện thoại. Tắt `Pi` vẫn làm tay.
- Gộp phần "tự bật sếp" của việc 103 và việc 102 (vùng đệm vào tệp) vào đây.

## 3. Việc 111 (Core, G4): một trang điều khiển (cổng 5001)

- Trang ở cầu nối (sống trước khi bật xe). Trạng thái: `연결 안 됨` → `연결 중` (kiểm đủ đội) → `준비 완료` → chạm ô xanh → `경로 확인` (vẽ tuyến dày, độ dài, khoảng cách tới đỏ) → `출발` → `이동 중` / `대기 중` / `탈출 중` / `방향 맞추는 중` → `도착`; `수동 조종` (pad ▲◀■▶▼ trong trang, chuyển tiếp sang `teleop_web`, ưu tiên 100); `도움이 필요해요` (ảnh + `수동 조종` / `다시 시도` / `정지`); `연결 해제`.
- Dòng `두뇌` hiện sếp đang nghĩ gì (sếp ghi tệp JSON mỗi 1 s; cầu nối đọc).
- **Đồng bộ với xe:** trang không giữ số riêng. Bản đồ = tệp `slam_dinh_vi_xuong.yaml` trỏ tới + costmap xuất lúc bật; vùng đỏ/vàng từ `footprint`, `inflation_radius` của `nav2_params_cho.yaml`; vị trí từ `tf` (qua sếp); tuyến dùng chung hàm `tinh_tuyen.py`. Thiếu tệp ⇒ báo lỗi, không dùng số dự phòng. Hiện cầu nối còn cố định `map_xuong_v2`, giao diện tự đặt 13/18,55 cm ⇒ phải sửa.
- Tâm xe trong vùng hồng (< 13 cm) ⇒ `두뇌` cảnh báo; `출발` thì thoát trước.

## 4. Ba điểm mở, chốt theo khuyến nghị (Thiên "chốt thiết kế" 03:47; sai thì sửa)

- (a) Ảnh khi báo = **ảnh LiDAR vẽ từ trên xuống** quanh xe (camera vẫn đóng băng theo quyết định 22/10).
- (b) Báo = **màn đỏ + rung/âm trên chính trang**; gửi ra ngoài là Stretch.
- (c) Thứ tự: **105 → 109 → 110 trong ngày 2/11; 111 ngay sau 109, mục tiêu buổi 10/11** (dùng lại bộ gửi đường của 109).

## 5. Bất ngờ phát hiện khi rà soát (mảnh 1–7)

1. `di_B` tuyến 1 của `cu`/`tt` chỉ có B ⇒ không ép khe; hai khe dài 2,05 / 2,02 m [tính].
2. `tt` `ve_A` điểm 1 nằm ô vàng, cách ô đỏ 9 cm; bảng `tt`/`cu` lập trên v2.
3. Đổi tuyến bắt đầu từ điểm 1 tuyến kia (lần 16:53: 166 s) [log].
4. Đổi tuyến khi đang kẹt vô ích: xe đứng ≈ 120 s sát K4 [log 17:32–17:34].
5. Mỗi điểm tốn ≈ 7,5 s đứng chờ `chuyen.sh` gửi điểm [log] ⇒ `xs` 6 điểm thêm ≈ 37 s/chuyến [tính].
6. RPP chặn khi **thân xe** sắp đè ô vật, không phải "sắp chạm đỏ" [suy luận ≈ 75 %].
7. Sếp mức B không xét kẹt, không xét vật lạ; `THOAT`/`DOI_HANH_LANG` không ai nhận [đọc code].
8. Đồng hồ "đứng yên 10 s" của sếp đặt lại giữa hai điểm [đọc code].
9. Bấm nút kết nối làm mất sếp và phụ lái; vùng đệm quay về 0,25 [đọc code].
10. Hai trang điều khiển, không trang nào đi được tới B; buông lái tay giữa chuyến thì Nav2 lái tiếp sau 0,3 s [đọc code].
11. Ba định nghĩa "đỏ" (costmap 8 cm, giao diện 13 cm, phụ lái 8 cm từ mép); `trang thai` ghi R = 25 cm nhưng mã 18,55 cm. Chọn B trên giao diện an toàn: 0 ô xanh giao diện rơi vào đỏ Nav2 [tính].
12. (mảnh 6) Phụ lái và sếp nghe "có đích" qua `/navigate_to_pose`, sếp đọc `/plan` ⇒ mù khi đổi sang `/follow_path` (đã ghi vào 109 mục 9).
13. (mảnh 6) `logic_quyet_dinh.py` cũng phát `/cmd_vel_avoid`; không có trong launch.
14. (mảnh 6) Mặc định launch (`bat_dong_co`, `bat_gyro`, `bat_mo_hinh_san` = false) khác cách xe chạy thật; chỉ dòng `ACCESS` của `bat_xe.sh` bật chúng ⇒ bật tay quên cờ là chạy cấu hình khác (việc 110 che).
15. (mảnh 7) `so_voi_pi.sh` **không so `sep.py`**, `slam_dinh_vi_xuong.yaml`, bản đồ ⇒ "9/9 GIONG" không chứng minh sếp trên `Pi` đúng bản kho. Sửa nhỏ (Core, G5) trước khi chép mã lên `Pi` ngày 2/11.
16. (mảnh 7) `ghi_bag.sh` không ghi `/sep/trang_thai`, `/cmd_vel_avoid`, `/cmd_vel_nav`, `/rosout` ⇒ bag không dựng lại được chuỗi chỉ huy; thêm khi làm 109. Đầu tệp vẫn ghi "CHUA CHAY TREN XE" dù đã có bag 31/10.
17. (mảnh 7) `tinh_tuyen.py` đọc cố định `du_lieu_0111/costmap_v3_020` và in dòng để chép tay vào `chuyen.sh` ⇒ 109/111 cần nó thành hàm gọi được trên `Pi` với costmap của lần bật. `mo_phong_he_thong.py`, `so_sanh_tuyen.py` là mô phỏng cũ, thừa sau khi có `gia_lap/` (để nguyên, ghi "cũ").

## 6. Bảng một trang: xe đang có gì (kho `c311c3c`, `Pi` còn bản cũ)

| # | Tính năng | Bật bằng / mặc định | Nói với ai | Đã thử | Hạn chế chính |
|---|---|---|---|---|---|
| 1 | `chuyen.sh` + bảng `cu`/`tt`/`xs` | `bash ~/chuyen.sh di_B\|ve_A`, `BANG=tt` mặc định, kiểm vị trí BẬT | `/navigate_to_pose`, `~/sep_lenh`, CSV, mã 0–4 | `tt` trên xe 1/11; `xs` chưa | Gửi từng điểm (≈ 7,5 s/điểm); đổi tuyến từ điểm 1; `tt` lập trên v2 |
| 2 | Nav2 (`NavFn` + RPP + `bt_cho`) | `NAV_PARAMS`; 0,25 trong tệp, 0,20 đặt tay | `/cmd_vel_nav` (20) | A→B 3/3; B→A khe dưới 0/3 | Chỉ "tiến hoặc đứng"; RPP không kiểm va chạm khi xoay |
| 3 | Sếp `sep.py` | tay, `ra_lenh` false (mức A) | 8 nguồn → nhật ký, `/sep/trang_thai`, `~/sep_lenh` | `Pi` bản cũ; mới 25/25 + giả lập | Không có "tay"; mức B không xét kẹt; lỗi lượt 20/3 |
| 4 | Bật/tắt `bat_xe.sh`, cầu nối | nút `차량 연결` / lệnh | systemd 5001 → `bat_xe.sh` | ≈ 55 s (n = 1); `tat` mới giả lập 2–3 s | Không bật sếp/phụ lái; không có nút tắt; bỏ qua mã `tat` |
| 5 | Lái tay `teleop_web` | cùng motor, cổng 5000 | `/cmd_vel_teleop` (100) | 1/11 vẽ bản đồ | Trang riêng; buông nút giữa chuyến thì Nav2 lái tiếp |
| 6 | Giao diện chọn B | cầu nối 5001 | chỉ `connect` | DEMO | 4/5 nút trả 501; bản đồ v2 |
| 7 | Phụ lái `lai_xe_lidar` | tay, `phat_lenh` false | `/cmd_vel_avoid` (60) | mô phỏng 26/26; chưa chạy xe | Chỉ ra tay ≤ 8 cm; không ai gọi; nghe kênh đích cũ |
| 8 | Camera, nhận diện người, `ket_detector`, `rf2o` | cờ launch false / ngoài launch | — | — | Đóng băng / Stretch |
| 9 | `tinh_tuyen.py` | PC | ra 4 dòng chép tay | chạy PC | Không chạy trực tuyến |
| 10 | Giả lập `gia_lap/` | PC, domain 87 | Nav2/SLAM thật, xe giả | bắt lỗi `tat`, lỗi lượt | Chưa hiệu chuẩn ⇒ chỉ bắt lỗi ghép |
| 11 | `so_voi_pi.sh`, `ghi_bag.sh`, `kiem_xe.sh` | PC / `Pi` | ssh md5 / bag / chỉ đọc | đã chạy | Thiếu `sep.py`; bag thiếu kênh chuỗi chỉ huy |

## 7. Công cụ (Thiên đồng ý 02/11 04:06–04:07)

| Công cụ | Vấn đề [đọc code] | Cách xử lý | Loại, cổng | Khi nào | Đường lui |
|---|---|---|---|---|---|
| `so_voi_pi.sh` | Danh sách 19 tệp thiếu `sep.py`, `slam_dinh_vi_xuong.yaml`, bản đồ ⇒ "GIONG" không chứng minh sếp trên `Pi` đúng bản kho | (1) Thêm `sep.py`, `tinh_tuyen.py` (khi lên `Pi`), `slam_dinh_vi_xuong.yaml`, `map_xuong_v3_pg.*` (so với `du_lieu_0111/`); dòng tổng `GIONG x/y`. (2) **Tự nhắc:** lướt `xe_scripts/`, tệp `.py/.sh/.yaml/.xml` chưa có trong danh sách ⇒ in `CHUA_CO_TRONG_DANH_SACH <tên>`. (3) Luật G thêm dòng đối chiếu "tệp mới đã vào danh sách `so_voi_pi.sh` chưa?" | Core, G5 | **Đầu tiên sáng 2/11**, trước khi chép mã lên `Pi` (≈ 10 phút) | Chỉ đọc; bỏ dòng mới |
| `ghi_bag.sh` | Không ghi "lời nói" của chuỗi chỉ huy | Thêm `/sep/trang_thai`, `/cmd_vel_nav`, `/cmd_vel_avoid`, `/cmd_vel_teleop`, `/rosout`, trạng thái `/follow_path` và `/navigate_to_pose`; sửa dòng đầu (đã chạy 31/10) | Core, G2 + G5 | Cùng 109, sau khi `/follow_path` có thật | Bản cũ trong git |
| `tinh_tuyen.py` | Chỉ chạy PC, in dòng để chép tay; đọc cố định `costmap_v3_020` | Tách hàm `tinh(costmap, điểm_đầu, đích) → đường dày`, giữ chế độ in cho PC. Costmap **xuất từ Nav2 lúc bật** (`map_saver_cli -t /global_costmap/costmap ... save_map_timeout:=10.0`, có `timeout`) ⇒ `~/costmap_lan_bat.*`, đặt trong `bat_xe.sh` trước `SAN SANG`; lỗi ⇒ `SAN SANG` báo thiếu dòng "tuyến" (định nghĩa việc 110). Không tự tính lại vùng đỏ (một nguồn, bài học 331) | Core, G2 + G4 | Phần đầu của 109 | Cờ về 6 điểm `xs` |
| `mo_phong_he_thong.py`, `so_sanh_tuyen.py` | Mô phỏng cũ, thừa sau `gia_lap/` | Ghi "CŨ" ở dòng đầu hoặc chuyển `cu/`; không xoá | Stretch | Khi rảnh | — |

Rủi ro: lệnh `ros2` trên `Pi` hay treo (`trang thai` mục 3) ⇒ bước xuất costmap phải có `timeout`.
