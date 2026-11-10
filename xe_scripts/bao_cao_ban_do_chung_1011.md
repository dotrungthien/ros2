# Bản đồ chung giữa sếp và Nav2 (Thiên duyệt 10/11 17:59)

Nhánh `bando-chung`, tách từ 1d175aa. Chưa chạy trên xe. Mọi kết quả trong tài liệu này là **đạt giả lập**:

- tường lấy từ `map_xuong_v3`;
- LiDAR giả 420 tia;
- Nav2 giả dùng costmap chung (tường 251 + đệm mềm, LiDAR 254 + inflation, lớp `vong_vat`), NavFn giả vạch lại 1 Hz.

## Gốc lỗi đã sửa

Trước đây Nav2 lập đường trên bản đồ có tường 251 không đệm, còn vật chỉ phình 0,08. Ống O11 đặt giữa khe dưới chừa mỗi bên 19 cm, nên Nav2 thấy còn qua được: khe mở 0,05 m. Sếp thì cần 16 + 6 + 6 = 28 cm nên chặn. Kết quả là lặp lại vòng "Nav2 lái vào, sếp chặn".

Bây giờ sếp gửi cho Nav2 một vòng an toàn quanh mỗi VẬT CHẮC. Với vòng này, khe dưới đóng (0,00 m) ngay trên costmap của Nav2 (test_04, test_11).

## 0. Hộp đen

- **Ghi khi nào:** ở mỗi sự kiện và mỗi 2 s khi xe đang đi. Các sự kiện: DOI_VAT, CHO_VAT, KHONG_TIEN, THOAT*, DIEM_*, KHE, Luot bi chan, CHO_NGUOI, DEN_NOI, HET_DUONG.
- **Ghi vào đâu:** `hop_den.py` ghi `~/hop_den/<mmdd_HHMMSS>_<huong>/<HHMMSS_mmm>_<su_kien>.npz`.
- **Nội dung mỗi tệp:**
  - global và local costmap, mỗi cái kèm mảng và `[w, h, res, ox, oy]`;
  - biến đổi map←odom, đường `/plan`, tư thế xe;
  - các điểm quét đã đổi sang khung map, kèm nhãn cụm: tuong / ong / vat / khac;
  - các vòng an toàn;
  - trạng thái sếp (`/sep/trang_thai`) và dòng log.
- **Dung lượng:** trần 20 MB mỗi chuyến (`hop_den_mb`). Vượt trần thì chính chương trình xoá tệp cũ nhất của chuyến đó. Lỗi ghi được bỏ qua, không làm hỏng chuyến.
- **CPU:** đo 1,1–1,2 ms mỗi tệp trên PC, tức khoảng 0,06 % một nhân khi ghi mỗi 2 s (dưới 3 %).
- **Vẽ trên PC:** `python3 ve_hop_den.py ~/hop_den/<chuyen> [thu_muc_ra]`.
  - Mỗi tệp ra một PNG: costmap, ô local ≥ 99, `/plan`, thân xe 25×16 kèm mũi, điểm tô màu theo nhãn, vòng nét đứt.
  - Thêm `_chuyen.png`: vết xe thật cùng các sự kiện.
- **Giới hạn:** điểm quét có nhãn cho mọi cụm trong 2,5 m, tính 2 lần/s cùng lúc với vòng. Tệp được ghi giữa hai lần tính thì dùng nhãn của lần gần nhất.

## 1. Vòng an toàn trong costmap của Nav2

**Cách tính vòng:**

- Mỗi cụm trong 2,5 m được phân loại bằng ba dấu hiệu. `TheoDoiOng` dùng riêng, không đếm chung với `danh_gia`.
- Cụm nhận **ong** hoặc **ngoai** (ngoài bản đồ) thì có vòng. Bán kính = bán kính cụm + 0,06 + 0,01.
  - Tâm vòng lấy theo khớp tròn Kasa, vì trọng tâm của nửa cung bị lệch về phía LiDAR.
  - Cụm dính vào tường thì vòng chỉ bao phần ngoài bản đồ, không ôm tường.
- Tường không có vòng.
- Giữ vòng 3 s sau lần thấy cuối (`GiuVong`).

**Đường đi của vòng:**

- gui_duong gửi `/vong_vat_ds` (String) tối đa 5 lần/s.
- `lop_huong_dan.py` giữ tiếp 3 s rồi phát `/vong_vat` (OccupancyGrid cùng hình học với `/map_tinh`, giá trị 0 hoặc 100). Mỗi lần phát thay toàn bộ lưới. Lưới 0 được phát ngay khi node bật.
- Nếu gui_duong chết, vòng tự hết sau 3 s.

**Trong Nav2:** thêm StaticLayer `vong_vat` vào cả global và local costmap. Giá trị 100 thành 254, rồi inflation nửa thân 0,08 ⇒ tâm xe cách mép vật ≥ 0,15, tức thân cách vật ≥ 0,07.

**Vì sao chọn StaticLayer trên topic động, không dùng KeepoutFilter:**

- Đây là cùng cơ chế với `/lop_huong_dan` đang chạy trên xe (Humble 1.1.20).
- Mỗi tin mới thay toàn bộ lớp.
- Tia scan không xoá được lớp này. `ClearEntireCostmap` chỉ reset cờ của StaticLayer, không xoá dữ liệu.
- KeepoutFilter cần thêm `costmap_filter_info_server`, lifecycle và topic mask, nhiều node hơn mà ra cùng kết quả.

**Ở local costmap** phải thêm `use_maximum: true` và `trinary_costmap: false` để lớp vòng chỉ nâng chi phí, không ghi đè ô của LiDAR.

**CPU:** `danh_gia` 2,73 ms mỗi lần quét. `vong_quet` 2,04 ms mỗi lần, chạy 2 lần/s.

- Ở 10 lần quét/s, trung bình thêm 0,41 ms mỗi lần quét, tức +15 % phần gác.
- Tính tuyệt đối là khoảng 4 ms mỗi giây.
- Để giữ nhẹ, cụm nằm trên tường bản đồ (ít hơn 2 điểm ngoài bản đồ) được bỏ qua mà không chạy khớp tròn.

## 2. Tường đệm mềm: bảng D(4)

Bảng đo trên giả lập (test_05):

- Cột khe là chiều rộng thông nhỏ nhất của khe với Nav2, tức các ô < 253.
- A, B là giá trị ô costmap tại đó (0 = trống).
- Khe dưới: x 0,61–1,16, giữa K4 hoặc tường dưới và H0. Khe trên: x 0,81–1,16, giữa H0 và K2.
- Bản đồ lệch so với tường thật: LiDAR thấy tường thật lệch so với `/map_tinh`.

| Cách tường | lệch | khe dưới | khe trên | A | B | đường A→B |
|---|---|---|---|---|---|---|
| giữ 251 (đang dùng) | 0 | 0,40 | 0,35 | 0 | 0 | có |
| giữ 251 | y ±0,08 | 0,35 / 0,40 | 0,30 | 0 | 0 | có |
| giữ 251 | x ±0,08 | 0,40 | 0,35 | 0 | 0 | có |
| lethal + inflation 0,30 / k 4 | 0 | 0,40 | 0,35 | 0 | 0 | có |
| lethal + inflation 0,30 / k 4 | y ±0,08 | **0,30** | **0,25** | 0 | 0 | có |
| lethal + inflation 0,30 / k 4 | x ±0,08 | 0,40 | 0,35 | 0 | 0 | có |
| **251 + đệm mềm (chọn)** | 0 | 0,40 | 0,35 | 0 | 0 | có |
| 251 + đệm mềm | y ±0,08 | 0,35 / 0,40 | 0,30 | 0 | 0 | có |
| 251 + đệm mềm | x ±0,08 | 0,40 | 0,35 | 0 | 0 | có |

Tâm đường Nav2 cách tường ở trong khe, khi bản đồ lệch y 0,08: giữ 251 còn 0,125 m ở khe trên; đệm mềm còn 0,175–0,225 m; lethal 0,225 m.

**Chọn đệm mềm** (`tuong_dem`, do `lop_huong_dan` cộng vào `/lop_huong_dan`):

- Ô cách tường d ≤ 0,08 có giá trị 98, tức chi phí 248, dưới 253.
- Ô có d ≤ 0,30 có giá trị 98·e^(−6(d−0,08)).

Lý do:

- Bản đồ không bao giờ đóng khe. Chỉ LiDAR thật và vòng vật mới chặn, nên xe vẫn đi sát được khi buộc phải.
- Đường Nav2 vẫn nằm giữa khe khi có chỗ.
- Cách lethal làm hẹp hai khe thêm 0,10 m khi lệch 0,08. Ngoài ra, khi xe sát tường mà vị trí lệch, ô xe có thể rơi vào vùng 253 của bản đồ, và NavFn chỉ dọn mỗi ô xe.
- Không cần tệp bản đồ mới: `map_xuong_v3_tinh.yaml` vẫn giữ tường 99, tức 251.

Không ca nào trong bảng làm đóng khe.

Phần "cửa K2" và "miệng H0" chưa được đo riêng:

- Trên bản đồ v3, lối vào khe trên phía trái (0,45–0,57; 0,81–0,85) rộng hơn 0,60 m, nên không phải chỗ hẹp. Ô tại (0,51; 0,83) bằng 0 ở mọi ca.
- Miệng H0 nằm trong đoạn đo của khe dưới.

## 3. Giữ khe mỗi lượt (`giu_khe`)

**Khi bắt đầu mỗi lượt** — lượt đầu, hoặc sau một lượt bị chặn — xe xét hai việc: xe và đích có ở hai phía khe không, và vị trí có tin được không. Nếu có cả hai:

1. Xe chọn khe thông theo costmap chung (`ban_do_chung.chon_khe`, ô < 99).
   - Ưu tiên theo tuyến: tuyến 1 = khe dưới.
   - Sau một lượt hỏng, khe kia được thử trước.
2. Xe in `KHE: chon <khe> (thong .. m theo costmap chung), diem dan (x; y), ...`.
3. Xe gửi `/chon_tuyen` của khe đó (khe kia có giá 251: tốn nhưng không chặn).
4. Xe đi tới điểm dẫn ở giữa khe (`DIEM_DAN`, hạn 20 s), rồi gửi đích.

**Giữ khe:** khe được giữ cho tới khi một lượt hỏng.

**Khi khe đã chọn đóng giữa chừng** (costmap thông 0):

- `DIEM_DAN_XONG: ... => HONG (khe <ten> dong voi Nav2)`, sau đó `DIEM_DAN: bo, di thang dich (... khong tinh luot)`;
- `/chon_tuyen` trở về "0" và Nav2 tự vạch.

**Khi cả hai khe đóng:** `KHE: ca hai khe dong voi Nav2`, Nav2 tự vạch.

`ve_A_diem_dan` cố định đã tắt. Mã vẫn còn, `true` là quay về cách cũ.

## 4. Đơn giản hoá

- `diem_vuot` đặt false.
- Giữ nguyên: phân loại A++, nhắc đợi ống 30 s, thoát, vùng đỏ.

## 5. Đường lui

**Nav2:**

- Bật bản mới bằng dòng `NAV_PARAMS=/home/project1/nav2_params_mat_rpp_chung.yaml` trong `~/cau_noi/cau_noi.env`. `bat_xe.sh` đã đọc dòng này từ trước.
- Về bản cũ bằng cách đổi dòng đó thành `NAV_PARAMS=/home/project1/nav2_params_mat_rpp.yaml`. Tệp cũ không bị sửa.
- Tệp mới chỉ dùng được với `MAT_LAI=true`, vì cần `lop_huong_dan.py` phát `/vong_vat`.

**Hành vi** (`hanh_vi_xe.yaml`):

- `vong_vat: false`: không gửi vòng (lưới `/vong_vat` luôn 0).
- `giu_khe: false`.
- `tuong_dem: false`.
- `ve_A_diem_dan: true` + `diem_vuot: true` + ba khoá trên false = 1d175aa.
- `hop_den: false`: tắt hộp đen.

**Test cũ:** được ghim theo đường lui (`CFG` / `CFG3` có `giu_khe=False`; IOThe không có `gui_vong`) và vẫn đạt.

## 6. Test (đạt giả lập)

`test_ban_do_chung.py`: 13/13.

| Ca | Kết quả |
|---|---|
| 17:48: ống O11 giữa khe dưới (0,68; 0,085), di_B | KHE chọn trên ngay ⇒ DEN_NOI; không lượt bị chặn, không CHO_NGUOI, một dòng KHE (không lật); thân cách ống ≥ 0,06 |
| Đối chứng tắt vòng | đường Nav2 đầu tiên đi qua khe dưới ⇒ DOI_VAT + lượt 1 bị chặn (gốc cũ) |
| 17:08: ống r 0,075 áp H0, khe 35 cm | KHE chọn dưới (thông 0,15) ⇒ qua khe dưới ⇒ DEN_NOI; thân cách ống ≥ 0,06 |
| Không ống | đi giữa khe: tâm cách mép K4 ≥ 0,248, cách H0 ≥ 0,226 |
| Ống trước mũi 0,09 giữa sàn | vẫn DOI_VAT vật chắc, CHO_VAT 5/30…; thân cách ống ≥ 0,06 (xem thêm bên dưới) |
| ve_A từ B, ống giữa khe dưới bị H0 che | lượt đầu chọn dưới; lại gần thấy ống ⇒ "khe duoi dong voi Nav2" ⇒ bỏ điểm dẫn, không tính lượt ⇒ DEN_NOI |

Với ca ống trước mũi: sau 30 s, thoát xoay bị huỷ 3 lần (trước 0,08 ≤ 0,10) ⇒ CHO_NGUOI. Khi tắt cả hai tính năng mới, ca này cũng ra CHO_NGUOI y hệt, nên đây là giới hạn có sẵn của bước thoát trong giả lập, không do việc này gây ra.

**Hồi quy:** gui_duong 68/68, phan_loai_cum 22/22, lop_huong_dan 10/10, sep 48/48, lai_xe_lidar 97/97, pytest (gac_vat_la, sep_lidar, hanh_vi) 27, trang OK, cau_noi OK, bat_kiem 11/11. kiem_hanh_vi: DAT (47 dòng).

## D(1): ai đọc các thứ mới

- `/v10` (`trang_v10.chuyen_cuoi_ct`): `KHE: chon ...` hiện là đang đi (DI). Các dòng `KHE: ca hai khe dong` và `DIEM_DAN: bo` giữ trạng thái trước đó. `DIEM_DAN: (` vẫn là DI như cũ.
- `~/kinh_nghiem.csv`: không có dòng mới, không thêm cột.
- cau_noi: không đọc `KHE:`, `/vong_vat` hay hộp đen.
- `kiem_xe.sh`: lệnh `lop_huong_dan --kiem-costmap` không đổi. Lớp `vong_vat` dùng cùng hình học nên vẫn khớp.
- `so_voi_pi.sh`: thêm ban_do_chung.py, hop_den.py, nav2_params_mat_rpp_chung.yaml.

## D(5): ảnh hưởng tới từng tính năng

| Tính năng | Không đổi / đổi | Rủi ro |
|---|---|---|
| A→B | đổi: có điểm dẫn giữa khe theo costmap; vòng đóng khe có ống | vòng sai (cụm ngoài bản đồ do bản đồ lệch) có thể đóng nhầm khe trong 3 s; còn khe kia, và Nav2 tự vạch |
| B→A | đổi, như trên; điểm dẫn K4 cố định đã tắt | như trên |
| Đổi khe | đổi: chọn đầu lượt, giữ tới khi lượt hỏng; khe đóng giữa chừng thì bỏ điểm dẫn, không tính lượt | khe chỉ đóng khi lại gần (vật bị che) thì đi vòng thêm một đoạn |
| Dừng trước ống | không đổi (vùng đỏ, nhắc đợi 30 s) | — |
| Hông | không đổi (VẬT 0,06, TƯỜNG 0,02) | — |
| Thoát | không đổi | ca ống 0,09 trước mũi giữa sàn vẫn CHO_NGUOI như cũ (giả lập) |
| Lượt / gần đích | không đổi; điểm dẫn không tính lượt và không tính vào hạn gần đích (K2, K3 như cũ) | — |
| Vị trí không tin | không chọn khe, không có điểm dẫn (`tin_sep`); vòng vẫn gửi theo TF lúc quét | vị trí sai thì vòng lệch theo, giống LiDAR trong costmap |
| Bật xe | Nav2 bản mới cần `/vong_vat`; `lop_huong_dan` phát lưới 0 ngay khi bật | dùng tệp mới mà `MAT_LAI=false` (không có `lop_huong_dan`) ⇒ planner chờ lớp mãi; khi đó về `NAV_PARAMS` cũ |
| Trang | thêm KHE = DI | — |
| Sếp quan sát | không đổi (`/sep/trang_thai` chỉ được ghi vào hộp đen) | — |
| Kinh nghiệm | không đổi | — |
| CPU | vòng: khoảng 4 ms/s (+15 % phần gác ở 10 lần quét/s); hộp đen 0,06 %; `lop_huong_dan` vẽ lưới 52×32 khi vòng đổi | chưa đo trên Pi |
| Đường lui | một dòng env (Nav2) + khoá trong yaml | — |

## Chưa làm / giới hạn

- **Chưa chạy trên Pi.** Cần thấy trên xe ba điều: `/vong_vat` vào được local costmap (StaticLayer ở rolling window cần TF map→odom); RPP `use_collision_detection` có dừng trước vòng hay không; CPU thật.
- **Nav2 giả không phải Nav2 thật:** NavFn giả, bám đường kiểu RPP, collision_monitor chỉ xét mũi.
- **Cửa K2 / miệng H0** không có mặt cắt hẹp riêng trên bản đồ v3 (xem mục 2).
- **Nhãn trong hộp đen** được tính 2 lần/s chứ không phải mỗi lần quét.
