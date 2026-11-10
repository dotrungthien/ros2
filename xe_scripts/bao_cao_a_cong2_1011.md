# A++ + mục 6 + mục 7 + sửa đặc tả 8 (Thiên 10/11 16:3x) — nhánh nao-xe-vung, CHƯA chạy trên Pi; giả lập là cành phụ.
Mốc 1: c703fe3 (A++ mục 1–5 + A1). Mốc 2: commit này (mục 6, 7, K1–K3, test).

## A++ (nhận ống chặt hơn; mũi trên bản đồ)
- gac_vat_la.ba_dau_hieu, khi `ong_kiem_tam` bật:
  - Góc cung thấy được phải ≥ `ong_goc_min_deg` 100°. Ống Ø18 thấy 140–170°; nhiễu trên tường hay góc chỉ cho cung 30–60°.
  - Tâm cung, khớp lại trên điểm bản đồ, không được sâu trong ô tường quá `ong_tam_sau_m` (`BanDo.do_sau`).
- `ong_on_dinh` (`TheoDoiOng`, Mat giữ một bộ cho cả chuyến): chỉ gọi "ong" khi trong 1 s có ≥ 3 lần quét thấy cụm đó (cách ≤ 0,05 m), và ≥ 60 % số lần ra cung.
  - Log: `(ong: cung r=0.09 rms=0.002, on dinh 10/10)`.
  - Cung bị loại ghi vào lý do: `bo cung r=.. chi 52 do` / `chua on dinh 1/4` / `tam sau 0.06 trong tuong`.
- `mui_tuong_khong_doi`: cụm ở mũi, trên bản đồ, không phải ống ⇒ TƯỜNG ⇒ không CHO_VAT / DOI_VAT 30 s. Mũi 0,07 vẫn là việc của collision_monitor.
- Đường lui: ba khoá = false cho kết quả giống hệt 50e4146. Test_14 chạy 40 cảnh (bản đồ / góc K4 bo tròn / ống, nhiễu 3 mm), so JSON giữa mã git archive 50e4146 và mã mới.
### A1 — ngưỡng "tâm cung sâu trong tường" (đo từ dựng lại, nhiễu ±5 mm, 20 lần mỗi tư thế)
- (i) Cung khớp nhầm ở góc K4 (bo tròn r 0,06, tư thế 16:27 / 15:18 lùi tới mũi 0,095): độ sâu tâm
  `[0×10, .001, .016, .021, .036, .044]`, `[0×10, .010, .013, .015, .023, .029]`, `[0×5, .023, .049]`, `[0×4, .019, .026]`.
  Góc K4 vuông: 0–0,048.
- (ii) Ống áp tường thật:
  - Áp tường y −0,365: 0,035–0,045.
  - Áp K4: 0.
  - Mô phỏng bản đồ lệch 8 / 12 cm: cách dựng của tôi SAI (dịch khung lẫn lộn, cho kết quả y hệt lệch 0) ⇒ CHƯA đo được. Ghi là chưa làm.
- **Hai nhóm CHỒNG nhau** (cung nhầm phần lớn có tâm ở ô trống, độ sâu 0; ống thật tới 0,045) ⇒ không có ngưỡng nằm giữa.
  - Chọn 0,05: trên mọi ống thật đo được, dư 0,005. Ngưỡng cũ 0,03 sẽ loại nhầm ống áp tường y −0,365.
  - Cái chặn cung nhầm thực sự là góc cung ≥ 100° và tính ổn định: test_10 cho 0 lần "ong" trên 4 × 38 lần quét; đường lui (50e4146) cho [2, 0, 1, 1].
## Mục 6 — điểm dẫn ve_A (`ve_A_diem_dan`, mặc định true)
- Tính từ bản đồ (`diem_dan_ve_A`): đoạn x 0,45–0,90, khe K4 (mép dưới) – H0 (mép trên).
  - **Điểm dẫn (0,675; 0,085)**, hướng −173° (về A), **cách mép trên K4 0,245 m, cách H0 0,245 m** (bản đồ v3).
- Dùng khi: ve_A, lượt 1 tuyến 1, xe còn ở phía B so với điểm dẫn (x > 0,775). Log: `DIEM_DAN: (0.68; 0.09) huong -173 do, cach K4 0.25 m, cach H0 0.25 m (ban do).`
- Bị chặn (vùng đỏ / Nav2 từ chối hay huỷ / quá 20 s) ⇒ `DIEM_DAN: bo, di thang A (Nav2 tu vach, khong tinh luot).` Không thoát, không tính lượt (K2), không tính vào hạn gần đích (K3).
## Mục 7 + K1 — điểm vượt (`diem_vuot`, mặc định true)
- Sau một lượt bị chặn tại P và THOAT xong (không KEP / HAN_GAN), `di_vuot` làm như sau:
  - Lấy LiDAR tại chỗ (phân loại: cụm TƯỜNG ⇒ tường; ống / vật chắc / ngoài bản đồ ⇒ vật) cộng ô tường bản đồ trong 1 m.
  - Ứng viên V phía trước P theo hướng tới đích, lệch 0 / ±20 / ±40 / ±60°, cách P 0,35 / 0,475 / 0,60 m, nằm ở ô trống.
  - Điều kiện đo theo **THÂN xe 25×16 tại V** (quay về đích):
    - Bậc 1: vật ≥ 0,12, tường ≥ 0,05.
    - Không có thì bậc 2: vật ≥ 0,08, tường ≥ 0,04.
  - Chọn: bậc nhỏ trước, rồi khe trên đường thẳng xe → V lớn (chỉ xếp hạng), rồi gần đích.
  - Log: `DIEM_VUOT: P (x; y) => V (x; y), phia trai|phai|thang, ly do bac 1, than cach vat .. / tuong .., duong cach them ..`.
  - Đi tới V với hạn 25 s. Không có V, hoặc tới V hỏng ⇒ bỏ, gửi đích như cũ. Không tính lượt (K2) và không tính vào hạn gần (K3).
- Mục 6e: điểm dẫn đang dùng mà vẫn bị chặn ⇒ lượt bị chặn ⇒ điểm vượt, như mọi chỗ khác.
## Test (đạt giả lập), test_phan_loai_cum.py 19/19
- **Mục 6** (test_15): ve_A từ B có điểm dẫn, cả ba ca đều DEN_NOI và không CHO_NGUOI:
  - không ống: tới điểm dẫn;
  - ống hông 23 cm: tới điểm dẫn;
  - ống hông 17,5 cm: điểm dẫn bị bỏ ("DIEM_DAN: bo"), xe vẫn đi thẳng A.
- **Mục 7 (i)** (test_16): góc K4 ca 16:27, tắt điểm dẫn ⇒ DEN_NOI, không CHO_NGUOI. **Nhưng giả lập KHÔNG bị chặn** (Nav2 giả ôm CACH 0,12 vẫn qua), nên điểm vượt không chạy. Bằng chứng yếu.
- **(iii)** (test_17): hộp 30 cm chặn giữa khe K4–H0. Điểm vượt chọn V ở SAU khe (V hợp lệ tại chỗ V); đi tới V bị huỷ ("HONG (truoc vat 0.10 <= 0.10)") ⇒ bỏ ⇒ Nav2 tự vạch ⇒ DEN_NOI. Không treo.
- **K1** (test_18): ống tạo khe 30 cm với K4. Có V hợp lệ nhưng V nằm SAU khe và là bậc 1, không phải bậc 2 trong khe. Chưa chạy xe qua khe trong giả lập.
- **(iv)**: điểm dẫn / điểm vượt tắt (CFG, CFG3 của test cũ) ⇒ test_gui_duong 68/68 và 15 test A+/A++ đạt như trước mục 6/7.
- Hồi quy: sep_lidar + gac_vat_la + hanh_vi 27, lai_xe_lidar 97, lop_huong_dan 10, sep 48, trang 61, cau_noi OK, bat_kiem 11. kiem_hanh_vi DAT 47 dòng.
## K4 — mọi lớp có thể dừng xe / làm Nav2 huỷ đích khi mũi gần tường (góc K4, mũi 0,08–0,10). Chỉ báo, không đổi tham số Nav2.
| Lớp | Ngưỡng (tệp) | Ở mũi 0,08–0,10 trước tường |
|---|---|---|
| collision_monitor ThanTruoc | polygon = footprint + 0,07 mũi; approach, time_before_collision 1,5 s, ≥ 2 điểm (nav2_params_mat*.yaml) | **Hạn chế ẩn**: approach cho phép v = (khoảng − 0,07)/1,5 ⇒ ở 0,10 còn 0,02 m/s < 0,092 m/s, bánh không quay ⇒ xe ĐỨNG mọi lần tiến khi mũi ≤ ≈ 0,07 + 0,092×1,5 ≈ 0,21 m về phía tường. Xoay cũng mô phỏng (tốc độ góc). |
| RPP use_collision_detection | footprint (padding 0) chạm ô 254 trong 1,0 s quỹ đạo | đường đâm vào góc ⇒ "collision ahead" ⇒ FollowPath hỏng ⇒ BT: xoá costmap, BackUp 0,25 |
| RPP điều tốc theo chi phí | regulated_linear_scaling_min_speed 0,06 < 0,092 | gần tường chi phí cao ⇒ tốc độ dưới ngưỡng bánh ⇒ không tiến ⇒ khong_tien 10 s |
| BT FollowPath recovery | BackUp 0,25 / 0,10 m/s (bt_mat.xml) | lùi khi controller hỏng |
| behavior_server BackUp / DriveOnHeading | simulate_ahead_time 0,7 ⇒ nhìn 0,06 m ở 0,10 m/s; footprint chạm 254 | thoát "tien" về phía tường ở 0,08 ⇒ Collision Ahead ⇒ ABORTED |
| Spin | quét footprint theo simulate 0,7 s (≈ 24° ở 0,6 rad/s) | góc thân quét vào tường ⇒ ABORTED |
| NavFn | ô ≤ 0,08 quanh ô LiDAR = 253 | không vạch tâm vào ≤ 0,08 tường LiDAR; tường bản đồ 251 không phình ⇒ đường ôm góc |
| gac_vat_la (gui_duong) | đỏ trước VẬT ≤ 0,10 (hành lang); hông VẬT ≤ 0,06; hông TƯỜNG ≤ 0,02 | góc K4 = TƯỜNG (A+/A++) ⇒ không đỏ |
| sep_lidar huỷ / chấm | trước mọi vật ≤ 0,07 khi đang giảm; vật 0,06 / tường 0,02 mọi mẫu (thoat_ra_xa) | thoát "tien" về phía tường bị cấm khi làm mũi ≤ 0,07 |
| DOI_VAT / CHO_VAT 30 s | vùng đỏ VẬT hoặc vật chắc ≤ 0,14 | ở góc K4: không còn (A++) |
## D(1)
- Chữ log mới: `DIEM_DAN: ...`, `   DIEM_DAN_XONG: (x; y) => TOI|...`, `DIEM_VUOT: ...`, `   DIEM_VUOT_XONG: ...`; lý do MEM thêm `on dinh k/n`, `bo cung ...`.
  - trang_v10: `_RE_MEM` vẫn khớp (phần lý do sau ":" tự do). `chuyen_cuoi_ct` coi DIEM_* là dòng không nghĩa (giữ trạng thái trước). cau_noi không bắt.
  - Chưa sửa trang để hiện DIEM_VUOT riêng (không cần).
- Khoá mới (hanh_vi_xe.yaml thoi_quen):
  - ong_kiem_tam, ong_on_dinh, mui_tuong_khong_doi, ong_tam_sau_m 0,05, ong_cach_cum_m, ong_cua_so_s, ong_ti_le, ong_so_quet_min, ong_goc_min_deg 100;
  - ve_A_diem_dan, diem_dan_cach_k4_m, diem_dan_han_s, diem_vuot, diem_vuot_han_s, diem_vuot_d_min/max_m, vuot_vat1/2_m, vuot_tuong1/2_m.
- /sep/trang_thai không đổi.
## D(4) (luật mới)
| Chỗ | Lớp còn chặn | Lối ra hướng dẫn |
|---|---|---|
| Cửa K2 | collision_monitor (đứng khi mũi ≤ ≈0,21 trước tường lúc tiến), RPP chạm, BackUp/Spin footprint | điểm vượt sau lượt chặn |
| Khe K4–H0 | như trên; góc K4 = TƯỜNG (không DOI_VAT); ống = VẬT (0,06) | điểm dẫn ve_A (0,675; 0,085), điểm vượt |
| Miệng H0 | như trên | điểm vượt |
| Quanh A / B | như trên; xoay: chỉ cấm khi chạm (vật 0,06 / tường 0,02) suốt vòng quét | — |
## CHƯA LÀM / yếu
- A1 (ii): ống áp tường khi bản đồ lệch 8 / 12 cm — chưa đo đúng.
- Mục 7 (i): giả lập không tái hiện kẹt ở góc K4, nên chưa chứng minh điểm vượt đưa xe qua.
- K1: chưa có ca V bậc 2 thật sự (V sau khe là bậc 1); chưa chạy xe qua khe 30 cm.
- collision_monitor approach + bánh chết dưới 0,092 là khoá ẩn ở mọi chỗ mũi gần tường (≈ 0,21 m). Chỉ báo, chưa đổi.

# Mục 9 — ảnh hưởng tới tính năng khác (commit sau c2d8875)
- **9a** (gac_vat_la.ba_dau_hieu trả thêm `co_cung`): cụm có cung dạng ống (r / rms / lồi / góc ≥ 100°) nhưng bị loại vì tâm sâu trong
  tường hoặc chưa ổn định ⇒ ở HÔNG là VẬT (6 cm), không mang cờ tường. Cung bị loại vì góc nhỏ (< 100°: nhiễu tường / góc K4) KHÔNG tính
  (nếu tính thì tường nhiễu lại thành VẬT ở hông — khoá như 15:53). test_19: ống áp tường dưới / mép trên K4 / mép dưới H0, vị trí tin
  lệch 0 / 8 / 12 cm về phía tường, nhiễu ±5 mm, 10 lần quét: hông 0,05 luôn VẬT + đỏ (ở mọi ca nhãn là "ong, on dinh 10/10").
- **9b** (chon_diem_vuot): bỏ V có hình chiếu theo hướng P→đích vượt quá đích; xe cách đích ≤ gan_dich_m 0,60 ⇒ V phải gần đích hơn P.
  test_20: ống gần B, xe (1,10; 0,70) ⇒ V (1,49; 0,79), gần B hơn P, không vượt B.
- **9c**: vị trí không tin (tin_sep: KHONG_TIN / BAO) ⇒ không dùng điểm dẫn, `DIEM_VUOT: bo (vi tri khong tin).` (test_21).
- **9d CPU** (PC, process_time, 600 lần quét = 60 s ở 10 Hz, nhiễu ±5 mm, 4 cảnh: góc K4, ống cạnh K4, sàn, ống giữa sàn; danh_gia +
  diem_loc_nhan + khe mỗi lần quét): 50e4146 2,64 ms/quét; mã mới ba khoá tắt 2,63; mã mới mặc định 2,68 ⇒ **+1,5 %** (< 20 %).
  Theo dõi cụm chỉ cho cụm có điểm trong vàng (≤ 0,30 trước / 0,12 hông / 0,10 sau), cửa sổ 1 s.
- **9e D(1)**: trang_v10 `chuyen_cuoi_ct`: `DIEM_DAN: (..)` và `DIEM_VUOT: P .. => V ..` = 이동 (DI); `DIEM_*: bo ..`, `.._XONG` giữ trạng
  thái trước (THOAT / CHO). ~/kinh_nghiem.csv: mục 6 / 7 KHÔNG ghi dòng mới, không thêm cột ⇒ người đọc (hanh_vi.doc_kinh_nghiem,
  lop_huong_dan, cach_kinh_nghiem) không đổi. cau_noi không đọc DIEM_*.
Test: test_phan_loai_cum 22/22 (gia lap); hồi quy như trên.
