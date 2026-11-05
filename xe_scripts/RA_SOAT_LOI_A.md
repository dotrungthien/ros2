# Rà soát lỗi A (05/11 19:33, kẹt góc K2): dựng lại `tinh_tuyen.py` trên costmap lúc kẹt

CHỈ ĐỌC: không sửa mã, không commit. Chưa giả lập cả hệ thống.

## Kiểm
1. `git rev-parse --short HEAD` = `d297dea` (đạt). `git status --short`: ngoài `du_lieu_0511/` (thư mục ở GỐC kho, không phải
   `xe_scripts/du_lieu_0511/`, chỉ chứa 3 tệp `*k2_1933*`) còn 5 mục chưa theo dõi có từ trước:
   `xe_scripts/KE_HOACH_109.md` và 4 tệp `xe_scripts/du_lieu_0211/chup_*.txt` ⇒ **không đúng "chỉ có du_lieu_0511/*k2_1933*"**.
2. `md5sum xe_scripts/tinh_tuyen.py` = `6d208304888be7c3d55593a3120cb41e` (khớp bản trên Pi 6d208304…).

## Dữ liệu và cách chạy
- `du_lieu_0511/costmap_ket_k2_1933.{pgm,yaml}`: 52×33, 0,05 m, gốc (−0,566; −0,455), `mode: raw` ⇒ điểm ảnh = giá trị
  OccupancyGrid 0..100, 255 = chưa biết. Đổi lại thành `data` (hàng 0 ở dưới, 255 → −1) rồi `Ban.tu_luoi` y như
  `gui_duong.py` dòng 388 trên Pi (ô ≥ 72 cấm khi tìm đường, ô ≥ 98 = ô CÓ VẬT). Có 728 ô ≥ 98, 1154 ô ≥ 72.
- Scan `scan_ket_k2_1933.txt`: stamp 1793874838 = 19:33:58, `laser_frame`; chưa dùng (không có tư thế xe).
- **Điểm đầu/đích chuyến kẹt: KHÔNG có trong log.** Mọi log trong `xe_scripts/du_lieu_0511/` dừng lúc 19:04
  (`ket_qua_chuyen.csv` cuối 19:02:45). Nên đã chạy (a) A→B và B→A đúng như lệnh; (b) vì `gui_duong` tính đường từ CHỖ XE
  ĐANG ĐỨNG (`pose` lúc gửi, cả sau TINH_LAI), quét mọi ô tự do làm điểm đầu, tìm ca cho đỉnh gần (0,51; 0,82).
- Đo mỗi đoạn: "tâm xe" = khoảng cách Euclid nhỏ nhất từ đoạn (mẫu 1 cm) tới **mép** ô ≥ 98; "thân xe" = hình chữ nhật
  footprint `nav2_params_cho.yaml` (x −0,105..0,145, y ±0,08: 25×16 cm) đặt dọc đoạn theo hướng đoạn, khoảng cách viền thân
  tới mép ô ≥ 98 (0 = CHẠM: có tâm ô ≥ 98 nằm trong thân). Cột `cach_do_doan` là số `tinh_tuyen` tự dùng (tâm ô tới tâm ô,
  mẫu 2 cm) để so với ngưỡng `R_XOAY_CAN` = 0,1856 + 0,05 = 0,2356 m.
- Script (scratchpad, không vào kho): `loi_a_k2.py` (md5 3ca3e08e…), `loi_a_quet.py` (md5 4f80f7b8…).

## Kết quả (a): xuất phát đúng A hoặc B — KHÔNG tái hiện
Cả 4 ca (A→B, B→A × tuyến 1, 2): không cảnh báo, không đỉnh nào gần (0,51; 0,82). Đầu ra đầy đủ:
```
costmap 52x33 res 0.05 goc (-0.566;-0.455); o >= 98: 728; o >= 72 (cam lop 1): 1154; nguong R_XOAY_CAN 0.2356 m

== A->B (di_B) tuyen 1, dau (0.0, 0.0) cuoi (1.6, 0.82)
  duong goc Dijkstra (41 diem): (0.00;0.00) (0.06;0.07) (0.11;0.07) (0.16;0.07) (0.21;0.07) (0.26;0.07) (0.31;0.07) (0.36;0.07) (0.41;0.07) (0.46;0.07) (0.51;0.07) (0.56;0.07) (0.61;0.07) (0.66;0.07) (0.71;0.07) (0.76;0.07) (0.81;0.07) (0.86;0.07) (0.91;0.07) (0.96;0.07) (1.01;0.02) (1.06;0.02) (1.11;0.02) (1.16;0.02) (1.21;0.07) (1.26;0.07) (1.31;0.12) (1.36;0.17) (1.41;0.22) (1.46;0.27) (1.51;0.32) (1.56;0.37) (1.56;0.42) (1.56;0.47) (1.56;0.52) (1.56;0.57) (1.56;0.62) (1.56;0.67) (1.56;0.72) (1.56;0.77) (1.60;0.82)
  dinh Douglas-Peucker (truoc nan): (0.000;0.000) (0.059;0.070) (1.259;0.070) (1.559;0.370) (1.600;0.820)
  dinh sau nan (gui di):           (0.000;0.000) (0.059;0.070) (1.259;0.070) (1.559;0.370) (1.600;0.820)
    dinh 1 (0.059;0.070): re 50 do, cach_do (tinh_tuyen, tam o-tam o) 0.364 m, can > 0.2356
    dinh 2 (1.259;0.070): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.255 m, can > 0.2356
    dinh 3 (1.559;0.370): re 40 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): khong
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (0.00;0.00)->(0.06;0.07) | 0.09 | 50 | 0.288 tai (0.00;0.00) | 0.173 tai (0.00;0.00) | 0.316 | tam >=, than 0.173 |
  | 2 (0.06;0.07)->(1.26;0.07) | 1.20 | 0 | 0.175 tai (0.44;0.07) | 0.095 tai (0.30;0.07) | 0.200 | tam <, than 0.095 |
  | 3 (1.26;0.07)->(1.56;0.37) | 0.42 | 45 | 0.177 tai (1.36;0.17) | 0.097 tai (1.43;0.24) | 0.212 | tam <, than 0.097 |
  | 4 (1.56;0.37)->(1.60;0.82) | 0.45 | 85 | 0.234 tai (1.60;0.82) | 0.123 tai (1.60;0.82) | 0.250 | tam <, than 0.123 |

== A->B (di_B) tuyen 2, dau (0.0, 0.0) cuoi (1.6, 0.82)
  duong goc Dijkstra (42 diem): (0.00;0.00) (0.01;0.07) (0.01;0.12) (0.01;0.17) (0.01;0.22) (0.01;0.27) (0.01;0.32) (0.01;0.37) (0.06;0.42) (0.11;0.47) (0.11;0.52) (0.16;0.57) (0.16;0.62) (0.21;0.67) (0.26;0.72) (0.31;0.77) (0.36;0.77) (0.41;0.77) (0.46;0.77) (0.51;0.77) (0.56;0.72) (0.61;0.72) (0.66;0.72) (0.71;0.72) (0.76;0.72) (0.81;0.72) (0.86;0.72) (0.91;0.72) (0.96;0.72) (1.01;0.72) (1.06;0.72) (1.11;0.72) (1.16;0.72) (1.21;0.72) (1.26;0.77) (1.31;0.77) (1.36;0.77) (1.41;0.77) (1.46;0.77) (1.51;0.77) (1.56;0.82) (1.60;0.82)
  dinh Douglas-Peucker (truoc nan): (0.000;0.000) (0.009;0.370) (0.309;0.770) (1.209;0.720) (1.600;0.820)
  dinh sau nan (gui di):           (0.000;0.000) (0.009;0.370) (0.309;0.770) (0.759;0.720) (1.209;0.720) (1.600;0.820)
    dinh 1 (0.009;0.370): re 35 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
    dinh 2 (0.309;0.770): re 59 do, cach_do (tinh_tuyen, tam o-tam o) 0.250 m, can > 0.2356
    dinh 3 (0.759;0.720): re 6 do, cach_do (tinh_tuyen, tam o-tam o) 0.150 m, can > 0.2356  [da doi]
    dinh 4 (1.209;0.720): re 14 do, cach_do (tinh_tuyen, tam o-tam o) 0.150 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): khong
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (0.00;0.00)->(0.01;0.37) | 0.37 | 89 | 0.288 tai (0.00;0.00) | 0.159 tai (0.00;0.00) | 0.335 | tam >=, than 0.159 |
  | 2 (0.01;0.37)->(0.31;0.77) | 0.50 | 53 | 0.155 tai (0.21;0.63) | 0.061 tai (0.31;0.77) | 0.180 | tam <, than 0.061 |
  | 3 (0.31;0.77)->(0.76;0.72) | 0.45 | -6 | 0.110 tai (0.62;0.74) | 0.029 tai (0.76;0.72) | 0.150 | tam <, than 0.029 |
  | 4 (0.76;0.72)->(1.21;0.72) | 0.45 | 0 | 0.125 tai (0.76;0.72) | 0.045 tai (0.76;0.72) | 0.150 | tam <, than 0.045 |
  | 5 (1.21;0.72)->(1.60;0.82) | 0.40 | 14 | 0.125 tai (1.21;0.72) | 0.021 tai (1.21;0.72) | 0.150 | tam <, than 0.021 |

== B->A (ve_A) tuyen 1, dau (1.6, 0.82) cuoi (0.0, 0.0)
  duong goc Dijkstra (41 diem): (1.60;0.82) (1.56;0.77) (1.56;0.72) (1.56;0.67) (1.56;0.62) (1.56;0.57) (1.56;0.52) (1.56;0.47) (1.56;0.42) (1.56;0.37) (1.51;0.32) (1.46;0.27) (1.41;0.22) (1.36;0.17) (1.31;0.12) (1.26;0.07) (1.21;0.07) (1.16;0.02) (1.11;0.02) (1.06;0.02) (1.01;0.02) (0.96;0.07) (0.91;0.07) (0.86;0.07) (0.81;0.07) (0.76;0.07) (0.71;0.07) (0.66;0.07) (0.61;0.07) (0.56;0.07) (0.51;0.07) (0.46;0.07) (0.41;0.07) (0.36;0.07) (0.31;0.07) (0.26;0.07) (0.21;0.07) (0.16;0.07) (0.11;0.07) (0.06;0.07) (0.00;0.00)
  dinh Douglas-Peucker (truoc nan): (1.600;0.820) (1.559;0.370) (1.259;0.070) (0.059;0.070) (0.000;0.000)
  dinh sau nan (gui di):           (1.600;0.820) (1.559;0.370) (1.259;0.070) (0.059;0.070) (0.000;0.000)
    dinh 1 (1.559;0.370): re 40 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
    dinh 2 (1.259;0.070): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.255 m, can > 0.2356
    dinh 3 (0.059;0.070): re 50 do, cach_do (tinh_tuyen, tam o-tam o) 0.364 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): khong
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (1.60;0.82)->(1.56;0.37) | 0.45 | -95 | 0.234 tai (1.60;0.82) | 0.145 tai (1.60;0.82) | 0.250 | tam <, than 0.145 |
  | 2 (1.56;0.37)->(1.26;0.07) | 0.42 | -135 | 0.177 tai (1.36;0.17) | 0.097 tai (1.45;0.26) | 0.212 | tam <, than 0.097 |
  | 3 (1.26;0.07)->(0.06;0.07) | 1.20 | 180 | 0.175 tai (0.78;0.07) | 0.095 tai (0.83;0.07) | 0.200 | tam <, than 0.095 |
  | 4 (0.06;0.07)->(0.00;0.00) | 0.09 | -130 | 0.288 tai (0.00;0.00) | 0.143 tai (0.00;0.00) | 0.316 | tam >=, than 0.143 |

== B->A (ve_A) tuyen 2, dau (1.6, 0.82) cuoi (0.0, 0.0)
  duong goc Dijkstra (42 diem): (1.60;0.82) (1.56;0.82) (1.51;0.77) (1.46;0.77) (1.41;0.77) (1.36;0.77) (1.31;0.77) (1.26;0.77) (1.21;0.77) (1.16;0.77) (1.11;0.77) (1.06;0.77) (1.01;0.77) (0.96;0.77) (0.91;0.77) (0.86;0.72) (0.81;0.72) (0.76;0.72) (0.71;0.72) (0.66;0.72) (0.61;0.72) (0.56;0.72) (0.51;0.77) (0.46;0.77) (0.41;0.77) (0.36;0.77) (0.31;0.77) (0.26;0.72) (0.26;0.67) (0.21;0.62) (0.16;0.57) (0.11;0.52) (0.11;0.47) (0.06;0.42) (0.01;0.37) (0.01;0.32) (0.01;0.27) (0.01;0.22) (0.01;0.17) (0.01;0.12) (0.01;0.07) (0.00;0.00)
  dinh Douglas-Peucker (truoc nan): (1.600;0.820) (0.859;0.720) (0.309;0.770) (0.009;0.370) (0.000;0.000)
  dinh sau nan (gui di):           (1.600;0.820) (0.859;0.720) (0.309;0.770) (0.009;0.370) (0.000;0.000)
    dinh 1 (0.859;0.720): re 13 do, cach_do (tinh_tuyen, tam o-tam o) 0.150 m, can > 0.2356
    dinh 2 (0.309;0.770): re 58 do, cach_do (tinh_tuyen, tam o-tam o) 0.250 m, can > 0.2356
    dinh 3 (0.009;0.370): re 35 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): khong
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (1.60;0.82)->(0.86;0.72) | 0.75 | -172 | 0.125 tai (0.86;0.72) | 0.026 tai (0.86;0.72) | 0.150 | tam <, than 0.026 |
  | 2 (0.86;0.72)->(0.31;0.77) | 0.55 | 175 | 0.104 tai (0.62;0.74) | 0.024 tai (0.77;0.73) | 0.141 | tam <, than 0.024 |
  | 3 (0.31;0.77)->(0.01;0.37) | 0.50 | -127 | 0.155 tai (0.21;0.64) | 0.075 tai (0.26;0.71) | 0.180 | tam <, than 0.075 |
  | 4 (0.01;0.37)->(0.00;0.00) | 0.37 | -91 | 0.288 tai (0.00;0.00) | 0.122 tai (0.00;0.00) | 0.335 | tam >=, than 0.122 |
```

## Kết quả (b): quét điểm xuất phát — TÁI HIỆN khi xe đứng sẵn trong góc K2
Ô chứa (0,51; 0,82) có giá trị 0, `cach_do` 0,141 m (tâm ô tới tâm ô vật). Các điểm đầu cho đỉnh gần (0,51; 0,82)
đều nằm sát góc: (0,56; 0,82), (0,56; 0,87), (0,51; 0,87), (0,56; 0,92), (0,61; 0,87).
```
o chua (0,51;0,82) = (21, 25), gia tri 0, cach_do 0.141 m; o lan can (hang tren xuong):
   y=0.970   94  94  94  91  83  94  99 100 100
   y=0.920   81  81  81  79  79  91  99  99 100
   y=0.870   70  70  70   0  74  83  91  99  99
   y=0.820    0   0   0   0   0  74  83  91  94
   y=0.770    0   0   0   0   0   0  74  79  81
   y=0.720    0   0   0   0   0  70  70  70  70
   y=0.670    0   0  70  74  79  81  81  81  81

di_B tuyen 1: 3 diem xuat phat cho dinh gan (0,51;0,82); 2 co canh_bao
  dau (0.56;0.82) -> dinh (0.509;0.770) cach_do 0.180; canh_bao [('0.51', '0.77', '0.180')]; dinh: (0.56;0.82) (0.51;0.77) (0.41;0.77) (0.36;0.77) (0.26;0.67) (0.01;0.37) (0.01;0.22) (0.16;0.07) (1.01;0.02) (1.21;0.02) (1.56;0.37) (1.60;0.82)
  dau (0.56;0.87) -> dinh (0.509;0.820) cach_do 0.141; canh_bao [('0.46', '0.77', '0.206'), ('0.31', '0.87', '0.150'), ('0.26', '0.82', '0.200'), ('0.26', '0.67', '0.180')]; dinh: (0.56;0.87) (0.51;0.82) (0.46;0.77) (0.31;0.87) (0.26;0.82) (0.26;0.67) (0.01;0.37) (0.01;0.22) (0.16;0.07) (1.01;0.02) (1.21;0.02) (1.56;0.37) (1.60;0.82)

di_B tuyen 2: 3 diem xuat phat cho dinh gan (0,51;0,82); 3 co canh_bao
  dau (0.51;0.87) -> dinh (0.459;0.820) cach_do 0.180; canh_bao [('0.46', '0.82', '0.180'), ('0.46', '0.77', '0.206')]; dinh: (0.51;0.87) (0.46;0.82) (0.46;0.77) (0.61;0.72) (1.21;0.72) (1.60;0.82)
  dau (0.56;0.87) -> dinh (0.509;0.820) cach_do 0.141; canh_bao [('0.51', '0.82', '0.141'), ('0.51', '0.77', '0.180'), ('0.61', '0.72', '0.150')]; dinh: (0.56;0.87) (0.51;0.82) (0.51;0.77) (0.61;0.72) (1.21;0.72) (1.60;0.82)
  dau (0.56;0.92) -> dinh (0.509;0.820) cach_do 0.141; canh_bao [('0.56', '0.87', '0.071'), ('0.51', '0.82', '0.141'), ('0.51', '0.77', '0.180'), ('0.61', '0.72', '0.150')]; dinh: (0.56;0.92) (0.56;0.87) (0.51;0.82) (0.51;0.77) (0.61;0.72) (1.21;0.72) (1.60;0.82)

ve_A tuyen 1: 3 diem xuat phat cho dinh gan (0,51;0,82); 2 co canh_bao
  dau (0.56;0.82) -> dinh (0.509;0.770) cach_do 0.180; canh_bao [('0.51', '0.77', '0.180')]; dinh: (0.56;0.82) (0.51;0.77) (0.41;0.77) (0.36;0.77) (0.26;0.67) (0.01;0.37) (0.00;0.00)
  dau (0.56;0.87) -> dinh (0.509;0.820) cach_do 0.141; canh_bao [('0.46', '0.77', '0.206'), ('0.31', '0.87', '0.150'), ('0.26', '0.82', '0.200'), ('0.26', '0.67', '0.180')]; dinh: (0.56;0.87) (0.51;0.82) (0.46;0.77) (0.31;0.87) (0.26;0.82) (0.26;0.67) (0.01;0.37) (0.00;0.00)

ve_A tuyen 2: 4 diem xuat phat cho dinh gan (0,51;0,82); 2 co canh_bao
  dau (0.56;0.82) -> dinh (0.509;0.770) cach_do 0.180; canh_bao [('0.51', '0.77', '0.180')]; dinh: (0.56;0.82) (0.51;0.77) (0.46;0.77) (0.31;0.77) (0.21;0.67) (0.01;0.37) (0.00;0.00)
  dau (0.61;0.87) -> dinh (0.509;0.820) cach_do 0.141; canh_bao [('0.56', '0.87', '0.071')]; dinh: (0.61;0.87) (0.56;0.87) (0.51;0.82) (0.36;0.77) (0.31;0.77) (0.11;0.52) (0.01;0.37) (0.00;0.00)
```

## Bảng đoạn: ca tái hiện đúng mô tả (di_B tuyến 2, đầu (0,559; 0,870)) và hai ca cùng góc
```

== di_B tuyen 2, dau (0.56;0.87) cuoi (1.6, 0.82)
  duong goc Dijkstra (25 diem): (0.56;0.87) (0.51;0.82) (0.51;0.77) (0.56;0.77) (0.61;0.72) (0.66;0.72) (0.71;0.72) (0.76;0.72) (0.81;0.72) (0.86;0.72) (0.91;0.72) (0.96;0.72) (1.01;0.72) (1.06;0.72) (1.11;0.72) (1.16;0.72) (1.21;0.72) (1.26;0.77) (1.31;0.77) (1.36;0.77) (1.41;0.77) (1.46;0.77) (1.51;0.77) (1.56;0.82) (1.60;0.82)
  dinh Douglas-Peucker (truoc nan): (0.559;0.870) (0.509;0.770) (0.609;0.720) (1.209;0.720) (1.600;0.820)
  dinh sau nan (gui di):           (0.559;0.870) (0.509;0.820) (0.509;0.770) (0.609;0.720) (1.209;0.720) (1.600;0.820)
    dinh 1 (0.509;0.820): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.141 m, can > 0.2356  [da doi]
    dinh 2 (0.509;0.770): re 63 do, cach_do (tinh_tuyen, tam o-tam o) 0.180 m, can > 0.2356
    dinh 3 (0.609;0.720): re 27 do, cach_do (tinh_tuyen, tam o-tam o) 0.150 m, can > 0.2356
    dinh 4 (1.209;0.720): re 14 do, cach_do (tinh_tuyen, tam o-tam o) 0.150 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): [('0.509', '0.820', '0.141 m'), ('0.509', '0.770', '0.180 m'), ('0.609', '0.720', '0.150 m')]
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (0.56;0.87)->(0.51;0.82) | 0.07 | -135 | 0.035 tai (0.56;0.87) | 0.000 tai (0.56;0.87) | 0.071 | tam <, than CHAM |
  | 2 (0.51;0.82)->(0.51;0.77) | 0.05 | -90 | 0.106 tai (0.51;0.82) | 0.000 tai (0.51;0.82) | 0.141 | tam <, than CHAM |
  | 3 (0.51;0.77)->(0.61;0.72) | 0.11 | -27 | 0.123 tai (0.58;0.74) | 0.000 tai (0.59;0.73) | 0.141 | tam <, than CHAM |
  | 4 (0.61;0.72)->(1.21;0.72) | 0.60 | 0 | 0.125 tai (0.61;0.72) | 0.045 tai (0.61;0.72) | 0.150 | tam <, than 0.045 |
  | 5 (1.21;0.72)->(1.60;0.82) | 0.40 | 14 | 0.125 tai (1.21;0.72) | 0.021 tai (1.21;0.72) | 0.150 | tam <, than 0.021 |

== di_B tuyen 1, dau (0.56;0.87) cuoi (1.6, 0.82)
  duong goc Dijkstra (59 diem): (0.56;0.87) (0.51;0.82) (0.46;0.77) (0.41;0.77) (0.36;0.82) (0.31;0.87) (0.26;0.82) (0.26;0.77) (0.26;0.72) (0.26;0.67) (0.21;0.62) (0.16;0.57) (0.11;0.52) (0.11;0.47) (0.06;0.42) (0.01;0.37) (0.01;0.32) (0.01;0.27) (0.01;0.22) (0.06;0.17) (0.11;0.12) (0.16;0.07) (0.21;0.07) (0.26;0.07) (0.31;0.07) (0.36;0.07) (0.41;0.07) (0.46;0.07) (0.51;0.07) (0.56;0.07) (0.61;0.07) (0.66;0.07) (0.71;0.07) (0.76;0.07) (0.81;0.07) (0.86;0.07) (0.91;0.07) (0.96;0.07) (1.01;0.02) (1.06;0.02) (1.11;0.02) (1.16;0.02) (1.21;0.02) (1.26;0.07) (1.31;0.12) (1.36;0.17) (1.41;0.22) (1.46;0.27) (1.51;0.32) (1.56;0.37) (1.56;0.42) (1.56;0.47) (1.56;0.52) (1.56;0.57) (1.56;0.62) (1.56;0.67) (1.56;0.72) (1.56;0.77) (1.60;0.82)
  dinh Douglas-Peucker (truoc nan): (0.559;0.870) (0.459;0.770) (0.309;0.870) (0.259;0.820) (0.259;0.670) (0.009;0.370) (0.009;0.220) (0.159;0.070) (1.009;0.020) (1.209;0.020) (1.559;0.370) (1.600;0.820)
  dinh sau nan (gui di):           (0.559;0.870) (0.509;0.820) (0.459;0.770) (0.309;0.870) (0.259;0.820) (0.259;0.670) (0.009;0.370) (0.009;0.220) (0.159;0.070) (1.009;0.020) (1.209;0.020) (1.559;0.370) (1.600;0.820)
    dinh 1 (0.509;0.820): re 0 do, cach_do (tinh_tuyen, tam o-tam o) 0.141 m, can > 0.2356  [da doi]
    dinh 2 (0.459;0.770): re 79 do, cach_do (tinh_tuyen, tam o-tam o) 0.206 m, can > 0.2356
    dinh 3 (0.309;0.870): re 79 do, cach_do (tinh_tuyen, tam o-tam o) 0.150 m, can > 0.2356
    dinh 4 (0.259;0.820): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.200 m, can > 0.2356
    dinh 5 (0.259;0.670): re 40 do, cach_do (tinh_tuyen, tam o-tam o) 0.180 m, can > 0.2356
    dinh 6 (0.009;0.370): re 40 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
    dinh 7 (0.009;0.220): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.381 m, can > 0.2356
    dinh 8 (0.159;0.070): re 42 do, cach_do (tinh_tuyen, tam o-tam o) 0.320 m, can > 0.2356
    dinh 9 (1.009;0.020): re 3 do, cach_do (tinh_tuyen, tam o-tam o) 0.292 m, can > 0.2356
    dinh 10 (1.209;0.020): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.255 m, can > 0.2356
    dinh 11 (1.559;0.370): re 40 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): [('0.459', '0.770', '0.206 m'), ('0.309', '0.870', '0.150 m'), ('0.259', '0.820', '0.200 m'), ('0.259', '0.670', '0.180 m')]
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (0.56;0.87)->(0.51;0.82) | 0.07 | -135 | 0.035 tai (0.56;0.87) | 0.000 tai (0.56;0.87) | 0.071 | tam <, than CHAM |
  | 2 (0.51;0.82)->(0.46;0.77) | 0.07 | -135 | 0.106 tai (0.51;0.82) | 0.001 tai (0.51;0.82) | 0.141 | tam <, than 0.001 |
  | 3 (0.46;0.77)->(0.31;0.87) | 0.18 | 146 | 0.125 tai (0.31;0.87) | 0.000 tai (0.34;0.85) | 0.150 | tam <, than CHAM |
  | 4 (0.31;0.87)->(0.26;0.82) | 0.07 | -135 | 0.125 tai (0.31;0.87) | 0.000 tai (0.31;0.87) | 0.150 | tam <, than CHAM |
  | 5 (0.26;0.82)->(0.26;0.67) | 0.15 | -90 | 0.146 tai (0.26;0.67) | 0.000 tai (0.26;0.69) | 0.180 | tam <, than CHAM |
  | 6 (0.26;0.67)->(0.01;0.37) | 0.39 | -130 | 0.138 tai (0.23;0.63) | 0.058 tai (0.26;0.67) | 0.141 | tam <, than 0.058 |
  | 7 (0.01;0.37)->(0.01;0.22) | 0.15 | -90 | 0.325 tai (0.01;0.37) | 0.220 tai (0.01;0.37) | 0.350 | tam >=, than 0.220 |
  | 8 (0.01;0.22)->(0.16;0.07) | 0.21 | -45 | 0.285 tai (0.16;0.07) | 0.138 tai (0.16;0.07) | 0.320 | tam >=, than 0.138 |
  | 9 (0.16;0.07)->(1.01;0.02) | 0.85 | -3 | 0.138 tai (0.79;0.03) | 0.058 tai (0.90;0.03) | 0.150 | tam <, than 0.058 |
  | 10 (1.01;0.02)->(1.21;0.02) | 0.20 | 0 | 0.226 tai (1.21;0.02) | 0.128 tai (1.01;0.02) | 0.255 | tam <, than 0.128 |
  | 11 (1.21;0.02)->(1.56;0.37) | 0.49 | 45 | 0.177 tai (1.36;0.17) | 0.097 tai (1.41;0.22) | 0.212 | tam <, than 0.097 |
  | 12 (1.56;0.37)->(1.60;0.82) | 0.45 | 85 | 0.234 tai (1.60;0.82) | 0.123 tai (1.60;0.82) | 0.250 | tam <, than 0.123 |

== ve_A tuyen 2, dau (0.61;0.87) cuoi (0.0, 0.0)
  duong goc Dijkstra (22 diem): (0.61;0.87) (0.56;0.87) (0.51;0.82) (0.46;0.77) (0.41;0.77) (0.36;0.77) (0.31;0.77) (0.26;0.72) (0.26;0.67) (0.21;0.62) (0.16;0.57) (0.11;0.52) (0.11;0.47) (0.06;0.42) (0.01;0.37) (0.01;0.32) (0.01;0.27) (0.01;0.22) (0.01;0.17) (0.01;0.12) (0.01;0.07) (0.00;0.00)
  dinh Douglas-Peucker (truoc nan): (0.609;0.870) (0.309;0.770) (0.109;0.520) (0.009;0.370) (0.000;0.000)
  dinh sau nan (gui di):           (0.609;0.870) (0.559;0.870) (0.509;0.820) (0.359;0.770) (0.309;0.770) (0.109;0.520) (0.009;0.370) (0.000;0.000)
    dinh 1 (0.559;0.870): re 45 do, cach_do (tinh_tuyen, tam o-tam o) 0.071 m, can > 0.2356  [da doi]
    dinh 2 (0.509;0.820): re 27 do, cach_do (tinh_tuyen, tam o-tam o) 0.141 m, can > 0.2356  [da doi]
    dinh 3 (0.359;0.770): re 18 do, cach_do (tinh_tuyen, tam o-tam o) 0.250 m, can > 0.2356  [da doi]
    dinh 4 (0.309;0.770): re 51 do, cach_do (tinh_tuyen, tam o-tam o) 0.250 m, can > 0.2356
    dinh 5 (0.109;0.520): re 5 do, cach_do (tinh_tuyen, tam o-tam o) 0.224 m, can > 0.2356
    dinh 6 (0.009;0.370): re 32 do, cach_do (tinh_tuyen, tam o-tam o) 0.350 m, can > 0.2356
  canh_bao (khong doi duoc, VAN GUI): [('0.559', '0.870', '0.071 m')]
  | doan | dai m | huong do | tam xe: min toi o>=98 (m) tai | than 25x16: min (m) tai | cach_do_doan tinh_tuyen | < 0,236? |
  | 1 (0.61;0.87)->(0.56;0.87) | 0.05 | 180 | 0.025 tai (0.61;0.87) | 0.000 tai (0.61;0.87) | 0.050 | tam <, than CHAM |
  | 2 (0.56;0.87)->(0.51;0.82) | 0.07 | -135 | 0.035 tai (0.56;0.87) | 0.000 tai (0.56;0.87) | 0.071 | tam <, than CHAM |
  | 3 (0.51;0.82)->(0.36;0.77) | 0.16 | -162 | 0.106 tai (0.51;0.82) | 0.000 tai (0.51;0.82) | 0.141 | tam <, than CHAM |
  | 4 (0.36;0.77)->(0.31;0.77) | 0.05 | 180 | 0.215 tai (0.36;0.77) | 0.080 tai (0.31;0.77) | 0.250 | tam <, than 0.080 |
  | 5 (0.31;0.77)->(0.11;0.52) | 0.32 | -129 | 0.160 tai (0.21;0.65) | 0.080 tai (0.21;0.65) | 0.180 | tam <, than 0.080 |
  | 6 (0.11;0.52)->(0.01;0.37) | 0.18 | -124 | 0.190 tai (0.11;0.52) | 0.079 tai (0.11;0.52) | 0.224 | tam <, than 0.079 |
  | 7 (0.01;0.37)->(0.00;0.00) | 0.37 | -91 | 0.288 tai (0.00;0.00) | 0.122 tai (0.00;0.00) | 0.335 | tam >=, than 0.122 |
```

## Đọc kết quả (ca di_B tuyến 2 từ (0,559; 0,870))
- Đỉnh (0,509; 0,820): rẽ 45°, cách ô vật 0,141 m < 0,2356 ⇒ nằm trong `canh_bao` ("không dời được") **nhưng vẫn gửi**:
  `doan_thang` chỉ ghi cảnh báo và giữ đỉnh cũ, `gui_duong` in cảnh báo rồi gửi. Khớp mong đợi (≈ (0,51; 0,82), ≈ 0,14 m).
- Đỉnh này KHÔNG có trong Douglas-Peucker ban đầu: nó do bước "đoạn cắt ô đỏ thì chèn điểm giữa" thêm vào (đoạn DP
  (0,559; 0,870)→(0,509; 0,770) cắt ô cấm), nên được đặt đúng chỗ đường gốc ôm góc.
- Ba đoạn đầu (dài 0,07 / 0,05 / 0,11 m): tâm xe cách mép ô ≥ 98 chỉ 0,035 / 0,106 / 0,123 m; **thân xe CHẠM ô ≥ 98**
  (đoạn 3 (0,51; 0,77)→(0,61; 0,72) chạm tại (0,59; 0,73): đây là đoạn cắt góc K2). Các đoạn sau tâm 0,125 m, thân 0,02–0,05 m.
- Ngay điểm đầu thân xe đã chạm ô ≥ 98 (xe đứng trong góc), nên mọi đường ra khỏi góc đều bắt đầu bằng đoạn chạm.
- Tuyến 1 cùng điểm đầu còn tệ hơn: 4 đỉnh cảnh báo, 4 đoạn thân chạm. ve_A tuyến 2 từ (0,609; 0,870): đỉnh (0,559; 0,870)
  cách 0,071 m, 3 đoạn đầu thân chạm.
- Luật hiện có chỉ kiểm khoảng cách tại ĐỈNH RẼ (≥ 20°) và "không gần vật hơn đường gốc − 3 cm"; không kiểm thân xe dọc đoạn,
  và đỉnh không dời được vẫn gửi. Đó là chỗ cho lỗi A lọt qua.

## Chưa chắc
- Điểm đầu thật lúc 19:33 không có trong log; (0,559; 0,870) là điểm quét được cho đúng đỉnh mô tả, chưa đối chiếu scan.
- Chưa so `gui_duong.py` trong kho (md5 6c937df6…) với bản trên Pi.

---

# Việc 2: so đường vạch với vết xe thật, kẹt T3 18:57 (CHỈ ĐỌC; chưa giả lập cả hệ thống)

Bước 0 đã làm: `mv du_lieu_0511/*k2_1933* xe_scripts/du_lieu_0511/ && rmdir du_lieu_0511`.
Kiểm (1) `git status --short`: 5 mục cũ (`KE_HOACH_109.md`, 4 `du_lieu_0211/chup_*.txt`) + `RA_SOAT_LOI_A.md` (tệp này, việc 1)
+ 3 tệp `xe_scripts/du_lieu_0511/*k2_1933*`. Không có gì khác.

Cách làm (script scratchpad `loi_a_t3.py`, không vào kho):
- `costmap_t3_1858` (52×32, gốc (−0,566; −0,416), raw 0..100, 255 = chưa biết) → `Ban.tu_luoi` như `gui_duong.py` trên Pi;
  `tinh_duong(ban, (0,29; 0,01), B, tuyến 2, yaw_cuoi=0)`, cùng hàm `doan_thang` lấy đỉnh và cảnh báo.
  Lưu ý: costmap lưu 18:58, sau lúc kẹt khoảng 1 phút.
- Vết: `vet_t3.txt` (tf2_echo map→base_footprint, ~1 Hz) trong 18:56:54–18:57:17 ⇒ **n = 23**. Thêm vết từ cột x, y, góc của
  `sep_nhat_ky.csv` (làm tròn 1 cm / 1°; 23 vị trí khác nhau) để thấy 18:57:17.
- Lệch ngang: khoảng cách có dấu tới đường gấp khúc của các đỉnh (+ = bên trái hướng đi). "Trong/ngoài": so với đỉnh rẽ gần
  nhất (đỉnh cuối đoạn nếu đã quá nửa đoạn).
- Thân xe 25×16 cm (footprint `nav2_params_cho.yaml`), khoảng cách viền thân tới mép ô **100 (= 254 LETHAL, cái RPP kiểm)**; thêm
  cột tới ô ≥ 98 để đối chiếu với luật `tinh_tuyen`. 0 = có tâm ô nằm trong thân.

## Đầu ra
```
costmap_t3_1858 52x32 goc (-0.566;-0.416); o 100 (254): 394; o >= 98: 746
dinh duong vach: (0.290;0.010) (0.009;0.259) (0.009;0.409) (0.359;0.809) (0.609;0.759) (0.909;0.759) (1.509;0.759) (1.600;0.820)
  dinh 1 (0.009;0.259): re -48 do (phai), cach_do 0.381 m
  dinh 2 (0.009;0.409): re -41 do (phai), cach_do 0.350 m
  dinh 3 (0.359;0.809): re -60 do (phai), cach_do 0.250 m
  dinh 4 (0.609;0.759): re +11 do (trai), cach_do 0.158 m
  dinh 5 (0.909;0.759): re +0 do (phai), cach_do 0.150 m
  dinh 6 (1.509;0.759): re +34 do (trai), cach_do 0.350 m
CANH BAO: khong co

| doan vach | huong | than->o100 min (m) tai | than->o>=98 min | tam->o100 min |
| 1 (0.29;0.01)->(0.01;0.26) | 138 | 0.082 tai (0.29;0.01) | 0.012 | 0.200 |
| 2 (0.01;0.26)->(0.01;0.41) | 90 | 0.230 tai (0.01;0.41) | 0.180 | 0.375 |
| 3 (0.01;0.41)->(0.36;0.81) | 49 | 0.113 tai (0.36;0.81) | 0.063 | 0.228 |
| 4 (0.36;0.81)->(0.61;0.76) | -11 | 0.106 tai (0.50;0.78) | 0.048 | 0.190 |
| 5 (0.61;0.76)->(0.91;0.76) | 0 | 0.095 tai (0.61;0.76) | 0.045 | 0.175 |
| 6 (0.91;0.76)->(1.51;0.76) | 0 | 0.095 tai (0.91;0.76) | 0.045 | 0.175 |
| 7 (1.51;0.76)->(1.60;0.82) | 34 | 0.119 tai (1.60;0.82) | 0.069 | 0.284 |

vet_t3.txt: 299 vi tri ca file; trong 18:56:54-18:57:17: n = 23
| gio | x | y | goc do | lech ngang m (+trai) | doan | phia | than->o100 m | than->o>=98 m |
| 18:56:54.21 | 0.292 | 0.013 | 7 | -0.004 | 1 | - | 0.109 | 0.053 |
| 18:56:55.21 | 0.292 | 0.013 | 7 | -0.004 | 1 | - | 0.109 | 0.053 |
| 18:56:56.21 | 0.292 | 0.013 | 7 | -0.004 | 1 | - | 0.109 | 0.053 |
| 18:56:57.21 | 0.292 | 0.013 | 23 | -0.004 | 1 | - | 0.121 | 0.055 |
| 18:56:58.21 | 0.292 | 0.013 | 56 | -0.004 | 1 | - | 0.096 | 0.027 |
| 18:56:59.21 | 0.292 | 0.013 | 91 | -0.004 | 1 | - | 0.076 | 0.025 |
| 18:57:00.21 | 0.292 | 0.013 | 125 | -0.004 | 1 | - | 0.094 | 0.025 |
| 18:57:01.21 | 0.283 | 0.017 | 155 | -0.001 | 1 | - | 0.077 | 0.016 |
| 18:57:02.21 | 0.201 | 0.057 | 154 | +0.024 | 1 | - | 0.162 | 0.092 |
| 18:57:03.21 | 0.111 | 0.103 | 152 | +0.049 | 1 | ngoai re dinh 1 | 0.264 | 0.195 |
| 18:57:04.21 | 0.055 | 0.137 | 141 | +0.061 | 1 | ngoai re dinh 1 | 0.308 | 0.248 |
| 18:57:05.21 | 0.029 | 0.148 | 130 | +0.070 | 1 | ngoai re dinh 1 | 0.292 | 0.241 |
| 18:57:06.21 | -0.006 | 0.199 | 117 | +0.055 | 1 | ngoai re dinh 1 | 0.293 | 0.230 |
| 18:57:07.21 | -0.019 | 0.242 | 103 | +0.033 | 1 | ngoai re dinh 1 | 0.320 | 0.260 |
| 18:57:08.21 | -0.021 | 0.288 | 84 | +0.030 | 2 | ngoai re dinh 1 | 0.305 | 0.255 |
| 18:57:09.21 | -0.067 | 0.345 | 65 | +0.076 | 2 | ngoai re dinh 2 | 0.253 | 0.189 |
| 18:57:10.21 | -0.024 | 0.415 | 57 | +0.034 | 2 | ngoai re dinh 2 | 0.204 | 0.154 |
| 18:57:11.21 | 0.027 | 0.492 | 56 | +0.041 | 3 | ngoai re dinh 2 | 0.127 | 0.077 |
| 18:57:12.21 | 0.064 | 0.613 | 47 | +0.093 | 3 | ngoai re dinh 2 | 0.059 | 0.000 |
| 18:57:13.21 | 0.113 | 0.640 | 28 | +0.074 | 3 | ngoai re dinh 2 | 0.084 | 0.017 |
| 18:57:14.21 | 0.219 | 0.697 | 28 | +0.032 | 3 | ngoai re dinh 3 | 0.085 | 0.016 |
| 18:57:15.21 | 0.334 | 0.769 | 25 | -0.008 | 3 | TRONG re dinh 3 | 0.171 | 0.109 |
| 18:57:16.21 | 0.394 | 0.791 | 15 | -0.011 | 4 | TRONG re dinh 3 | 0.172 | 0.111 |

sep_nhat_ky.csv cung khoang: 27 dong co toa do, 23 vi tri khac nhau
| gio | x | y | goc do | lech ngang m | doan | phia | than->o100 m |
| 18:56:54 | 0.29 | 0.01 | 7 | -0.000 | 1 | - | 0.106 |
| 18:56:57 | 0.29 | 0.01 | 18 | -0.000 | 1 | - | 0.116 |
| 18:56:57 | 0.29 | 0.01 | 25 | -0.000 | 1 | - | 0.119 |
| 18:56:57 | 0.29 | 0.01 | 35 | -0.000 | 1 | - | 0.118 |
| 18:56:57 | 0.29 | 0.01 | 42 | -0.000 | 1 | - | 0.114 |
| 18:56:58 | 0.29 | 0.01 | 51 | -0.000 | 1 | - | 0.104 |
| 18:56:58 | 0.29 | 0.01 | 54 | -0.000 | 1 | - | 0.100 |
| 18:56:58 | 0.29 | 0.01 | 62 | -0.000 | 1 | - | 0.086 |
| 18:56:58 | 0.29 | 0.01 | 72 | -0.000 | 1 | - | 0.072 |
| 18:56:58 | 0.29 | 0.01 | 78 | -0.000 | 1 | - | 0.068 |
| 18:56:59 | 0.29 | 0.01 | 89 | -0.000 | 1 | - | 0.071 |
| 18:56:59 | 0.29 | 0.01 | 96 | -0.000 | 1 | - | 0.080 |
| 18:56:59 | 0.29 | 0.01 | 103 | -0.000 | 1 | - | 0.088 |
| 18:56:59 | 0.29 | 0.01 | 113 | -0.000 | 1 | - | 0.094 |
| 18:56:59 | 0.29 | 0.01 | 117 | -0.000 | 1 | - | 0.095 |
| 18:57:00 | 0.29 | 0.01 | 133 | -0.000 | 1 | - | 0.088 |
| 18:57:00 | 0.29 | 0.01 | 136 | -0.000 | 1 | - | 0.085 |
| 18:57:00 | 0.29 | 0.01 | 146 | -0.000 | 1 | - | 0.073 |
| 18:57:00 | 0.29 | 0.01 | 149 | -0.000 | 1 | - | 0.070 |
| 18:57:02 | 0.19 | 0.06 | 154 | +0.029 | 1 | - | 0.173 |
| 18:57:08 | -0.02 | 0.28 | 85 | +0.029 | 2 | ngoai re dinh 1 | 0.307 |
| 18:57:16 | 0.43 | 0.80 | 14 | +0.005 | 4 | ngoai re dinh 3 | 0.134 |
| 18:57:17 | 0.46 | 0.81 | 5 | +0.021 | 4 | ngoai re dinh 3 | 0.092 |
```

Log quanh lúc kẹt (`log_bat_xe_nav.txt`): 18:56:56.41 controller nhận đích; 18:57:17.02–.22 ba lần
`RegulatedPurePursuitController detected collision ahead!` rồi `Controller patience exceeded`, ABORTED sau 20,8 s.
`sep_nhat_ky.csv` 18:57:16: `TRUY_VET,VAT_LA_TRUOC,vat la (nguoi; hop),ban do: o trong`. Phụ lái 18:57:11–13: `TR-TRAI=0.10`,
`TRAI=0.11–0.14`. Trên costmap toàn cục T3, ô 100 gần nhất cách chỗ kẹt (0,46; 0,81) 0,29 m (tâm tới tâm).

## Đọc kết quả
- Đường vạch: 6 đỉnh, **không có CANH BAO**. Quanh K2 (đoạn 3–4, đỉnh 3 (0,359; 0,809) rẽ phải 60°, đỉnh 4 (0,609; 0,759)
  trái 11°): thân xe dọc đường vạch cách ô 254 nhỏ nhất 0,106 m (tại (0,50; 0,78)), cách ô ≥ 98 chỉ 0,048 m.
- Vết quanh K2 (18:57:14–18:57:17): lệch −0,008 / −0,011 m (phía TRONG khúc rẽ đỉnh 3), rồi +0,005 / +0,021 m (sếp, phía
  ngoài). **Không chỗ nào > 5 cm về phía trong.** Thân xe thật lúc đó cách ô 254: 0,17 / 0,17 / 0,13 / 0,09 m.
- Lệch lớn nhất là ở khúc rẽ đỉnh 2 (0,009; 0,409), không phải ở K2: +0,093 m về phía NGOÀI lúc 18:57:12; khi đó thân chạm ô ≥ 98
  và cách ô 254 0,059 m.
- RPP dừng vì "collision ahead" trên **local costmap** (không có trong dữ liệu), trong khi trên costmap toàn cục xe không chạm 254.
  Sếp cùng lúc thấy vật lạ phía trước ở chỗ bản đồ là ô trống.

Kết luận 1 dòng: không phải do xe lệch (≤ 1,1 cm về phía trong ở K2). Kẹt do đường vạch đã sát K2 (thân cách vùng ≥ 98 chỉ
4,8 cm, cách ô 254 khoảng 10 cm), cộng với vật/ô mới trên local costmap mà costmap toàn cục không có. Muốn chốt cần local costmap
hoặc scan lúc 18:57:17.

---

# Việc 3: định vị lúc kẹt 19:33 và góc thân (lỗi B) (CHỈ ĐỌC; chưa giả lập cả hệ thống)

Kiểm (1): `git status --short` không đổi so với việc 2 (chỉ nội dung `RA_SOAT_LOI_A.md`, tệp chưa theo dõi, dài thêm).

## Dữ liệu và giả định
- **Scan bị cắt:** `scan_ket_k2_1933.txt` là `ros2 topic echo` mặc định, mảng bị cắt sau 128 phần tử (`'...'`).
  Kiểm (2): đọc được **128** giá trị `ranges` (không phải ≈ 270; scan đầy đủ có 420 tia, −180..180°, bước 0,859°),
  121 điểm hợp lệ, chỉ phủ góc LiDAR **−179,1° .. −70,9°** (sau–phải xe). K2 (trước–trái) và vùng lớp 2 (trước mũi) KHÔNG có
  trong dữ liệu. Lần sau chụp bằng `ros2 topic echo --once --full-length /scan`.
- Tư thế tin: log trong `du_lieu_0511` dừng 19:04 ⇒ dùng (0,57; 0,85), góc −17° (log 19:28) như lệnh.
- TF LiDAR theo kho: `lai_xe_lidar.LIDAR_X = 0,025` m về phía mũi, không xoay (`scan_to_xy`, `lidar_angle_offset_deg` mặc định 0).
  Chữ `inverted` không có ở đâu trong kho (cấu hình driver YDLidar nằm trên Pi). Đã thử cả góc như `/scan` và góc lật (−θ).
  Góc lật đặt điểm ra ngoài bản đồ (y 1,05..2,50 m) ⇒ loại; dữ liệu `/scan` đã đúng chiều (driver đã áp `inverted`).
- Vật: H0 = các điểm có y < 0,60 (hộp dưới), K2 = y > 0,85 và x > 0,55 (theo `mo_phong_he_thong.py`: H0 (0,44–1,04; 0,37–0,52),
  K2 (0,88–1,28; 0,97–1,12)). Ô vật = giá trị 100 (= 254 LETHAL) trên costmap `costmap_ket_k2_1933`.
- Script scratchpad (không vào kho): `loi_b_scan.py`, `loi_b_tia.py`.

## Đầu ra
```
scan: 128 gia tri ranges doc duoc (BI CAT: ros2 topic echo in '...' sau 128 phan tu); scan day du co 420 tia (-180.0..180.0 do, buoc 0.859 do)
  co 121 diem hop le (range_min 0.10..range_max 12.0); goc LiDAR cac diem: -179.1..-70.9 do
tu the xe tin: (0.57; 0.85) goc -17 do (log 19:28: log trong du_lieu_0511 dung 19:04, khong co 19:33)

== (a) [o 100 = 254] goc nhu /scan, tu the tin ==
  H0  :  89 diem; cach mep o 100: tb 0.000 m, trung vi 0.000, max 0.000; diem trong o vat 89; vecto tb diem->o (0.001; 0.003) m (huong 72 do)
        lech toi BE MAT vat (mep o 100, + truoc mat / - lun vao): tb -0.071 m, trung vi -0.075, |.| tb 0.071
  K2  : 0 diem
  khac:  32 diem; cach mep o 100: tb 0.004 m, trung vi 0.000, max 0.052; diem trong o vat 25; vecto tb diem->o (0.003; -0.005) m (huong -60 do)
        lech toi BE MAT vat (mep o 100, + truoc mat / - lun vao): tb -0.008 m, trung vi -0.005, |.| tb 0.015
  vung the gioi cua cac diem: x -0.57..0.61, y -0.43..1.03
  do luoi (cham = |lech toi be mat| tb): tot nhat (0.66; 0.94) goc -16 do: lech so voi tin dx +0.09 dy +0.09 dgoc +1 do; |lech| tb 0.010 m, trung vi 0.007, 0/121 diem dung tren mep (tu the tin: |lech| tb 0.056)
  5 tu the tot nhat: (0.66;0.94;-16 do) 0.010; (0.67;0.93;-17 do) 0.010; (0.66;0.93;-16 do) 0.011; (0.57;0.77;-20 do) 0.011; (0.66;0.93;-17 do) 0.011

== (a) [o 100 = 254] goc LAT (-th), tu the tin ==
  H0  : 0 diem
  K2  :  41 diem; cach mep o 100: tb 0.168 m, trung vi 0.029, max 1.198; diem trong o vat 6; vecto tb diem->o (-0.009; -0.182) m (huong -93 do)
        lech toi BE MAT vat (mep o 100, + truoc mat / - lun vao): tb +0.166 m, trung vi +0.029, |.| tb 0.171
  khac:  80 diem; cach mep o 100: tb 0.588 m, trung vi 0.626, max 1.305; diem trong o vat 15; vecto tb diem->o (0.034; -0.605) m (huong -87 do)
        lech toi BE MAT vat (mep o 100, + truoc mat / - lun vao): tb +0.584 m, trung vi +0.626, |.| tb 0.593
  vung the gioi cua cac diem: x -0.29..0.82, y 1.05..2.50
  (bo do luoi cho goc LAT: diem roi ra ngoai ban do)

== (b) goc nhu /scan: than 25x16 -> diem scan gan nhat 0.277 m tai (0.149; -0.357) than, goc -67 do (0 = mui, + = trai); diem trong vung lop 2 (x 0,145-0,25; |y|<=0,10): 0
   goc cac diem trong khung than: -179..-67 do (phia phai-sau)
   goc -180..-150:  33 diem, than->diem min 0.500 m
   goc -150..-120:  33 diem, than->diem min 1.035 m
   goc -120.. -90:  28 diem, than->diem min 0.334 m
   goc  -90.. -60:  27 diem, than->diem min 0.277 m

== (b) goc LAT: than 25x16 -> diem scan gan nhat 0.277 m tai (0.149; 0.357) than, goc 67 do (0 = mui, + = trai); diem trong vung lop 2 (x 0,145-0,25; |y|<=0,10): 0
   goc cac diem trong khung than: 67..179 do (phia trai-sau)
   goc   60..  90:  27 diem, than->diem min 0.277 m
   goc   90.. 120:  28 diem, than->diem min 0.334 m
   goc  120.. 150:  33 diem, than->diem min 1.035 m
   goc  150.. 180:  33 diem, than->diem min 0.500 m

(c) nguong 72 tren thang topic 0..100 <=> chi phi Nav2 >= 185 (bang Costmap2DPublisher 1 + 97(c-1)/251; 253 -> 99, 254 -> 100)

(c) nguong 98 tren thang topic 0..100 <=> chi phi Nav2 >= 252 (bang Costmap2DPublisher 1 + 97(c-1)/251; 253 -> 99, 254 -> 100)
--- phan xu bang tia (loi_b_tia.py) ---
tin         (0.57; 0.85) -17 do: tia xuyen o 100 truoc diem cham 100/121; |lech be mat| tb 0.056 m
cuc tieu 1  (0.66; 0.94) -16 do: tia xuyen o 100 truoc diem cham 121/121; |lech be mat| tb 0.010 m
cuc tieu 2  (0.57; 0.77) -20 do: tia xuyen o 100 truoc diem cham 114/121; |lech be mat| tb 0.011 m
luoi 2 cm / 1 do, xep theo (so tia xuyen, |lech|): (0.63;0.91;-10 do) xuyen 13 lech 0.026; (0.63;0.93;-9 do) xuyen 13 lech 0.027; (0.63;0.93;-8 do) xuyen 13 lech 0.031; (0.65;0.91;-11 do) xuyen 13 lech 0.031; (0.65;0.93;-10 do) xuyen 13 lech 0.032; (0.65;0.91;-10 do) xuyen 13 lech 0.033
```

## Bảng (a): chiếu scan bằng tư thế tin
| Vật | Số điểm | Lệch tới bề mặt vật (mép ô 100) | Hướng |
|---|---|---|---|
| H0 | 89 | trung bình −0,071 m (lún VÀO vật 7 cm) | về phía nam (−y): LiDAR thấy mặt trên H0 xa hơn mép ô vật khoảng 7 cm |
| tường tây/khác | 32 | trung bình −0,008 m | khớp (≤ 1 cm) |
| K2 | 0 | — | không có trong 128 tia |

Dò lưới ±0,20 m (1 cm), ±10° (1°), chấm = |lệch tới bề mặt| trung bình: **không ra một đáp án**.
- Hai cực tiểu gần bằng nhau: (0,66; 0,94; −16°) 0,010 m (lệch so với tin +0,09; +0,09; +1°) và (0,57; 0,77; −20°)
  0,011 m (0; −0,08; −3°): hai hướng ngược nhau theo y.
- Thêm điều kiện tia (không xuyên ô 100 trước điểm chạm, bỏ 5 cm cuối): tư thế tin 100/121 tia xuyên (vì điểm H0 lún 7 cm);
  tốt nhất (0,63; 0,91; −10°) còn 13/121 (lệch +0,06; +0,06; +7°).
- Chỉ thấy một góc phần tư (mặt trên H0 + tường tây) nên x/y/góc bù trừ nhau. Kết luận được: tư thế tin lệch khỏi chỗ scan khớp
  khoảng 6–9 cm. Hướng (về K2 hay về H0) chưa chốt được.

## Bảng (b): khung thân xe (góc như `/scan`)
| Đo | Kết quả | Thiên đo |
|---|---|---|
| Thân 25×16 → điểm scan gần nhất | 0,277 m tại thân (0,149; −0,357), góc −67° (phải-trước, mép vùng có dữ liệu): mặt trên H0 | H0: 31 cm |
| Thân → K2 | không đo được (K2 ở trước–trái, ngoài 128 tia) | K2: 5,5 cm |
| Điểm trong vùng lớp 2 (x 0,145–0,25; \|y\| ≤ 0,10) | 0, nhưng vùng này không nằm trong dữ liệu (vô nghĩa) | — |
Theo góc: −180..−150° min 0,500 m; −150..−120° 1,035 m; −120..−90° 0,334 m; −90..−67° 0,277 m. H0 kéo dài ra ngoài −67° nên
khoảng cách thật tới H0 có thể nhỏ hơn 0,277 m. Số này đã nhỏ hơn 31 cm Thiên đo 3,3 cm.

## (c) Thang ngưỡng 72 và 98
`tinh_tuyen.Ban.tu_luoi` (đường `gui_duong.py` dùng trên Pi) so `data` của `/global_costmap/costmap` (OccupancyGrid) ⇒ **thang
topic 0–100**, không phải chi phí 0–252. Theo bảng Nav2 `Costmap2DPublisher` (c → 1 + 97(c−1)/251; 253 → 99; 254 → 100):
72 ⇔ chi phí ≥ 185; 98 ⇔ chi phí ≥ 252 (252, 253 inscribed, 254 lethal). Đường `Ban(yaml)` đọc PGM 3 màu, không dùng trên Pi.
