# Rà soát lỗi B, việc 119 bước 1: lớp 2 không thấy vật ở góc thân xe (CHỈ ĐỌC; chưa sửa mã; chưa giả lập cả hệ thống)

## 1) Mã hiện tại
- `sep.py` `_lop2` (dòng 810–835): chỉ xét khi có `SAP_GUI` mới trong `~/gui_duong_luot`; `L.vung_lui(pts)` đúng ⇒ ghi `RA_LUI`
  (lý do `vat_truoc_mui_<cach_truoc>`), sai ⇒ `KHONG_CAN vung_trong`. `pts` = `L.scan_to_xy(scan, lidar_angle_offset_deg)`.
- `lai_xe_lidar.py` `vung_lui` (dòng 582): ≥ `dem_diem_do` = 2 điểm trong chữ nhật `X_MUI ≤ x ≤ lop2_x_xa`, `|y| ≤ lop2_w`
  = **0,145 ≤ x ≤ 0,25 m, |y| ≤ 0,10 m**.
- `lai_xe_lidar.py` RA2 (`ra2_bat_dau` dòng 399, `_ra2` dòng 410): lùi thẳng `−v_esc` = −0,10 m/s, góc 0. Dừng theo thứ tự:
  chỉ nhìn; HUY; phía sau có ≥ 2 điểm trong `ra2_sau` 0,05 m; `cach_truoc(pts, lop2_w) ≥ ra2_cach` 0,30 m; đã lùi
  `ra2_di_max` 0,30 m; quá 6 s. `cach_truoc` (dòng 587) = khoảng cách từ TÂM xe tới điểm gần nhất có `x ≥ 0,145` và
  `|y| ≤ 0,10`; không có điểm thì INF.
- **Frame điểm LiDAR:** khung thân xe, gốc = tâm xe (`base_footprint`), x về mũi, y sang trái. `scan_to_xy`:
  `(LIDAR_X + r·cos θ, r·sin θ)`, **LiDAR tại (0,025; 0)**, không xoay (`lidar_angle_offset_deg` mặc định 0).
  Thân `X_DUOI..X_MUI` = −0,105..0,145, `Y_HONG` ±0,08 (25 × 16 cm); LiDAR cách mũi 0,12 m, cách đuôi 0,13 m.

## 2–4) Tính (script scratchpad `loi_b_119.py`)
```
than: x -0.105..0.145, y +-0.080 (tam xe = base_footprint); LiDAR tai (0.025; 0), khong xoay; vung lop 2 hien tai: 0.145<=x<=0.250, |y|<=0.10, >= 2 diem

(2) vat cach goc truoc than 5,5 cm:
| goc than | vi tri vat | diem vat (than) | cach LiDAR (m) | goc tia (do) | > range_min 0,10? | trong vung cu? | cach_truoc (m) |
| truoc-trai | canh hong (ngang goc) | (0.145; +0.135) | 0.181 | +48.4 | co | khong | INF (khong co diem) |
| truoc-trai | cheo 45 do tu goc | (0.184; +0.119) | 0.198 | +36.8 | co | khong | INF (khong co diem) |
| truoc-trai | truoc mui (doc goc) | (0.200; +0.080) | 0.192 | +24.6 | co | CO | 0.215 |
| truoc-phai | canh hong (ngang goc) | (0.145; -0.135) | 0.181 | -48.4 | co | khong | INF (khong co diem) |
| truoc-phai | cheo 45 do tu goc | (0.184; -0.119) | 0.198 | -36.8 | co | khong | INF (khong co diem) |
| truoc-phai | truoc mui (doc goc) | (0.200; -0.080) | 0.192 | -24.6 | co | CO | 0.215 |

(3) vung moi de xuat: x >= X_MUI - 0,05 (= 0.095) va khoang_cach_mep(than) <= 0,105 m (bang tam voi mui cu: 0,25 - 0,145)
   truoc-trai canh hong          (0.145; +0.135): vung cu False vung moi True
   truoc-trai cheo 45            (0.184; +0.119): vung cu False vung moi True
   truoc-trai truoc mui          (0.200; +0.080): vung cu True  vung moi True
   truoc-trai hong sau goc 4 cm  (0.105; +0.135): vung cu False vung moi True
   truoc-phai canh hong          (0.145; -0.135): vung cu False vung moi True
   truoc-phai cheo 45            (0.184; -0.119): vung cu False vung moi True
   truoc-phai truoc mui          (0.200; -0.080): vung cu True  vung moi True
   truoc-phai hong sau goc 4 cm  (0.105; -0.135): vung cu False vung moi True
   hanh lang 45 cm, xe giua: tuong cach hong 0,145 m -> vung moi False; xe lech 4,5 cm: cach 0,10 -> True

    scan 19:33: 128 gia tri (cat), 121 diem hop le, goc -180.0..-70.9 do; THIEU 292 tia goc -70.0..180.0 do (gom ca vung truoc mui va 2 goc truoc)
    diem trong vung cu: 0; trong vung moi: 0; khoang_cach_mep min 0.277 m
    tia LiDAR toi goc than: goc truoc-trai +33.7 do, goc truoc-phai -33.7 do; vung moi nam trong goc tia khoang -75..+75 do

(4) lui thang 0.30 m (ra2_di_max), v_esc 0.10 m/s; vat cach goc 5,5 cm; khoang cach than->vat nho nhat tren duong lui:
| vat | thang (delta 0) | delta +-2 do | +-5 do | +-10 do | delta toi han (cham) |
| canh hong trai (0,145; 0,135) | 0.055 | 0.055 | 0.055 | 0.055 | > 90 do |
| cheo trai (0,184; 0,119) | 0.055 | 0.055 | 0.055 | 0.055 | > 90 do |
| truoc mui trai (0,200; 0,080) | 0.055 | 0.055 | 0.055 | 0.055 | > 90 do |
| tuong doc hong trai y = 0,135 (x <= 0,145) | 0.055 | 0.053 | 0.049 | 0.042 | 35.2 do |
goc lech khi lui 0,30 m ma goc truoc-trai cham tuong doc hong: delta ~ 35.2 do; goc ap tren duong (lech ngang tam ~ d*delta/2 = 0.013 m o 5 do)
xoay tai cho (khong lui) toi khi goc truoc-trai cham vat canh hong: delta = nan do
```

## Đọc kết quả
**(2)** Vật cách góc trước 5,5 cm thì cách LiDAR 0,18–0,20 m, ở góc tia ±24,6° (trước mũi) đến ±48,4° (ngang hông). Mọi ca đều
> `range_min` 0,10 m, nên LiDAR thấy được. Vùng cũ chỉ bắt ca "trước mũi" (|y| = 0,08). Ca ngang hông (|y| = 0,135) và chéo 45°
(|y| = 0,119) **nằm ngoài |y| ≤ 0,10** ⇒ `vung_lui` sai ⇒ `KHONG_CAN`. Tia tới chính góc thân là ±33,7°.

**(3) Vùng mới đề xuất** (theo hình thân): điểm có `khoang_cach_mep(than) ≤ 0,105 m` (bằng tầm mũi cũ 0,25 − 0,145) và
`x ≥ X_MUI − 0,05 = 0,095` (nửa trước, gồm 5 cm hông sau góc); vẫn ≥ 2 điểm. Bắt cả 8 ca ở hai góc. Vùng cũ gần như nằm trọn trong
vùng mới: chỉ hai góc xa (0,25; ±0,10) cách thân 0,107 m, ra ngoài 0,105 chưa tới 2 mm.
- Rủi ro kích nhầm: hành lang 45 cm, xe ở giữa thì tường cách hông 0,145 m (không kích); **xe lệch ≥ 4,5 cm về một bên là kích**
  (tường vào trong 0,105 m). Nếu thấy nhiều, giảm bề hông (ví dụ 0,07) mà giữ bề mũi 0,105.
- **Phải đổi cùng `cach_truoc`** (điều kiện dừng RA2): hiện chỉ nhìn |y| ≤ 0,10. Với vật ở góc (|y| > 0,10) thì `c0` = `c` = INF
  ≥ 0,30 ⇒ **RA2 dừng ngay bước đầu "CACH_30CM 0 cm"**, không lùi. Đề xuất: điều kiện dừng = khoảng cách THÂN tới điểm gần
  nhất trong vùng mới ≥ ngưỡng, ví dụ 0,15 m (tương đương 0,30 tâm − 0,145 mũi).
- Dựng lại trên `scan_ket_k2_1933.txt`: bản này **bị cắt ở 128 phần tử**, còn 121 điểm ở góc −180..−70,9°; **thiếu 292 tia
  −70..+180°**, gồm toàn bộ vùng cũ, vùng mới và hai góc trước (tia ±33,7°). Vùng cũ 0 điểm, vùng mới 0 điểm, nhưng **không kết
  luận được** (vùng không có dữ liệu). Thân → điểm gần nhất trong phần còn lại: 0,277 m.
- Kích nhầm trên T1: trong `xe_scripts/du_lieu_0511/` không có scan nào của T1 (chỉ có `scan_ket_k2_1933.txt`; `sep_nhat_ky.csv`
  chỉ có dòng `LOP2` kết quả, không có điểm) ⇒ **chưa kiểm được**.

**(4) Lùi thẳng khi vật ở góc:** không quệt.
- Vật điểm ở góc (ngang hông, chéo, trước mũi): khoảng cách nhỏ nhất luôn là 0,055 m lúc bắt đầu, lùi chỉ làm xa thêm, kể cả lệch
  góc tới 10°. Xoay tại chỗ cũng không chạm: góc thân cách tâm 0,166 m, vật cách tâm ≥ 0,198 m.
- Ca xấu nhất là tường chạy dọc hông trái (y = 0,135): lùi 0,30 m với lệch góc tích luỹ 2° / 5° / 10° còn 0,053 / 0,049 / 0,042 m;
  phải lệch **35°** mới chạm. Lệch ngang tâm khi lệch 5° chỉ ≈ 0,013 m.
- Ca "đuôi cách vật sau < 5 cm" thì RA2 đã dừng bằng `ra2_sau`.

---

# Việc 119 bước 2: đã sửa (CHƯA commit; CHƯA chạy trên Pi; **chưa giả lập cả hệ thống**)

## Sửa gì
- `lai_xe_lidar.py`:
  - Hằng mới, có ghi giá trị cũ cạnh từng hằng: `LOP2_D_THAN = 0.105` (cũ: chữ nhật tới x 0,25), `LOP2_X_TU = X_MUI − 0.05 = 0.095`
    (cũ: x ≥ 0,145), `LOP2_W_CU = 0.10` (|y| cũ, không còn dùng, giữ để trả lại), `RA2_CACH_THAN = 0.15` (cũ: `ra2_cach` 0,30 từ tâm).
  - `DEFAULT` (cũng là tham số ROS của nút): `lop2_x_xa`, `lop2_w`, `ra2_cach` → `lop2_d_than`, `lop2_x_tu`, `ra2_cach_than`.
    `bat_xe.sh` không truyền tham số nào trong số này (đã kiểm), nên không phải sửa chỗ khác.
  - `vung_lui(pts)`: ≥ 2 điểm có `x ≥ 0,095` và `khoang_cach_mep ≤ 0,105`.
  - `cach_truoc(pts)`: khoảng cách từ THÂN (cũ: từ tâm) tới điểm gần nhất có `x ≥ 0,095` (cũ: x ≥ 0,145 và |y| ≤ 0,10).
  - `_ra2`: thứ tự dừng = chỉ nhìn; HUY; sau < 5 cm; **TRUOC_TRONG** (phía trước x ≥ 0,095 không có điểm nào: lý do riêng, thay cho
    `CACH_30CM 0 cm`); thân-trước ≥ 0,15 ⇒ `CACH_15CM`; đã lùi 0,30 m; quá 6 s. Câu bắt đầu: `RA2 BAT DAU LUI (than-truoc …)`.
- `sep.py` `_lop2`: lý do `vat_truoc_mui_<m>` → `vat_cach_than_<m>` (thân → vật). Vùng gọi `L.vung_lui(pts)` như cũ.

**Hiểu lệnh khác chữ (xin Thiên duyệt):** "dừng khi thân cách điểm gần nhất trong vùng mới ≥ 0,15 m". Vùng mới chỉ tới 0,105 m
nên trong vùng không bao giờ có điểm ≥ 0,15. Vì thế vùng đo để DỪNG lấy cùng hình (x ≥ 0,095) nhưng không giới hạn 0,105. Hệ quả:
tường hai bên nằm ở nửa trước cũng được tính. Trong khe mà hông cách tường < 0,15 m (ví dụ khe 52 cm lệch ≥ 3 cm) thì RA2 không bao
giờ đạt `CACH_15CM`, lùi đủ 0,30 m (`DA_LUI_30CM`) hoặc dừng vì sau < 5 cm.

## Thử (`test_lai_xe_lidar.py` 86/86; `test_sep.py` 45/45; `test_gui_duong.py` 29/29)
Ca cũ đổi theo luật mới (nói rõ):
- `cach_truoc` vật 8 cm trước mũi: nay 0,08 (thân), cũ 0,225 (tâm).
- Lùi tới `CACH_15CM` (cũ `CACH_30CM`): vẫn lùi 7 cm như cũ.
- Hai ca `ra2_cach=0.60` → `ra2_cach_than=0.60`.

Ca mới (scan dựng tay bằng `make_scan`, 410 tia):
```
PASS  119 vat cach goc than 5,5 cm (8 ca): vung moi kich, lui that (CACH_15CM, >= 5 cm, khong cham, khong xoay)  | [('trai ngang hong', True, False, 0.055, 'RA XONG CACH_15CM 15 cm', 0.15, False),
    ('trai cheo 45', True, False, 0.055, 'RA XONG CACH_15CM 11 cm', 0.11, False),
    ('trai truoc mui', True, True, 0.055, 'RA XONG CACH_15CM 10 cm', 0.1, False),
    ('trai hong sau goc 4 cm', True, False, 0.055, 'RA XONG CACH_15CM 19 cm', 0.19, False),
    ('phai ngang hong', True, False, 0.055, 'RA XONG CACH_15CM 15 cm', 0.15, False),
    ('phai cheo 45', True, False, 0.055, 'RA XONG CACH_15CM 11 cm', 0.11, False),
    ('phai truoc mui', True, True, 0.055, 'RA XONG CACH_15CM 10 cm', 0.1, False),
    ('phai hong sau goc 4 cm', True, False, 0.055, 'RA XONG CACH_15CM 19 cm', 0.19, False)]
PASS  119 vung cu bo sot ca ngang hong / cheo / hong sau goc (6/8), chi bat truoc mui  | [('trai ngang hong', False),
    ('trai cheo 45', False),
    ('trai truoc mui', True),
    ('trai hong sau goc 4 cm', False),
    ('phai ngang hong', False),
    ('phai cheo 45', False),
    ('phai truoc mui', True),
    ('phai hong sau goc 4 cm', False)]
PASS  119 ong thang truoc mui 0,21 m: vung moi va vung cu deu kich, lui toi than cach >= 0,15 m  | RA XONG CACH_15CM 9 cm 0.090
      khe (rong, lech): (vung moi, vung cu) = {(0.52, 0.0): (False, False),
    (0.52, 0.05): (False, False),
    (0.52, 0.07): (False, False),
    (0.52, 0.09): (True, False),
    (0.56, 0.0): (False, False),
    (0.56, 0.05): (False, False),
    (0.56, 0.07): (False, False),
    (0.56, 0.09): (False, False)}
PASS  119 khe 52 cm: lech 0/5/7 cm khong kich, 9 cm kich (tuong cach hong 9 cm <= 10,5); khe 56 cm: 0..9 cm khong kich; vung cu khong kich  | {(0.52, 0.0): (False, False),
    (0.52, 0.05): (False, False),
    (0.52, 0.07): (False, False),
    (0.52, 0.09): (True, False),
    (0.56, 0.0): (False, False),
    (0.56, 0.05): (False, False),
    (0.56, 0.07): (False, False),
    (0.56, 0.09): (False, False)}
PASS  119 RA2 ma phia truoc khong co diem nao: ly do TRUOC_TRONG ngay buoc dau (khong ghi CACH_15CM)  | RA XONG TRUOC_TRONG 1 cm
PASS  119 hang ten co gia tri cu canh ben, DEFAULT dung hang
```
- 8 ca góc (hộp 2 cm, mặt gần nhất cách góc thân 5,5 cm; trái/phải × ngang hông, chéo 45°, trước mũi, hông sau góc 4 cm): vùng mới
  kích cả 8. Lùi thật 0,10 / 0,11 / 0,15 / 0,19 m (trước mũi / chéo / ngang hông / hông sau góc) tới `CACH_15CM`, không chạm, không
  xoay. Vùng cũ chỉ bắt 2 ca trước mũi.
- Ống Ø18 thẳng trước mũi, mặt cách tâm 0,21 m: vùng cũ và vùng mới đều kích; lùi 9 cm tới `CACH_15CM`.
- Khe (tường dài hai bên), xe lệch e về một bên:
  | khe | e = 0 | 5 cm | 7 cm | 9 cm | bắt đầu kích khi |
  |---|---|---|---|---|---|
  | 52 cm | không | không | không | **kích** | e ≥ 7,5 cm (hông cách tường ≤ 10,5 cm) |
  | 56 cm | không | không | không | không | e ≥ 9,5 cm |
  Vùng cũ không kích ở mọi ca khe.
- RA2 khi phía trước không có điểm: `RA XONG TRUOC_TRONG` ngay bước đầu (con số cm là ước theo lệnh 0,1 s, như cũ).

## Kiểm tích hợp mục D(1)
| Chữ / tệp / tên | Bên ghi | Bên đọc | Khớp? |
|---|---|---|---|
| `LOP2` (loại dòng nhật ký) | `sep.py` `_lop2` (`ghi('LOP2', …)`, 3 chỗ) | người đọc `sep_nhat_ky.csv` | khớp, không đổi |
| `RA_LUI` trong `~/sep_thoat` | `sep.py` `ly_ra2` (`'RA_LUI', '<ma_luot> <ma>'`) | `lai_xe_lidar.doc_sep_ra2` (cả `gui_duong.py` dòng 442 dùng hàm này) | khớp, không đổi |
| `RA_XONG` (tên trạng thái phụ lái) | `lai_xe_lidar._ket_thuc(…'RA_XONG'…)` | `sep.py` (`t_ra_xong`, `DANG_LAM`), `gui_duong.py` (`e[1] == 'RA_XONG'`) | khớp, không đổi |
| câu `RA XONG <ly> <cm> cm` | `lai_xe_lidar` | `sep.doc_rosout` KHÔNG bắt câu này (`test_sep` dòng 475) | lý do mới `TRUOC_TRONG`, `CACH_15CM` không ai so khớp |
| `~/gui_duong_luot` | `gui_duong.py` (`SAP_GUI`/`HUY`) | `sep.py` (`tep_luot`), `lai_xe_lidar.luot_huy` | khớp, không đổi |
| `vung_lui`, `cach_truoc` | `lai_xe_lidar.py` | `sep.py` dòng 829–831 (gọi với mặc định) | khớp; chữ ký đổi (`d`, `x_tu`), sep không truyền đối số |
| tham số `lop2_d_than`, `lop2_x_tu`, `ra2_cach_than` | `lai_xe_lidar.DEFAULT` (tham số ROS nút phụ lái) | `NguoiLai.p`; `sep.py` dùng mặc định mô-đun | khớp; tên cũ chỉ còn trong chú thích. Như trước: đổi tham số ROS của phụ lái thì sếp KHÔNG theo |
| lý do `vat_cach_than_<m>` | `sep.py` | chỉ nhật ký; không tệp nào so chữ `vat_truoc_mui` | khớp |
| `bat_xe.sh` | không truyền `lop2_*`/`ra2_*` | — | khớp |

`python3 -m py_compile sep.py lai_xe_lidar.py gui_duong.py`: đạt.

md5 tệp đã sửa: `lai_xe_lidar.py` 34a8ff44a440e930ffce046b2353ed56, `sep.py` c8111ff53835b9523858bfa25dcf92bb,
`test_lai_xe_lidar.py` 2a71191d7ae852df3de92a807ada069b (`gui_duong.py` không đổi: 6c937df6922ed236ca46f41f4b762877).
