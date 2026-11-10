# VIỆC A+ (Thiên duyệt 10/11 15:34) — nhánh nao-xe-vung từ e9f0f94, CHƯA COMMIT. Chưa chạy trên Pi; giả lập là cành phụ.

## Bước 0 (chỉ đọc)
- Phân loại cụm: gac_vat_la.py `danh_gia` / `lidar_noi` (LiDAR: ngắn + hở hai bên ⇒ VAT, dài ≥ 0,40 ⇒ TUONG, còn lại KHONG_CHAC ⇒ VAT;
  bản đồ chỉ ghi LECH). Dùng ở: gui_duong `Mat.quet` → `SL.vung_do` (= GV.do_vat), `SL.vat_chac`, `truoc_khi_giao`, `theo_doi_mat`;
  sep_lidar `khe/cham/thieu/chon_dong_tac` dùng ĐIỂM THÔ (mọi loại, kể cả tường) qua `diem_loc`.
- Log 15:18–15:25 (Pi, chỉ đọc, chép về scratchpad): góc K4 (0,60..0,63; −0,17..−0,22) ⇒ "(vat) LECH ban do=tuong" ⇒ DOI_VAT 30 s ⇒ THOAT.
- Điều kiện từ chối THOAT:
  - (1) `thieu()` đòi hông+đuôi ≥ 0,06 với MỌI điểm (kể cả tường) ⇒ cạnh K4 0,04 ⇒ "không làm tốt lên" ⇒ KEP / HONG.
  - (2) `cham()` cấm xoay khi có điểm cách tâm ≤ 0,186.
  - (3) `ly_do_huy` (VAT hông ≤ 0,06, trước VAT ≤ 0,10) với K4 bị gọi là VAT.
  - (4) `di_diem` bỏ k0 truyền vào (gán lại None), nên k0 lấy lúc xe đã chạy ⇒ "truoc vat 0.10 <= 0.10" huỷ diem.
  - THOAT hỏng 3 lần ⇒ CHO_NGUOI.
- Vì sao Nav2 vạch sát K4 (không đổi tham số):
  - Tường bản đồ /map_tinh = 99 ⇒ 251, KHÔNG được inflation (InflationLayer chỉ phình ô 254).
  - Chỉ ô LiDAR (254) phình: r_ins 0,08 (padding 0), chi phí giảm nhanh (k 10: hở 0,05 ⇒ 152).
  - NavFn đổi chi phí lấy độ dài ở góc trong khúc rẽ ⇒ đường ôm góc K4 (plan 15:23:37 y −0,046 ở x 0,63–0,73, góc K4 y −0,165 ⇒ hở thân ≈ 0,04).

## Đã làm
1. gac_vat_la.py `ba_dau_hieu` (`phan_loai_cum`: cu | ba_dau_hieu, mặc định ba_dau_hieu):
   - a. Cung: chuỗi đoạn ngắn (DP 0,005, ≤ 0,06), ≥ 5 điểm, Kasa r 0,06–0,12, RMS ≤ 0,01, lồi về LiDAR.
   - b. Ngoài bản đồ: ≥ 2 điểm cách ô tường > 0,08.
   - c. Thẳng / gãy 90 ± 20 (DP 0,01).
   - Quyết: ống ⇒ VAT; có phần ngoài bản đồ ⇒ VAT; trên bản đồ + thẳng/gãy ⇒ TUONG; còn lại KHONG_CHAC: mũi/đuôi VAT, hông theo bản đồ.
   - Không có bản đồ (cụm không phải cung) ⇒ luật LiDAR cũ cho cụm đó.
   - MEM: `(tuong: tren ban do, goc 92 do)`, `(ong: cung r=0.09 rms=0.004)`, `(vat: ngoai ban do 29 diem)`, `(vat: khong chac, ...)`.
2. Hông + TƯỜNG: không đỏ, không DOI_VAT; chỉ `cham` khi kc ≤ khong_cham_m 0,02 (do_vat).
   - sep_lidar `khe` tách hs_v / hs_t; `thieu`: VAT ≥ 0,06, TUONG ≥ 0,02; `kep`: VAT hông ≤ 0,06 hoặc TUONG ≤ 0,02.
   - VAT CHẮC (nhắc đợi) = ống / ngoài bản đồ. Mũi / đuôi giữ nguyên.
3. thoat_ra_xa (mặc định true):
   - `cham`: bỏ cấm xoay theo 0,186; thêm "khe nhỏ nhất (mọi điểm, trong cho_chat) không giảm" suốt quãng quét (cấm bước làm gần hơn).
   - `di_diem` giữ k0 lúc trước khi chạy.
4. Đường lui: `phan_loai_cum: cu` + `thoat_ra_xa: false` = 629a17b.
   - test_08: 40 cảnh ngẫu nhiên (seed cố định), mã 629a17b (git archive) và mã mới cu/false ⇒ JSON giống hệt: danh_gia, MEM, do_vat, khe,
     cham 17 động tác, chon_dong_tac, thieu, ly_do_huy.
   - Các tệp test cũ (gui_duong 68, sep_lidar, gac_vat_la) ghim cu/false ⇒ đạt hết như 629a17b.
5. Khoá mới (yaml thoi_quen + hanh_vi.MAC_DINH; kiểu chuỗi được hanh_vi chấp nhận): phan_loai_cum, thoat_ra_xa, cung_r_min_m,
   cung_r_max_m, cung_rms_m, cung_so_diem, ngoai_ban_do_m, ngoai_so_diem, thang_rms_m, goc_gay_lech_deg, khong_cham_m.
   kiem_hanh_vi +3 dòng (46): phan_loai_cum ∈ {cu, ba_dau_hieu}; r_min < r_max, so_diem ≥ 3; 0 < khong_cham ≤ 0,06.
6. trang_v10: `_RE_MEM` nhận thêm `ong` và phần ly do sau dấu ":"; html: loai ≠ tuong ⇒ 물체.

## Test (đạt giả lập)
test_phan_loai_cum.py 10/10 (gia lap):
- Ống giữa sàn / sát tường / sát K4 0 / 1 / 3 cm / sát H0 ⇒ luôn VẬT (ống cung r 0,09; sát tường dính cụm với tường ⇒ VAT "ngoai ban do").
- Góc K4 từ 4 tư thế (gồm (0,76; −0,07; ±172°)), tường thẳng, góc H0 ⇒ TƯỜNG.
- Hộp lạ ⇒ VẬT chắc.
- Tường hông 0,04 không đỏ, 0,015 đỏ (chạm).
- thoat_ra_xa: lùi / xoay ra xa được khi đang đỏ; xoay vào gần bị cấm; cũ cấm xoay.
- (thêm trước commit) Xoay xét CẢ VÒNG QUÉT: `cham()` kiểm mọi mẫu 1° (tịnh tiến 5 mm), mọi điểm LiDAR so với đa giác thân 25×16 ở
  tư thế mẫu — không chỉ tư thế cuối. Ngưỡng không chạm (thoat_ra_xa): VẬT 0,06, TƯỜNG 0,02 (không nhãn: coi VẬT); lúc đầu đã dưới
  ngưỡng thì khoảng cách đó không được giảm ở mẫu nào; mũi (hành lang thân, mọi vật) không vào ≤ 0,07. Bỏ hai luật cũ (khe hông+đuôi
  không giảm, trước ≤ 0,10) khi thoat_ra_xa (đường lui cu/false giữ nguyên: test_08 vẫn giống hệt 629a17b).
  test_05b: ống Ø18 trước-trái cách hông 0,07: xoay trái 30 / 90 cấm ("cham vat", chạm ở mẫu giữa vòng 30°), xoay phải 30 được,
  xoay phải 90 cấm (góc sau-trái quét vào ống — đúng).
- (thêm trước commit) Ca 15:53 (không ống): test_09 — góc K4 ở hông 0,06 ⇒ TƯỜNG, ve_A ⇒ DEN_NOI, không DOI_VAT, không THOAT.
- Phát lại 15:18–15:25 (khe K4–ống 39,5 cm theo thước) ⇒ DEN_NOI, không CHO_NGUOI, không DOI_VAT, qua giữa khe.
  **Giới hạn:** cùng thế giới, đường lui cũng tới A (giả lập không tái hiện khoá cứng thật: bản đồ = thế giới, không lệch 4 cm,
  Nav2 giả không ôm góc) ⇒ test này chỉ chứng minh A+ đi qua, không chứng minh khác biệt. Không có scan thô của 15:23 (ảnh chụp chỉ có costmap/plan).
- du_lieu_0911: không có scan thô để phát lại (chỉ log), không phát lại được.

Hồi quy: test_gui_duong 68/68, test_sep_lidar + test_gac_vat_la + test_hanh_vi 27 (pytest), test_lai_xe_lidar 97/97, test_lop_huong_dan 10/10,
test_sep 48/48, test_trang_v10 61 OK, test_cau_noi OK, test_bat_kiem_cham 11/11; kiem_hanh_vi DAT 46; py_compile, yaml load đạt; ACCESS[=] diff 0.

## Tự hỏi "có trao quyền chặn cho thứ không phải giới hạn xe không?"
- Ống sát tường dính cụm với tường ⇒ CẢ cụm (gồm đoạn tường dính) là VAT ⇒ đoạn tường đó có đỏ hông 0,06 (bảo thủ, đúng luật "ống luôn VẬT").
- Không bản đồ ⇒ quay về luật LiDAR cũ (KHONG_CHAC hông = VAT) cho cụm đó: có thể khoá như cũ khi mất /map_tinh hay TF bản đồ.
- `cham()` thêm điều kiện "khe nhỏ nhất không giảm" (thoat_ra_xa): là "không làm gần hơn" của luật, không phải ngưỡng mới.

## D(1)
- Chữ log mới: MEM thêm `(tuong|vat|ong: <ly do>)`. Người đọc: trang_v10 `_RE_MEM` (đã sửa, test trang OK); cau_noi_xe.py KHÔNG đọc MEM
  (DONG_BAT/DONG_CHUYEN/DONG_TAT không khớp "MEM:"). Các chữ SEP_LIDAR / GAC / THOAT không đổi.
- /sep/trang_thai: KHÔNG đổi (sep.py không sửa); cau_noi / trang không đọc khoá mới.
- Tên tham số: 11 khoá thoi_quen mới (mục 5), dung_o ghi trong yaml.

## D(4) — khoá cứng theo luật mới (giả lập + đọc mã; số khe thật của Thiên)
| Chỗ | Luật mới | Lớp còn chặn |
|---|---|---|
| Cửa K2 (11/25 cm) | khung cửa trên bản đồ, thẳng/gãy ⇒ TUONG: hông chỉ chạm 0,02 | collision_monitor mũi 0,07; RPP thân thể; BackUp đuôi |
| Khe K4–H0 (K4–ống 39,5 / hông K4 6 cm) | góc K4 ⇒ TUONG (test 02); ống ⇒ VAT 0,06 hông; thoát ra xa không cấm bước không làm gần hơn | như trên + VAT ống |
| Miệng H0 | góc H0 ⇒ TUONG (test 02); chưa đo khe thật | như trên |
| Quanh A / B | tường thẳng ⇒ TUONG; xoay đầu: hết cấm 0,186, chỉ cấm làm gần hơn | như trên |
Nav2 vạch sát tường bản đồ (251 không phình) VẪN CÒN (không đổi tham số Nav2 theo lệnh) — hướng dẫn, không phải khoá.

## Tệp lên Pi (KHÔNG gồm bat_xe.sh, kiem_xe.sh)
| Tệp | md5 | Trên Pi |
|---|---|---|
| gac_vat_la.py | aaf050626501cc042fd9ba451aeeec01 | ~/ |
| sep_lidar.py | 8db645782eff52faf4c8efcec0bcc260 | ~/ |
| gui_duong.py | 2029de3574bdc85face98992a8f212a8 | ~/ |
| hanh_vi.py | 0100f88e1c8fa49d9ed494495886f06a | ~/ |
| hanh_vi_xe.yaml | b04a4c8230037fc4b7490b69afb290c3 | ~/ |
| kiem_hanh_vi.py | 5c77e8643593bb427f7ecfd9fb0a7f23 | ~/ |
| trang_v10.py | 8d38836e09bef90fee397b2cc904b51a | ~/cau_noi/ |
| trang_v10.html | 67dbb5298e6385727843df2fdfe340a2 | ~/cau_noi/ |

## Lên Pi (VIẾT SẴN, CHƯA CHẠY). C = commit A+ (Claude báo). Thay tệp: ghi `.moi` cạnh tệp rồi `mv` (không cp đè tệp đang chạy);
## không `rm` trên Pi; lệnh ssh / scp tách riêng; khởi động lại cau_noi_xe bằng systemctl, dán riêng.
```bash
# --- PC (1): xuat 8 tep tu commit, so md5 voi bang tren ---
C=<commit>
cd ~/ros2_ws/src/ros2
mkdir -p /tmp/moi_1011c
for f in gac_vat_la.py sep_lidar.py gui_duong.py hanh_vi.py hanh_vi_xe.yaml kiem_hanh_vi.py trang_v10.py trang_v10.html; do git show $C:xe_scripts/$f > /tmp/moi_1011c/$f; done
md5sum /tmp/moi_1011c/*
```
```bash
# --- PC (2): tao thu muc tren Pi (ssh, rieng) ---
ssh project1@192.168.0.2 'mkdir -p /tmp/moi_1011c'
```
```bash
# --- PC (3): chep len Pi (scp, rieng) ---
scp /tmp/moi_1011c/* project1@192.168.0.2:/tmp/moi_1011c/
```
```bash
# --- Pi (ssh project1@192.168.0.2): sao luu .bak_1011c (cp -n), ghi .moi canh tep, kiem, roi mv ---
cd /tmp/moi_1011c && ok=1
for f in gac_vat_la.py sep_lidar.py gui_duong.py hanh_vi.py hanh_vi_xe.yaml kiem_hanh_vi.py; do
  cp -n ~/$f ~/$f.bak_1011c && cp $f ~/$f.moi && cmp ~/$f.moi $f || ok=0; done
for f in trang_v10.py trang_v10.html; do
  cp -n ~/cau_noi/$f ~/cau_noi/$f.bak_1011c && cp $f ~/cau_noi/$f.moi && cmp ~/cau_noi/$f.moi $f || ok=0; done
[ $ok = 1 ] && python3 -c "import sys; [compile(open(f).read(), f, 'exec') for f in sys.argv[1:]]; print('PY_OK')" \
   ~/gac_vat_la.py.moi ~/sep_lidar.py.moi ~/gui_duong.py.moi ~/hanh_vi.py.moi ~/kiem_hanh_vi.py.moi ~/cau_noi/trang_v10.py.moi \
 && python3 -c "import yaml; yaml.safe_load(open('$HOME/hanh_vi_xe.yaml.moi')); print('YAML_OK')" && echo KIEM_MOI_DAT || echo KIEM_MOI_KHONG_DAT_DUNG_LAI
```
```bash
# --- Pi: chi khi KIEM_MOI_DAT: mv .moi vao cho (doi ten nguyen tu), roi kiem ---
for f in gac_vat_la.py sep_lidar.py gui_duong.py hanh_vi.py hanh_vi_xe.yaml kiem_hanh_vi.py; do mv ~/$f.moi ~/$f; done
for f in trang_v10.py trang_v10.html; do mv ~/cau_noi/$f.moi ~/cau_noi/$f; done
cd ~ && python3 ~/kiem_hanh_vi.py | tail -1
md5sum ~/gac_vat_la.py ~/sep_lidar.py ~/gui_duong.py ~/hanh_vi.py ~/hanh_vi_xe.yaml ~/kiem_hanh_vi.py ~/cau_noi/trang_v10.py ~/cau_noi/trang_v10.html
```
```bash
# --- Pi: khoi dong lai cau_noi_xe (trang) - dan rieng ---
sudo systemctl restart cau_noi_xe
```
Sau đó: bật lại hệ thống từ trang (gui_duong / hanh_vi đọc yaml lúc khởi động). PC: `bash xe_scripts/so_voi_pi.sh` ⇒ 8 tệp GIONG.

## Hai lệnh lui (Pi)
```bash
# (i) chi doi 2 dong yaml ve duong lui (= 629a17b), giu ma moi: ghi .moi, kiem, mv
sed -e 's/^  phan_loai_cum: {gia_tri: ba_dau_hieu,/  phan_loai_cum: {gia_tri: cu,/' -e 's/^  thoat_ra_xa: {gia_tri: true,/  thoat_ra_xa: {gia_tri: false,/' ~/hanh_vi_xe.yaml > ~/hanh_vi_xe.yaml.moi \
 && [ "$(grep -c 'phan_loai_cum: {gia_tri: cu,\|thoat_ra_xa: {gia_tri: false,' ~/hanh_vi_xe.yaml.moi)" = 2 ] \
 && python3 -c "import yaml; yaml.safe_load(open('$HOME/hanh_vi_xe.yaml.moi')); print('YAML_OK')" \
 && mv ~/hanh_vi_xe.yaml.moi ~/hanh_vi_xe.yaml && cd ~ && python3 ~/kiem_hanh_vi.py | tail -1
```
```bash
# (ii) tra ban truoc .bak_1011c: chep bak ra .moi, mv (bak giu nguyen)
for f in gac_vat_la.py sep_lidar.py gui_duong.py hanh_vi.py hanh_vi_xe.yaml kiem_hanh_vi.py; do cp ~/$f.bak_1011c ~/$f.moi && mv ~/$f.moi ~/$f; done
for f in trang_v10.py trang_v10.html; do cp ~/cau_noi/$f.bak_1011c ~/cau_noi/$f.moi && mv ~/cau_noi/$f.moi ~/cau_noi/$f; done
python3 -m py_compile ~/gac_vat_la.py ~/sep_lidar.py ~/gui_duong.py ~/hanh_vi.py ~/kiem_hanh_vi.py && cd ~ && python3 ~/kiem_hanh_vi.py | tail -1
```
```bash
# (ii) tiep: khoi dong lai cau_noi_xe - dan rieng
sudo systemctl restart cau_noi_xe
```
Ghi chú: thư mục /tmp/moi_1011c trên Pi để lại (không rm); xoá khi Thiên muốn.
