# Sửa lỗi bật xe 09/11 21:29 — nhánh nao-xe-vung (629a17b + sửa chưa commit). Chưa chạy trên Pi; không push.
Thiên chốt bước 0 (10/11): mã 2 không tự tắt; ngân sách đọc lại tổng 30 s; test ngân sách + trap không lặp.

## bat_xe.sh (+18 dòng)
- Ngay sau dòng bật launch chính (B1, tiến trình nền đầu tiên): `trap tu_tat_khi_loi EXIT`, `trap 'exit 130' INT`, `trap 'exit 143' TERM`.
- Thoát mã ≠ 0 (lỗi B1, thiếu NAV_PARAMS, Nav2 không lên, `cd ~`, KQ=1 cuối, 130 Ctrl-C, 143 TERM): gỡ trap rồi in
  `TU_TAT_SAU_LOI (ma N): bash <tệp> tat`, chạy `bash "$0" tat`; tắt lỗi ⇒ `TU_TAT_SAU_LOI: tat chua sach (ma M), …; khong thu lai`;
  thoát giữ mã cũ. Mã 2 (thiếu ~/kiem_xe.sh): `CON_CHAY_DE_DOC_TAY: tat bang: bash ~/bat_xe.sh tat`, không tắt.
- Không lặp: trap tự gỡ trước khi tắt; nhánh `tat` thoát trước dòng đặt trap. `exec` (bật lại Nav2 lần 2) không chạy trap cũ, bản mới tự đặt lại.
  Nhánh bật lại mà "tat chưa sạch" (dòng 278, exit 1) ⇒ trap thử tắt thêm 1 lần (chấp nhận).

## kiem_xe.sh (+48 −3)
- Đọc lại (tối đa 3 lần, cách KIEM_DOC_LAI_S = 3 s, in `DOC_LAI    <muc>: lan n/3 sau 3 s (han H s, ngan sach con C s)`) chỉ khi ĐỌC LỖI:
  yaw_nguon, gyro_scale, dau_yaw, bt_xml (rỗng hoặc HET_GIO); vị trí (tf2_echo không ra số); ros2 node list (qua hạn / < 3 node);
  8d /lop_huong_dan (lệnh không in gì; "0" là giá trị ⇒ không đọc lại); 8e ros2 action list (rỗng). Giá trị sai ⇒ KHONG DAT ngay.
- Ngân sách tổng KIEM_DOC_LAI_TONG_S = 30 s mỗi lần chạy kiem_xe: chỉ bắt đầu đọc lại khi còn > 3 s; hạn lệnh đọc lại = min(hạn thường,
  còn − 3); tính thời gian thật + 1 s mỗi lần (SECONDS nguyên giây) ⇒ không vượt 30 s. Hết ⇒ `HET_NGAN_SACH_DOC_LAI: da dung x/30 s; muc
  con lai khong doc lai (<muc>)` một lần, các mục sau báo như cũ ("khong doc duoc"). Không đổi ngưỡng nào (hạn 10 s / 8 s lần đầu giữ).
- Ngoài phạm vi: hai bước python (lop_huong_dan --kiem-costmap, gac_vat_la --kiem-map) có hạn chờ riêng, không in "khong doc duoc".

## Thời gian (mục 3)
- Chậm thêm tối đa do đọc lại: **30 s / một lượt kiem_xe**, **90 s / 3 lượt** (bat_xe chạy kiem_xe tối đa 3 lượt, mỗi lượt là tiến trình
  mới ⇒ ngân sách mới) = 19 % của hạn 8 phút (480 s kể từ con quay lấy số 0).
- So với hạn (kể từ số 0, xấu nhất mọi lệnh ros2 đều treo hết hạn): B2 Nav2 ≤ 120 s + B2b 5 s + daemon 5 s + 3 lượt kiem (mỗi lượt kiểu mắt
  ≈ 117 s: 3×10 + 8 + 10 + 10 + 8 + 10 + ~10 + ~1 + ~10 + 10) + 2×5 s chờ = 491 s TRƯỚC sửa (đã quá 480), **581 s** sau sửa.
  Thường (lệnh trả lời): một lượt kiem cỡ 30–40 s, 3 lượt chỉ khi KHONG DAT; đọc lại chỉ chạy khi có mục đọc lỗi.

## D(1)
- Chữ mới DOC_LAI, HET_NGAN_SACH_DOC_LAI, TU_TAT_SAU_LOI, CON_CHAY_DE_DOC_TAY: KHÔNG tệp nào khác đọc (grep cau_noi_xe.py, trang_v10.py,
  trang_v10.html, chon_b_v4.html, sep.py, gui_duong.py: 0). Chúng nằm trong log bật của cau_noi; DONG_BAT
  `^(SAN SANG|CHUA SAN SANG|QUA 8 PHUT|DA BAT|LOI)` không khớp ⇒ dòng kết bật trên /nut và /v10 giữ dòng lỗi gốc (test).
  `CON_CHAY_DE_DOC_TAY` khớp DONG_TAT (`^CON_CHAY`) nhưng DONG_TAT chỉ đọc log nút TAT (bat_xe.sh tat thoát trước trap, không in dòng này).
- Lối thoát của tắt trong trap in KIEM_XONG / CON_CHAY / DA_DON vào log bật: không khớp DONG_BAT.
- **Rủi ro (chưa sửa, ngoài lệnh):** cau_noi_xe.py `job_max_s` 420 s tính từ lúc chạy bat_xe.sh; quá hạn ⇒ `killpg SIGKILL` ⇒ trap KHÔNG chạy
  (SIGKILL) ⇒ tiến trình còn lại như 21:29. Xấu nhất B1 60 + B2 120 + 10 + 3 lượt kiem đã ≈ 551 s > 420 trước sửa; nay +90 s. Đề xuất:
  cau_noi gửi TERM trước (trap tắt, mã 143) rồi mới KILL, hoặc tăng job_max_s; Thiên chọn.

## Test (test_bat_kiem_cham.sh, mới; lệnh giả trong PATH, HOME tạm, không đụng tiến trình / /dev/shm thật): 11/11
kiem_xe: đọc lỗi 2 lần rồi được ⇒ DAT; lỗi mãi ⇒ 3 lần rồi KHONG DAT "khong doc duoc"; giá trị sai ⇒ KHONG DAT ngay, 0 DOC_LAI;
treo 10 s (HET_GIO) rồi được + vị trí rỗng rồi được ⇒ DAT; ngân sách 5 s hết giữa chừng (HET_NGAN_SACH 1 lần, vị trí không đọc lại,
cả lần ≤ 5 s đọc lại); ngân sách 8 s với lệnh treo (đọc lại hạn 7 s, rồi hết; cả lần chạy 18 s).
bat_xe: launch thoát sớm ⇒ TU_TAT_SAU_LOI (ma 1), tắt sạch, mã 1; tắt tự lỗi ⇒ 1 lần TU_TAT_SAU_LOI + "tat chua sach", không lặp, mã 1;
Ctrl-C lúc chờ B1 ⇒ (ma 130), tắt sạch, mã 130; mã 2 ⇒ CON_CHAY_DE_DOC_TAY, không tắt; D(1) DONG_BAT.
Hồi quy: test_cau_noi 8 OK, test_trang_v10 61 OK, test_gui_duong 68/68, test_hanh_vi 6; bash -n 2 tệp đạt.
ACCESS[=] trong diff: 0 (cả grep -a; dòng thêm/bớt chứa ACCESS: 0; test chỉ có "grep -v ACCESS").
so_voi_pi.sh: không tệp mới lên Pi (test_* loại trừ); `--nhac` có thêm the_gioi_gia.py (từ 629a17b, không phải lần này).

## md5
| Tệp | md5 | Trên Pi |
|---|---|---|
| bat_xe.sh | b8feed4df45688d2c191d8eaa7f8b3d1 (cả tệp, bản kho); bỏ dòng ACCESS (cách so_voi_pi.sh so): af3860e2c1d93f82ac3d674f16f31f12 | ~/ |
| kiem_xe.sh | dc153a42913dabe2cb1c623a502e4803 | ~/ |
| test_bat_kiem_cham.sh | e4bc24351dc85b23e569ec5659ea1531 | không lên Pi |
Lưu ý: bat_xe.sh trên Pi có mã thật ở dòng ACCESS ⇒ chép tay phần sửa (khối trap 18 dòng ngay sau dòng setsid launch chính), không chép đè tệp.

## Việc sau 11/11
- Rủi ro SIGKILL 420 s của cau_noi_xe.py (`job_max_s`, `killpg SIGKILL` ⇒ trap không chạy ⇒ tiến trình còn sót). Để sau buổi chấm; chưa sửa
  cau_noi_xe.py. Hướng: TERM trước (trap tự tắt, mã 143) rồi mới KILL, hoặc tăng job_max_s.

## Lên Pi (VIẾT SẴN, CHƯA CHẠY). Không chép tay khối trap. Không lệnh nào in dòng ACCESS (chỉ md5 / số dòng / cmp).
C = commit của báo cáo này (`git rev-parse --short nao-xe-vung` sau commit; Claude báo trong chat).
```bash
# --- PC ---
C=<commit>
cd ~/ros2_ws/src/ros2 && mkdir -p /tmp/moi_1011b
git show $C:xe_scripts/bat_xe.sh      > /tmp/moi_1011b/bat_xe.sh
git show 629a17b:xe_scripts/bat_xe.sh > /tmp/moi_1011b/bat_xe_629a17b.sh
git show $C:xe_scripts/kiem_xe.sh     > /tmp/moi_1011b/kiem_xe.sh
grep -av ACCESS /tmp/moi_1011b/bat_xe.sh | md5sum      # mong af3860e2c1d93f82ac3d674f16f31f12
md5sum /tmp/moi_1011b/kiem_xe.sh                         # mong dc153a42913dabe2cb1c623a502e4803
ssh project1@192.168.0.2 'mkdir -p /tmp/moi_1011b'
scp /tmp/moi_1011b/bat_xe.sh /tmp/moi_1011b/bat_xe_629a17b.sh /tmp/moi_1011b/kiem_xe.sh project1@192.168.0.2:/tmp/moi_1011b/

# --- Pi (ssh project1@192.168.0.2) ---
cd /tmp/moi_1011b
# 5a: bat_xe.sh tren Pi chi khac ban 629a17b o dong ACCESS (cung so dong, cung vi tri dong ACCESS, phan con lai y het)
[ "$(wc -l < ~/bat_xe.sh)" = "$(wc -l < bat_xe_629a17b.sh)" ] \
 && diff <(grep -an ACCESS ~/bat_xe.sh | cut -d: -f1) <(grep -an ACCESS bat_xe_629a17b.sh | cut -d: -f1) \
 && cmp <(grep -av ACCESS ~/bat_xe.sh) <(grep -av ACCESS bat_xe_629a17b.sh) \
 && [ "$(grep -ac ACCESS ~/bat_xe.sh)" = "$(grep -ac ACCESS bat_xe.sh)" ] && echo 5A_DAT || echo 5A_KHONG_DAT_DUNG_LAI
# 5b (chi khi 5A_DAT): ghep ban moi, thay tung dong ACCESS bang dong ACCESS cua Pi theo thu tu
awk 'NR==FNR { if ($0 ~ /ACCESS/) a[++n] = $0; next } /ACCESS/ { print a[++m]; next } { print }' ~/bat_xe.sh bat_xe.sh > bat_xe_ghep.sh
[ "$(grep -av ACCESS bat_xe_ghep.sh | md5sum | cut -d' ' -f1)" = af3860e2c1d93f82ac3d674f16f31f12 ] \
 && cmp <(grep -a ACCESS ~/bat_xe.sh) <(grep -a ACCESS bat_xe_ghep.sh) \
 && bash -n bat_xe_ghep.sh \
 && cp -n ~/bat_xe.sh ~/bat_xe.sh.bak_1011b && cp bat_xe_ghep.sh ~/bat_xe.sh \
 && bash -n ~/bat_xe.sh && grep -av ACCESS ~/bat_xe.sh | md5sum && echo 5B_DAT || echo 5B_KHONG_DAT
# kiem_xe.sh: chep thang
cp -n ~/kiem_xe.sh ~/kiem_xe.sh.bak_1011b && cp kiem_xe.sh ~/kiem_xe.sh && cmp ~/kiem_xe.sh kiem_xe.sh \
 && bash -n ~/kiem_xe.sh && md5sum ~/kiem_xe.sh && echo KIEM_DAT || echo KIEM_KHONG_DAT    # mong dc153a42913dabe2cb1c623a502e4803
rm -f /tmp/moi_1011b/bat_xe_ghep.sh     # ban ghep co dong ACCESS that: xoa khoi /tmp

# --- Tra ve ban truoc (Pi) ---
cp ~/bat_xe.sh.bak_1011b ~/bat_xe.sh && cp ~/kiem_xe.sh.bak_1011b ~/kiem_xe.sh \
 && bash -n ~/bat_xe.sh && bash -n ~/kiem_xe.sh && grep -av ACCESS ~/bat_xe.sh | md5sum && md5sum ~/kiem_xe.sh
```
Sau khi lên: `bash xe_scripts/so_voi_pi.sh` trên PC ⇒ bat_xe.sh, kiem_xe.sh GIONG.
