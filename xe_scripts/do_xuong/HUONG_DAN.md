# do_xuong: đo ở xưởng (việc 118 B10, soạn 06/11/2026, CHƯA CHẠY TRÊN XE)

Ba công cụ chỉ đo, không sửa gì trên xe. Chỉ `do_toc_do.sh` phát lệnh chạy, các công cụ còn lại chỉ nghe.

| Tệp | Làm gì | Chạy ở đâu |
|---|---|---|
| `do_toc_do.sh` | Cho xe đi thẳng bằng `/cmd_vel_teleop` 0,05 m/s trong 10 s, in odom đầu/cuối. Thiên đo thước để so. | Pi |
| `ghi_sua_vi_tri.py` | Chỉ nghe `/tf`, ghi **mọi** lần SLAM sửa map → odom (giờ, độ lớn) ra CSV | Pi |
| `khop_scan_A.py` | `chup`: lấy 1 scan + tư thế tin (Pi). Không có `chup`: khớp scan với map_xuong_v3 như RA_SOAT_118 B2 (máy tính) | Pi / máy tính |

Hai tab:
- tab **`aaa@`**: máy tính (WSL), dấu nhắc `aaa@...:~$`;
- tab **`project1@`**: Pi, mở bằng `ssh -t project1@192.168.0.2`, dấu nhắc `project1@...:~$`.

## Phần 0: chép lên Pi (một lần)

Tab **`aaa@`**:

```
scp -r ~/ros2_ws/src/ros2/xe_scripts/do_xuong project1@192.168.0.2:~/ && ssh project1@192.168.0.2 'bash -n ~/do_xuong/do_toc_do.sh && python3 -m py_compile ~/do_xuong/ghi_sua_vi_tri.py ~/do_xuong/khop_scan_A.py && echo CHEP_OK'
```

Mong: `CHEP_OK`.

## Phần 1: đo tốc độ thật so với odom (`do_toc_do.sh`)

**Trước khi chạy:**
- Xe bật bằng `bat_xe.sh` như mọi lần. Cần odom_tu_lenh, twist_mux, dieu_khien đang chạy.
- **Nav2 không có đích**: không chạy `chuyen.sh` / `gui_duong.py`.
- Phía trước mũi xe trống **ít nhất 1,6 m**. Thêm khoảng 25 cm mỗi bên vì xe có thể đi cong: trong giả lập có tật M2, xe lệch phải khoảng 18° sau 10 s.
- Dán băng dính đánh dấu **một điểm cố định trên xe** (ví dụ mép trước bánh phải) lên sàn, gọi là **vạch đầu**.

Tab **`project1@`** (chạy ở tab hiện ra, để Ctrl+C dùng được):

```
source ~/ros2_ws/install/setup.bash; bash ~/do_xuong/do_toc_do.sh
```

Mong thấy:

```
CHAY: 0.05 m/s thang trong 10 s. Ctrl+C = HUY (xe dung ngay); tu tab khac: kill -INT <so>
DA DUNG (Twist 0 trong 1 s).
odom dau  (...)
odom cuoi (...)
XONG: lenh 0.05 m/s trong 10.0 s | odom: quang thang 0.9.. m, doc huong dau ... m, goc doi ... do
```

**Đo thước:**
- Đánh dấu **vạch cuối** đúng điểm cũ trên xe.
- Đo **quãng thẳng** từ vạch đầu tới vạch cuối, ghi cạnh số `odom: quang thang`.
- Đo thêm khoảng lệch ngang so với đường thẳng hướng đầu, nếu tiện.
- Làm **3 lần**. Mỗi lần nối thêm một dòng vào `~/do_xuong/do_toc_do.csv`.

**HUỶ:**
- **Ctrl+C**: xe nhận Twist 0 liên tục 1 s rồi script thoát, mã 130, dòng ghi là `HUY`.
- Mất ssh cũng dừng như vậy.
- Từ tab khác: `kill -INT <so>` (số in ở dòng `CHAY`).
- Nếu script chết cứng (kill -9), twist_mux tự bỏ lệnh teleop sau 0,3 s.

**Đọc số [đọc code odom_tu_lenh.py:267–283, thử trong giả lập]:**
- Với lệnh 0,05 m/s, cả hai bánh nằm dưới `duty_min` 0,21 (bánh bị nâng). Odom luôn tính 0,21 × 0,437 = **0,0918 m/s**, nên sau 10 s odom ≈ **0,92 m**, bất kể tốc độ thật.
- Giả lập: tật M2 0,12 → thật **1,25 m**, odom 0,92 m; không tật → thật 0,96 m, odom 0,92 m.
- Thước ra **≈ 1,2 m** ⇒ tật M2 có trên xe thật (thật / odom ≈ 1,3). Thước ra **≈ 0,92 m** ⇒ odom đúng ở tốc độ này.

## Phần 2: ghi mọi lần SLAM sửa vị trí (`ghi_sua_vi_tri.py`)

Bật **trước** khi cho xe chạy tuyến, ở một tab **`project1@`** riêng:

```
source ~/ros2_ws/install/setup.bash; python3 ~/do_xuong/ghi_sua_vi_tri.py
```

- Mong: `map -> odom dau tien: (...). Ghi vao /home/project1/sua_vi_tri_<ngay_gio>.csv`.
- Mỗi lần sửa ≥ 5 cm in một dòng `SUA ... cm`. Mọi lần sửa (kể cả < 1 mm) đều vào CSV.
- Xong buổi: **Ctrl+C**, mong `DUNG: <n> lan doi da ghi vao ...`.

Lấy tệp về, ở tab **`aaa@`**:

```
scp 'project1@192.168.0.2:~/sua_vi_tri_*.csv' ~/ros2_ws/src/ros2/xe_scripts/du_lieu_0711/
```

(tạo thư mục trước: `mkdir -p ~/ros2_ws/src/ros2/xe_scripts/du_lieu_0711`; đổi tên theo ngày đo)

## Phần 3: khớp scan với map_xuong_v3 (`khop_scan_A.py`)

**Bước 1: chụp.** Xe **đứng yên** (ví dụ ở vạch A, hoặc ở chỗ kẹt cửa K2), SLAM đang chạy.

Tab **`project1@`**:

```
source ~/ros2_ws/install/setup.bash; python3 ~/do_xuong/khop_scan_A.py chup
```

Mong: `DA CHUP /home/project1/scan_<gio>.txt: ... tia, tu the tin (x; y; goc do), stamp ...`. Ghi lại xe đang đứng ở đâu (đo thước nếu được).

**Bước 2: khớp trên máy tính.** Tab **`aaa@`**:

```
scp 'project1@192.168.0.2:~/scan_*.txt' ~/ros2_ws/src/ros2/xe_scripts/du_lieu_0711/ && cd ~/ros2_ws/src/ros2/xe_scripts && for f in du_lieu_0711/scan_*.txt; do python3 do_xuong/khop_scan_A.py $f; done
```

**Đọc:**
- Dòng `CAO NHAT: ... | than: doc +x ngang +y cm`: xe thật ở **trước** tư thế tin x cm (doc +), lệch **trái** y cm (ngang +).
- `tap hoa diem`: khoảng không chắc chắn. Khúc tường song song thì một phương có thể rộng vài cm.
- `CANH BAO: diem cao nhat < 0,90`: scan khớp kém (vật lạ, lệch quá ±12 cm / ±6°). Đừng tin số đó.
- Đối chứng [thử 06/11]: `scan_ket_5b2.txt` với tư thế tin (0,313; 0,796; 8,5°) ra **doc +9,5, ngang +6,2 cm**, đúng như RA_SOAT_118 B2b:

```
python3 do_xuong/khop_scan_A.py du_lieu_0511_toi/scan_ket_5b2.txt 0.313 0.796 8.5
```

## Gửi lại cho Claude
- `~/do_xuong/do_toc_do.csv` + số thước của từng lần;
- `sua_vi_tri_*.csv`;
- `scan_*.txt` + vị trí đo thước nếu có;
- giờ chạy từng tuyến.
