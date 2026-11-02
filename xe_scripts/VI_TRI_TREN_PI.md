# Vị trí các tệp trên Pi

Nguồn: `cai_cau_noi.sh`, `HUONG_DAN_CAU_NOI.md`, `HUONG_DAN_THU_31_10.md`. Chỉ ghi điều ba tệp này nói. Chỗ nào không nói thì ghi "chưa rõ".
Ba tệp nguồn đều ghi người dùng trên Pi là `project1`, nên `~` = `/home/project1`.
Cả ba tệp nguồn đều ghi chưa chạy trên Pi thật (`cai_cau_noi.sh`: "CHUA CHAY TREN Pi THAT"); các đường dẫn dưới đây là đường dẫn theo hướng dẫn, chưa đối chiếu với Pi.

## Tệp trong `xe_scripts/`

| Tên tệp | Đường dẫn trên Pi | Ghi chú |
|---|---|---|
| `bat_xe.sh` | `~/bat_xe.sh` | Bản v3. `cai_cau_noi.sh` kiểm `~/bat_xe.sh` và dừng nếu thiếu. Pi giữ bản cũ ở `~/bat_xe.sh.bak` (tạo bằng `cp -n`). |
| `bat_xe.sh.v3` | chưa rõ | Ba tệp nguồn chỉ nói "bản v3" của `bat_xe.sh`; không nói tệp `.v3` này chép lên đâu, cũng không nói nó khác `bat_xe.sh` ở chỗ nào. |
| `kiem_xe.sh` | `~/kiem_xe.sh` | Bản v3. Bản cũ giữ ở `~/kiem_xe.sh.bak`. |
| `chuyen.sh` | `~/chuyen.sh` | Bản v4. Ghi kết quả vào `~/ket_qua_chuyen.csv`. |
| `ghi_bag.sh` | `~/ghi_bag.sh` | Bản v3. |
| `bt_cho.xml` | `~/bt_cho.xml` (`/home/project1/bt_cho.xml`) | Truyền cho Nav2 qua `NAV_PARAMS`. Bước 3a sửa tại chỗ trên Pi, bản 60 giây giữ ở `~/bt_cho.xml.60s`. |
| `nav2_params_cho.yaml` | `~/nav2_params_cho.yaml` (`/home/project1/nav2_params_cho.yaml`) | Là giá trị `NAV_PARAMS` mặc định mà `cai_cau_noi.sh` ghi vào `cau_noi.env` (`/home/<người dùng>/nav2_params_cho.yaml`). |
| `cai_cau_noi.sh` | `~/cai_cau_noi.sh` | `scp` lên `~`, chạy 1 lần: `bash ~/cai_cau_noi.sh`. Script kiểm `cau_noi_xe.py` và `chon_b_v4.html` ở thư mục hiện tại, nên phải chạy khi đang ở `~`. |
| `cau_noi_xe.py` | `~/cau_noi_xe.py`, rồi `cai_cau_noi.sh` chép sang `~/cau_noi/cau_noi_xe.py` | Dịch vụ systemd chạy bản trong `~/cau_noi/`. Cổng 5001. |
| `chon_b_v4.html` | `~/chon_b_v4.html`, rồi chép sang `~/cau_noi/chon_b.html` | Đổi tên khi chép (bỏ `_v4`). |
| `lai_xe_lidar.py` | `~/lai_xe_lidar.py` | Chạy bằng `python3 ~/lai_xe_lidar.py` sau khi `source ~/ros2_ws/install/setup.bash`. |
| `HUONG_DAN_CAU_NOI.md` | chưa rõ | Ba tệp nguồn không nói chép tài liệu lên Pi. |
| `HUONG_DAN_THU_31_10.md` | chưa rõ | Như trên. |
| `THIET_KE_LAI_XE_LIDAR.md` | chưa rõ | `HUONG_DAN_THU_31_10.md` chỉ nhắc đây là thiết kế đầy đủ; không nói vị trí trên Pi. |
| `BAO_CAO_SU_CO_31_10_CHIEU.md` | chưa rõ | Không được ba tệp nguồn nhắc tới. |
| `LENH_CLAUDE_CODE.txt` | chưa rõ | Không được ba tệp nguồn nhắc tới. |
| `mo_phong_he_thong.py` | chưa rõ | Không được ba tệp nguồn nhắc tới. |
| `so_sanh_tuyen.py` | chưa rõ | Không được ba tệp nguồn nhắc tới. |
| `test_lai_xe_lidar.py` | chưa rõ | Không được ba tệp nguồn nhắc tới. |

## Tệp có trên Pi nhưng không nằm trong `xe_scripts/`

| Tệp | Đường dẫn trên Pi | Ghi chú |
|---|---|---|
| Bản đồ | `~/map_xuong_v2.pgm` (hoặc `.png`) và `~/map_xuong_v2.yaml` | `HUONG_DAN_CAU_NOI.md` bảo chép từ `goi_ve_nha_30_10.tgz`. Hoặc đặt `CAU_NOI_MAP` trong `cau_noi.env` (đường dẫn không có đuôi). Có trong kho hay không: chưa rõ. |
| Tệp môi trường cầu nối | `~/cau_noi/cau_noi.env` | `cai_cau_noi.sh` tạo nếu chưa có. Đổi `NAV_PARAMS` ở đây rồi `sudo systemctl restart cau_noi_xe`. |
| Dịch vụ systemd | `/etc/systemd/system/cau_noi_xe.service` | `cai_cau_noi.sh` ghi bằng `sudo tee`. `KillMode=process`. |
| Tệp PIN | `~/.cau_noi_pin` | Tuỳ chọn, `chmod 600`. Nội dung không được ghi vào kho. |
| Log cầu nối | `~/log_cau_noi.txt` | Log của lần bật. |
| Log hệ thống | `~/log_bat_xe_nav.txt`, `~/log_bat_xe_*` | Chỉ biết đúng tên `log_bat_xe_nav.txt` và mẫu `log_bat_xe_*`; các tệp còn lại chưa rõ. |
| Kết quả chuyến | `~/ket_qua_chuyen.csv`, `~/chuyen_*.txt` | `chuyen.sh` ghi ra. |
| Dữ liệu ghi bag | `~/bag_*` | `ghi_bag.sh` ghi ra; thư mục chính xác chưa rõ. |
| Gói đã đóng cuối buổi | `~/dulieu_31_10.tgz` | Tạo bằng `tar` ở cuối buổi thử. |
| Workspace ROS 2 | `~/ros2_ws/install/setup.bash` | Cần `source` trước khi chạy `ros2`. |
| Launch Nav2 | gói `my_bot`, `navigation_my_bot.launch.py` | Nằm trong workspace trên; đường dẫn tệp cụ thể chưa rõ. |

## Đối chiếu luật G (01/11, đọc mã; CHƯA chạy trên Pi)

Luật G: bên ghi và bên đọc phải dùng cùng tên/đường dẫn. Cột "khớp?" chỉ nói về tên trong mã, không nói tệp có thật trên Pi (việc đó là `so_voi_pi.sh`). **Chưa giả lập cả hệ thống (công cụ chỉ đọc).**

| Bên ghi | Bên đọc | Tên/đường dẫn phải khớp | Khớp? |
|---|---|---|---|
| `cai_cau_noi.sh` chép | systemd `ExecStart`, `cau_noi_xe.py` (trang `HERE/chon_b.html`) | `~/cau_noi/cau_noi_xe.py`, `~/cau_noi/chon_b.html` (đổi tên từ `chon_b_v4.html`) | khớp |
| `cai_cau_noi.sh` ghi `cau_noi.env` | `EnvironmentFile` → `cau_noi_xe.py` → `bat_xe.sh` (con thừa kế biến môi trường) | `NAV_PARAMS=/home/$ME/nav2_params_cho.yaml` = `~/nav2_params_cho.yaml` | khớp nếu người dùng là `project1`; việc thừa kế biến chưa thử |
| `nav2_params_cho.yaml` | `bt_navigator`; `bat_xe.sh` đọc `MONG_BT` bằng `sed` | `/home/project1/bt_cho.xml` = `~/bt_cho.xml` | khớp (đường tuyệt đối, như trên) |
| `bat_xe.sh` (`SLAM` mặc định) | `mapping_my_bot.launch.py` → `slam_toolbox` | `/home/project1/slam_dinh_vi_xuong.yaml` = `~/slam_dinh_vi_xuong.yaml` | khớp; bản kho ở `du_lieu_0111/`, đã vào `so_voi_pi.sh` |
| `slam_dinh_vi_xuong.yaml` (`map_file_name`) | `slam_toolbox`, `mode: localization` | `/home/project1/map_xuong_v3_pg` → `~/map_xuong_v3_pg.posegraph` + `.data` | khớp tên; không cần `.pgm`; đã vào `so_voi_pi.sh` |
| `cau_noi_xe.py` (`CAU_NOI_MAP`) | trang chọn B; `cai_cau_noi.sh` bước [2] | `~/map_xuong_v2.*` | khớp nội bộ, nhưng **LỆCH với SLAM dùng `map_xuong_v3_pg`**: trang vẽ v2, định vị theo v3. Chưa quyết (xem `HUONG_DAN_CAU_NOI.md` mục 5) |
| `bat_xe.sh` (`LOG2`) | `cau_noi_xe.py` (`nav_log`, tìm `CRITICAL FAILURE`) | `~/log_bat_xe_nav.txt` | khớp |
| `cau_noi_xe.py` (`log`) | chính nó (`doc_cuoi`) | `~/log_cau_noi.txt` | khớp |
| `bat_xe.sh` gọi | `kiem_xe.sh` | `~/kiem_xe.sh` | khớp |
| `chuyen.sh` ghi CSV | `sep.py` (`csv_chuyen`) | `~/ket_qua_chuyen.csv`; 10 cột `gio…tuoi_costmap_s` giống `COT_CSV` | khớp tên và cột |
| `sep.py` ghi (`ra_lenh`, mặc định false) | `chuyen.sh` (`SEP_LENH`) | `~/sep_lenh`; đồng hồ `CLOCK_BOOTTIME` = số đầu `/proc/uptime` | khớp tên; cùng đồng hồ là lời `sep.py` ghi, chưa thử |
| `sep.py` ghi `~/sep_nhat_ky.csv` | chưa có bên đọc trong kho (`du_lieu_0111/sep_nhat_ky.csv` là bản chép về) | `~/sep_nhat_ky.csv` | không áp dụng |
| `sep.py` | `import lai_xe_lidar` cùng thư mục | `~/sep.py` và `~/lai_xe_lidar.py` | khớp; cả hai đã vào `so_voi_pi.sh` |
| `tinh_tuyen.py` | chính nó đọc `<thư mục tệp>/du_lieu_0111/costmap_v3_020.yaml` | chép riêng lên `~/tinh_tuyen.py` thì **thiếu `du_lieu_0111/`** | công cụ chạy ở máy; để `tuychon` trong `so_voi_pi.sh` |
| `ghi_bag.sh` | hướng dẫn cuối buổi (`tar ~/bag_*`) | `$HOME/bag_*` | khớp |
| `NODES` (`bat_xe.sh`, 19 tên) | `CHINH` (`cau_noi_xe.py`, 12 tên) | cùng danh sách tên tiến trình | **LỆCH có chủ ý**: 7 tên thêm 31/10 chỉ có ở `NODES`; `CHINH` chưa đổi theo lệnh Thiên |
| **Tệp mới đã vào danh sách `so_voi_pi.sh` chưa?** | `sep.py`, `tinh_tuyen.py`, `slam_dinh_vi_xuong.yaml`, `map_xuong_v3_pg.posegraph/.data` | có. Còn ngoài danh sách: `costmap_v3_020.*`, `sep_nhat_ky.csv` (dữ liệu máy, thuộc `du_lieu_*/`). Từ nay `so_voi_pi.sh` tự nhắc `CHUA_CO_TRONG_DANH_SACH` | đã vào; tự nhắc trên kho: 0 dòng |

## Đối chiếu luật G — việc 105 (02/11; đọc mã + `test_sep.py` 28/28; CHƯA chạy trên xe)

Chữ `sep.py` ghi ra mà nơi khác đọc. **Chưa giả lập cả hệ thống:** `gia_lap/bat_gia.sh` không chạy được trên PC này (không có `~/ros2_ws/install`, thiếu gói `my_bot`, `twist_mux`, `joint_state_publisher`), nên không chạy lại ca kẹt sát K4 trong giả lập. Phần đã thử chỉ là hàm thuần + `Sep.vong` với LiDAR/bản đồ giả (`mo_phong_he_thong.py`, chưa hiệu chuẩn: chỉ bắt lỗi ghép).

| Bên ghi | Bên đọc | Tên/đường dẫn phải khớp | Khớp? |
|---|---|---|---|
| `sep.py` → `~/sep_nhat_ky.csv`, 11 cột `COT_NHAT_KY` | chưa có tệp nào ngoài `sep.py`/`test_sep.py` đọc (grep toàn kho) | tiêu đề cột; chữ mới: `TRUY_VET,KET_SAT_VAT` (`THOAT luot n/3…` hoặc `HET 3 LUOT, DUNG_VA_BAO`), `TU_THE,DUNG_VA_BAO` (lý do `HET_3_LUOT_THOAT_VAN_KET`), `L1,BAT_DAU_THOAT` (`khong cong (KET_SAT_VAT dang bat)…`, `het 3 luot, khong cong`) | không có bên đọc để lệch; **chữ mới chưa có trong tài liệu nào** (người đọc nhật ký phải biết) |
| `sep.py` → `~/sep_lenh` (`DI\|CHO\|DUNG` + lý do) | `chuyen.sh` `doc_sep` / `theo_sep` | ba chữ `DI`, `CHO`, `DUNG`; đồng hồ `CLOCK_BOOTTIME` = `/proc/uptime` | khớp; **việc 105 không đổi `di_hay_khong`**, nên `het_luot` / `DUNG_VA_BAO` KHÔNG làm sếp ra lệnh `DUNG` (chỉ nằm ở tư thế ghi nhật ký). Cố ý, chờ Thiên quyết |
| `sep.py` → `/sep/trang_thai` (JSON) | chưa có bên đọc trong kho (chỉ tài liệu nói `ghi_bag.sh` sẽ ghi) | khoá `luot_thoat`, `tu_the`; giá trị `tu_the` mới có thể là `DUNG_VA_BAO` do KET_SAT_VAT | không thêm khoá; `luot_thoat` nay ≤ 3 (trước đây vượt 3) |
| `chuyen.sh` → `~/ket_qua_chuyen.csv` cột `ket_qua` | `sep.py` `_csv` (đặt lại `luot_thoat`, `het_luot` khi `KIEM_DAT`) | `KIEM_DAT`, `DEN_NOI`, `HET_DUONG`, `SAI_VI_TRI`, `DUNG_SEP` | khớp cả 5 tên |
| `lai_xe_lidar.py` log `/rosout` `'%s -> %s (%s)%s \| %s'` | `sep.py` `doc_rosout` (cộng/không cộng lượt khi `THOAT`) | khuôn câu log; `test_15` kiểm nguyên văn | khớp |
| `sep.py` `P_MAC_DINH` (thêm `sat_vat_tat`) | `declare_parameter` trong nút; không launch/yaml nào đặt | tên tham số `sat_vat_tat` (m, 0,12); mã dùng `max(sat_vat_tat, sat_vat)` nên đặt nhỏ hơn `sat_vat` cũng không đảo khoảng trễ | khớp (không bên nào khác đặt tên này) |
| Tệp mới/đổi đã vào `so_voi_pi.sh` chưa? | `sep.py` có trong danh sách (`~/sep.py`); `test_sep.py` thuộc luật loại trừ `test_*` | `bash xe_scripts/so_voi_pi.sh --nhac` | đã vào |

## Đối chiếu luật G — việc 109c, commit c1 (02/11; đọc mã + `test_sep.py` 32/32 + `test_lai_xe_lidar.py` 36/36; CHƯA chạy trên xe)

c1 chỉ có phần "nói": sếp ghi `~/sep_thoat`, phụ lái đọc và **in** "SE LAM", không phát tốc độ. **Chưa giả lập cả hệ thống (PC trường thiếu `install`); nút ROS của hai tệp chưa chạy ở đâu**, chỉ hàm thuần và đọc mã.

| Bên ghi | Bên đọc | Tên/khuôn phải khớp | Khớp? |
|---|---|---|---|
| `sep.py` `ghi_lenh(~/sep_thoat, BOOTTIME, 'THOAT', ly_thoat(...))` | `lai_xe_lidar.py` `doc_sep_thoat` | một dòng `<giây BOOTTIME> THOAT <xP\|-> <yP\|-> <lượt> <mã>` (6 trường); tuổi ≤ 3 s; `-` = sếp không có P tin | khớp: `test_32` ghi bằng `ly_thoat` rồi đọc bằng `doc_sep_thoat`, cả tuổi quá hạn, tệp hỏng, `P=-` |
| `sep.py` tham số `ra_thoat` (mặc định false), `tep_thoat` | người bật tay (`-p ra_thoat:=true`) | tên `ra_thoat`; đường dẫn `~/sep_thoat` | khớp: `test_32` kiểm chữ `declare_parameter('ra_thoat', False)`; `lai_xe_lidar.py` khai `tep_thoat` cùng mặc định `~/sep_thoat` |
| `sep.py` `di_hay_khong` trả `DUNG` (lượt 4, `HET_3_LUOT_THOAT`) vào `~/sep_lenh` (chỉ khi `ra_lenh=true`) | `chuyen.sh` `doc_sep` / `theo_sep` (`read -r T C L`, nhánh `DUNG)`) | chữ `DUNG` | khớp: `test_32` kiểm hai dòng trong `chuyen.sh`; `chuyen.sh` chỉ đọc ở ranh giới nên có thể muộn tới `TO` = 130 s |
| `lai_xe_lidar.py` câu log `SE LAM (chi nhin…)` (qua `/rosout`) | `sep.py` `doc_rosout` (chỉ bắt `->` và `trang_thai=`) | câu không chứa `->` và `trang_thai=` | khớp: `test_32` kiểm `doc_rosout('lai_xe_lidar', cau) is None`; sếp **không** bắt câu này (muốn sếp ghi lại thì là việc sau) |
| `sep.py` dòng nhật ký `TRUY_VET,KET_SAT_VAT` (thêm `phia ket X, P=(x;y)\|-`) | người đọc nhật ký; chưa có chương trình đọc | chữ `phia ket`, `P=` | không có bên đọc để lệch (như bảng việc 105) |
| `sep.py` `/sep/trang_thai`, khoá mới `thoat` (dict hoặc null) và `ra_thoat` | chưa có bên đọc trong kho | khoá `thoat`, `ra_thoat` | không có bên đọc |
| `chuyen.sh` ghi `KIEM_DAT` | `sep.py` `_csv` đặt lại `het_luot` | `KIEM_DAT` | khớp (cũ). **Rủi ro:** `chuyen.sh` ghi `KIEM_DAT` rồi hỏi sếp ngay; nếu `DUNG` của chuyến trước còn trong `~/sep_lenh` thì có thể đọc trúng trước khi sếp kịp đặt lại (cửa sổ ≈ 0,1 s) |
| Tệp đổi đã vào `so_voi_pi.sh` chưa? | `sep.py`, `lai_xe_lidar.py` đã có; không tệp mới | `bash xe_scripts/so_voi_pi.sh --nhac` | đã vào; tự nhắc: 0 dòng |

## Đối chiếu luật G — việc 109c, commit c2 (02/11; đọc mã + `test_lai_xe_lidar.py` 47/47 + `test_sep.py` 33/33; CHƯA chạy trên xe)

c2 = phụ lái **ra tay** pha RA (đi thẳng về phía thoáng) khi `phat_lenh=true` và có `~/sep_thoat` mới; chưa xoay (c3). **Chưa giả lập cả hệ thống (PC trường thiếu `install`); nút ROS chưa chạy ở đâu**, chỉ `NguoiLai` + xe lý tưởng trong `test_lai_xe_lidar.py` (không trễ, không lệch).

| Bên ghi | Bên đọc | Tên/khuôn phải khớp | Khớp? |
|---|---|---|---|
| `lai_xe_lidar.py` phát `Twist` ở `_tick` (10 lần/s, `create_timer(0.1)`, chỉ khi `phat_lenh`) | `twist_mux` (`twist_mux.yaml`, mục `avoid`) | topic `/cmd_vel_avoid`; ưu tiên 60 > Nav2 20 (`/cmd_vel_nav`); `timeout 0.2` s > chu kỳ phát 0,1 s | khớp: kiểm bằng đọc `twist_mux.yaml` và mã (ca "luat G"). Hết RA thì ngừng phát, 0,2 s sau `twist_mux` trả bánh cho Nav2 |
| `sep.py` `ghi_lenh(~/sep_thoat…)` (c1) | `lai_xe_lidar.py` `doc_sep_thoat` → `ra_bat_dau` | khuôn `<giây BOOTTIME> THOAT <xP\|-> <yP\|-> <lượt> <mã>`; tuổi ≤ 3 s; mã mới | khớp (c1, `test_32`); c2 chỉ dùng thêm: có tệp mới ⇒ bắt đầu RA, `P` chưa dùng (c3) |
| `lai_xe_lidar.py` log `RA BAT DAU …`, `RA XONG <lý do> <cm> cm`, `RA KHONG DI: …` | người đọc log; `sep.py` `doc_rosout` | lý do: `DU_CHO_XOAY`, `DA_DI_30CM`, `QUA_10S`, `DO_PHIA_DI`, `MAT_DICH`; chữ `RA XONG` | sếp **không** bắt ba câu này (`test_33`); không bên nào khác đọc |
| `lai_xe_lidar.py` câu đổi trạng thái `… -> RA_LUI \| RA_TIEN \| RA_XONG (…)` | `sep.py` `doc_rosout` (`l1`) | tên không bắt đầu bằng `THOAT` hay `CHAN_` | khớp: `test_33` kiểm sếp bắt được thành `l1` mà **không cộng lượt** `luot_thoat`; `chon_tu_the` không coi `RA_*` là phản xạ (chưa biết có nên: việc sau) |
| Tệp đổi đã vào `so_voi_pi.sh` chưa? | `lai_xe_lidar.py` đã có; không tệp mới | `bash xe_scripts/so_voi_pi.sh --nhac` | đã vào; tự nhắc 0 dòng |

Điều rút ra từ ca thử (cần Thiên biết): vật cách mũi **5,5 cm** (K4 đo 01/11) thì mọi điểm LiDAR đã cách tâm 20 cm > 18,56 cm (`R_XOAY`), nên RA **không đi** (`RA XONG DU_CHO_XOAY 0 cm`); RA chỉ đi khi vật gần hơn ≈ 4,1 cm trước mũi, hoặc hông/sau chật hơn. Hết RA là xoay được ngay (c3).

## Đối chiếu luật G — việc 109c, commit c3 (02/11; đọc mã + `test_lai_xe_lidar.py` 59/59 + `test_sep.py` 34/34; CHƯA chạy trên xe)

c3 = sau RA (kể cả `RA XONG DU_CHO_XOAY 0 cm`), pha XOAY tại chỗ về hướng **thoáng nhất trong ±45° quanh hướng tới P**. **Chưa giả lập cả hệ thống (PC trường thiếu `install`); nút ROS chưa chạy ở đâu**, chỉ `NguoiLai` + xe lý tưởng (không trễ tf, không trễ motor, không vùng chết).

| Bên ghi | Bên đọc | Tên/khuôn phải khớp | Khớp? |
|---|---|---|---|
| `odom_tu_lenh.py` phát `/odom` (`Odometry`; `yaw_nguon=gyro` khi chạy thật) | `lai_xe_lidar.py` nghe `/odom`, chỉ lấy **góc đã xoay** = hiệu yaw so với lúc bắt đầu | topic `/odom`; kiểu `Odometry`; yaw tuyệt đối không cần khớp khung bản đồ | khớp: kiểm bằng đọc mã (`create_publisher(Odometry, '/odom'` / `create_subscription(Odometry, '/odom'`); ca thử đưa vào `/odom` lệch hằng số 0,7 rad so với bản đồ mà vẫn xoay đúng. `/odom` cũ > 0,5 s ⇒ không xoay, log |
| `sep.py` `~/sep_thoat` (c1), cột P | `lai_xe_lidar.py` `doc_sep_thoat` → `thoat_p`; vị trí xe từ tf `map→base_footprint` (`_pose_ban_do`, cũ > `tf_tuoi_toi_da` = 1 s ⇒ `None`) | P = `(x, y)` trên bản đồ; `-` ⇒ không xoay | khớp (khuôn c1, `test_32`); tf cũ hay `P=-` ⇒ log `XOAY KHONG: khong co huong toi P …` |
| `lai_xe_lidar.py` phát `Twist` (c2, 10 lần/s) với `w` khi XOAY (1,0 rad/s, 0,6 rad/s khi còn < 45°) | `twist_mux` (`/cmd_vel_avoid`, 60, timeout 0,2 s) | như c2 | khớp (ca "luat G" của c2). 0,6 rad/s = `min_rotational_vel` của `behavior_server` (`nav2_params_cho.yaml`, né vùng chết) |
| `lai_xe_lidar.py` log `XOAY XONG <lý do> <độ>`, `XOAY KHONG: …`; câu đổi trạng thái `… -> XOAY_P \| XOAY_XONG (…)` | `sep.py` `doc_rosout` | lý do: `DEN_HUONG`, `TRUOC_DUOI_12CM`, `DIEM_TRONG_R_XOAY`, `QUA_10S`, `MAT_DICH`, `MAT_ODOM`; tên không bắt đầu bằng `THOAT`/`CHAN_` | `test_34`: sếp không bắt hai kiểu câu `XOAY XONG`/`XOAY KHONG`, bắt `XOAY_P` thành `l1` mà **không cộng lượt** |
| Tham số mới trong `DEFAULT` (`xoay_p_nua` 45, `xoay_p_truoc` 0,12, `xoay_p_w_cham` 0,6, `xoay_p_t_max` 10) | nút `lai_xe_lidar` (`declare_parameter` theo `DEFAULT`) | tên | khớp (vòng khai báo/đọc chung với mọi tham số `DEFAULT`) |
| Tệp đổi đã vào `so_voi_pi.sh` chưa? | `lai_xe_lidar.py` đã có; không tệp mới | `bash xe_scripts/so_voi_pi.sh --nhac` | đã vào; tự nhắc 0 dòng |

Điều rút ra từ ca thử: hướng đích **không phải lúc nào cũng là hướng P**. Khi vật trước mũi che P (ca "P +40°"), hướng chọn thành 80°; khi P sau tường, chọn 130° (thoáng 0,50 m so với 0,22 m ở hướng P). Dải bám đường dùng bề rộng quét của thân xe (rộng hơn 16 cm khi chéo), nên chéo gần vật hay bị coi là bị chắn. Dừng cách hướng đích tới 20° (`xoay_den_deg`); phần còn lại để RPP (xoay tự khi lệch > 45°).

## Đối chiếu luật G — việc 110 rút gọn (02/11; `bash -n`, thử `bat_xe.sh` với `ros2` giả và HOME giả; CHƯA chạy trên Pi)

Đổi: `nav2_params_cho.yaml` `inflation_radius` 0,25 → 0,20 (global và local); `bat_xe.sh` đọc `NAV_PARAMS` từ `~/cau_noi/cau_noi.env` khi chưa đặt, bật nền `sep.py` và `lai_xe_lidar.py` sau khi Nav2 active, chỉ in `SAN SANG` khi cả hai còn sống sau 5 s. **Chưa giả lập cả hệ thống:** thử giả chỉ có `bat_xe.sh` + `ros2` giả + `sep.py`/`lai_xe_lidar.py` giả (ngủ), không có Nav2, SLAM, `sep.py` thật.

| Bên ghi | Bên đọc | Tên/đường dẫn phải khớp | Khớp? |
|---|---|---|---|
| `bat_xe.sh` `-p ra_lenh:= -p ra_thoat:=` | `sep.py` `declare_parameter('ra_lenh', False)` / `('ra_thoat', False)` | tên tham số | khớp (đọc mã); giá trị chỉ `true`/`false`, kiểm trước khi bật gì |
| `bat_xe.sh` `-p phat_lenh:=` | `lai_xe_lidar.py` `declare_parameter('phat_lenh', False)` | tên tham số | khớp (đọc mã) |
| người gõ / `cau_noi.env` đặt `SEP_RA_LENH`, `SEP_RA_THOAT`, `PHU_LAI_PHAT_LENH` | `bat_xe.sh` | ba tên biến môi trường; mặc định false | chưa ai đặt: `cai_cau_noi.sh` chỉ tạo dòng `NAV_PARAMS`; nút kết nối truyền biến cho `bat_xe.sh` qua kế thừa môi trường (`Popen` không đặt `env`) [đọc mã, chưa thử] |
| `bat_xe.sh` → `~/log_sep.txt`, `~/log_phu_lai.txt` (bản cũ `.cu`) | chưa có chương trình đọc | đường dẫn | không có bên đọc (`cau_noi_xe.py` chỉ đọc `log_cau_noi.txt`, `log_bat_xe_nav.txt`) |
| `cai_cau_noi.sh` ghi `~/cau_noi/cau_noi.env` (`NAV_PARAMS=/home/$ME/nav2_params_cho.yaml`) | `bat_xe.sh` (`sed -n 's/^NAV_PARAMS=//p'`, bỏ dấu nháy và CR; không `source`) | dòng `NAV_PARAMS=`; tệp tham số phải tồn tại | khớp (thử giả cả dạng có nháy và CRLF). **Đổi hành vi:** bật bằng `ssh` không `export NAV_PARAMS` trước đây chạy cây hành vi mặc định (có quay và lùi), nay mặc định dùng `bt_cho` |
| `bat_xe.sh` mẫu `TAY_SEP`, `TAY_LAI` (python3 + tên tệp) | `bat_xe.sh` `tat`, chốt "không bật hai bản" (`con_chay_tat`) | cùng khuôn với `TAY` | khớp: chuỗi `TAY` sau khi tách `mau_py` giống từng ký tự bản cũ; thử giả `tat` tắt cả hai; `bash -c`, `test_sep.py`, `sep.pyc`, `ssh … sep.py` không bị khớp |
| `cau_noi_xe.py` `CHINH` (12 tên) | trạng thái `on` của nút kết nối | tên tiến trình | **không có `sep`, `lai_xe_lidar`**: phụ trợ chết sau khi bật thì nút vẫn báo `on`; lúc bật thiếu thì `bat_xe.sh` ra mã 1 nên nút báo `fail`. Không đổi `CHINH` (theo lệnh Thiên) |
| `nav2_params_cho.yaml` 0,20 | `gia_lap/bat_gia.sh` (`INFLATION` mặc định 0,20, `sed` đè dòng) ; `config/nav2_params.yaml` của kho (0,25) | giá trị | giả lập khớp; bản của kho còn 0,25 (đường lui: bản cũ trong git) |
| Tệp đổi đã vào `so_voi_pi.sh` chưa? | `bat_xe.sh`, `nav2_params_cho.yaml` đã có; không tệp mới | `bash xe_scripts/so_voi_pi.sh --nhac` | đã vào; tự nhắc 0 dòng. Hai tệp sẽ ra `KHAC` tới khi chép lên Pi (nhớ giữ dòng `ACCESS` của Pi) |
