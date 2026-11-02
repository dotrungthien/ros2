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
