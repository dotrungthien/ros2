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
