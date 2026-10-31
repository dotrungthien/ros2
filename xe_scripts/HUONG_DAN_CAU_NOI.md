# Cầu nối điện thoại ↔ Pi (việc 92, Core) — hướng dẫn thử ở trường 31/10

Trạng thái: VIẾT Ở NGOÀI ĐƯỜNG, CHƯA CHẠY TRÊN Pi THẬT. Đã thử với `bat_xe.sh` giả trong máy chủ (trạng thái, 409, 501, PIN, mất tiến trình, khởi động lại) [đo, n = 1 mỗi ca].
Loại: **Core**, phục vụ **G4** (giao diện) và **G5** (bật nguồn tới xong, x/10 lần) — Thiên nâng lên Core 31/10.

## Nó là gì (3 dòng)

- Một chương trình nhỏ `cau_noi_xe.py` chạy sẵn trên `Pi` ngay khi bật nguồn (cổng 5001), không cần ROS.
- Nút `차량 연결` trên điện thoại = gọi `bash ~/bat_xe.sh tat` rồi `bash ~/bat_xe.sh` thay bạn gõ terminal.
- Trang bản 4: bản đồ có 3 vùng (xanh, vàng, đỏ) và vành an toàn 8 cm, 15 cm quanh thân xe.
- Nó đọc danh sách tiến trình thật để báo `연결됨` / `연결 실패`. Không đoán.

Chưa nối (trả 501, màn hình báo rõ): `출발`, `정지`, `다시 시도`, `수동 조종` thật. Vị trí xe trên bản đồ cũng chưa đọc từ `tf`. Xe dừng khẩn vẫn bằng công tắc motor hoặc trang lái tay cổng 5000.

## Phần 1. Cài (1 lần, ≈ 5 phút)

Trong tab `aaa@`, thư mục có 4 tệp của bộ này. `bat_xe.sh` bản v3 phải ĐÃ nằm ở `~` của `Pi`:

```
scp cau_noi_xe.py chon_b_v4.html cai_cau_noi.sh project1@192.168.0.2:~/
```

```
ssh -t project1@192.168.0.2 'bash ~/cai_cau_noi.sh'
```

Mong thấy 5 dòng `[1]`–`[5]`: `[4]` là `active`, `[5]` là một dòng JSON có `"link": "off"`. Dòng `[3]` mà báo CẢNH BÁO nghĩa là `sudo` đòi mật khẩu: chỉnh giờ từ điện thoại sẽ lỗi khi `Pi` lệch giờ > 30 s.

Bản đồ: cần `~/map_xuong_v2.pgm` và `~/map_xuong_v2.yaml` trên `Pi` (dòng `[2]`). Chưa có thì chép từ `goi_ve_nha_30_10.tgz`.

Mở trên điện thoại (cùng Wi-Fi `project`): `http://192.168.0.2:5001/`

PIN (nên đặt, không bắt buộc): gõ ở `project1@` `echo 'số-bạn-chọn' > ~/.cau_noi_pin; chmod 600 ~/.cau_noi_pin`. Có tệp này thì lần đầu bấm nút điện thoại sẽ hỏi PIN. KHÔNG ghi PIN vào tệp trong Project.

## Phần 2. Bốn phép thử (mỗi cái 1 việc)

| # | Việc | Mong đợi | Ghi lại |
|---|---|---|---|
| 1 | Bật `Pi` (xe ở A, công tắc motor OFF). Mở trang trên điện thoại | trang hiện `연결 안 됨` trong ≤ 60 s kể từ lúc bật nguồn | số giây [đo] |
| 2 | Bấm `차량 연결`, trả lời hộp xác nhận. Không chạm xe | `연결 중 (n초)` rồi `대기 중`, chấm góc phải `연결됨` | số giây tới `대기 중`; ≤ 3 phút theo thiết kế [ước] |
| 3 | Ở `project1@` gõ `curl -s localhost:5001/api/b/status` | `"link": "on"`, `"thieu": []`, `han_s` ≈ 480 trừ thời gian đã trôi | cả dòng |
| 4 | Giả lỗi: `pkill -f planner_server` | trong ≤ 3 s trang hiện `연결 실패`; bấm `다시 연결` về `대기 중` | có / không |

Phép 3 là phép QUAN TRỌNG NHẤT: nếu `"thieu"` không rỗng trong khi hệ thống chạy bình thường, nghĩa là tên tiến trình mình đoán sai (lấy từ `bat_xe.sh`, chưa so với `Pi` thật). Khi đó chạy `python3 ~/cau_noi/cau_noi_xe.py --kiem` và gửi cho mình kết quả. Đừng sửa tay.

Sau khi phép 1–4 đạt: lặp phép 2 mười lần (bấm `차량 재시작` mỗi lần; xe đưa về A mỗi lần). Ghi x/10 lần ra `대기 중` và số giây. Con số này chính là phép đo cho G5.

## Phần 3. Đường lui

- Tắt cầu nối: `sudo systemctl disable --now cau_noi_xe`. Quy trình `ssh` cũ (Quy trinh chuan 71) không bị đổi gì.
- Cầu nối khởi động lại không giết hệ thống ROS đang chạy (`KillMode=process`) [chưa kiểm trên Pi].
- Đổi tệp tham số Nav2: sửa `~/cau_noi/cau_noi.env` (dòng `NAV_PARAMS=`), rồi `sudo systemctl restart cau_noi_xe`.
- Xem lỗi: `journalctl -u cau_noi_xe -n 30`; log của lần bật: `~/log_cau_noi.txt`.

## Phần 4. Chưa kiểm (đang giả định)

1. `sudo` không cần mật khẩu (dòng `[3]`).
2. Múi giờ `Pi` = múi giờ điện thoại (quy trình cũ cũng giả định vậy).
3. Tên 9 tiến trình chính khớp ở đối số đầu của dòng lệnh (phép 3 kiểm).
4. `bat_xe.sh` chạy được khi không có terminal (không có `ssh -t`): `ros2 daemon start` và `sudo` có thể phản ứng khác.
5. Đường dẫn bản đồ `~/map_xuong_v2.*` và việc nó khớp hệ toạ độ `slam_dinh_vi_xuong.yaml`.
6. Ngày 10/11 dùng hotspot điện thoại: địa chỉ `Pi` đổi, chưa đo.
