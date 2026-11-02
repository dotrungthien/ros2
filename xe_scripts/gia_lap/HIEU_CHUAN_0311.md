# Hiệu chuẩn giả lập 03/11 (việc 106) — CHƯA HIỆU CHUẨN

**Kết luận: giả lập chưa hiệu chuẩn: Q1 đạt, Q2 không đạt (lạc quan: 2/2 về tới A, 0 NHAY_VI_TRI so với 5 lần trên xe).**
Chưa dùng giả lập để quyết thay đổi trên xe. Hình dung sai: `hieu_chuan_0311.png`.

## Cấu hình giả lập (HEAD 500cdc0, WSL tf2 0.25.24, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=87)
- `bat_gia.sh`: launch `mapping_my_bot` + Nav2 thật với bản chép `nav2_params_cho.yaml` (inflation 0.20), odom_tu_lenh yaw_nguon=gyro; sau Nav2 bật `sep.py` + `lai_xe_lidar.py` như `bat_xe.sh` B2b; liên kết `~/gui_duong.py`, `~/tinh_tuyen.py`.
- `xe_gia.py`: mô hình sàn của odom_tu_lenh (duty 0,21 ⇒ 0,092 m/s, he_so_xoay 0,46, trễ xoay 0,5 s), banh_trai_that = 1/0,92; `/gyro_yaw` = góc thật 50 Hz; `/scan` 10 Hz, 500 tia trên `map_xuong_v3.png`.
- Q1: mức B (SEP_RA_LENH/SEP_RA_THOAT/PHU_LAI_PHAT_LENH=true), rotate_to_heading_min_angle 0.35 (yaml), `BANG=day`, `BO_QUA_KIEM=1`; 3 vòng di_B rồi ve_A.
- Q2: như xe 02/11 16:43–17:03: ROTATE_MIN=0.785, SEP_RA_LENH=true, SEP_RA_THOAT=false, PHU_LAI_PHAT_LENH=false, `BANG=xs`, `BO_QUA_KIEM=1`; 2 lần di_B rồi ngay ve_A (không bật lại ở B).

## Dung sai ghi trước
- Q1 (ghi 03/11 00:51, hình + lời Thiên): 6/6 DEN_NOI; ≥ 5/6 tới ở lần 1; cách đích ≤ 0,15 m; mỗi chiều 30–150 s; tâm xe không vào ô đỏ.
- Sửa TRƯỚC khi chạy Q2 (ghi 01:12:49): Q1 chấm "cách đích" theo vị trí xe tin (cùng kiểu số với log xe); thời gian Q1 chỉ ghi (giả lập không có độ chậm Pi). Q2a bỏ ngưỡng thời gian (chỉ ghi).
- Q2 (ghi 01:12:49): Q2a di_B DEN_NOI 6/6 điểm. Q2b 0/2 lần tới A VÀ ≥ 1/2 lần kẹt với vị trí xe tin cách (0,79; −0,08) ≤ 0,20 m, tâm cách ô đỏ ≤ 0,10 m, sếp ghi KET_SAT_VAT.

## Q1 (03/11 01:03–01:07, BANG=day) — ĐẠT (chấm lại theo xe tin)
| vòng | chiều | kết quả | lần | cách đích tin / thật (m) | giây | tâm→đỏ thật (m) |
|---|---|---|---|---|---|---|
| 1 | di_B | DEN_NOI | 1/3 | 0,137 / 0,138 | 25 | 0,161 |
| 1 | ve_A | DEN_NOI | 1/3 | 0,140 / 0,195 | 25 | 0,173 |
| 2 | di_B | DEN_NOI | 1/3 | 0,144 / 0,144 | 25 | 0,155 |
| 2 | ve_A | DEN_NOI | 1/3 | 0,131 / 0,223 | 24 | 0,164 |
| 3 | di_B | DEN_NOI | 1/3 | 0,133 / 0,130 | 26 | 0,159 |
| 3 | ve_A | DEN_NOI | 1/3 | 0,140 / 0,203 | 27 | 0,162 |

Cả 3 vòng: 0 "Transform data too old", sếp 0 KET_SAT_VAT / 0 THOAT, phụ lái 0 tin `/cmd_vel_avoid`. Theo vị trí thật chỉ 3/6 ≤ 0,15 m (ve_A lệch 6–8 cm, xem giới hạn).

## Q2 (03/11 01:13–01:17, BANG=xs) — KHÔNG ĐẠT
| lần | di_B (Q2a) | ve_A (Q2b) | đứng yên lâu nhất (tin) | tin gần (0,79; −0,08) nhất | sếp |
|---|---|---|---|---|---|
| 1 | DEN_NOI 6/6 điểm, 43 s | DEN_NOI 6/6, 45 s, mã 0 | 3 s ở (1,15; −0,05; 168°) | 0,113 m | 0 KET_SAT_VAT, 0 THOAT, 0 DUNG_SEP |
| 2 | DEN_NOI 6/6 điểm, 44 s | DEN_NOI 6/6, 42 s, mã 0 | 3 s ở (1,14; −0,05; 174°) | 0,070 m | như trên |

Q2a ĐẠT 2/2. Q2b KHÔNG ĐẠT: 2/2 về tới A (cần 0), 0/2 kẹt đúng kiểu. Xe 02/11: di_B_xs 70 s; ve_A_xs hỏng ở điểm 4 rồi DUNG_SEP 17:05:58; sếp ghi 5 lần NHAY_VI_TRI (0,11–0,16 m) lúc 16:43–17:06, giả lập 0.

## Giới hạn đã biết
- LiDAR giả dừng tia ở mép gần của ô chiếm (ô 5 cm): thấy vật gần hơn thật 0–5 cm.
- Odom đúng tuyệt đối: xe giả và odom_tu_lenh dùng cùng mô hình sàn; không trượt bánh.
- Con quay không trôi, không nhiễu (góc thật); LiDAR không nhiễu, 10 Hz (xe ≈ 12 Hz).
- Không có độ chậm của Pi (thời gian mỗi chiều ngắn hơn xe).
- SLAM bám A lệch cố định: ve_A thật − tin 6–8 cm ở A (lúc bật: −4; −4 cm).
- Hệ quả: SLAM trong giả lập không nhảy vị trí (0 NHAY_VI_TRI), giả lập lạc quan.

## Việc 107 kế tiếp (đề xuất, Thiên chưa chốt)
Làm giả lập bi quan đủ để tái hiện Q2b, mỗi lần đổi một thứ và chạy lại Q2: (1) bắn tia tới tâm ô / nhiễu khoảng cách LiDAR, 12 Hz; (2) odom lệch khỏi xe giả (trượt bánh, he_so_xoay thật khác 0,46); (3) con quay trôi bias. Đích: có NHAY_VI_TRI và kẹt quanh (0,79; −0,08) như xe 02/11.
