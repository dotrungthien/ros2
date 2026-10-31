# Hướng dẫn thử ngày 31/10 (bản 3: sửa đêm 30→31/10 ở nhà, CHƯA CHẠY TRÊN XE)

Mục tiêu: xe có hành vi rõ ràng và ổn định. Không quay vô định, không lùi, không vào góc chết.
Quyết định đã chốt (Thiên, 30/10): bỏ quay và lùi; chờ đứng yên (tối đa khoảng 60 giây), rồi đổi tuyến; B = (1,60; 0,82).
Ngày 31/10 chỉ thử và hiệu chỉnh. Luật: mỗi lô đổi MỘT thứ; mỗi bài ghi x/n; cổng Core chưa đạt thì không làm Stretch.

## Bản 3 khác bản 1 ở đâu (Phần 0 đến Phần 2 không đổi cách xe chạy; Phần 3 mới)

| Tệp | Đổi gì | Đường lui |
|---|---|---|
| `chuyen.sh` (v4) | kiểm vị trí trước MỖI điểm (bản 1 chỉ kiểm điểm đầu); ghi thêm 2 số vào CSV: lệch và tuổi tin của costmap toàn cục; `TRY` mặc định 1 (theo chính sách 31/10: chờ rồi đổi tuyến); **bảng tuyến tiếp tuyến (biến `BANG`, mặc định `tt`; `BANG=cu` là bảng cũ)** | dùng `chuyen.sh` bản 1 trong tệp `.tgz` cũ |
| `kiem_xe.sh` (v3) | thêm Bước 8: cây hành vi đang chạy, tiến trình/node trùng, costmap | `kiem_xe.sh.bak` |
| `bat_xe.sh` (v3) | `tat` thêm bước `kill -9` khi còn tiến trình sót; có `NAV_PARAMS` thì tự đối chiếu cây hành vi; ghi số tệp `/dev/shm` | `bat_xe.sh.bak` |
| `ghi_bag.sh` (v3) | ghi thêm `published_footprint` của hai costmap và `/behavior_tree_log` | không dùng |
| `bt_cho.xml`, `nav2_params_cho.yaml` | KHÔNG đổi | bật Nav2 không có `NAV_PARAMS` |

## Phần 0: chép lên Pi (làm một lần, ở tab `aaa@`)

Tải 6 tệp về thư mục Tải xuống, rồi chạy (đổi `USER` nếu khác). Bản `.bak` của `bat_xe.sh`, `kiem_xe.sh` giữ bản đang có trên Pi:

```
ssh project1@192.168.0.2 'cp -n ~/bat_xe.sh ~/bat_xe.sh.bak; cp -n ~/kiem_xe.sh ~/kiem_xe.sh.bak'; scp /mnt/c/Users/USER/Downloads/{bt_cho.xml,nav2_params_cho.yaml,chuyen.sh,ghi_bag.sh,bat_xe.sh,kiem_xe.sh} project1@192.168.0.2:~/ && ssh project1@192.168.0.2 'for f in chuyen.sh ghi_bag.sh bat_xe.sh kiem_xe.sh; do bash -n ~/$f && echo "$f cu phap OK"; done; ls -l ~/bt_cho.xml ~/nav2_params_cho.yaml'
```

Kiểm `ros2 bag` có trên Pi chưa:

```
ssh -t project1@192.168.0.2 'source ~/ros2_ws/install/setup.bash; ros2 bag --help | head -3 || echo CHUA_CO_ROS2_BAG'
```

Nếu in `CHUA_CO_ROS2_BAG`: `sudo apt install -y ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins` (cần mạng).

## Phần 0b: hai phép kiểm chỉ đọc (khoảng 2 phút, ở tab `aaa@`, chưa bật gì)

**Kiểm 1: launch Nav2 có nhận `params_file` không (chỉ là bảo hiểm: ĐÃ xác nhận trong mã nhánh `pi-30-10`, `navigation_my_bot.launch.py` dòng 82–85 khai báo `params_file`; có thể bỏ qua nếu thiếu giờ).** Theo trí nhớ của Claude về `ros2 launch` (≈ 80%, chưa thử), tham số không khai báo bị bỏ qua IM LẶNG: Nav2 sẽ chạy cây cũ (có quay, lùi) mà không báo lỗi.

```
ssh project1@192.168.0.2 'source ~/ros2_ws/install/setup.bash; ros2 launch my_bot navigation_my_bot.launch.py --show-args | grep -i -A1 params_file'
```

Mong: thấy một dòng có chữ `'params_file'`.

**Kiểm 2: `bt_cho.xml` khác cây gốc ở đâu.** Đếm thẻ của cây gốc Humble và của `bt_cho.xml`:

```
ssh project1@192.168.0.2 'diff <(grep -o "<[A-Za-z]*" /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml | sort | uniq -c) <(grep -o "<[A-Za-z]*" ~/bt_cho.xml | sort | uniq -c)'
```

Mong (theo trí nhớ của Claude về cây gốc, ≈ 85%, CHƯA đối chiếu): mất đúng `BackUp`, `RoundRobin`, `Spin`. Nếu `GoalUpdated` và `ReactiveFallback` cũng giảm từ 3 xuống 1, nghĩa là `bt_cho.xml` còn bỏ hai lớp bọc bên trong (không nguy hiểm, nhưng khác dòng mô tả "đổi đúng một chỗ"): gửi kết quả cho Claude, KHÔNG tự sửa.

## Phần 1: bật hệ thống (xe ở vạch A, đo 40/30/30)

Chạy như `Quy trinh chuan 71`, nhưng dùng `bat_xe.sh` và truyền tệp tham số mới:

```
ssh -t project1@192.168.0.2 "NAV_PARAMS=/home/project1/nav2_params_cho.yaml bash ~/bat_xe.sh '$(date +'%Y-%m-%d %H:%M:%S')'"
```

Mong thấy: `Mong doi cay hanh vi: /home/project1/bt_cho.xml`, rồi `DAT  bt_xml = /home/project1/bt_cho.xml`, rồi `SAN SANG. Con ... giay`.
Nếu thấy `KHONG DAT  bt_xml = ...` hoặc `KHONG DAT  planner_server co 2 tien trinh`: `bash ~/bat_xe.sh tat`, báo lại, chưa chạy xe.
Nếu `bat_xe.sh` lỗi (lần đầu chạy trên Pi), quay về cách thủ công của `Quy trinh chuan 71`, Nav2 vẫn bật bằng:
`ros2 launch my_bot navigation_my_bot.launch.py params_file:=/home/project1/nav2_params_cho.yaml`

**Phép thử đường đọc vị trí của `chuyen.sh`** (chỉ đọc, tab `project1@` mới, xe vẫn ở A):

```
source ~/ros2_ws/install/setup.bash; timeout 8 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -m1 Translation
```

Mong: một dòng `- Translation: [0.0.., 0.0.., 0.000]` (gần 0, 0). Không in gì: `chuyen.sh` sẽ báo không đọc được vị trí; khi đó chạy `BO_QUA_KIEM=1 bash ~/chuyen.sh ...` và báo lại.

Bật ghi dữ liệu (tab `project1@`):

```
bash ~/ghi_bag.sh bat
```

## Phần 2: các bài thử (mỗi bài ghi x/n, dừng khi cổng Core chưa đạt)

Mỗi lần gửi chuyến: công tắc motor ON, tay sát công tắc, gõ lệnh. Dừng khẩn = tắt công tắc.
Cứ 3 vòng (khoảng 8 phút) thì: **đo 3 số xe đang đứng (trước khi bê xe)**, `bash ~/bat_xe.sh tat`, đặt xe về A (40/30/30), bật lại (quy tắc 8 phút của con quay). Ba số đo trước khi bê chính là độ lệch dừng thật, không tốn thêm việc: đít xe tới tường sau, bánh phải tới tường phải, mũi xe tới tường trước (cm, từ mép ngoài).

**"Quay hoặc lùi vô định"** (định nghĩa đề xuất, Thiên sửa nếu muốn): xe lùi, hoặc xoay tại chỗ liên tục hơn 10 giây mà không tiến. Xoay có chủ đích ở đầu chuyến (quay mũi theo đường) và ở A (quay về hướng cuối) KHÔNG tính.

**Dòng `KIEM:` đầu tiên của mỗi chuyến** phải in ba cặp số gần nhau: tf, bộ lập đường, costmap. Có cặp trống `( ; )` thì ghi lại và báo.

**Bài 1: A→B và về A, 10 vòng.** Mỗi vòng:

```
bash ~/chuyen.sh di_B
bash ~/chuyen.sh ve_A
```

Đạt khi: 9/10 vòng về tới A, không có lần xe quay hoặc lùi vô định. Làm theo lô 3 vòng (lô 1 gõ `BANG=cu bash ~/chuyen.sh ve_A`, lô 2 gõ `bash ~/chuyen.sh ve_A`, xem "Bảng điểm tuyến TIẾP TUYẾN" bên dưới): lô nào dưới 2/3 thì DỪNG, `bash ~/ghi_bag.sh tat`, giữ dữ liệu, không đổi gì thêm. Sau bài, đếm:

```
awk -F, '$6=="DEN_NOI"||$6=="HET_DUONG"||$6=="SAI_VI_TRI"{t[$2]++; if($6=="DEN_NOI") ok[$2]++} END{for(k in t) print k": "ok[k]+0"/"t[k]" chuyen"}' ~/ket_qua_chuyen.csv
```

**Bài 2: chặn hành lang dưới (giữa K4 và H0) bằng tấm foam cao 30 cm, rộng ≥ 56 cm.** Có hai cách đặt, làm riêng:

- **2a: đặt tấm chắn TRƯỚC khi gõ lệnh.** Mong (suy luận ≈ 65%, chưa kiểm): bộ lập đường tự chọn hành lang trên, xe đi luôn, KHÔNG chờ 60 giây (chờ chỉ xảy ra khi không còn đường). Làm 3 lần: `bash ~/chuyen.sh ve_A` từ B.
- **2b: đặt tấm chắn SAU khi mũi xe đã vào hành lang dưới** (qua mép phía B của K4 khoảng 20 cm). Mong: xe dừng cách tấm chắn ≥ 6 cm; xoay đầu có chủ đích trong hành lang rồi đi đường kia, hoặc chờ rồi đổi tuyến. Bấm giờ từ lúc xe dừng tới lúc đi lại. Làm 3 lần.
- Từ bản 3, `TRY` mặc định 1: xe chờ hết thời gian của cây hành vi (60 giây, hoặc 30 giây sau khi làm Phần 3a) rồi đổi tuyến. `TRY=2` thì chờ gấp đôi trước khi đổi.

**Bài 3: "người" (ống Ø18 cm) đứng giữa hành lang 15 giây rồi lấy ra.** Mong: xe dừng cách ≥ 6 cm, đứng yên, lấy ống ra thì đi tiếp. Làm 3 lần.

## Nếu có điều gì khác mong đợi

| Dấu hiệu | Việc làm |
|---|---|
| `chuyen.sh` in `DUNG: Nav2 thay sai vi tri xe` (mã 3) | xe chưa chạy thêm. Chép nguyên 2 dòng `KIEM:` (ba cặp số) cho Claude, rồi tắt hết và bật lại. Cách đọc: tf đúng, bộ lập đường sai, costmap đúng ⇒ lỗi riêng của bộ lập đường; tf đúng, bộ lập đường VÀ costmap cùng sai (hoặc tuổi tin lớn) ⇒ costmap trong Nav2 thấy tf cũ; tf sai ⇒ lỗi ở SLAM |
| `kiem_xe.sh` báo `KHONG DAT ... co 2 tien trinh` hoặc `node trung ten` | còn tiến trình cũ. `bash ~/bat_xe.sh tat`, bật lại, ghi lại là lần thứ mấy sau khi bật Pi |
| xe vẫn quay hoặc lùi vô định | kiểm `DAT  bt_xml = /home/project1/bt_cho.xml` và log Nav2 (`~/log_bat_xe_nav.txt`) có lỗi nạp cây không |
| `HET_DUONG` | cả hai tuyến bị chặn hoặc toạ độ điểm sai (±5 cm đọc từ bản đồ): ghi ảnh chỗ xe dừng |
| xe kẹt sát vách ở cửa hành lang | `bash ~/ghi_bag.sh tat`, chụp ảnh, đo khoảng cách mép xe tới K4 (cm); chưa đổi gì, báo lại |

**Bảng điểm tuyến TIẾP TUYẾN (v4; Thiên chốt 31/10 12:10; rủi ro 13 và nghi ngờ xe xoay vô định).** Nav2 xoay xe tại chỗ khi hướng xe lệch quá 45° so với đường đi (lúc xuất phát) hoặc lệch quá 17,2° so với hướng của điểm đích (lúc tới điểm) [đọc cấu hình: `rotate_to_heading_min_angle` 0,785 rad, `yaw_goal_tolerance` 0,3 rad; cách Nav2 xoay là theo hiểu biết về bộ bám đường, chưa kiểm trên xe, ≈ 75%]. Bảng cũ cho mỗi điểm một hướng cố định, không theo hướng đi, nên tuyến 1 buộc xoay ≈ 102°, rồi ≈ 78°, rồi ≈ 175° ở A; tuyến 2 buộc 4 lần (≈ 158°, 22°, 67°, 113°) [tính từ toạ độ]. Hướng cuối ở A tạm không chấm (Thiên 31/10), nên không cần quay lại +x.

Bảng mới: hướng mỗi điểm = hướng đi TỚI điểm đó; mỗi chỗ rẽ ≤ 36° (thêm điểm). Còn một lần xoay lớn ở B' (≈ 120° tuyến 1, ≈ 167° tuyến 2) vì B' là ngõ cụt: không tránh được nếu không lùi.

| Tuyến | Bảng cũ (`BANG=cu`) | Bảng tiếp tuyến (`BANG=tt`, mặc định) | Xoay tại chỗ (cũ → mới) |
|---|---|---|---|
| `ve_A` 1 (hành lang dưới) | (1,45; 0,13) → A | (1,30; 0,30) → (1,00; 0,10) → A | 3 lần (≈ 102°, 78°, 175°) → 1 lần (≈ 120° ở B') |
| `ve_A` 2 (hành lang trên) | (1,45; 0,76) → (0,30; 0,72) → A | (1,30; 0,75) → (0,45; 0,76) → (0,15; 0,60) → (0,00; 0,30) → A | 4 lần → 1 lần (≈ 167° ở B') |
| `di_B` 1 | không đổi | không đổi (A→B đã 11/11 ngày 29/10) | ≈ 27° ở B (cả hai) |

**Cách chạy (không cần sửa tệp, không dùng `sed`).** Lô 1 (3 vòng) bằng bảng cũ để có số so sánh, lô 2 (3 vòng) bằng bảng mới:

```
BANG=cu bash ~/chuyen.sh ve_A
```

```
bash ~/chuyen.sh ve_A
```

(Đi `di_B` thì gõ như cũ, không cần `BANG`.) Lệnh đếm x/n ở Bài 1 tự tách thành hai dòng `ve_A_cu` và `ve_A_tt` vì tên bảng nằm trong cột 2 của `~/ket_qua_chuyen.csv`. Đường lui: dùng `BANG=cu`.

Cần quan sát ở lô 2 (một việc duy nhất): ở B' xe xoay một lần rồi đi thẳng xuống; KHÔNG xoay thêm ở `(1,30; 0,30)`, `(1,00; 0,10)` và A. Nếu xe xoay thêm, ghi điểm nào, khoảng bao nhiêu độ, và báo lại.

Mô phỏng (Nav2 đồ chơi, KHÔNG phải xe thật, 20 lần mỗi dòng): tuyến 1 bảng cũ 20/20, khe nhỏ nhất 5,9 cm, 31 giây; bảng tiếp tuyến 20/20, 8,6 cm, 27 giây. Tuyến 2 bảng cũ 18/20 (khe nhỏ nhất 0,5 cm); bảng tiếp tuyến 20/20 (4,9 cm). Mô phỏng chỉ cho biết bảng mới không tệ hơn trong mô hình đồ chơi, không chứng minh xe thật chạy tốt hơn.

Đánh đổi: nhiều điểm hơn nghĩa là nhiều lần bắt đầu chuyến, mà lỗi `0 poses` hay `SUCCEEDED` tức thì (việc 90) xảy ra lúc bắt đầu chuyến. Tuyến 1 tăng từ 2 lên 3 điểm, tuyến 2 từ 3 lên 5. Nếu số lần `KHONG_DAT` ngay điểm đầu tăng so với bảng cũ thì dừng và báo.

## Phần 3: chính sách 31/10 và "người lái" LiDAR (mới; làm SAU Bài 1, chỉ khi còn giờ)

Chính sách Thiên chốt 31/10 01:10: bị chặn phía trước thì chờ 30 giây; hết 30 giây hoặc kẹt hông/góc thì thoát bằng LiDAR (lùi/xoay có kiểm tra) rồi đi đường khác; hết đường thì dừng, chụp ảnh, báo, hiện nút lái tay. Thiết kế đầy đủ: `THIET_KE_LAI_XE_LIDAR.md`. Mỗi bước dưới đây là MỘT thay đổi; bước nào không đạt thì dừng ở bước đó.

**3a. Đổi chờ 60 → 30 giây** (chỉ đổi số lần thử của cây hành vi; một chu kỳ khoảng 4 giây nên 8 lần ≈ 30 giây [ước, bấm giờ để kiểm]). Đồng hồ 30 giây là của cây hành vi; nút người lái chỉ xen vào sau 40 giây (`t_cho`) nếu xe vẫn đứng yên, để hai đồng hồ không cộng dồn. Ở tab `aaa@`, rồi tắt và bật lại Nav2 (`bat_xe.sh tat`, bật lại như Phần 1):

```
ssh project1@192.168.0.2 'cp ~/bt_cho.xml ~/bt_cho.xml.60s && sed -i "s/number_of_retries=\"15\"/number_of_retries=\"8\"/" ~/bt_cho.xml && grep -n number_of_retries ~/bt_cho.xml'
```

Mong: dòng `NavigateRecovery` có `number_of_retries="8"`, hai dòng kia vẫn `"1"` (dòng 6 là chú thích cũ còn ghi 15, bỏ qua). Đường lui: `cp ~/bt_cho.xml.60s ~/bt_cho.xml`. Bài 2b và Bài 3: bấm giờ từ lúc xe đứng yên đến lúc xe đi lại hoặc đổi tuyến.

**3b. Chép và chạy chế độ "chỉ nhìn"** (nút không gửi lệnh nào). Chép từ `aaa@`, rồi chạy ở tab `project1@` MỚI, SAU khi `bat_xe.sh` báo sẵn sàng:

```
scp /mnt/c/Users/USER/Downloads/lai_xe_lidar.py project1@192.168.0.2:~/
```

```
source ~/ros2_ws/install/setup.bash; python3 ~/lai_xe_lidar.py
```

Mong thấy: `lai_xe_lidar san sang. phat_lenh=False (false = chi nhin)`, rồi mỗi giây một dòng `vung gan nhat: TRUOC=0.xx TRAI=0.xx ...` (4 vùng gần vật nhất, tính từ mép thân xe). Báo lại nếu thấy lỗi `ModuleNotFoundError` hoặc không có dòng nào.

**3c. So với thước** (motor OFF, xe nằm yên, chưa Nav2 cũng được). Đặt mũi xe cách tường trước đúng 10 cm bằng thước, đọc `TRUOC=`. Rồi đặt hông trái cách tường đúng 5 cm, đọc `TRAI=`. Mong: sai lệch không quá 1,5 cm [đề xuất]. Lệch hơn thì DỪNG, ghi 2 cặp số (thước, máy), không qua 3d.

**3d. Cho gửi lệnh** (chỉ sau 3c đạt; tay sát công tắc motor; mở tab `project1@` khác để gõ lệnh này):

```
source ~/ros2_ws/install/setup.bash; ros2 param set /lai_xe_lidar phat_lenh true
```

Tắt ngay: cùng lệnh với `false`, hoặc `Ctrl+C` nút (xe quay về như cũ). Thử theo thứ tự: Bài 3 (ống Ø18 cm chắn 15 giây: xe phải đứng yên, không lùi, rồi đi tiếp); Bài 2b; rồi một ca kẹt thật (xe sát tường bên hông). Đếm x/3 mỗi ca. Dòng nhật ký có chữ `[chi nhin: SE gui ...]` cho biết nút ĐỊNH làm gì khi còn ở chế độ chỉ nhìn.

**3e. Phân biệt địa hình với vật lạ** (hướng A; làm sau 3c; trong lúc có đích đang chạy, nút còn ở chế độ chỉ nhìn). Nút phải nhận được bản đồ và vị trí: ở tab chạy nút mong thấy dòng `nhan ban do 51x32, 0.05 m/o` (kích thước theo bản đồ `map_xuong_v2`). Không thấy: chạy `ros2 topic list | grep map` và báo lại.
- Ca 1: mũi xe cách tường xưởng 6 cm (tường CÓ trên bản đồ), gửi một đích bất kỳ. Mong dòng nhật ký `CHAN_TRUOC_LA_DIA_HINH` và `[chi nhin: SE gui ...]` ngay (không đếm `CHO_`). Làm 3 lần.
- Ca 2: ống Ø18 cm (KHÔNG có trên bản đồ) cách mũi 6 cm giữa hành lang. Mong `CHO_0s`, `CHO_1s`, ... đếm lên. Làm 3 lần.
- Cảnh báo `tf map->base_footprint cu ... s` xuất hiện: ghi giờ (đây là manh mối cho việc 90, vì cùng nguồn `tf` với bộ lập đường). Muốn tắt hướng A: `ros2 param set /lai_xe_lidar dung_ban_do false`.

**3f. Quyền phủ quyết** (nút nghe `/cmd_vel_nav`, tức lệnh Nav2). Chế độ chỉ nhìn: hông xe cách tường 4 cm (thước), gửi một đích cần quay đầu (ví dụ `chuyen.sh di_B` khi xe đang quay mặt về A). Mong dòng nhật ký `CHAN_XOAY` và `[chi nhin: SE gui v=0.00 w=0.00]` ngay khi Nav2 muốn quay. Nếu bật `phat_lenh`, xe không quay vào tường: đứng yên, sau khoảng 2,5 giây nút đi cung ra xa tường rồi cho Nav2 quay. Làm 3 lần.

**Chưa làm (tối 31/10 hoặc 1/11 ở nhà):** bước 5, chụp ảnh + báo trên giao diện + nút lái tay (`THIET_KE_LAI_XE_LIDAR.md` mục 6).

## Cuối buổi

```
bash ~/ghi_bag.sh tat
cd ~ && tar czf ~/dulieu_31_10.tgz bag_* ket_qua_chuyen.csv chuyen_*.txt log_bat_xe_* 2>/dev/null; ls -l ~/dulieu_31_10.tgz
```

Nếu đã thử Phần 3, chép thêm nhật ký của nút (cửa sổ `lai_xe_lidar`) vào tệp bằng cách chạy nó với `| tee ~/lai_xe_lidar.log`. Rồi chép về máy (`scp`), đính kèm vào chat, gõ "ghi giai đoạn 61". Tắt Pi bằng lệnh có chặn (`SACH_TAT_PI`).

## Điều chưa kiểm (đọc trước khi thử)

- `chuyen.sh`, `kiem_xe.sh`, `bat_xe.sh` đã thử bằng `ros2`, `pgrep`, `pkill` giả (`chuyen.sh` 9 tình huống, `kiem_xe.sh` 5, phần `tat` của `bat_xe.sh` 2), chưa thử trên Pi thật. Định dạng đầu ra của `ros2 action send_goal` và `ros2 topic echo` là theo trí nhớ về Humble.
- Đọc `tf2_echo` qua ống (`| grep`) chưa kiểm trên Pi (xem "Phép thử đường đọc vị trí").
- Toạ độ điểm trung gian đọc từ ảnh bản đồ, sai số ≈ 5 cm: hành lang dưới tâm y ≈ 0,09, hành lang trên tâm y ≈ 0,76 [đọc `map_xuong_v2.png`, chưa đo thước]. Từ điểm (1,45; 0,13) tới A đường thẳng đi qua miệng hành lang (x ≈ 1,04) ở y ≈ 0,09, tức gần đúng tâm [tính]. Dư tới K4 khi xe đi đúng đường ≈ 18 cm; nếu xe lệch khỏi đường 16–18 cm như ngày 30/10 thì còn ≈ 0,4–2,4 cm [tính, từ số đọc ảnh ± 5 cm, nên chưa biết có chạm hay không]: cần đo, không giả định.
- Sai số dừng của điểm trung gian là 15 cm (`xy_goal_tolerance` 0,15): hai điểm đầu của bảng cũ `(1,45; 0,13)` đặt cao hơn tâm hành lang để dư chỗ. Bảng tiếp tuyến: các điểm cách tường và hộp ≥ 20 cm theo mô hình hình học xưởng (đọc từ ảnh bản đồ ± 5 cm), chưa đo thước; điểm cách nhau ≥ 30 cm để khỏi "tới" ngay khi vừa dừng ở điểm trước.
- Cây gốc Humble "theo trí nhớ" (xem Kiểm 2). Cây `bt_cho.xml` chưa nạp thử vào Nav2 thật.
- Nguyên nhân gốc của "bộ lập đường tin sai vị trí xe" vẫn chưa biết: bản này chỉ giúp BẮT và PHÂN LOẠI (ba nguồn vị trí, tiến trình trùng, số tệp `/dev/shm`), chưa chữa.
