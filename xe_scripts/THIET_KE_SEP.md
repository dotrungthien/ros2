# Thiết kế "sếp" (bộ não điều phối) của xe: bản 1.5, 01/11/2026 04:00, ở nhà

Thay bản 1.4. **Bản 1.5** ghi 10 chốt của bước 1 (#22), 6 chỗ Claude Code tự hiểu đã được duyệt (#23), quyết định **tư duy trước, hoàn thiện sau** và ba mức A/B/C cho buổi chấm (#24); vòng lặp 10 lần/giây; rủi ro lệch góc. **Bản 1.4** sửa thiết kế cho khớp mã thật sau bước 0 (Claude Code rà soát chỉ đọc, 01/11 ≈ 02:00): 5 quyết định #17–#21 (Thiên "đồng ý cả 5" 01/11 02:05) và các chỗ lệch nhỏ (định nghĩa "kẹt", `so_quet_ket` ≈ 3,0 s, đồng hồ chờ, điều kiện của lớp an toàn, sếp đếm lượt thoát). **Bản 1.3** ghi 8 câu trả lời tình huống của Thiên (01/11 01:42) thành quyết định #10–#16 ở mục 1b, thêm **bảng quyền ra lệnh** (mục 2b), **độ tin cậy vị trí bằng số** (mục 3, 4), **lớp an toàn chạy độc lập và luật sếp treo** (mục 9) và **bảng đo** (mục 10b). Bỏ ý "đoán người lao tới" (#11). Bản 1.2 chỉ đổi một chỗ: **hai câu hỏi mở ở mục 12 đã được Thiên trả lời (01/11 01:26)**, ghi thành quyết định #8, #9 ở mục 1b và sửa theo ở mục 3, 4, 5, 7. Bản 1.1 đã thêm câu trả lời của Thiên lúc 23:22 (mục 1b), cách sếp suy nghĩ (mục 4) và ba kịch bản (mục 6).
Loại: **Core** (Thiên chốt 31/10), phục vụ cổng **G2** (B→A ổn định), **G3** (dừng trước vật, không đâm) và **G5** (chạy ổn định, không cần người can thiệp).
Trạng thái: **bước 1 (chỉ nhìn) đã có mã: `xe_scripts/sep.py` + `test_sep.py`, commit `f38d197`, 15/15 ca thử [log]; chưa chạy trên `Pi`.** Bước 2–4 chưa có mã. Người thiết kế là chat; **người viết, mô phỏng và kiểm mã là Claude Code** (luật phân vai 31/10). Mọi ngưỡng ghi [đề xuất] là con số khởi điểm, chưa đo.

## 1. Mục đích và tiêu chí

Một người **sếp** đứng trên cùng, nhìn được toàn cảnh và **ra lệnh cho cấp dưới**. Mục tiêu: xe tới đích đã chỉ định một cách ổn định và an toàn.

Hai nguyên tắc của Thiên:
1. **Phòng bệnh hơn chữa bệnh:** phát hiện và tránh trước khi xe vào chỗ hẹp, thay vì để xe chui vào góc kẹt rồi mới tìm cách chạy ra.
2. **Khi vẫn kẹt (phần còn lại ≈ 10%), sếp dẫn cấp dưới thoát ra có kế hoạch**, không để xe hoảng loạn rồi bỏ cuộc.

Tiêu chí "ổn định 90% thời gian": **trong 10 chuyến, ít nhất 9 chuyến xe tới đích mà không cần người can thiệp.**

## 1b. Quyết định của Thiên

| # | Quyết định | Hệ quả cho thiết kế |
|---|---|---|
| 1 | **Sếp quyết định dựa trên thông tin và bức tranh toàn cảnh đang có.** Cấp dưới là người đưa thông tin. Sếp tổng hợp, vẽ bức tranh toàn cảnh, vạch chiến lược rõ ràng rồi mới quyết hành động và hướng đi. **Sếp quyết sai thì truy ngược xuống cấp dưới, tìm bộ phận nào đưa thông tin sai và giải quyết tận gốc.** (31/10 23:22) | Mục 4 và mục 5 |
| 2 | **Vùng mù dưới 10 cm: chấp nhận.** LiDAR là đôi mắt duy nhất; vật rơi vào vùng mù thì xe mù. Sếp chỉ chịu trách nhiệm với vật ở ngoài 10 cm. (31/10 23:22) | Mục 8 |
| 3 | **Thầy chấp nhận chờ lâu.** (31/10 23:22) | Bỏ giới hạn thời gian cứng cho mỗi chuyến; vẫn giữ giới hạn *số lượt* thoát và *số lần* khởi động lại |
| 4 | **Vùng đỏ: không tới được** (giao diện đã ghi rõ). (31/10 23:22) | Sếp không phải xử lý đích nằm trong vùng đỏ |
| 5 | **Thêm kiểm vị trí giữa chuyến:** nhảy lớn thì dừng và kiểm lại. (31/10 23:22) | Mục 4 |
| 6 | **Xưởng mini không có vật thấp hơn 17,5 cm** (Thiên loại bỏ hết). (31/10 23:22) | Điều kiện bắt buộc của môi trường chấm (mục 8) |
| 7 | Câu hỏi tình huống phải cụ thể và xây được cây tư duy của sếp. (31/10 23:22) | Mục 4 và mục 6 |
| 8 | **Sếp được tự khởi động lại Nav2, tối đa 2 lần mỗi chuyến; mỗi lần ghi một dòng nhật ký và hiện lên màn hình.** Quá 2 lần thì DỪNG VÀ BÁO. (Thiên "đồng ý" 01/11 01:26) | Mục 5 (dòng Nav2 mất nhịp tim); nhật ký giữ đủ dấu vết để không che lỗi gốc; việc tìm gốc vẫn là phép thử G (việc 93) |
| 9 | **Chờ 30 s mà vật lạ vẫn chắn giữa đường ⇒ đổi hành lang ngay, không chờ thêm.** Riêng vật chắn **đúng tại B**: chờ 2 lượt × 30 s rồi báo (không có đường khác tới B). (Thiên "theo khuyến nghị" 01/11 01:26) | Mục 4 (tư thế CHỜ), mục 7 |
| 10 | **Phản xạ an toàn luôn đi trước chiến lược:** khi `lai_xe_lidar` đang phản xạ hoặc thoát (vùng đỏ), sếp chờ nó xong (≤ 3 s [đề xuất]) rồi mới thi hành quyết định mới (ví dụ đổi hành lang). (Thiên "oke, có vẻ rất có ích" 01/11 01:42) | Mục 2b |
| 11 | **Không làm "đoán người lao tới":** trong xưởng mini, "người" là ống trụ do Thiên điều khiển, không lao tới. Ngưỡng dừng: xem #20. (Thiên 01/11 01:42) | Ý này chuyển sang Stretch; ràng buộc ≥ 6 cm vẫn kiểm bằng phép đo quãng dừng (mục 10b) |
| 12 | **Ngoại lệ của #9:** hết 30 s mà khe đang **rộng dần** (vật đang đi ra) thì chờ thêm tối đa **10 s** [đề xuất]; khe không rộng thêm thì đổi hành lang ngay. (Thiên "đồng ý đề xuất" 01/11 01:42) | Mục 4 (tư thế CHỜ) |
| 13 | **Bản đồ méo làm tụt độ tin cậy ở cùng một đoạn mọi chuyến ⇒ quét lại bản đồ trước 3/11**, không nới ngưỡng. (Thiên "đồng ý" 01/11 01:42) | Mục 4, mục 10b; đổi thứ tự so với quyết định 31/10 "quét lại sau G2" khi có dấu hiệu này |
| 14 | **Sếp treo rồi sống lại: bắt đầu lại từ bước A**, bỏ quyết định cũ, ghi nhật ký thời gian treo. (Thiên "đồng ý" 01/11 01:42) | Mục 9 |
| 15 | **Sau khi khởi động lại Nav2: kiểm "đi hay không đi" trước** (vị trí khớp, đường hợp lý) rồi mới gửi lại đích. **`SUCCEEDED` mà còn cách đích > 0,15 m: gửi lại 1 lần;** lần sau vẫn lệch ≤ 0,20 m thì coi là tới và hiện "lệch X cm" lên màn hình. (Thiên "kiểm trước", "gửi lại 1 lần" 01/11 01:42) | Mục 5 |
| 16 | **Vị trí không tin (độ tin cậy thấp) thì cấm mọi động tác thoát và đổi hành lang:** chỉ đứng yên CHỜ và kiểm lại vị trí; vẫn thấp sau 15 s thì DỪNG VÀ BÁO. (Thiên "đồng ý" 01/11 01:42) | Mục 4 |
| 17 | **Sếp huỷ đích thì kèm một chữ trong tệp `~/sep_lenh` (`CHO` / `DOI` / `DUNG`);** `chuyen.sh` đọc tệp khi đích bị huỷ để biết chờ, đổi tuyến hay dừng. Trước đây `chuyen.sh` coi mọi đích bị huỷ là hỏng và đổi tuyến ngay. (Thiên "đồng ý cả 5" 01/11 02:05) | Mục 2b; sửa `chuyen.sh` (Claude Code, từ bước 2) |
| 18 | **THẬN TRỌNG không làm xe chậm lại**, chỉ kiểm LiDAR mỗi vòng và sẵn sàng huỷ đích. Giới hạn tốc độ qua Nav2 (`speed_limit`) là Stretch. Lý do: xe đã chạy gần tốc độ tối thiểu ≈ 0,092 m/s [tính], và đó là quyền mới chưa kiểm. (01/11 02:05) | Mục 4 |
| 19 | **Làm rõ #10 và #16:** khi vị trí không tin, `lai_xe_lidar` **vẫn được** làm động tác chỉ dựa vào LiDAR (dừng, lùi thẳng, đi cung ra xa tường); **cấm** những việc dựa vào bản đồ (chọn hướng theo đích, đổi hành lang). Lý do: LiDAR đo trực tiếp, không cần vị trí đúng. (01/11 02:05) | Mục 4, mục 7 |
| 20 | **Bỏ ngưỡng dừng 25–30 cm** (chỉ là đề xuất, không bộ phận nào làm). Giữ cách đang có: Nav2 dừng xe ≈ 8 cm từ mũi trước ống (9/9 lần ≥ 6 cm [Thiên đo 29–30/10]) và vùng vàng 15 cm của `lai_xe_lidar`; kiểm 10/10 ≥ 6 cm theo bảng đo. (01/11 02:05) | Mục 4, mục 10b; thay phần "giữ ngưỡng 25–30 cm" của #11 |
| 21 | **Khởi động lại Nav2 chỉ bằng một đường: `bat_xe.sh`** (thêm chế độ chỉ bật lại Nav2). Không dùng cách khác song song. (01/11 02:05) | Mục 5; Claude Code đọc `bat_xe.sh` ở bước 1 |
| 22 | **10 chốt của bước 1** (Thiên 01/11 02:21–03:16): (1) đích = điểm cuối `/plan` cuối cùng trước `SUCCEEDED`; (2) "chuyến" theo đuôi `~/ket_qua_chuyen.csv` (`KIEM_DAT` bắt đầu; `DEN_NOI` / `HET_DUONG` / `SAI_VI_TRI` kết thúc); (3) độ tin cậy trên mọi điểm ≤ 3 m; (4) "nhảy" = độ đổi `map→odom`; (5) trạng thái `lai_xe_lidar` đọc từ `/rosout`, có ca thử canh câu log; (6) vòng lặp theo đồng hồ 10 lần/giây, `/scan` cũ > 0,5 s ⇒ `MAT_SCAN`; (7) `0 poses` ghi cả `/plan` rỗng và `ABORTED` < 2 s; (8) hiện lên màn hình và (9) đo khe 26 cm để bước 3; (10) danh sách cần kiểm trên `Pi` ghi ở đầu `sep.py` | Mục 3, 4, 5 |
| 23 | **6 chỗ Claude Code tự hiểu, đã duyệt** (01/11 03:30): bỏ nghe `/cmd_vel_nav`, `/cmd_vel_avoid` (dùng `/ket` và `/rosout`); mức NGHI bị chắn thì chỉ CHỜ, không đổi hành lang; `SUCCEEDED` lệch thì luôn gửi lại 1 lần trước, lần 2 vẫn > 0,20 m mới là Nav2 báo giả; 15 s của mức BÁO tính từ lúc độ tin cậy bắt đầu tụt; ca 14 kiểm không có `subprocess`/`os.system`, không action client, chỉ một publisher String | Mục 4, 5 |
| 24 | **Tư duy trước, hoàn thiện sau** (Thiên 01/11 03:46–03:49): mọi buổi chấm bắt buộc có sếp chạy trên xe. **Mức A** chỉ nhìn, ghi "tôi sẽ làm gì" (sàn, bắt buộc; thầy xem bằng `ros2 topic echo /sep/trang_thai`); **mức B** = A + "đi hay không đi"; **mức C** = B + tự bật lại Nav2 + chờ 30 s rồi đổi tuyến (bước 3 rút gọn, chưa đo khe). B, C chỉ bật khi ≥ 9/10; chưa đạt thì lùi mức, không dưới A. Sếp làm Nav2 tắt nhiều hơn ⇒ làm sếp nhẹ đi (bỏ nghe `/rosout`, giảm 5 lần/giây), không bỏ sếp | Mục 9, 10 |

**Các chỗ lệch nhỏ sửa ở bản 1.4 (theo báo cáo bước 0):** (a) **một định nghĩa "kẹt"** là của `lai_xe_lidar`: sát vật **và** Nav2 không gửi lệnh tiến hay xoay, trong `so_quet_ket` vòng; (b) vòng của `lai_xe_lidar` chạy **10 lần/giây** nên `so_quet_ket` 30 ≈ **3,0 s** (không phải 2,5 s); (c) **đồng hồ chờ chính là 30 s của sếp** (và của cây hành vi); 40 s của `lai_xe_lidar` chỉ là dự phòng; (d) tuổi `tf`: `lai_xe_lidar` coi tf cũ hơn 1 s là "không dùng bản đồ được" (cho phản xạ), sếp coi tf cũ hơn 5 s là "định vị hỏng" (cho quyết định); hai ngưỡng khác mục đích, giữ cả hai; (e) **sếp đếm 3 lượt thoát**, vì `lai_xe_lidar` tự nó không giới hạn số lượt.

## 2. Kiến trúc: sếp và cấp dưới

```
                 SẾP (một chương trình chạy liên tục, nghe một lần)
   nhận thông tin từ cấp dưới -> tổng hợp -> bức tranh -> chiến lược -> quyết
        │ ra lệnh: đi / không đi, chọn hành lang,
        │          chờ, thoát, đổi tuyến, khởi động lại, báo người
   ┌────┼───────────────┬──────────────────┐
  Nav2             lai_xe_lidar        chuyen.sh
 (vạch đường,      (phản xạ sát vật,   (gửi từng điểm
  bám đường)        động tác thoát)     của tuyến)
```

- Cấp dưới **giữ nguyên việc của mình** và **đưa thông tin** cho sếp. Sếp không viết lại Nav2; sếp quyết *việc gì, lúc nào, theo hướng nào*.
- `lai_xe_lidar.py` là cấp dưới phụ trách phản xạ và động tác thoát; sếp cho nó biết kế hoạch (hướng tới đích, hành lang đang đi, ngõ cụt B′).
- `chuyen.sh` vẫn là **đường lui**: tắt sếp thì hệ thống chạy như trước.

## 2b. Bảng quyền ra lệnh (ai được điều khiển bánh xe)

Bánh xe chỉ nghe **một** lệnh tốc độ mỗi lúc, do `twist_mux` chọn theo ưu tiên (đã có, chạy thật).

| Thứ tự | Ai | Kênh (ưu tiên) | Khi nào được phát lệnh | Ghi chú |
|---|---|---|---|---|
| 0 | **Công tắc motor** | phần cứng | bất cứ lúc nào | dừng khẩn, thắng mọi thứ |
| 1 | Người lái tay (`teleop_web`) | `/cmd_vel_teleop` (100) | khi người bấm | thắng mọi phần mềm |
| 2 | `lai_xe_lidar` (phản xạ, thoát) | `/cmd_vel_avoid` (60) | **chỉ** khi vùng đỏ, kẹt, hoặc khi sếp giao động tác thoát; ngoài lúc đó im lặng | **Phản xạ đi trước chiến lược (#10)**; vẫn chạy khi sếp treo hoặc chết. **Chỉ có hiệu lực khi `phat_lenh=true`** (mặc định đang tắt; bật sau phép thử việc 96). Ngoài ra còn phát tốc độ 0 khi mũi xe cách vật ≤ 15 cm (vùng vàng) để giữ xe đứng chờ |
| 3 | Nav2 | `/cmd_vel_nav` (20) | khi có đích | |
| — | **Sếp** | **không phát tốc độ** | ra lệnh gián tiếp: huỷ đích Nav2 **kèm một chữ trong tệp `~/sep_lenh`** (#17), bật cờ thoát cho `lai_xe_lidar`, khởi động lại Nav2 qua `bat_xe.sh` (#21) | Sếp không giành bánh xe, nên sếp có lỗi cũng không làm xe chạy bậy |

Mâu thuẫn được xử lý theo bảng này, không theo "ai nói sau". Sếp muốn xe dừng thì **ghi `CHO`, `DOI` hoặc `DUNG` vào `~/sep_lenh` rồi huỷ đích Nav2**; `chuyen.sh` thấy đích bị huỷ thì đọc tệp đó: `CHO` ⇒ chờ rồi gửi lại cùng điểm, `DOI` ⇒ đổi tuyến, `DUNG` ⇒ dừng hẳn và báo (#17). Không có tệp thì `chuyen.sh` làm như cũ (đường lui). Nếu cần giữ yên tuyệt đối thì yêu cầu `lai_xe_lidar` phát tốc độ 0. **Sếp không có cách làm xe chạy chậm lại** (#18).

## 3. Sếp nhận thông tin gì từ cấp dưới

Mỗi thông tin đi kèm **ba nhãn: nguồn, giờ, độ tin cậy**. Nhãn này là thứ cho phép truy vết (mục 5).

| Nguồn (cấp dưới) | Thông tin | Ghi chú |
|---|---|---|
| Định vị (`tf` `map → base_footprint`) | xe ở đâu | tuổi ≥ 5 s là dấu hiệu cũ |
| Bộ lập đường (`/plan`) | đường đi dự định | lỗi 30/10: điểm đầu cách xe ≈ 1,1 m |
| LiDAR (`/scan`, ≈ 12 Hz) | khoảng cách quanh thân xe | `range_min` 0,10 m; **dưới 10 cm là vùng mù, chấp nhận** |
| Bản đồ và bản đồ chi phí (`/map`, costmap) | khoảng thoáng, địa hình hay vật lạ | cần định vị đúng |
| Trạng thái mục tiêu Nav2 | xe đang đi, chờ hay hỏng | `ABORTED` ≠ xe tới đích; `SUCCEEDED` ≠ xe tới đúng chỗ |
| Nhịp tim Nav2, nhật ký `CRITICAL FAILURE` | phần mềm còn sống không | 31/10: Nav2 tự tắt sau 97 s và 183 s; **sếp được khởi động lại tối đa 2 lần mỗi chuyến (#8)** |
| `time.monotonic()` | mọi hẹn giờ | không dùng giờ hệ thống (`Pi` không có pin giữ giờ) |
| **Độ tin cậy vị trí (0–1, sếp tự tính)** | phần trăm điểm LiDAR (mọi điểm ≤ 3 m) rơi trúng ô tường của bản đồ (lệch ≤ 1 ô), tính trên mỗi lần quét. **Kém nhạy lệch góc** (10° vẫn 0,71–0,84 [mô phỏng]) | **≥ 0,70 tin; 0,50–0,70 nghi (THẬN TRỌNG); < 0,50 trong 5 s ⇒ không tin** [đề xuất, đặt lại theo bảng đo]. Cùng phép tính với "mất vị trí 5 s / 15 s" đã chốt 31/10 |

Sếp là **một chương trình nghe một lần** và giữ trạng thái trong bộ nhớ, không gọi lệnh `ros2` lặp (giả thuyết G ≈ 40% [ước]: các lệnh lặp làm Nav2 mất nhịp tim).

## 4. Sếp suy nghĩ thế nào (cây tư duy)

Mỗi vòng (**10 lần/giây theo đồng hồ**, lấy lần quét LiDAR mới nhất; #22), sếp đi qua năm bước:

**Bước A. Thu thập.** Nhận thông tin từ cấp dưới cùng nhãn nguồn, giờ, độ tin cậy.

**Bước B. Tổng hợp thành bức tranh toàn cảnh.** Một bản ghi duy nhất, gồm: vị trí xe và độ tin cậy; đích; đường đi và điểm đầu có khớp vị trí xe không; khoảng thoáng phía trước, sau, hai hông; vật phía trước là địa hình hay vật lạ; hành lang nào thông; phần mềm nào khỏe; chuyến đang ở giai đoạn nào.
Ở đây sếp **đối chiếu chéo**: các nguồn có nhất quán không (định vị với đường đi; LiDAR với bản đồ; trạng thái Nav2 với chuyển động thật). **Nếu mâu thuẫn, sếp không quyết dựa vào nguồn nghi sai** mà chuyển sang mục 5 (truy vết).

**Bước C. Chiến lược.** Từ bức tranh, sếp chọn một trong sáu *tư thế*:

| Tư thế | Khi nào |
|---|---|
| ĐI THƯỜNG | vị trí tin cậy, đường phía trước thoáng |
| THẬN TRỌNG | sắp vào chỗ hẹp (hành lang 50–56 cm, góc K4); hoặc độ tin cậy vị trí 0,50–0,70: **kiểm LiDAR mỗi vòng, sẵn sàng huỷ đích; không làm chậm xe (#18)** |
| CHỜ | vật lạ phía trước đã làm xe dừng (Nav2 dừng ≈ 8 cm từ mũi; vùng vàng 15 cm của `lai_xe_lidar`) (#20): đứng im chờ **30 s**; nếu khe **đang rộng dần** thì chờ thêm tối đa 10 s (#12); **vị trí không tin thì chỉ được CHỜ (#16), trừ động tác chỉ dựa vào LiDAR (#19)** |
| ĐỔI HÀNH LANG | chỗ hẹp sắp tới dưới 26 cm; hoặc **chờ đủ 30 s (+ tối đa 10 s nếu khe đang rộng dần) mà vật vẫn chắn giữa đường (#9, #12)**; **cấm khi vị trí không tin (#16)** |
| THOÁT | xe kẹt (mục 7); **cấm khi vị trí không tin (#16)** |
| DỪNG VÀ BÁO | thông tin mâu thuẫn không giải được; Nav2 chết lần thứ 3 trong chuyến (#8); vật chắn tại B sau 2 × 30 s (#9); vị trí không tin quá 15 s (#16); hết đường |

**Thứ tự ưu tiên khi nhiều chuyện cùng lúc:** (1) phản xạ an toàn của `lai_xe_lidar` (#10); (2) vị trí có tin được không (#16); (3) phần mềm còn sống không (#8); (4) vật cản và đường đi. Ví dụ: người chắn phía trước **và** độ tin cậy 0,60 ⇒ chỉ CHỜ đứng yên và kiểm lại vị trí, không đổi hành lang.

**Bước D. Hành động.** Sếp gửi lệnh cho cấp dưới **kèm một kỳ vọng**: "gửi mục tiêu X, kỳ vọng sau 3 s xe tiến ít nhất 0,2 m, khoảng thoáng phía trước vẫn ≥ 0,25 m". Phòng bệnh nằm ở đây: sếp *nhìn xa* bằng cách lấy 0,6 m [đề xuất] đầu của `/plan` đặt lên LiDAR trực tiếp, và đổi tư thế *trước khi* vào chỗ hẹp.

**Bước E. Kiểm kỳ vọng.** Vòng sau, sếp so kết quả thật với kỳ vọng. Khớp thì tiếp tục. **Không khớp thì sếp không đoán mà truy vết nguồn thông tin sai** (mục 5), rồi mới đổi tư thế.

**Kiểm giữa chuyến (quyết định #5).** Mỗi vòng sếp theo dõi *nhảy vị trí*: nếu vị trí xe trên bản đồ nhảy lớn (khởi điểm: ≥ 0,10 m hoặc ≥ 5° trong chưa tới 1 s [đề xuất]) mà con quay và lệnh không giải thích được, sếp **dừng xe**, đối chiếu lại định vị với đường đi và LiDAR. Khớp ⇒ đi tiếp. Không khớp ⇒ dừng và báo. SLAM được phép sửa tối đa ±21° và ±0,26 m một lần, nên nhảy là chuyện có thể xảy ra, và sếp phải kiểm *sau* nhảy chứ không cấm nhảy.

## 5. Truy vết nguồn thông tin sai

Khi mâu thuẫn hoặc kỳ vọng không khớp, sếp hỏi: *thông tin nào sai, do cấp dưới nào đưa?* Bảng dưới là các ca đã gặp thật:

| Dấu hiệu | Nghi nguồn | Phép kiểm rẻ | Xử lý |
|---|---|---|---|
| Điểm đầu `/plan` cách xe > 0,30 m (30/10: ≈ 1,1 m) | bộ lập đường nhận sai vị trí xe | gọi dịch vụ lập đường với `start` đúng (30/10 làm tay: 79 chấm thay vì 6) | không đi; khởi động lại phần lập đường; ghi nhật ký |
| `tf` cũ ≥ 5 s | định vị hoặc đường truyền `tf` | đọc tuổi `tf`; kiểm `/scan` còn đến không | dừng; chờ `tf` mới; hết hạn mức thì phục hồi |
| `lai_xe_lidar` báo kẹt (sát vật **và** Nav2 không gửi lệnh trong 30 vòng ≈ 3,0 s; định nghĩa duy nhất); hoặc có lệnh tiến mà vị trí không đổi | bánh bị chặn (kẹt thật) hoặc odom sai | khoảng cách LiDAR không đổi ⇒ kẹt thật; LiDAR đổi mà vị trí không đổi ⇒ vị trí sai | kẹt thật ⇒ THOÁT (sếp đếm lượt); vị trí sai ⇒ dừng và kiểm định vị |
| Nav2 báo `SUCCEEDED` mà còn cách đích > 0,15 m, hoặc chỉ sau 0,05 s | trạng thái Nav2 giả | so vị trí `tf` với đích | **gửi lại đích 1 lần (#15)**; lần sau vẫn lệch ≤ 0,20 m thì coi là tới và hiện "lệch X cm"; lệch > 0,20 m hoặc `SUCCEEDED` sau 0,05 s ⇒ khởi động lại cả chồng Nav2 (bài học 268), tính vào giới hạn 2 lần |
| Vị trí nhảy lớn giữa chuyến | SLAM sửa hoặc trượt bánh | so với góc con quay và lệnh | dừng, đối chiếu, đi tiếp hoặc báo |
| Nav2 mất nhịp tim (`CRITICAL FAILURE`) | Nav2 hoặc mạng ROS | đọc nhật ký; kiểm tiến trình còn sống | dừng xe; **khởi động lại Nav2, tối đa 2 lần mỗi chuyến (#8)**; mỗi lần ghi nhật ký và hiện lên màn hình; **lên lại thì kiểm "đi hay không đi" trước khi gửi lại đích (#15)**; lần thứ 3 ⇒ DỪNG VÀ BÁO |
| Vật phía trước: LiDAR thấy nhưng bản đồ nói ô trống | vật lạ (người, hộp) hoặc định vị lệch | hỏi bản đồ (hướng A); kiểm vị trí | vật lạ ⇒ CHỜ 30 s rồi ĐỔI HÀNH LANG (#9); định vị lệch ⇒ kiểm định vị |

**Nhật ký truy vết:** mỗi lần mâu thuẫn, sếp ghi một dòng gồm giờ, dấu hiệu, nguồn nghi, phép kiểm và kết quả, xử lý. Nhờ đó khi xe quyết sai, người đọc nhật ký biết *bộ phận nào đưa thông tin sai* và sửa đúng chỗ, không vá triệu chứng.

## 6. Ba kịch bản cụ thể (đi qua cây tư duy)

**Kịch bản 1: phòng bệnh. Ống chắn giữa hành lang dưới, xe chưa vào.**
Xe ở (1,45; 0,13), đường Nav2 vạch qua hành lang dưới rộng 56 cm. Ống Ø 18 cm đứng giữa hành lang.
- A: LiDAR báo vật cách mũi xe 0,55 m. B: bản đồ nói ô đó trống ⇒ **vật lạ**; mỗi bên ống còn ≈ 19 cm [tính], dưới ngưỡng 26 cm.
- C: tư thế **ĐỔI HÀNH LANG** (không chờ vào tận nơi).
- D: sếp bảo `chuyen.sh` chuyển sang tuyến 2 (hành lang trên rộng 52 cm), kỳ vọng sau 5 s xe quay về hướng hành lang trên.
- E: xe quay đúng, tiếp tục. Không bao giờ chui vào hành lang dưới. Đây là *phòng bệnh*.

**Kịch bản 2: chữa bệnh. Xe đã vào, hông trái cách tường 5 cm, Nav2 không tiến được.**
- A: lệnh tiến có 3 s, vị trí không đổi, LiDAR hông trái 5 cm. B: không mâu thuẫn ⇒ **kẹt thật**, kẹt kiểu hông (H).
- C: tư thế **THOÁT**. Bước 1 hiểu kiểu kẹt (H). Bước 2 chọn hướng: ra xa tường trái và về phía đích. Bước 3 một động tác: đi cung 0,10 m/s, ω 0,3 rad/s ra xa tường, kiểm LiDAR sau mỗi lần quét.
- D: kỳ vọng sau 3 s hông cách tường ≥ 12 cm. E: đạt ⇒ trả quyền cho Nav2. Không đạt ⇒ lượt thoát 2 (đổi động tác), tối đa 3 lượt; hết thì đổi hành lang; hết đường thì DỪNG VÀ BÁO.

**Kịch bản 3: thông tin mâu thuẫn. Bộ lập đường nhận sai vị trí (lỗi 30/10).**
- A: định vị nói xe ở (1,17; 0,64); `/plan` bắt đầu ở (0,07; 0,14), cách xe ≈ 1,1 m. B: **mâu thuẫn** giữa hai nguồn.
- Sếp không đoán. Truy vết: nghi bộ lập đường nhận sai vị trí (bảng mục 5); phép kiểm: gọi lập đường với `start` đúng.
- C/D: tư thế **KHÔNG ĐI**; khởi động lại phần lập đường; ghi nhật ký. E: lập đường lại cho 79 chấm bắt đầu đúng chỗ xe ⇒ đi. Nếu hai lần chưa được ⇒ DỪNG VÀ BÁO.

## 7. Cứu xe khi kẹt (nhánh THOÁT)

| Bước | Sếp làm gì | Dựa vào |
|---|---|---|
| 1. Hiểu kẹt kiểu gì | dừng; xác định bị chặn phía trước, sau, hông hay góc | LiDAR theo vùng quanh thân (`lai_xe_lidar`: F, B, H) |
| 2. Chọn hướng thoát | hướng thoáng nhất **và** gần đường tới đích nhất; tránh ngõ cụt B′ | bản đồ, `/plan`, LiDAR |
| 3. Một động tác nhỏ | xoay (chỉ khi mọi điểm LiDAR cách tâm xe > 0,1855 m), lùi thẳng hoặc đi cung ra xa tường; kiểm LiDAR sau **mỗi lần quét** | bảng thoát mục 4 của `THIET_KE_LAI_XE_LIDAR.md` |
| 4. Hết hạn mức thì đổi cách | tối đa **3 lượt** [Thiên chốt], **sếp đếm** (vì `lai_xe_lidar` tự nó không giới hạn số lượt), mỗi lượt ≤ 10 s; chưa ra thì đổi hành lang (cấm khi vị trí không tin, #19) | sếp; `chuyen.sh` (`TRY`) |
| 5. Hết đường | dừng tại chỗ, chụp ảnh phía trước, báo người phụ trách, hiện nút lái tay | `teleop_web`; **chưa có mã** |

Bị chặn phía trước bởi vật lạ (không phải kẹt): CHỜ 30 s ⇒ vẫn chắn thì **ĐỔI HÀNH LANG ngay** (#9). Vật chắn đúng tại B: chờ 2 × 30 s ⇒ DỪNG VÀ BÁO.
Khác với cách đã bỏ ngày 30/10 (quay trái rồi lùi mặc định của Nav2): hướng được **chọn bằng thông tin**, số lần **có giới hạn**, mỗi động tác **được kiểm**, và **bỏ cuộc có kiểm soát**.
Hai giới hạn thật: **xoay tại chỗ cần ≥ 45,6 cm** [tính], sát trong hành lang 50–56 cm nên lùi thẳng an toàn hơn; **ngõ cụt B′** (quay đầu ≈ 120° đến 167°) phải được sếp biết trước.

## 8. Giới hạn đã chấp nhận và điều kiện môi trường

- **Vùng mù dưới 10 cm** (LiDAR là mắt duy nhất): chấp nhận (Thiên 31/10). Vật rơi vào vùng mù thì xe mù, sếp không bù.
- **Điều kiện bắt buộc của xưởng khi chấm:** không có vật thấp hơn mặt phẳng laser (≈ 17,5 cm) như dây điện, mép thảm; mọi vật trong xưởng cao 30 cm hoặc là ống Ø 18 cm cao 30 cm trở lên. Kiểm trước mỗi buổi chấm.
- **Vùng đỏ không tới được**; giao diện chặn và ghi rõ.
- **Thầy chấp nhận chờ lâu**: không đặt giới hạn thời gian cứng; vẫn giữ giới hạn số lượt thoát (3) và số lần khởi động lại Nav2 (2).

## 9. Nguyên tắc ổn định

1. **Mặc định chỉ nhìn:** sếp in "tôi sẽ làm gì" và ghi nhật ký nhưng không gửi lệnh nào. Bật ra lệnh bằng một lệnh `ros2 param set`.
2. **Tắt sếp thì xe chạy như trước** (Nav2 + `chuyen.sh`). Không có thứ gì chỉ chạy được khi có sếp.
3. **Một chương trình, nghe một lần.** Không gọi `ros2` CLI lặp.
4. **Hẹn giờ bằng `time.monotonic()`.**
5. **Mỗi lần đổi một thứ**, thử x/10 rồi mới đổi thứ kế tiếp.
6. **Không im lặng:** mất `/scan` hoặc `tf` thì dừng xe và ghi nhật ký, không đoán. Mỗi lần khởi động lại Nav2 đều hiện lên màn hình.
7. Thêm sếp **sau** phép thử G (việc 93): sếp là thêm một node ROS, nên không đưa vào khi Nav2 còn tự tắt chưa rõ vì sao.
8. **Lớp an toàn chạy độc lập:** vùng đỏ của `lai_xe_lidar` chặn xe đi về phía vật **dù sếp đang treo hoặc đã chết**. `lai_xe_lidar` không cần sếp để làm phản xạ. **Điều kiện:** `phat_lenh=true` (mặc định đang tắt, tức chỉ nhìn). Trước khi bật, lớp an toàn duy nhất là Nav2 và công tắc motor.
9. **Sếp tự canh mình:** mỗi vòng ghi giờ; một vòng chậm hơn 1 s [đề xuất] thì ghi "sếp treo X s". Khi sống lại, **bỏ quyết định cũ, bắt đầu lại từ bước A (#14)**. Trong lúc sếp treo, sếp không giữ bánh xe (mục 2b) nên xe chỉ chịu lệnh của Nav2 và phản xạ.

## 10. Kế hoạch từng bước (mỗi bước thử được, có đường lui)

| Bước | Việc | Đạt khi | Đường lui |
|---|---|---|---|
| 0 | Claude Code rà soát `lai_xe_lidar.py`, `chuyen.sh` và các tệp liên quan (chỉ đọc); chạy mô phỏng `mo_phong_he_thong.py` | báo cáo bảng; mô phỏng chạy | không đổi gì |
| 1 | Sếp **chỉ nhìn**: thu thập, tổng hợp, ghi nhật ký "sẽ làm gì" và truy vết, trong lúc `chuyen.sh` chạy như cũ | nhật ký khớp thực tế (ví dụ báo "không đi" đúng chuyến `0 poses`; báo mâu thuẫn đúng chỗ) | tắt sếp |
| 2 | Bật quyết định "đi hay không đi" | trong 10 chuyến, không còn chuyến nào bắt đầu với đường sai | `ros2 param set` tắt |
| 3 | Thêm chọn hành lang (gồm đổi ngay sau 30 s chờ), kiểm giữa chuyến và khởi động lại Nav2 tối đa 2 lần qua `bat_xe.sh` (#21); `chuyen.sh` đọc `~/sep_lenh` (#17) | ≥ 9/10 chuyến B→A tới đích; Nav2 tự tắt thì xe dừng, khởi động lại và đi tiếp | tắt từng quyết định |
| 4 | Thêm cứu xe (nhánh THOÁT) dùng phần thoát của `lai_xe_lidar` | ca kẹt hông/góc dựng sẵn: thoát ≥ 9/10, không chạm | tắt, quay về bước 3 |

Thứ tự ở trường: `so_voi_pi.sh` → phép thử G → bước 1 trên xe (10 chuyến chỉ nhìn).
**Trạng thái 01/11 04:00:** bước 0 xong; bước 1 có mã (15/15 ca thử), chưa chạy trên `Pi`. **Lịch [Thiên báo]:** 1/11 ở xưởng tới 18:00 (bước 1 trên xe); tối 1/11 Claude Code viết bước 2 và bước 3 rút gọn; 2/11 ở xưởng tới 20:00 (bước 2, 3 x/10) ⇒ chốt mức A/B/C cho 3/11 (#24).

## 10b. Bảng đo: biến [đề xuất] thành [đo]

Mỗi ngưỡng chỉ được dùng để ra lệnh (bước 2 trở đi) khi đã có số đo. Đo bằng sếp ở chế độ chỉ nhìn (bước 1) cộng thước, lúc xe chạy bình thường bằng `chuyen.sh`.

| Ngưỡng | Đang đặt | Cách đo | Đạt khi | Đặt lại thành |
|---|---|---|---|---|
| Khoảng dừng trước vật lạ (ràng buộc cứng ≥ 6 cm) | Nav2 ≈ 8 cm từ mũi (9/9 ≥ 6 cm [Thiên đo]); vùng vàng `lai_xe_lidar` 15 cm (#20) | ống Ø 18 cm đặt giữa hành lang, xe chạy tới 10 lần; thước đo mũi xe → ống sau khi dừng hẳn | **10/10 lần ≥ 6 cm** | không đạt 10/10 thì mới tính thêm lớp dừng sớm (khi đó quyết lại) |
| Độ tin cậy vị trí (0,70 / 0,50) | [đề xuất] | ghi độ tin cậy mỗi lần quét trong 10 vòng bình thường | 0 lần xuống dưới 0,50 khi xe đúng chỗ | 0,70 → (thấp nhất đo được − 0,10) |
| Nhảy vị trí giữa chuyến (0,10 m / 5°) | [đề xuất] | ghi mỗi lần SLAM sửa vị trí trong 10 vòng | 0 báo nhầm | 2 × nhảy lớn nhất khi bình thường |
| `LECH_MAX` (điểm đầu đường lệch xe) | 0,30 m [ước] | cột `lech_kiem_m` của `~/ket_qua_chuyen.csv` qua 10 chuyến | 0 báo nhầm | 2–3 × lệch lớn nhất khi bình thường |
| `so_quet_ket` (xác nhận kẹt) | 30 vòng ≈ 3,0 s ở 10 lần/giây [đề xuất] | 10 vòng bình thường + 10 ca kẹt dựng sẵn | 0 báo nhầm, bắt ≥ 9/10 ca kẹt | số nhỏ nhất đạt cả hai |
| Khe tối thiểu 26 cm | [Thiên chốt] | ống giữa hành lang (khe 16–19 cm) và ống sát tường (khe 32–38 cm), mỗi kiểu 5 lần | xe chọn đúng (đổi hành lang / chui qua) ≥ 9/10 | giữ 26 nếu đạt |
| Khởi động lại Nav2 | tối đa 2 lần | đếm số lần Nav2 lên lại được và thời gian lên, trong mọi lần tự tắt | lên lại ≥ 9/10, mỗi lần ≤ 60 s | giữ 2 lần nếu đạt |
| Sếp treo (1 s) | [đề xuất] | ghi độ dài vòng lặp của sếp trong 10 vòng | vòng dài nhất < 0,5 s | 2 × vòng dài nhất |
| Độ tin cậy vị trí theo đoạn đường (#13) | — | ghi độ tin cậy theo toạ độ trong 10 vòng | không có đoạn nào thấp ở mọi vòng | có đoạn thấp ở mọi vòng ⇒ quét lại bản đồ trước 3/11 |

## 11. Rủi ro và điều chưa biết

- Còn 2 ngày tới 3/11 và 9 ngày tới 10/11; sếp là thay đổi lớn nên chia bước, bước nào không đạt thì **dừng ở bước trước**.
- Mọi ngưỡng ở mục 4 và 7 là [đề xuất] hoặc [ước], chưa đo trên xe (n = 0).
- **Độ tin cậy kém nhạy lệch góc:** lệch 10° vẫn "tin" [mô phỏng]; đi 1 m trôi ngang ≈ 17 cm [tính], đáng lo trong hành lang 50–56 cm. Hiện chỉ dựa vào luật nhảy ≥ 5°; đo trên xe trước khi thêm phép kiểm góc.
- **Chính sếp có thể làm Nav2 tự tắt** (thêm việc cho mạng ROS, giả thuyết G): chạy sếp trên `Pi` chỉ sau phép thử G; so số lần Nav2 tắt có và không có sếp.
- `mo_phong_he_thong.py` chưa có bảng `tt`, chưa mô phỏng huỷ đích, Nav2 chết hay SLAM nhảy: chưa thử được tính năng nào của sếp bằng mô phỏng (cần bổ sung trước bước 2).
- `lai_xe_lidar.py` mới thử trong mô phỏng lý tưởng (không trượt bánh, không duty tối thiểu, không trễ). Xe thật có thể thoát chậm hơn.
- Định vị lệch ≥ 10 cm làm phân biệt tường và vật lạ coi tường là vật lạ: chỉ chậm thêm 30 s, không nguy hiểm.
- Nav2 tự tắt chưa rõ nguyên nhân: khởi động lại (#8) chữa triệu chứng, chưa chữa gốc. Nhật ký mỗi lần khởi động lại là dữ liệu cho phép thử G.
- Bản đồ `map_xuong_v2` hơi méo [Thiên báo]: có thể làm độ tin cậy vị trí tụt ở cùng một đoạn mọi chuyến; nếu bảng đo thấy vậy thì quét lại trước 3/11 (#13), nghĩa là phải đo lại A, B và bảng tuyến.
- Không đoán người lao tới (#11): đúng với xưởng mini (ống do Thiên điều khiển); với người thật đi nhanh thì ngưỡng cố định có thể không đủ (Stretch).
- Bức tranh toàn cảnh chỉ tốt bằng thông tin cấp dưới đưa. Truy vết (mục 5) giúp phát hiện nguồn sai nhưng không thay được việc sửa chính nguồn đó.

## 12. Câu hỏi mở cho Thiên

Không còn câu hỏi mở về thiết kế. Hai câu của bản 1.1 đã trả lời (01/11 01:26): **khởi động lại Nav2 tối đa 2 lần mỗi chuyến** (#8); **chờ 30 s vật vẫn chắn thì đổi hành lang ngay** (#9). Tám câu tình huống của bản 1.2 đã trả lời (01/11 01:42): #10–#16. Năm chỗ chưa khớp mã ở bước 0 đã chốt (01/11 02:05): #17–#21. Mười câu mơ hồ của bước 1 đã chốt (01/11 03:16): #22.
Câu mở còn lại (để bước 3): cách đo khe 26 cm từ LiDAR; cách hiện chữ của sếp lên trang lái tay.

## Việc của Claude Code (không phải chat)

Rà soát mã hiện có theo mục 10 bước 0; viết, mô phỏng và đánh giá mã của sếp; đóng gói vào `bat_xe.sh`; commit. Chat chỉ thiết kế, soạn lệnh dán và hướng dẫn chạy từng bước trên `Pi`.
