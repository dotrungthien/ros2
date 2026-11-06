# BÁO CÁO VIỆC 121: 정지 không dừng xe (CHỈ ĐỌC, 06/11/2026 ~04:10)

Giả lập chưa hiệu chuẩn, chưa thử trên xe. Không sửa tệp, không commit (tệp này chưa theo dõi).

Đã đọc: `xe_scripts/gui_duong.py` @ `579cda5` (md5 `ff496a60`, giống cây chính), log giả lập trong scratchpad `v10/simhome`.

Nhãn: [log] = log giả lập; [đọc code] = mã trong kho; [tính] = hiệu thời gian từ log; [ước] = suy từ kịch bản thử; [suy luận] = chưa chứng minh.

## 1. Traceback lần hỏng

**Nguồn và giới hạn:**
- Tệp `simhome/log_nut_chuyen.txt` đã bị ghi đè: mỗi chuyến `bat_dau_chuyen` mở tệp bằng `"w"` [đọc code `cau_noi_xe.py`].
- Bản còn lại duy nhất là đầu ra `tail -25` mà mình đọc lúc **03:57:02** (bản ghi phiên, mốc 18:57:02Z).
- Bản này bị cắt hai chỗ:
  - mất dòng đầu `Traceback (most recent call last):`;
  - mất mọi thứ phía trên, nên **không biết có chuỗi "During handling of the above exception" (KeyboardInterrupt trước đó) hay không**.

**Giờ** [log, tính]:
- Đích nhận: Nav2 controller "Received a goal" lúc 03:56:50.30; CSV `KIEM_DAT` 03:56:50.
- Bấm 정지 ≈ 03:56:56 [ước: kịch bản `di` rồi `sleep 6` rồi `dung`].
- Không có dòng "Goal was canceled".
- Xe vẫn tới đích: "Reached the goal!" lúc 03:57:22.57.

Đường dẫn `/tmp/.../simhome/gui_duong.py` là liên kết tới `xe_scripts/gui_duong.py` của worktree.

```
  File ".../simhome/gui_duong.py", line 545, in <module>
    main()
  File ".../simhome/gui_duong.py", line 533, in main
    ma = chay_co_ngat(node, a[0], int(a[1]) if len(a) > 1 else 1, node.cfg, bo_qua_ngat)
  File ".../simhome/gui_duong.py", line 171, in chay_co_ngat
    return chay(io, huong, tuyen_dau, cfg)
  File ".../simhome/gui_duong.py", line 133, in chay
    kq = theo_doi(io, duong, cfg)
  File ".../simhome/gui_duong.py", line 289, in theo_doi
    io.ngu(0.1)
  File ".../simhome/gui_duong.py", line 359, in ngu
    rclpy.spin_once(self, timeout_sec=min(con, 0.05))
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 208, in spin_once
    executor.spin_once(timeout_sec=timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 812, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 809, in _spin_once_impl
    raise handler.exception()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 309, in _execute_coroutine_step
    result = coro.send(None)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 478, in handler
    arg = take_from_wait_list(entity)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 400, in _take_subscription
    msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
RuntimeError: Unable to convert call argument to Python object (compile in debug mode for details)
```

Sau đó `ket_qua_chuyen.csv` không có dòng `NGAT` cho chuyến 03:56:50 (dòng kế tiếp là `KIEM_DAT` 03:57:26) [log]. Mã thoát 1 [log: `/v10` `chuyen.ma`].

## 2. Đọc `gui_duong.py`: tín hiệu, hủy, tắt

| Việc | Tệp:dòng | Ghi chú [đọc code] |
|---|---|---|
| import `SignalHandlerOptions` | `gui_duong.py:299` | |
| `rclpy.init` | `gui_duong.py:518` | **có** `signal_handler_options=SignalHandlerOptions.NO`: rclpy không cài bộ bắt tín hiệu (chú thích 516–517: để context còn sống mà hủy đích) |
| Hàm bắt tín hiệu `ngat` | `gui_duong.py:520–521` | `raise KeyboardInterrupt` |
| Đăng ký | `gui_duong.py:523–524` | `signal.signal(SIGINT, ngat)`, `signal.signal(SIGTERM, ngat)` |
| Chặn tín hiệu lần 2 `bo_qua_ngat` | `gui_duong.py:527–529` | SIG_IGN cho SIGINT, SIGTERM |
| Bắt ngắt `chay_co_ngat` | `gui_duong.py:167–178` | **chỉ** `except KeyboardInterrupt` (dòng 172) |
| Ghi HUY cho lớp 2 `huy_luot` | gọi `:174`, định nghĩa `:431–438` | ghi `~/gui_duong_luot` |
| Hủy đích khi ngắt `huy_khi_ngat` | gọi `:175`, định nghĩa `:501–508` | `self.goal.cancel_goal_async()` (dòng 505), quay `ngu(0.05)` tối đa 2 s |
| Hủy đích thường `huy` (DUNG / QUA_GIO / TINH_LAI) | `:494–499` | `cancel_goal_async()` dòng 496 |
| Tắt | `:533–538` | `try: chay_co_ngat … finally: node.destroy_node()` (535); `if rclpy.ok(): rclpy.shutdown()` (536–537); `sys.exit(ma)` (538) |

**Khi nhận SIGINT, trường hợp đúng** [đọc code]. Cầu nối gửi 1 SIGINT bằng `killpg`. `chuyen.sh` `exec` gui_duong nên pid = pgid, và `dung_chuyen` bỏ gửi riêng cho pid trùng nhóm [đọc code `cau_noi_xe.py`].
1. Python chạy `ngat` giữa hai lệnh bytecode, ở bất kỳ chỗ nào đang chạy → `KeyboardInterrupt`.
2. Lỗi đi lên tới `chay_co_ngat:172`.
3. `bo_qua_ngat()` → `huy_luot()` → `huy_khi_ngat()`: hủy và chờ ≤ 2 s → in `HUY_DICH_KHI_NGAT ok`.
4. Ghi CSV `NGAT`, trả 130.
5. `finally`: `destroy_node`, `shutdown`, rồi `sys.exit(130)`.

**Trường hợp hỏng** [log + đọc code]:
1. Lỗi đi lên dưới dạng `RuntimeError` (từ `take_message`), nên `except KeyboardInterrupt` không bắt.
2. Bỏ qua cả bước 3–4: không HUY cho lớp 2, không hủy đích, không ghi NGAT.
3. `finally` vẫn `destroy_node` + `shutdown`. Lỗi không bắt nên Python thoát mã 1.
4. Đích FollowPath ở Nav2 vẫn sống, xe đi tiếp.

**Vì sao thành RuntimeError** [suy luận, chưa chứng minh]:
- `take_message` là mã C++ (pybind11) dựng tin Python và gọi ngược vào Python.
- Nếu `ngat` chạy đúng lúc đó, `KeyboardInterrupt` sinh ra bên trong lời gọi pybind; pybind báo lỗi chuyển kiểu "Unable to convert call argument…" thay vì để `KeyboardInterrupt` đi lên.
- Đây cũng là lý do mình chưa biết có chuỗi KeyboardInterrupt phía trên hay không (mục 1).
- gui_duong nhận tin liên tục từ `/tf`, `/tf_static` (TransformListener dùng chung executor, `:319`), `/rosout` (`:323`) và `/global_costmap/costmap` (`:322`). Vì vậy `spin_once` thường xuyên ở trong `take_message`, khả năng trúng không nhỏ.

## 3. Lúc bấm, gui_duong đang ở đâu (7 lần SIGINT vào gui_duong thật)

Sửa lại cách đếm trong BAO_CAO_111 mục 2:
- Trong "6/7 đạt" có 5 lần qua `/v10` (có đo `/odom`).
- Lần đạt thứ 6 là lần test chạy nền vô tình gọi `dung_chuyen()` lúc 03:53:04, không đo `/odom`.
- Lần "1" trong `thu_dung_n` là `KHONG_CO_CHUYEN` (xe đã ở B), không tính.

| # | Đích nhận [log] | Hủy đích / dừng [log] | Đích → dừng [tính] | Pha | Kết quả |
|---|---|---|---|---|---|
| 1 | 03:53:03.82 | 03:53:04.72 | 0,9 s | đang có đích (`theo_doi`) | 130 NGAT, `HUY_DICH_KHI_NGAT ok` |
| 2 (**hỏng**) | 03:56:50.30 | không có hủy; bấm ≈ 03:56:56 [ước] | ≈ 6 s [ước] | đang có đích: traceback `theo_doi:289` → `ngu:359` → `spin_once` → `take_message` | mã 1, `RuntimeError`, xe tới B 03:57:22.57 |
| 3 | 03:57:37.99 | 03:57:41.99 | 4,0 s | đang có đích | 130 NGAT, `/odom` đứng yên 3 s |
| 4 | 03:57:51.46 | 03:57:57.66 | 6,2 s | đang có đích | 130 NGAT, `/odom` đứng yên |
| 5 | 03:58:05.49 | 03:58:09.19 | 3,7 s | đang có đích | 130 NGAT, `/odom` đứng yên |
| 6 | 03:58:17.51 | 03:58:20.31 | 2,8 s | đang có đích | 130 NGAT, `/odom` đứng yên |
| 7 | 03:58:30.96 | 03:58:34.76 | 3,8 s | đang có đích | 130 NGAT, `/odom` đứng yên |

**Kết luận** [log + suy luận]:
- Cả 7 lần đều ở **cùng một pha**: đã gửi FollowPath, đang lặp trong `theo_doi`.
- Không lần nào rơi vào lúc chờ đường (`cho_tf_costmap`) hay chờ RA_XONG (`cho_lop2`): giả lập không có vật, sếp `ra_lenh=false`, nên lớp 2 trả lời ngay.
- Pha không phân biệt được đạt hay hỏng. Khác biệt là **lệnh nào đang chạy khi tín hiệu tới**: trong `take_message` thì hỏng, chỗ khác thì đạt.
- Các pha chờ RA_XONG / chờ đường cũng gọi `ngu()` → `spin_once`, nên cũng có thể trúng [suy luận]. Lúc đó chưa có đích, nhưng nếu đang lùi lớp 2 thì `huy_luot` cũng bị bỏ qua.
- Chưa đo tần suất thật: 1/7 là mẫu nhỏ [log].

---

# BƯỚC S1 (sửa `gui_duong.py`, chưa commit)

Làm trong `../ros2_trang111` (nhánh `trang-111`); cây chính không đụng. Giả lập chưa hiệu chuẩn, chưa thử trên xe.

Tệp đổi (chưa commit):
- `xe_scripts/gui_duong.py`: md5 `ff496a60` → `4c464381`, +34/−12 dòng.
- `xe_scripts/test_gui_duong.py`: md5 `b936ad38` → `f8f80f46`, +88 dòng.

## S1.1 Thay đổi (đúng 3 điểm Thiên duyệt) [đọc code]

1. **Hàm `ngat` đặt cờ rồi mới raise.**
   - Đưa `ngat(signum, frame)` ra phần thuần: `DA_NGAT[0] = True`, rồi `raise KeyboardInterrupt`.
   - `main` vẫn đăng ký `ngat` cho SIGINT và SIGTERM như cũ; `rclpy.init(signal_handler_options=NO)` giữ nguyên.
2. **`chay_co_ngat` bắt `BaseException`:**
   - Bị ngắt = `isinstance(e, KeyboardInterrupt) or da_ngat()`; mặc định `da_ngat` đọc `DA_NGAT`.
   - Gọi `bo_qua_ngat()`.
   - Nếu bị ngắt **hoặc còn đích** (`io.co_dich()`, phương thức mới của nút = `goal is not None and kq is None`) → ghi HUY lớp 2 (`huy_luot`).
   - Luôn gọi `huy_khi_ngat()`: hủy đích nếu còn, chờ ≤ 2 s, như cũ.
   - Bị ngắt → in `HUY_DICH_KHI_NGAT <kq>`, ghi CSV `NGAT`, trả 130 (như cũ).
   - Lỗi khác → in `LOI_GUI_DUONG <Loại>: <lỗi>; HUY_DICH <kq>`, ghi CSV `LOI_GUI_DUONG`, trả `MA_LOI = 5`. Mã 5 không trùng 0, 1, 2, 3, 4, 130.
3. **Chặn tín hiệu lần 2:** vẫn `bo_qua_ngat()` (SIG_IGN) trước khi dọn, như cũ.

Ngoài 3 điểm trên chỉ thêm 1 dòng tài liệu mã thoát ở đầu tệp.

## S1.2 Kiểm 1: test [log]

- `test_gui_duong.py` **33/33** (30 cũ + 3 mới); `test_sep.py` **46/46**; `test_lai_xe_lidar.py` **97/97**. Thêm `test_trang_v10` 42/42 và `test_cau_noi` 8/8 (cùng worktree, sau khi tắt giả lập).
- **test_31:** gửi SIGINT **thật** cho tiến trình (đã đăng ký `G.ngat`), nuốt `KeyboardInterrupt` rồi ném `RuntimeError` giống pybind, lúc đang có đích. Kết quả: HUY ghi trước, hủy đích (`HUY_DICH_KHI_NGAT ok`), CSV `NGAT`, mã 130. Kiểm thêm: `ngat` đặt cờ trước khi raise.
- **test_32:** `RuntimeError` khi có đích, không có SIGINT → vẫn ghi HUY và hủy đích (`HUY_DICH ok`), CSV `LOI_GUI_DUONG`, mã 5, không có `NGAT`.
- **test_33:** lỗi ở bước KIEM, trước khi gửi đích → không ghi HUY, `HUY_DICH khong_co_dich`, mã 5. Kèm luật G: `chuyen.sh` `exec` gui_duong, không có `exit 5`; cầu nối `MA_CHUYEN.get(rc, "LOI")` và `DONG_CHUYEN` có `LOI`.
- **Đối chứng:** chép 3 test mới chạy với `gui_duong.py` @ 579cda5 (thư mục scratch) → **30/33**, đúng 3 test mới hỏng: test bắt được lỗi cũ.

## S1.3 Kiểm 2: giả lập, dựng lại lỗi [log]

**Cách làm:**
- `gia_lap/bat_gia.sh` của worktree (Nav2 + SLAM + xe giả + sếp + phụ lái, domain 87, HOME giả). Cầu nối `env -i`, gọi `POST /api/nut/dung` (đúng hàm `dung_chuyen`).
- Bấm sau U(1; 6) s kể từ lúc `/follow_path` ở EXECUTING.
- Nút rclpy nghe `/odom` (20 Hz [log]): "còn chạy" = có twist ≠ 0 trong khoảng 2–3,2 s sau khi bấm; "lần cuối chuyển động" = mẫu twist ≠ 0 cuối cùng sau khi bấm.

**Loại bỏ một loạt đo hỏng:**
- Loạt `moi` đầu tiên: từ lần 17 xe **kẹt** ở (0,62; 0,11; 53°), RPP báo "collision ahead" [log], nên lúc bấm xe đã đứng yên (14 lần `khong_co_dich`). Loạt này **không tính**.
- Đã sửa cách đo: chỉ tính lần mà 0,5 s trước khi bấm xe đang chạy; 3 lần liền đứng yên thì bật lại giả lập (xe về A).
- Loạt `cu` chạy trước khi sửa: 30 lần, có 2 lần "lần cuối +0,00 s" (#7, #21), có thể lúc đó xe đứng yên [suy luận].

| Loạt | Bản | Số lần tính | Xe còn chạy 2–3 s sau | Mã thoát | Lần cuối chuyển động sau khi bấm | Ghi chú |
|---|---|---|---|---|---|---|
| `cu` | 579cda5 | 30 | **0** | 130 × 30 | max 0,39 s, trung vị 0,13 s | **chưa dựng lại được lỗi** bằng dừng ngẫu nhiên |
| `moi2` | mới | 30 (bỏ 5 lần đứng yên, bật lại 1 lần) | **0** | 130 × 30 | max 0,38 s, trung vị 0,13 s; **30/30 ≤ 1 s** | `HUY_DICH_KHI_NGAT ok` × 30 |
| `tiemcu` (tiêm lỗi) | 579cda5 | 10 | **10** | 1 (HET_DUONG) × 10 | ≥ 3,15 s, hết cửa sổ đo; xe đi tiếp | dòng cuối `RuntimeError…` × 10: **dựng lại đúng lỗi 03:56** |
| `tiemmoi` (tiêm lỗi) | mới | 30 | **0** | 130 × 30 | max 0,34 s, trung vị 0,11 s; **30/30 ≤ 1 s** | `ok` × 28; `khong_tra_loi` × 1 (#2: hủy chưa được xác nhận trong 2 s, dừng mất 2211 ms, nhưng xe đứng ở +0,04 s); `khong_co_dich` × 1 (#25: đích vừa kết thúc ngay trước khi bấm) |

**Tiêm lỗi** (tệp scratch `tiem_cu/gui_duong.py`, `tiem_moi/gui_duong.py`; không sửa kho):
- Một tệp khởi chạy cùng tên chạy bản thật qua `runpy`, sau khi vá hai chỗ:
  - `signal.signal`: bọc handler SIGINT của gui_duong. Handler gốc vẫn chạy (bản mới đặt `DA_NGAT`); `KeyboardInterrupt` bị nuốt và đánh dấu.
  - `Executor._take_subscription`: lần lấy tin kế tiếp ném `RuntimeError('Unable to convert call argument…')`.
- Đây là mô phỏng cơ chế đã thấy trong traceback, **không phải** chứng minh pybind thật làm vậy [suy luận].
- Bằng chứng lỗi tiêm có nổ:
  - `tiemcu`: 10/10 lần ra `RuntimeError` [log].
  - `tiemmoi`: chỉ còn log chuyến cuối (`[TIEM LOI] RuntimeError trong _take_subscription` rồi `HUY_DICH_KHI_NGAT ok`) [log]. 29 lần kia suy từ việc lỗi tiêm là tất định [suy luận].

## S1.4 Kiểm 3: D(1)

| Bên ghi | Bên đọc | Phải khớp | Khớp? |
|---|---|---|---|
| `gui_duong.py` mã thoát 130 (ngắt) | `chuyen.sh` (`exec`, mã đi thẳng), `cau_noi_xe.MA_CHUYEN` | 130 → NGAT | khớp, không đổi [đọc code, log 60/60] |
| `gui_duong.py` mã thoát **5** (lỗi khác) | `chuyen.sh` | mã đi thẳng qua `exec` | khớp. **Chú thích đầu `chuyen.sh`** (dòng 38–39, "Ma thoat: 0…4; 130") **chưa nhắc mã 5** [đọc code]; `chuyen.sh` là tệp lái, không sửa |
| mã 5 | `cau_noi_xe.MA_CHUYEN.get(rc, "LOI")` | → `LOI` | khớp [tính: chạy hàm]. `/nut` hiện `LOI` thô (bảng `han()` không có `LOI`) |
| mã 5 | `/v10` `suy_trang_thai` | → 도움이 필요해요 `LOI_THOAT` | khớp [tính: chạy hàm] |
| stdout `HUY_DICH_KHI_NGAT <kq>` | `cau_noi_xe.DONG_CHUYEN` | khớp `^HUY_DICH_KHI_NGAT` | khớp, không đổi |
| stdout `LOI_GUI_DUONG …` (mới) | `cau_noi_xe.DONG_CHUYEN` `^(…\|LOI\|…)` | khớp tiền tố `LOI` | khớp [tính: `DONG_CHUYEN.match` = True] |
| CSV `NGAT` | `sep.py _csv` (đóng chuyến khi DEN_NOI, HET_DUONG, SAI_VI_TRI, DUNG_SEP, NGAT; `sep.py:901`) | đóng chuyến | khớp, không đổi |
| CSV **`LOI_GUI_DUONG`** (mới) | `sep.py _csv` | đóng chuyến | **KHÔNG khớp** [đọc code `sep.py:895–903`]: sếp không đóng chuyến. `KIEM_DAT` sau đó chỉ mở chuyến khi `self.chuyen is None` → chuyến sau **không đếm lại** `luot_thoat`, `nav2_lan`, `het_luot` → có thể DUNG_VA_BAO sớm [suy luận]. Sửa cần thêm `'LOI_GUI_DUONG'` vào bộ chữ đóng chuyến của `sep.py` (tệp lái; Thiên dặn không đổi gì khác → **cần quyết**) |
| `~/gui_duong_luot` dòng `<t> HUY` (`dong_huy`, không đổi khuôn) | `lai_xe_lidar.luot_huy` (`o[1] == 'HUY'`), `sep.doc_sap_gui` (bỏ dòng không phải SAP_GUI) | khuôn `'%.2f HUY'` | khớp. Mới: còn ghi khi lỗi khác **mà còn đích**. Lỗi khác lúc **chưa có đích** (vd. đang chờ RA_XONG) thì **không** ghi HUY (đúng điểm 2 Thiên duyệt), nên phụ lái lùi lớp 2 tự hết theo giới hạn của nó [đọc code] |
| `cau_noi_xe.dung_chuyen` | đọc "con sống" sau SIGINT | báo `DA_DUNG` khi tiến trình chết | không đổi. Với bản mới, `DA_DUNG` đi kèm hủy đích thật (60/60 lần [log]). Riêng lần #2 `tiemmoi` mất 2211 ms do `huy_khi_ngat` chờ đủ 2 s; vẫn dưới `dung_cho_s` = 3 s |

## S1.5 Còn mở

1. CSV `LOI_GUI_DUONG` chưa được `sep.py` đóng chuyến (bảng trên).
2. Chú thích mã thoát trong `chuyen.sh` chưa có mã 5.
3. `/nut` hiện `LOI` thô, không dịch.
4. Lỗi tự nhiên chưa dựng lại được trong 30 lần ngẫu nhiên. Lần hỏng duy nhất là 03:56 (1 trong 7 lần ở việc 111) [log]. Tần suất thật chưa biết.
5. Đồng hồ máy nhảy từ ~04:xx sang ~11:xx giữa các lần đo (log Nav2/sếp dùng giờ mới); không ảnh hưởng số đo (dùng `monotonic`).

---

# BƯỚC S2 (sếp đóng chuyến khi `LOI_GUI_DUONG`, chú thích mã 5, `/nut` dịch `LOI`; commit 1 lần)

Giả lập chưa hiệu chuẩn, chưa thử trên xe. Làm trong `../ros2_trang111` (nhánh `trang-111`); cây chính không đụng.

## S2.1 Thay đổi [đọc code]

| Tệp | Đổi | md5 trước → sau |
|---|---|---|
| `sep.py` | `_csv` (dòng 901): bộ chữ đóng chuyến thêm `'LOI_GUI_DUONG'`. Nhật ký sếp ghi `CHUYEN,KET_THUC,,<huong>,LOI_GUI_DUONG` (đúng chữ đó, vì `kq` ghi nguyên văn). Không đổi gì khác | 4fa9ffd7 → cc663629 |
| `chuyen.sh` | chỉ thêm 1 dòng chú thích mã 5 dưới dòng mã 130 | d2314189 → a05eee9a |
| `cau_noi_xe.py` | bảng `HAN` của `/nut` thêm `['LOI','오류']` (chỉ chữ hiển thị). `\b` nên `LOI_GUI_DUONG`, `LOI_THOAT` **không** bị dịch; `LOI` và `LOI: …` thì có | e6f8f8d8 → 62eeb25d |
| `test_sep.py` | thêm `test_47_LOI_GUI_DUONG_dong_chuyen_chuyen_sau_dem_lai_tu_0` | 697f9656 → 600f5706 |
| `test_cau_noi.py` | **sửa mã md5 ghim trang `/nut`** (`MD5_TRANG_NUT`, ghim từ việc 111 để `/nut` giữ nguyên từng byte). Đổi `HAN` thì md5 trang đổi theo, nên `test_01` hỏng. Đã so trang cũ/mới: khác **đúng 1 dòng**, chỉ thêm `['LOI','오류']` [tính: difflib] | d78d3627 → 7e52a409 |
| `gui_duong.py`, `test_gui_duong.py` | như S1, không đổi thêm | 4c464381, f8f80f46 |

## S2.2 Test [log]

| Bộ | Kết quả |
|---|---|
| `test_gui_duong.py` | 33/33 |
| `test_sep.py` | **47/47** (46 cũ + 1 mới) |
| `test_lai_xe_lidar.py` (phụ lái) | 97/97 |
| `test_trang_v10.py` | 42/42 |
| `test_cau_noi.py` | 8/8 (sau khi sửa md5 ghim; trước đó 7/8, hỏng đúng `test_01_trang_nut_giu_nguyen`) |
| Đối chứng: `test_sep.py` mới chạy trên `sep.py` @HEAD (`git show`, bản sao trong scratch) | 46/47, hỏng đúng `test_47` → test bắt được lỗi cũ |

`test_47`: chuyến `di_B_day` mở, Nav2 mất nhịp 1 lần, đặt `luot_thoat = 3`, `het_luot = True`; dòng CSV `LOI_GUI_DUONG` ⇒ `chuyen is None` và có dòng nhật ký chứa cả `KET_THUC` lẫn `LOI_GUI_DUONG`; `KIEM_DAT` chuyến `ve_A_day` ⇒ `BAT_DAU`, `nav2_lan = 0`, `luot_thoat = 0`, `het_luot = False`.

## S2.3 Giả lập: ca tiêm lỗi có sếp, 5 vòng [log]

Cách làm (scratch, không sửa kho): bộ chạy `tiem_loi/gui_duong.py` bọc `gui_duong.py` thật; khi có tệp cờ, ném `RuntimeError` (đúng chữ của traceback 03:56) **một lần** trong `_take_subscription` sau 1–6 s ngẫu nhiên kể từ `send_goal_async` đầu, **không có SIGINT**. Mỗi vòng: chuyến tiêm lỗi rồi một chuyến thường, qua `/api/nut` của cầu nối worktree (cổng 5093). Sếp chạy từ `sep.py` worktree (`bat_gia.sh`, `ra_lenh=false ra_thoat=false`).

| Vòng | Chuyến tiêm (giây sau đích) | Mã / kết quả | Chuyển động cuối so với lúc cầu nối thấy thoát | 2–3 s sau | Sếp: chuyến tiêm | Chuyến sau | Sếp: chuyến sau |
|---|---|---|---|---|---|---|---|
| 1 | di_B 1,37 | 5 / LOI | −0,25 s | đứng | `KET_THUC LOI_GUI_DUONG` | di_B, 0 DEN_NOI | `BAT_DAU` (đếm lại Nav2 0/2, thoát 0/3), 0 DUNG_VA_BAO |
| 2 | ve_A 3,39 | 5 / LOI | −0,08 s | đứng | như trên | ve_A, 0 DEN_NOI | như trên |
| 3 | di_B 1,31 | 5 / LOI | +0,07 s | đứng | như trên | di_B, 0 DEN_NOI | như trên |
| 4 | ve_A 1,14 | 5 / LOI | −0,02 s | đứng | như trên | ve_A, 0 DEN_NOI | như trên |
| 5 | di_B 5,07 | 5 / LOI | −0,19 s | đứng | như trên | di_B, 0 DEN_NOI | như trên |

Tổng: **5/5 vòng đạt** [log]. Ở cả 5 chuyến tiêm, xe đang chạy trong 10 s trước khi thoát [log]. Dòng stdout cầu nối thấy: `LOI_GUI_DUONG RuntimeError: Unable to convert call argument…` [log].

Giới hạn:
- Kết quả `HUY_DICH <kq>` (`ok`/`khong_tra_loi`) **không lưu được**: bộ đo cắt dòng ở 90 ký tự, log chuyến bị ghi đè mỗi chuyến, `~/gui_duong_luot` chỉ giữ dòng cuối. "Xe đứng" ở bảng trên là đo `/odom` [log], không phải xác nhận hủy từ Nav2.
- Trong giả lập, bộ đếm của chuyến hỏng đều bằng 0 nên "DUNG_VA_BAO sớm" **không thể xảy ra** dù có hay không có sửa. Giả lập chỉ chứng minh `KET_THUC LOI_GUI_DUONG` + `BAT_DAU` đếm lại; việc chặn DUNG_VA_BAO sớm do bộ đếm còn sót được chứng minh bằng `test_47` (đối chứng hỏng trên bản cũ).
- Không chạy đối chứng giả lập với `sep.py` cũ.

## S2.4 D(1) cho 4 tệp đổi

| Bên ghi | Bên đọc | Phải khớp | Khớp? |
|---|---|---|---|
| `gui_duong.py` mã 5 + stdout `LOI_GUI_DUONG …` + CSV `LOI_GUI_DUONG` | `chuyen.sh` `exec` → `cau_noi_xe` `MA_CHUYEN.get(5,"LOI")`, `DONG_CHUYEN` | `LOI`, dòng khớp tiền tố `LOI` | khớp [log: 5/5 ra `ma 5 LOI`] |
| CSV `LOI_GUI_DUONG` | `sep.py _csv` | đóng chuyến, ghi đúng chữ | **khớp** (S1 không khớp) [log 5/5, test_47] |
| Nhật ký sếp `CHUYEN,BAT_DAU` | `/v10` `trang_v10.py:239` (dấu kẹt/chắn tính từ `BAT_DAU` cuối) | chuyến sau có `BAT_DAU` mới | khớp; trước đây sau chuyến lỗi không có `BAT_DAU` mới nên `/v10` giữ dấu của chuyến hỏng [đọc code, suy luận] |
| Nhật ký sếp `KET_THUC` | `trang_v10.py` khác, cầu nối | — | không ai khác đọc chữ `KET_THUC` theo danh sách cố định [đọc code: grep] |
| `chuyen.sh` chú thích | người đọc | có mã 5 | khớp; chỉ chú thích, hành vi không đổi |
| `cau_noi_xe.py` bảng `HAN` | `/nut` `han()`; `/v10` `han_js()` lấy nguyên văn `HAN` + `han()` từ `TRANG_NUT` | `LOI` → `LOI (오류)`; `LOI_GUI_DUONG`, `LOI_THOAT` giữ nguyên | khớp cả hai trang [tính: mô phỏng regex `han()` bằng Python, máy không có `node`]. `/v10` dòng "지난 주행: …" (`trang_v10.html:92`) giờ hiện `LOI (오류)` |
| `cau_noi_xe.py` | `test_cau_noi.py` md5 ghim trang `/nut` | md5 mới | đã sửa ghim (S2.1) |

## S2.5 Còn mở

1. `/v10` khi mã 5 vẫn hiện "Nav2 목표가 아직 살아 있을 수 있어요: 모터 스위치를 끄세요" (`trang_v10.html:101`, chung cho `LOI_THOAT`). Bản mới đã hủy đích trước khi thoát, nhưng nếu hủy `khong_tra_loi` thì lời nhắc vẫn đúng; không đổi (ngoài phạm vi).
2. Mục 4 S1.5 vẫn mở: lỗi tự nhiên chưa dựng lại; tần suất thật chưa biết.
3. Chưa thử trên xe.
