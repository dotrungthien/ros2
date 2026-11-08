# Việc 118 B12: launch truyền v_banh_nang (07/11) — chưa giả lập cả hệ thống; chưa chạy trên Pi

Worktree `/home/aaa/ros2_ws/src/ros2_odom118`, nhánh `odom-118` (từ `origin/odom-118` = e2e7427). Commit `c19a6df`, chưa push.

## Sửa (chỉ `dev_ws/src/my_bot/launch/mapping_my_bot.launch.py`, +8 −1)
- Thêm `declare_v_banh_nang_cmd = DeclareLaunchArgument('v_banh_nang', default_value='0.121', ...)` sau khai báo
  `bat_mo_hinh_san`; chú thích: đo 07/11 do_toc_do 3 lần; '-1' = như cũ.
- Node `odom_tu_lenh` (khối `odom_cmd`, dòng 132–147): thêm vào dict parameters
  `'v_banh_nang': ParameterValue(LaunchConfiguration('v_banh_nang'), value_type=float)`. Dòng duy nhất bị ĐỔI (không chỉ thêm) là
  `value_type=bool)}])` → `value_type=bool),` (phải mở dict để thêm khoá).
- Thêm `ld.add_action(declare_v_banh_nang_cmd)` sau `declare_bat_mo_hinh_san_cmd`.
- Nhánh này không có dòng 129–130 đúng như lệnh ghi: khối node odom ở dòng 127–140 trước khi sửa.

## Kiểm
1. `git diff --stat`: 1 tệp (`mapping_my_bot.launch.py | 9 ++++++++-`).
2. `python3 -m py_compile` tệp launch: đạt.
3. D(1):
   | Bên ghi | Bên đọc | Tên / kiểu | Khớp? |
   |---|---|---|---|
   | launch arg `v_banh_nang` → `ParameterValue(..., float)` | `odom_tu_lenh.py:83` `declare_parameter('v_banh_nang', -1.0)` | `v_banh_nang`, float | khớp |
   | `bat_xe.sh:207` gọi launch | — | không truyền `v_banh_nang` | trên xe dùng mặc định **0.121** |
   | `gia_lap/bat_gia.sh:61` gọi launch | `bat_gia.sh:70–76` TẮT odom_tu_lenh của launch rồi bật lại chính nó với `$ODOM_THAM` | mặc định `ODOM_THAM` rỗng ⇒ `-1` (như cũ) | gia_lap KHÔNG nhận 0.121 từ launch; muốn giống xe: `ODOM_THAM="-p v_banh_nang:=0.121"` |

md5: `odom_tu_lenh.py` 93b2d4ad06e965566da00b04404bc18c, `mapping_my_bot.launch.py` 00e819184b4625d1255d9c0e7ec317c6.
Lưu ý: trên Pi, launch được đọc từ `~/ros2_ws/install/my_bot/share/my_bot/launch/` ⇒ cần `colcon build` (hoặc symlink-install)
sau khi chép tệp, nếu không thì launch cũ vẫn chạy.
