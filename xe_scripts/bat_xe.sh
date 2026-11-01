#!/usr/bin/env bash
# bat_xe.sh: bat ca he thong bang mot lenh (buoc 5, 6, 7, 8 cua Quy trinh chuan 71, ban xuong mini).
# Chay tren Pi (dau nhac project1@). Xe phai o vach A, DUNG YEN, cong tac motor OFF.
# Script KHONG bat motor, KHONG gui muc tieu. Chi bat 2 launch chay nen, roi kiem tham so va vi tri.
#
# Cach dung (tren Pi):
#   bash ~/bat_xe.sh                       bat he thong (gio Pi giu nguyen)
#   bash ~/bat_xe.sh "2026-10-29 15:40:00" chinh gio Pi truoc, roi bat
#   bash ~/bat_xe.sh tat                   tat Nav2 va launch chinh, don tien trinh sot
#
# Tu dien thoai / PC (1 lenh, tu lay gio may gui; doi 192.168.0.2 neu Pi o mang khac):
#   ssh -t project1@192.168.0.2 "bash ~/bat_xe.sh '$(date +'%Y-%m-%d %H:%M:%S')'"
#
# Hai launch chay nen (setsid + nohup): mat ssh thi he thong VAN CHAY.
# Log: ~/log_bat_xe_launch.txt va ~/log_bat_xe_nav.txt (lan chay truoc giu o duoi .cu)
# Ma thoat: 0 = san sang, vi tri dat; 1 = loi hoac vi tri khong dat;
#           2 = da bat xong nhung chua tu kiem duoc (thieu ~/kiem_xe.sh), phai doc tay.
#
# Ban 30/10/2026 (sua tai truong): danh sach NODES them planner_server, controller_server,
# behavior_server, velocity_smoother, lifecycle_manager (Ctrl+C hay de sot cac tien trinh Nav2).
# Ban 30/10 toi: them bien NAV_PARAMS (xem B2). CHUA CHAY TREN PI THAT.
# Ban v3 (30/10 dem, sua o nha, CHUA CHAY TREN PI): (1) "tat" co them buoc KILL -9 neu con tien trinh sot;
#   (2) co NAV_PARAMS thi kiem_xe.sh doi chieu cay hanh vi (MONG_BT); (3) ghi so tep /dev/shm truoc khi bat.

ACCESS="${ACCESS:-XOA_KHOI_BAN_NAY}"   # gia tri mac dinh that da duoc xoa khoi ban gui cho Claude Code
SLAM="${SLAM:-/home/project1/slam_dinh_vi_xuong.yaml}"
LOG1="$HOME/log_bat_xe_launch.txt"
LOG2="$HOME/log_bat_xe_nav.txt"
T1="${T1:-60}"    # giay cho dong "bias" cua launch chinh
T2="${T2:-120}"   # giay cho dong "Managed nodes are active" cua Nav2
HAN="${HAN:-480}" # 8 phut ke tu luc con quay lay so 0
# Danh sach ten tien trinh (khop lenh don trong chat 30/10; Quy trinh chuan 71 buoc 14, 15 can cap nhat theo).
NODES='[g]yro_mpu6050|[o]dom_tu_lenh|[s]lam_toolbox|[d]ieu_khien|[y]dlidar|[t]eleop_web|[b]t_navigator|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[v]elocity_smoother|[l]ifecycle_manager|[r]obot_state_publisher|[j]oint_state_publisher|[t]wist_mux|[s]tatic_transform_publisher|[c]amera_csi|[r]f2o_laser_odometry_node|[k]et_detector'
# 31/10 chieu: them 7 ten cua launch ma truoc day thieu (rsp/jsp/twist_mux/static_tf sot lai sau "tat" -> bo mo coi, xem BAO_CAO muc 4). Duong lui: cp bat_xe.sh.v3 bat_xe.sh

# 0 neu con node hoac launch cua he thong dang chay
con_chay() {
  pgrep -f "$NODES" > /dev/null || pgrep -f "[r]os2 launch my_bot" > /dev/null
}

# in cac tien trinh con chay (de nhin)
liet_ke() {
  pgrep -af "$NODES"
  pgrep -af "[r]os2 launch my_bot"
}

# viec 101: tien trinh ROS ngoai NODES (chay tay, lenh ros2, daemon); phai tat het truoc khi don /dev/shm
# chuong trinh chay tay: chi khop tien trinh python3 co dung ten tep (khong khop vo "bash -c" co chu do, bai hoc 303)
TAY='^([^ ]*/)?python3[^ ]*( [^ ]+)*[ /]([s]ep|[l]ai_xe_lidar|[t]heo_doi)\.py( |$)'
ROS_KHAC="[_]ros2_daemon|[r]os2 (bag|topic|action|param|run|service|node|launch)|$TAY"

# nhu con_chay, them cac chuong trinh chay tay (chi dung trong "tat"; phan bat van dung con_chay)
con_chay_tat() {
  con_chay || pgrep -f "$TAY" > /dev/null
}

# in pid, so giay da song, ten cua moi tien trinh ROS con song
ps_ros() {
  local P
  P=$(pgrep -f "$NODES|$ROS_KHAC" | paste -sd, -)
  [ -n "$P" ] && ps -o pid,etimes,comm -p "$P"
}

# viec 101: moc thoi gian tung buoc cua "tat" ($1 = so buoc), kem tien trinh ROS con song
moc() {
  echo "T+$((SECONDS - T_TAT)) buoc $1"
  ps_ros
}

# ---------- tat ----------
if [ "$1" = "tat" ]; then
  T_TAT=$SECONDS
  echo "== Tat ghi vi tri, sep, lai_xe_lidar, Nav2, launch chinh =="
  moc 1
  pkill -INT -f "$TAY"
  pkill -INT -f "[r]os2 launch my_bot navigation_my_bot"
  sleep 3
  moc 2
  pkill -INT -f "[r]os2 launch my_bot mapping_my_bot"
  i=0
  while [ "$i" -lt 20 ] && con_chay_tat; do
    sleep 1
    i=$((i + 1))
  done
  if con_chay_tat; then
    moc 3
    echo "== Sau 20 giay con tien trinh sot, gui lenh tat manh =="
    pkill -f "[r]os2 launch my_bot"
    pkill -f "$NODES"
    pkill -f "$TAY"
    sleep 3
  fi
  if con_chay_tat; then
    # v3: slam_toolbox hay sot sau Ctrl+C (kill -9 moi tat duoc, giai doan 60.4)
    moc 4
    echo "== Van con tien trinh sot, gui tin hieu KILL (-9) =="
    pkill -KILL -f "$NODES"
    pkill -KILL -f "[r]os2 launch my_bot"
    pkill -KILL -f "$TAY"
    sleep 2
  fi
  moc 5
  if con_chay_tat; then
    echo "CON_CHAY: van con cac tien trinh sau:"
    liet_ke
    pgrep -af "$TAY"
    echo "Chay lai: bash ~/bat_xe.sh tat   (van con thi tat Pi roi bat lai)"
    exit 1
  fi
  # ---------- don /dev/shm (viec 101): chi khi khong con tien trinh ROS nao ----------
  [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
  timeout 10 ros2 daemon stop > /dev/null 2>&1     # daemon giu tep shm; dung no truoc
  sleep 1
  moc 6
  SONG=$(pgrep -af "$NODES|$ROS_KHAC")
  DUNG=""
  if command -v fuser > /dev/null; then
    DUNG=$(fuser /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null)   # PID con mo hoac mmap tep shm
  else
    echo "KHONG_CO_FUSER: bo lop kiem tep dang mo, chi dua vao danh sach ten"
  fi
  N=$(ls /dev/shm | grep -c 'fastrtps_')
  if [ -n "$SONG" ] || [ -n "$DUNG" ]; then
    echo "CON_ROS_SONG: khong don /dev/shm ($N tep). Con:"
    [ -n "$SONG" ] && echo "$SONG"
    [ -n "$DUNG" ] && ps -o pid=,comm= -p "$(echo $DUNG | tr ' ' ',')"
    exit 1
  fi
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
  echo "DA_DON $N tep /dev/shm"
  echo "KIEM_XONG: da tat sach."
  exit 0
fi

# ---------- kiem doi so ----------
if [ -n "$1" ] && ! printf '%s' "$1" | grep -Eq '^[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}$'; then
  echo "SAI DOI SO: '$1'"
  echo "Dung: bash ~/bat_xe.sh   |   bash ~/bat_xe.sh \"YYYY-MM-DD HH:MM:SS\"   |   bash ~/bat_xe.sh tat"
  exit 1
fi

# ---------- khong bat hai ban ----------
if con_chay; then
  echo "KHONG BAT: van con he thong hoac tien trinh sot dang chay:"
  liet_ke
  echo "Chay: bash ~/bat_xe.sh tat   roi chay lai."
  exit 1
fi

# ---------- chinh gio ----------
if [ -n "$1" ]; then
  echo "== Chinh gio Pi: $1 =="
  sudo timedatectl set-ntp false || { echo "LOI: khong tat duoc dong bo gio."; exit 1; }
  sleep 5
  sudo timedatectl set-time "$1" || { echo "LOI: khong chinh duoc gio."; exit 1; }
  sudo systemctl stop apt-daily.timer apt-daily-upgrade.timer
  date
  pgrep -af "[u]nattended-upgrade"
fi

if [ ! -f ~/ros2_ws/install/setup.bash ]; then
  echo "LOI: khong thay ~/ros2_ws/install/setup.bash"
  exit 1
fi
source ~/ros2_ws/install/setup.bash
# ghi log ra tep thi Python mac dinh gom dong lai; tat de dong "bias" hien ngay
export PYTHONUNBUFFERED=1

# doi($1 = tep log, $2 = chuoi can thay, $3 = so giay toi da, $4 = mau tien trinh phai con song)
# tra ve 0 = thay chuoi, 1 = het gio, 2 = tien trinh da thoat som
doi() {
  local f="$1" s="$2" t="$3" p="$4" i=0
  while [ "$i" -lt "$t" ]; do
    grep -aqF "$s" "$f" 2>/dev/null && return 0
    if [ "$i" -ge 3 ] && ! pgrep -f "$p" > /dev/null; then
      return 2
    fi
    sleep 1
    i=$((i + 1))
  done
  return 1
}

# v3: ghi so tep trong /dev/shm (nghi van DDS sot sau sap nguon, chi do, khong doi gi)
echo "[shm] /dev/shm co $(ls /dev/shm 2>/dev/null | wc -l) tep truoc khi bat ($(date +%T))"

# ---------- B1 ----------
echo "== B1: bat he thong (XE DUNG YEN, KHONG CHAM VAO XE) =="
[ -f "$LOG1" ] && mv -f "$LOG1" "$LOG1.cu"
setsid nohup ros2 launch my_bot mapping_my_bot.launch.py access_code:="$ACCESS" bat_dong_co:=true bat_lidar:=true bat_rf2o:=false bat_camera:=false bat_gyro:=true bat_mo_hinh_san:=true slam_params_file:="$SLAM" > "$LOG1" 2>&1 < /dev/null &

doi "$LOG1" "보정 완료" "$T1" "[r]os2 launch my_bot mapping_my_bot"
R=$?
if [ "$R" -eq 0 ]; then
  T0=$(date +%s)
  echo "Con quay da lay so 0 luc $(date +%T). Han chot gui muc tieu: $(date -d "@$((T0 + HAN))" +%T) (8 phut)."
  grep -a "보정 완료\|raw std" "$LOG1" | tail -3
else
  if [ "$R" -eq 2 ]; then
    echo "LOI: launch chinh da thoat som."
  else
    echo "LOI: sau $T1 giay chua thay dong 'bias' cua con quay."
  fi
  echo "10 dong cuoi cua log:"
  tail -10 "$LOG1"
  echo "Thu: bash ~/bat_xe.sh tat   roi chay lai."
  exit 1
fi

# ---------- B2 ----------
echo "== B2: bat Nav2 =="
[ -f "$LOG2" ] && mv -f "$LOG2" "$LOG2.cu"
# NAV_PARAMS (tuy chon): duong dan tep tham so Nav2, vi du /home/project1/nav2_params_cho.yaml (cay hanh vi cho, khong quay/lui)
if [ -n "$NAV_PARAMS" ]; then
  echo "Nav2 dung tham so: $NAV_PARAMS"
  setsid nohup ros2 launch my_bot navigation_my_bot.launch.py params_file:="$NAV_PARAMS" > "$LOG2" 2>&1 < /dev/null &
else
  setsid nohup ros2 launch my_bot navigation_my_bot.launch.py > "$LOG2" 2>&1 < /dev/null &
fi

doi "$LOG2" "Managed nodes are active" "$T2" "[r]os2 launch my_bot navigation_my_bot"
R=$?
if [ "$R" -eq 0 ]; then
  echo "Nav2 san sang luc $(date +%T)."
else
  if [ "$R" -eq 2 ]; then
    echo "LOI: Nav2 da thoat som."
  else
    echo "LOI: sau $T2 giay chua thay 'Managed nodes are active'."
  fi
  echo "10 dong cuoi cua log:"
  tail -10 "$LOG2"
  echo "Thu: bash ~/bat_xe.sh tat   roi chay lai."
  exit 1
fi

# ---------- B3 ----------
echo "== B3: kiem tham so va vi tri (A = x 0, y 0, goc 0) =="
# giong buoc 6 cua Quy trinh chuan 71: tranh loi "Node not found"
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 3

# v3: neu bat Nav2 bang tep tham so rieng (NAV_PARAMS), kiem_xe.sh se doi chieu cay hanh vi dang chay voi tep do.
# (tham so launch khong khai bao thi bi bo qua im lang -> Nav2 chay cay cu co quay va lui ma khong bao loi)
if [ -n "$NAV_PARAMS" ]; then
  MONG_BT=$(sed -n 's/^ *default_nav_to_pose_bt_xml: *//p' "$NAV_PARAMS" | head -1 | tr -d ' \r')
  export MONG_BT
  echo "Mong doi cay hanh vi: ${MONG_BT:-'(tep tham so khong khai default_nav_to_pose_bt_xml)'}"
fi

if [ -f ~/kiem_xe.sh ]; then
  i=1
  while true; do
    bash ~/kiem_xe.sh 0 0 0
    KQ=$?
    [ "$KQ" -eq 0 ] && break
    [ "$i" -ge 3 ] && break
    i=$((i + 1))
    echo "-- Kiem lai lan $i/3 sau 5 giay (slam_toolbox co the chua dinh vi xong) --"
    sleep 5
  done
else
  echo "Khong co ~/kiem_xe.sh (viec 82 chua lam). Doc tay 3 tham so va vi tri:"
  for tp in "/odom_tu_lenh yaw_nguon" "/gyro_mpu6050 gyro_scale" "/gyro_mpu6050 dau_yaw"; do
    echo "$tp:"
    # $tp co chu y de tach thanh hai doi so (ten node, ten tham so)
    # shellcheck disable=SC2086
    ros2 param get $tp 2>&1 | tail -1
  done
  echo "Mong: gyro / 131.0 / -1.0"
  timeout 8 ros2 run tf2_ros tf2_echo map base_link
  echo "Mong: x 0 +-0.10 m, y 0 +-0.05 m, goc (so thu ba cua RPY degree) 0 +-5 do"
  KQ=2
fi

# ---------- ket ----------
echo "=========="
CON=$((T0 + HAN - $(date +%s)))
HAN_GIO=$(date -d "@$((T0 + HAN))" +%T)
if [ "$KQ" -eq 1 ]; then
  echo "CHUA SAN SANG -> bash ~/bat_xe.sh tat, dat lai xe o vach A, chay lai."
elif [ "$CON" -le 0 ]; then
  echo "QUA 8 PHUT ke tu luc lay so 0 (con quay da troi): bash ~/bat_xe.sh tat, roi bat lai."
  KQ=1
elif [ "$KQ" -eq 0 ]; then
  echo "SAN SANG. Con $CON giay de gui muc tieu (han chot $HAN_GIO)."
  echo "Viec tiep theo (lam bang tay): tab project1@ moi, bat ghi vi tri (buoc 9), go san lenh muc tieu (buoc 10), bat cong tac motor (buoc 11)."
else
  echo "DA BAT, CHUA TU KIEM (thieu ~/kiem_xe.sh). Doc tay o tren; dat thi lam tiep nhu SAN SANG. Han chot $HAN_GIO."
fi
exit $KQ
