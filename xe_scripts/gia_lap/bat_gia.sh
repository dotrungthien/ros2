#!/usr/bin/env bash
# Bat gia lap vong kin tren PC (WSL): launch mapping_my_bot + Nav2 that, chi xe va LiDAR la gia (xe_gia.py).
# Tep that khong sua: chep slam_dinh_vi_xuong.yaml va nav2_params_cho.yaml vao $TAM, doi /home/project1.
# Tat: bash xe_scripts/bat_xe.sh tat (xe_gia.py tu thoat khi launch mapping chet; odom_tu_lenh rieng khop mau NODES;
#   sep.py, lai_xe_lidar.py, gui_duong.py khop mau TAY).
# 106: sau Nav2 active bat sep.py + lai_xe_lidar.py nhu bat_xe.sh B2b (cung bien SEP_RA_LENH, SEP_RA_THOAT, PHU_LAI_PHAT_LENH);
#   lien ket ~/gui_duong.py, ~/tinh_tuyen.py cho chuyen.sh BANG=day; ROTATE_MIN ghi de rotate_to_heading_min_angle.
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87
GIA=$(cd "$(dirname "$0")" && pwd)
XS=$(dirname "$GIA")
TAM="${TAM:-/tmp/gia_lap}"
T_CHO="${T_CHO:-90}"
# ---- tham so hieu chuan (01/11 tren xe) ----
INFLATION="${INFLATION:-0.20}"     # inflation_radius cua global va local costmap (ban chep), nhu xe 01/11
VI_TRI_DAU="${VI_TRI_DAU:-0 0 0}"  # "x y goc_do": vi tri that dau cua xe_gia, cung la map_start_pose cua SLAM
# (banh_trai_that = 1/he_so_trai: tham so o dau xe_gia.py)
ROTATE_MIN="${ROTATE_MIN:-}"       # rong = giu rotate_to_heading_min_angle cua yaml; vi du 0.785 (xe 02/11 16:43)
# 06/11 viec 118 B7b: tat cua xe_gia (rong = nhu cu), vd XE_GIA_THAM="-p v_nang_that:=0.12"; SLAM_MIN_TRAVEL ghi de
# minimum_travel_distance CHI trong ban chep slam.yaml cua gia lap (rong = giu 0.25 nhu xe)
XE_GIA_THAM="${XE_GIA_THAM:-}"
SLAM_MIN_TRAVEL="${SLAM_MIN_TRAVEL:-}"
# 07/11 viec 118 B12: odom_tu_lenh nao chay va tham so them (rong = nhu cu), vd ODOM_THAM="-p v_banh_nang:=0.12"
ODOM_PY="${ODOM_PY:-$HOME/ros2_ws/install/my_bot/lib/my_bot/odom_tu_lenh.py}"
ODOM_THAM="${ODOM_THAM:-}"
# ---- sep va phu lai, nhu bat_xe.sh (mac dinh false) ----
SEP_RA_LENH="${SEP_RA_LENH:-false}"; SEP_RA_THOAT="${SEP_RA_THOAT:-false}"; PHU_LAI_PHAT_LENH="${PHU_LAI_PHAT_LENH:-false}"
for b in "SEP_RA_LENH=$SEP_RA_LENH" "SEP_RA_THOAT=$SEP_RA_THOAT" "PHU_LAI_PHAT_LENH=$PHU_LAI_PHAT_LENH"; do
  case "${b#*=}" in true|false) ;; *) echo "LOI: $b (chi true hoac false)"; exit 1 ;; esac
done
mkdir -p "$TAM"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

sed "s#/home/project1/#$XS/du_lieu_0111/#g" "$XS/du_lieu_0111/slam_dinh_vi_xuong.yaml" > "$TAM/slam.yaml"
sed "s#/home/project1/#$XS/#g" "$XS/nav2_params_cho.yaml" > "$TAM/nav2.yaml"
# 08/11 bo 1e: MAT_LAI=true => nav2_params_mat.yaml (bt_mat, collision_monitor, map_server /map_tinh, lop_huong_dan) va tep
# launch CUA KHO (mat_lai:=true; ban da cai co the cu). ~/goc_dang_chay ghi de nhu bat_xe.sh. Mac dinh (false) y nhu cu.
MAT_LAI="${MAT_LAI:-false}"
NAV_MAT="${NAV_MAT:-nav2_params_mat.yaml}"   # 08/11 buoc 2e: tep tham so kieu mat (vd nav2_params_mat_rpp.yaml)
if [ "$MAT_LAI" = "true" ] && [ "$PHU_LAI_PHAT_LENH" = "true" ]; then echo "CANH BAO: MAT_LAI=true => ep PHU_LAI_PHAT_LENH=false (/cmd_vel_avoid chi cho behavior_server)"; PHU_LAI_PHAT_LENH=false; fi   # 09/11 buoc 3 #8 (nhu bat_xe.sh)
if [ "$MAT_LAI" = "true" ]; then
  sed -e "s#/home/project1/map_xuong_v3.yaml#$XS/du_lieu_0111/map_xuong_v3.yaml#g" -e "s#/home/project1/#$XS/#g" \
    "$XS/$NAV_MAT" > "$TAM/nav2.yaml"
  echo "NAV_MAT=$NAV_MAT"
  export LOP_HUONG_DAN_PY="$XS/lop_huong_dan.py" LOP_HUONG_DAN_YAML="$XS/du_lieu_0111/map_xuong_v3.yaml"
  echo mat > ~/goc_dang_chay
else
  echo cu > ~/goc_dang_chay
fi
echo "MAT_LAI=$MAT_LAI ~/goc_dang_chay=$(cat ~/goc_dang_chay)"
sed -i -E "s/^( *inflation_radius:) .*/\1 $INFLATION/" "$TAM/nav2.yaml"
[ -n "$ROTATE_MIN" ] && sed -i -E "s/^( *rotate_to_heading_min_angle:) .*/\1 $ROTATE_MIN/" "$TAM/nav2.yaml"
echo "rotate_to_heading_min_angle dung: $(sed -n 's/^ *rotate_to_heading_min_angle: *\([^ ]*\).*/\1/p' "$TAM/nav2.yaml") (ROTATE_MIN=${ROTATE_MIN:-khong dat, giu yaml})"
read -r X0 Y0 G0 <<< "$(echo "$VI_TRI_DAU" | awk '{printf "%.4f %.4f %.4f", $1, $2, $3}')"   # so thuc cho -p
G0_RAD=$(awk -v g="$G0" 'BEGIN{printf "%.6f", g * atan2(0, -1) / 180}')
sed -i -E "s/^( *map_start_pose:).*/\1 [$X0, $Y0, $G0_RAD]/" "$TAM/slam.yaml"
[ -n "$SLAM_MIN_TRAVEL" ] && sed -i -E "s/^( *minimum_travel_distance:) .*/\1 $SLAM_MIN_TRAVEL/" "$TAM/slam.yaml"
# B8: SLAM_THAM="ten=gia_tri ..." ghi de them tham so CHI trong ban chep slam.yaml (ten phai co san trong tep)
for kv in ${SLAM_THAM:-}; do
  k=${kv%%=*}; v=${kv#*=}
  grep -qE "^ *$k:" "$TAM/slam.yaml" || { echo "LOI: SLAM_THAM $k khong co trong slam.yaml"; exit 1; }
  sed -i -E "s/^( *$k:) .*/\1 $v/" "$TAM/slam.yaml"
done
grep -h "minimum_travel\|scan_buffer_size\|loop_match_minimum_chain_size\|correlation_search_space_smear" "$TAM/slam.yaml" | tr -s ' ' | tr '\n' ' '; echo "(SLAM_MIN_TRAVEL=${SLAM_MIN_TRAVEL:-khong dat} SLAM_THAM=${SLAM_THAM:-khong}) XE_GIA_THAM=${XE_GIA_THAM:-khong}"
grep -h "inflation_radius\|map_start_pose" "$TAM/nav2.yaml" "$TAM/slam.yaml" | tr -s ' ' | tr '\n' ' '; echo
if grep -n project1 "$TAM/slam.yaml" "$TAM/nav2.yaml"; then echo "LOI: con /home/project1 trong ban chep"; exit 1; fi
grep -h "map_file_name\|default_nav_to_pose_bt_xml" "$TAM/slam.yaml" "$TAM/nav2.yaml"
# ~/ ma chuyen.sh BANG=day can: lien ket toi ban trong kho; tep that (khong phai lien ket ve kho) thi giu nguyen
for f in gui_duong.py tinh_tuyen.py; do
  if [ "$(readlink -f ~/"$f")" = "$XS/$f" ]; then echo "~/$f: da lien ket toi kho"
  elif [ -e ~/"$f" ] || [ -L ~/"$f" ]; then echo "~/$f: GIU tep co san (khong de)"
  else ln -s "$XS/$f" ~/"$f" && echo "~/$f: lien ket -> $XS/$f"; fi
done

T0=$SECONDS
setsid nohup ros2 launch my_bot mapping_my_bot.launch.py bat_lidar:=false bat_dong_co:=false bat_gyro:=false \
  bat_camera:=false bat_mo_hinh_san:=true access_code:=gia slam_params_file:="$TAM/slam.yaml" \
  > "$TAM/log_launch.txt" 2>&1 < /dev/null &
P1=$!
setsid nohup python3 "$GIA/xe_gia.py" --ros-args -p canh_pid:=$P1 \
  -p x0:="$X0" -p y0:="$Y0" -p goc0_do:="$G0" $XE_GIA_THAM > "$TAM/log_xe_gia.txt" 2>&1 < /dev/null &
echo "launch mapping PID $P1, xe_gia PID $!"
# Xe that lay goc tu con quay (bat_gyro:=true), nhung bat_gyro:=true se bat node gyro_mpu6050 that. Nen: tat
# odom_tu_lenh do launch bat (yaw_nguon=lenh) roi bat lai chinh no voi yaw_nguon=gyro; /gyro_yaw do xe_gia phat.
ODOM_LAUNCH='python3 [^ ]*/[o]dom_tu_lenh\.py --ros-args .*--params-file'
for i in $(seq 50); do pgrep -f "$ODOM_LAUNCH" > /dev/null && break; sleep 0.1; done
pkill -TERM -f "$ODOM_LAUNCH"   # TERM: luc moi exec, INT con bi bo qua (thua tu launch) toi khi rclpy.init
for i in $(seq 50); do pgrep -f "$ODOM_LAUNCH" > /dev/null || break; sleep 0.1; done
if pgrep -f "$ODOM_LAUNCH" > /dev/null; then echo "LOI: khong tat duoc odom_tu_lenh cua launch"; exit 1; fi
setsid nohup python3 "$ODOM_PY" --ros-args -r __node:=odom_tu_lenh \
  -p yaw_nguon:=gyro -p mo_hinh_san:=true $ODOM_THAM > "$TAM/log_odom.txt" 2>&1 < /dev/null &
echo "odom_tu_lenh (yaw_nguon=gyro) PID $! ODOM_PY=$ODOM_PY ODOM_THAM=${ODOM_THAM:-khong}"
sleep 5
if [ "$MAT_LAI" = "true" ]; then
  setsid nohup ros2 launch "$XS/../dev_ws/src/my_bot/launch/navigation_my_bot.launch.py" params_file:="$TAM/nav2.yaml" mat_lai:=true \
    > "$TAM/log_nav.txt" 2>&1 < /dev/null &
else
setsid nohup ros2 launch my_bot navigation_my_bot.launch.py params_file:="$TAM/nav2.yaml" \
  > "$TAM/log_nav.txt" 2>&1 < /dev/null &
fi
for i in $(seq "$T_CHO"); do
  if grep -q "Managed nodes are active" "$TAM/log_nav.txt"; then
    grep -h "Managed nodes are active" "$TAM/log_nav.txt"
    echo "NAV2_SAN_SANG sau $((SECONDS - T0)) s"
    break
  fi
  if [ $((SECONDS - T0)) -ge "$T_CHO" ]; then
    echo "NAV2_CHUA_SAN_SANG sau $((SECONDS - T0)) s (xem $TAM/log_nav.txt)"
    exit 1
  fi
  sleep 1
done
# ---- sep va phu lai, nhu bat_xe.sh B2b (chay tu ban trong kho; ghi ~/sep_* nhu tren xe) ----
echo "== bat sep (ra_lenh=$SEP_RA_LENH ra_thoat=$SEP_RA_THOAT) va phu lai (phat_lenh=$PHU_LAI_PHAT_LENH) =="
NGHE_FP=""; [ "$MAT_LAI" = "true" ] && NGHE_FP="-p nghe_follow_path:=false"
SEP_MAT=""; [ "$MAT_LAI" = "true" ] && SEP_MAT="-p dem_luot_thoat:=false"   # 08/11 1e: phu lai chi nhin => sep khong dem luot thoat   # 08/11 bo 1e, nhu bat_xe.sh
setsid nohup python3 "$XS/sep.py" --ros-args -p ra_lenh:="$SEP_RA_LENH" -p ra_thoat:="$SEP_RA_THOAT" $NGHE_FP $SEP_MAT \
  > "$TAM/log_sep.txt" 2>&1 < /dev/null &
setsid nohup python3 "$XS/lai_xe_lidar.py" --ros-args -p phat_lenh:="$PHU_LAI_PHAT_LENH" -p sep_ra_thoat:="$SEP_RA_THOAT" $NGHE_FP \
  > "$TAM/log_phu_lai.txt" 2>&1 < /dev/null &
sleep 5
n=0
pgrep -f "^python3 $XS/[s]ep\.py " > /dev/null || { echo "THIEU sep.py (xem $TAM/log_sep.txt)"; n=1; }
pgrep -f "^python3 $XS/[l]ai_xe_lidar\.py " > /dev/null || { echo "THIEU lai_xe_lidar.py (xem $TAM/log_phu_lai.txt)"; n=1; }
[ "$n" -eq 0 ] && echo "SEP_PHU_LAI_SONG sau $((SECONDS - T0)) s"
exit "$n"
