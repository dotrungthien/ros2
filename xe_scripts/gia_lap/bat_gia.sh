#!/usr/bin/env bash
# Bat gia lap vong kin tren PC (WSL): launch mapping_my_bot + Nav2 that, chi xe va LiDAR la gia (xe_gia.py).
# Tep that khong sua: chep slam_dinh_vi_xuong.yaml va nav2_params_cho.yaml vao $TAM, doi /home/project1.
# Tat: bash xe_scripts/bat_xe.sh tat (xe_gia.py tu thoat khi launch mapping chet; odom_tu_lenh rieng khop mau NODES).
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87
GIA=$(cd "$(dirname "$0")" && pwd)
XS=$(dirname "$GIA")
TAM="${TAM:-/tmp/gia_lap}"
T_CHO="${T_CHO:-90}"
# ---- tham so hieu chuan (01/11 tren xe) ----
INFLATION="${INFLATION:-0.20}"     # inflation_radius cua global va local costmap (ban chep), nhu xe 01/11
VI_TRI_DAU="${VI_TRI_DAU:-0 0 0}"  # "x y goc_do": vi tri that dau cua xe_gia, cung la map_start_pose cua SLAM
# (banh_trai_that = 1/he_so_trai: tham so o dau xe_gia.py)
mkdir -p "$TAM"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

sed "s#/home/project1/#$XS/du_lieu_0111/#g" "$XS/du_lieu_0111/slam_dinh_vi_xuong.yaml" > "$TAM/slam.yaml"
sed "s#/home/project1/#$XS/#g" "$XS/nav2_params_cho.yaml" > "$TAM/nav2.yaml"
sed -i -E "s/^( *inflation_radius:) .*/\1 $INFLATION/" "$TAM/nav2.yaml"
read -r X0 Y0 G0 <<< "$(echo "$VI_TRI_DAU" | awk '{printf "%.4f %.4f %.4f", $1, $2, $3}')"   # so thuc cho -p
G0_RAD=$(awk -v g="$G0" 'BEGIN{printf "%.6f", g * atan2(0, -1) / 180}')
sed -i -E "s/^( *map_start_pose:).*/\1 [$X0, $Y0, $G0_RAD]/" "$TAM/slam.yaml"
grep -h "inflation_radius\|map_start_pose" "$TAM/nav2.yaml" "$TAM/slam.yaml" | tr -s ' ' | tr '\n' ' '; echo
if grep -n project1 "$TAM/slam.yaml" "$TAM/nav2.yaml"; then echo "LOI: con /home/project1 trong ban chep"; exit 1; fi
grep -h "map_file_name\|default_nav_to_pose_bt_xml" "$TAM/slam.yaml" "$TAM/nav2.yaml"

T0=$SECONDS
setsid nohup ros2 launch my_bot mapping_my_bot.launch.py bat_lidar:=false bat_dong_co:=false bat_gyro:=false \
  bat_camera:=false bat_mo_hinh_san:=true access_code:=gia slam_params_file:="$TAM/slam.yaml" \
  > "$TAM/log_launch.txt" 2>&1 < /dev/null &
P1=$!
setsid nohup python3 "$GIA/xe_gia.py" --ros-args -p canh_pid:=$P1 \
  -p x0:="$X0" -p y0:="$Y0" -p goc0_do:="$G0" > "$TAM/log_xe_gia.txt" 2>&1 < /dev/null &
echo "launch mapping PID $P1, xe_gia PID $!"
# Xe that lay goc tu con quay (bat_gyro:=true), nhung bat_gyro:=true se bat node gyro_mpu6050 that. Nen: tat
# odom_tu_lenh do launch bat (yaw_nguon=lenh) roi bat lai chinh no voi yaw_nguon=gyro; /gyro_yaw do xe_gia phat.
ODOM_LAUNCH='python3 [^ ]*/[o]dom_tu_lenh\.py --ros-args .*--params-file'
for i in $(seq 50); do pgrep -f "$ODOM_LAUNCH" > /dev/null && break; sleep 0.1; done
pkill -TERM -f "$ODOM_LAUNCH"   # TERM: luc moi exec, INT con bi bo qua (thua tu launch) toi khi rclpy.init
for i in $(seq 50); do pgrep -f "$ODOM_LAUNCH" > /dev/null || break; sleep 0.1; done
if pgrep -f "$ODOM_LAUNCH" > /dev/null; then echo "LOI: khong tat duoc odom_tu_lenh cua launch"; exit 1; fi
setsid nohup python3 ~/ros2_ws/install/my_bot/lib/my_bot/odom_tu_lenh.py --ros-args -r __node:=odom_tu_lenh \
  -p yaw_nguon:=gyro -p mo_hinh_san:=true > "$TAM/log_odom.txt" 2>&1 < /dev/null &
echo "odom_tu_lenh (yaw_nguon=gyro) PID $!"
sleep 5
setsid nohup ros2 launch my_bot navigation_my_bot.launch.py params_file:="$TAM/nav2.yaml" \
  > "$TAM/log_nav.txt" 2>&1 < /dev/null &
for i in $(seq "$T_CHO"); do
  if grep -q "Managed nodes are active" "$TAM/log_nav.txt"; then
    grep -h "Managed nodes are active" "$TAM/log_nav.txt"
    echo "NAV2_SAN_SANG sau $((SECONDS - T0)) s"
    exit 0
  fi
  [ $((SECONDS - T0)) -ge "$T_CHO" ] && break
  sleep 1
done
echo "NAV2_CHUA_SAN_SANG sau $((SECONDS - T0)) s (xem $TAM/log_nav.txt)"
exit 1
