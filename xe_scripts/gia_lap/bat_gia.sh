#!/usr/bin/env bash
# Bat gia lap vong kin tren PC (WSL): launch mapping_my_bot + Nav2 that, chi xe va LiDAR la gia (xe_gia.py).
# Tep that khong sua: chep slam_dinh_vi_xuong.yaml va nav2_params_cho.yaml vao $TAM, doi /home/project1.
# Tat: bash xe_scripts/bat_xe.sh tat (xe_gia.py tu thoat khi launch mapping chet).
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87
GIA=$(cd "$(dirname "$0")" && pwd)
XS=$(dirname "$GIA")
TAM="${TAM:-/tmp/gia_lap}"
T_CHO="${T_CHO:-90}"
mkdir -p "$TAM"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

sed "s#/home/project1/#$XS/du_lieu_0111/#g" "$XS/du_lieu_0111/slam_dinh_vi_xuong.yaml" > "$TAM/slam.yaml"
sed "s#/home/project1/#$XS/#g" "$XS/nav2_params_cho.yaml" > "$TAM/nav2.yaml"
if grep -n project1 "$TAM/slam.yaml" "$TAM/nav2.yaml"; then echo "LOI: con /home/project1 trong ban chep"; exit 1; fi
grep -h "map_file_name\|default_nav_to_pose_bt_xml" "$TAM/slam.yaml" "$TAM/nav2.yaml"

T0=$SECONDS
setsid nohup ros2 launch my_bot mapping_my_bot.launch.py bat_lidar:=false bat_dong_co:=false bat_gyro:=false \
  bat_camera:=false bat_mo_hinh_san:=true access_code:=gia slam_params_file:="$TAM/slam.yaml" \
  > "$TAM/log_launch.txt" 2>&1 < /dev/null &
P1=$!
setsid nohup python3 "$GIA/xe_gia.py" --ros-args -p canh_pid:=$P1 > "$TAM/log_xe_gia.txt" 2>&1 < /dev/null &
echo "launch mapping PID $P1, xe_gia PID $!"
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
