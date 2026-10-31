#!/usr/bin/env bash
# ghi_bag.sh (soan 30/10/2026, CHUA CHAY TREN XE): ghi lai du lieu de phan tich o nha khi xe hong.
# Dung (tren Pi, sau khi bat_xe.sh san sang, TRUOC khi gui muc tieu):
#   bash ~/ghi_bag.sh bat      bat ghi nen (moi lan ~ vai MB moi phut)
#   bash ~/ghi_bag.sh tat      dung ghi, in ten thu muc bag
# Ghi: /tf, /tf_static, /scan, /odom, /cmd_vel, /plan, /received_global_plan, hai ban do chi phi, /gyro_yaw,
#   (v3) published_footprint cua hai costmap (costmap nghi xe o dau) va /behavior_tree_log (nut nao cua cay hanh vi that bai).
# Cuoi buoi nen lai: tar czf ~/bag_ngay.tgz ~/bag_* roi scp ve may.
cd "$HOME" || exit 1
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
case "$1" in
  bat)
    if pgrep -f "[r]os2 bag record" > /dev/null; then echo "DANG GHI SAN ROI. Chay: bash ~/ghi_bag.sh tat"; exit 1; fi
    TEN="bag_$(date +%m%d_%H%M%S)"
    setsid nohup ros2 bag record -o "$HOME/$TEN" /tf /tf_static /scan /odom /cmd_vel /plan /received_global_plan \
      /global_costmap/costmap /local_costmap/costmap /gyro_yaw \
      /global_costmap/published_footprint /local_costmap/published_footprint /behavior_tree_log > "$HOME/$TEN.log" 2>&1 < /dev/null &
    sleep 4
    if pgrep -f "[r]os2 bag record" > /dev/null; then echo "DANG GHI: $HOME/$TEN"; else echo "LOI: khong bat duoc. Xem $HOME/$TEN.log"; tail -5 "$HOME/$TEN.log"; exit 1; fi ;;
  tat)
    pkill -INT -f "[r]os2 bag record"
    sleep 4
    if pgrep -f "[r]os2 bag record" > /dev/null; then echo "CHUA DUNG. Chay lai lenh tat."; exit 1; fi
    echo "DA DUNG. Cac thu muc bag:"; ls -d "$HOME"/bag_* 2>/dev/null; du -sh "$HOME"/bag_* 2>/dev/null ;;
  *) echo "Dung: bash ~/ghi_bag.sh bat | tat"; exit 2 ;;
esac
