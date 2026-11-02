#!/usr/bin/env bash
# chup_loi.sh (viec 113, 02/11): chup trang thai controller_server de so luc tot (goc) va luc "Transform data too old" (loi).
# CHI DOC: khong giet, khong bat lai gi. Chay tren Pi bang user project1:  bash ~/chup_loi.sh goc|loi
# Ghi ~/chup_loi_HHMMSS_<nhan>.txt, 5 muc theo thu tu:
#   (1) date, uptime, ps (gio bat tung tien trinh ros/python3; ma truy cap trong dong lenh bi xoa thanh XOA)
#   (2) dong Udp cua /proc/net/snmp + sudo -n ss -uamp (Recv-Q, skmem; socket cua controller_server danh dau >>CS)
#   (3) top -H mot lan + trang thai, wchan tung luong /proc/<pid>/task/*
#   (4) timeout 30 ros2 topic info -v /tf, nap moi truong nhu bat_xe.sh (chi source ~/ros2_ws/install/setup.bash)
#   (5) CUOI CUNG: sudo -n timeout 60 gdb -p <pid> -batch -ex "thread apply all bt" (tien trinh dung vai giay)
# Khong thay pid: ghi KHONG_THAY; sudo -n hong: ghi KHONG_SUDO; ca hai deu chay tiep. Luon thoat 0, in duong dan tep.

[ -n "$1" ] || { echo "Dung: bash ~/chup_loi.sh goc|loi"; exit 2; }
NHAN=$(printf '%s' "$1" | tr -c 'A-Za-z0-9_-' '_')
TEP=~/chup_loi_$(date +%H%M%S)_$NHAN.txt
: > "$TEP"

bao() { echo "$*"; echo "$*" >> "$TEP"; }   # dong quan trong: ra man hinh va vao tep
muc() { echo "" >> "$TEP"; echo "===== $* =====" >> "$TEP"; }
xoa_ma() { sed -E 's/(access_code:=)[^ ]*/\1XOA/Ig; s/(ACCESS[=])[^ ]*/\1XOA/g'; }

# pid controller_server: pgrep -f (bai 303: [c] de khong bat chinh lenh nay); comm bi cat 15 ky tu (bai 304)
UNG=$(pgrep -f '[c]ontroller_server')
PID=""
for p in $UNG; do
  [ "$(cat /proc/$p/comm 2>/dev/null)" = "controller_serv" ] && { PID=$p; break; }
done
[ -z "$PID" ] && PID=$(echo $UNG | awk '{print $1}')
SUDO=1; timeout 5 sudo -n true 2>/dev/null || SUDO=0

bao "chup_loi $NHAN: ung vien controller_server = $(echo ${UNG:-(khong)}); chon pid = ${PID:-KHONG_THAY}"
[ -z "$PID" ] && bao "KHONG_THAY: khong co tien trinh controller_server (bo qua phan theo pid o muc 2, 3, 5)"
[ "$SUDO" = 0 ] && bao "KHONG_SUDO: sudo -n hong (ss khong co -p, khong chay gdb)"

muc "(1) gio, uptime, tien trinh ros/python3"
{ timeout 5 date '+%F %T.%N %z'; timeout 5 uptime
  timeout 10 ps -eo pid,lstart,etime,args | awk 'NR == 1 || /ros|python3/' | grep -v "chup_loi" | xoa_ma
} >> "$TEP" 2>&1

muc "(2) Udp /proc/net/snmp + ss -uam(p)"
{ timeout 5 grep '^Udp:' /proc/net/snmp
  if [ "$SUDO" = 1 ]; then SS=$(timeout 15 sudo -n ss -uamp 2>&1); else SS=$(timeout 15 ss -uam 2>&1); fi
  if [ -n "$PID" ]; then printf '%s\n' "$SS" | awk -v p="pid=$PID," 'index($0, p) {print ">>CS " $0; next} {print}'
  else printf '%s\n' "$SS"; fi
} >> "$TEP" 2>&1

muc "(3) luong cua controller_server (pid ${PID:-KHONG_THAY})"
if [ -n "$PID" ]; then
  { timeout 10 top -H -b -n1 -p "$PID"
    echo "-- tid trang_thai wchan ten (tu /proc/$PID/task/*/stat)"
    for t in /proc/$PID/task/*; do
      st=$(timeout 2 cat "$t/stat" 2>/dev/null) || continue
      ten=${st#*(}; ten=${ten%)*}; sau=${st##*) }
      echo "${t##*/} ${sau%% *} $(timeout 2 cat "$t/wchan" 2>/dev/null) ($ten)"
    done
  } >> "$TEP" 2>&1
else
  echo "KHONG_THAY" >> "$TEP"
fi

muc "(4) ros2 topic info -v /tf"
{ if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; else echo "KHONG_THAY ~/ros2_ws/install/setup.bash"; fi
  echo "moi truong lenh nay: ROS_DOMAIN_ID=${ROS_DOMAIN_ID-} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION-} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY-} FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE-}"
  if [ -n "$PID" ]; then
    echo "moi truong controller_server: $(tr '\0' '\n' < /proc/$PID/environ 2>/dev/null | grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY|FASTRTPS_|FASTDDS_)' | tr '\n' ' ')"
  fi
  timeout 30 ros2 topic info -v /tf; echo "ma ros2 topic info = $?"
} >> "$TEP" 2>&1

muc "(5) gdb thread apply all bt (pid ${PID:-KHONG_THAY})"
if [ -z "$PID" ]; then
  echo "KHONG_THAY" >> "$TEP"
elif [ "$SUDO" = 0 ]; then
  echo "KHONG_SUDO" >> "$TEP"
elif ! command -v gdb > /dev/null; then
  echo "KHONG_CO_GDB (sudo apt install gdb)" >> "$TEP"
else
  { timeout 70 sudo -n timeout 60 gdb -p "$PID" -batch -ex "thread apply all bt"; echo "ma gdb = $?"; } >> "$TEP" 2>&1
fi

echo "$TEP"
exit 0
