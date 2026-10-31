#!/usr/bin/env bash
# kiem_xe.sh v3 (30/10 toi): them Buoc 8 (cay hanh vi, tien trinh trung, costmap). Ban v2 giu o kiem_xe.sh.bak.
# kiem_xe.sh: kiem xe sau khi khoi dong (buoc 5 va buoc 7 cua Quy trinh chuan 71).
# CHI DOC. Khong dieu khien xe, khong sua tham so, khong dung toi motor.
# Bien tuy chon: MONG_BT = duong dan cay hanh vi mong doi (vi du /home/project1/bt_cho.xml); sai thi KHONG DAT.
# Chay tren Pi (dau nhac project1@), o mot tab MOI, SAU KHI launch va Nav2 da len.
#
# Cach dung:
#   bash ~/kiem_xe.sh              kiem 3 tham so; in x, y, goc de ban ghi lai (chua so sanh vi tri)
#   bash ~/kiem_xe.sh X Y          them so sanh vi tri voi X, Y (met); goc mong doi = 0 do
#   bash ~/kiem_xe.sh X Y GOC      nhu tren, goc mong doi = GOC (do)
# Nguong (theo Quy trinh chuan 71): x +-0.10 m, y +-0.05 m, goc +-5 do.
# Ma thoat: 0 = dat, 1 = khong dat.

[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

KQ=0

lay() {
  ros2 param get "$1" "$2" 2>/dev/null | sed -n 's/.*value is: *//p' | tail -1
}

kiem_tp() {
  local ten="$1" node="$2" tp="$3" mong="$4" v
  v=$(lay "$node" "$tp")
  if [ -z "$v" ]; then
    echo "KHONG DAT  $ten: khong doc duoc ($node $tp)"
    KQ=1
    return
  fi
  if python3 - "$v" "$mong" <<'PY'
import sys
a, b = sys.argv[1], sys.argv[2]
try:
    ok = abs(float(a) - float(b)) < 1e-6
except ValueError:
    ok = (a == b)
sys.exit(0 if ok else 1)
PY
  then
    echo "DAT        $ten = $v"
  else
    echo "KHONG DAT  $ten = $v (mong $mong)"
    KQ=1
  fi
}

echo "== Dien ap Pi (0x0 = chua tung thieu dien/qua nong tu luc bat) =="
if command -v vcgencmd > /dev/null 2>&1; then
  vcgencmd get_throttled
else
  echo "(khong co vcgencmd)"
fi

echo "== Buoc 5: 3 tham so =="
kiem_tp "yaw_nguon" /odom_tu_lenh yaw_nguon gyro
kiem_tp "gyro_scale" /gyro_mpu6050 gyro_scale 131.0
kiem_tp "dau_yaw" /gyro_mpu6050 dau_yaw -1.0

echo "== Buoc 7: vi tri xe tren ban do (doc khoang 8 giay) =="
TF=$(timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1)
POSE=$(printf '%s\n' "$TF" | python3 -c '
import re, sys
t = sys.stdin.read()
num = r"([-+0-9.eE]+)"
tr = re.findall(r"Translation: \[" + num + ", " + num + ", " + num + r"\]", t)
dg = re.findall(r"RPY \(degree\) \[" + num + ", " + num + ", " + num + r"\]", t)
if tr and dg:
    print(tr[-1][0], tr[-1][1], dg[-1][2])
')

if [ -z "$POSE" ]; then
  echo "KHONG DAT  khong doc duoc vi tri (Nav2 hoac SLAM chua len, hoac chua co TF map -> base_link)"
  KQ=1
else
  read -r PX PY PG <<< "$POSE"
  if [ -n "$1" ] && [ -n "$2" ]; then
    python3 - "$PX" "$PY" "$PG" "$1" "$2" "${3:-0}" <<'PY'
import sys
x, y, g, ex, ey, eg = map(float, sys.argv[1:7])
dx, dy = x - ex, y - ey
dg = (g - eg + 180.0) % 360.0 - 180.0
ok = abs(dx) <= 0.10 and abs(dy) <= 0.05 and abs(dg) <= 5.0
print("%s  vi tri: x=%.3f m (lech %+.3f, cho phep 0.10), y=%.3f m (lech %+.3f, cho phep 0.05), goc=%.1f do (lech %+.1f, cho phep 5.0)"
      % ("DAT       " if ok else "KHONG DAT ", x, dx, y, dy, g, dg))
sys.exit(0 if ok else 1)
PY
    [ $? -eq 0 ] || KQ=1
  else
    echo "GHI LAI    x=$PX m, y=$PY m, goc=$PG do (chua co so mong doi nen chua so sanh)"
    CHUA_SO_SANH=1
  fi
fi

echo "== Buoc 8: cay hanh vi, tien trinh trung, costmap (v3, chi doc) =="
# 8a. cay hanh vi Nav2 dang dung. Co MONG_BT thi phai khop (bat_xe.sh tu dat MONG_BT khi co NAV_PARAMS).
if [ -n "$MONG_BT" ]; then
  kiem_tp "bt_xml" /bt_navigator default_nav_to_pose_bt_xml "$MONG_BT"
else
  BTV=$(lay /bt_navigator default_nav_to_pose_bt_xml)
  echo "GHI LAI    bt_xml = ${BTV:-'(rong hoac khong doc duoc)'}  (khong khai MONG_BT nen chua so sanh)"
fi
# 8b. tien trinh Nav2 bi trung (danh sach don cu thieu 5 ten): moi loai phai dung 1 tien trinh
for p in planner_server controller_server bt_navigator; do
  n=$(pgrep -f "[${p:0:1}]${p:1}" | wc -l)
  if [ "$n" -gt 1 ]; then
    echo "KHONG DAT  $p co $n tien trinh (mong 1): con tien trinh cu. bash ~/bat_xe.sh tat roi bat lai"
    KQ=1
  else
    echo "DAT        $p: $n tien trinh"
  fi
done
NL=$(ros2 node list 2>/dev/null)
NN=$(printf '%s\n' "$NL" | grep -c '^/')
DUP=$(printf '%s\n' "$NL" | sort | uniq -d)
if [ "$NN" -lt 3 ]; then
  echo "GHI LAI    khong doc duoc danh sach node (chi thay $NN); bo qua buoc nay"
elif [ -n "$DUP" ]; then
  echo "KHONG DAT  node trung ten: $DUP"
  KQ=1
else
  echo "DAT        $NN node, khong trung ten"
fi
# 8c. costmap toan cuc nghi xe o dau va tin cu bao lau (chi ghi lai, khong tinh dat/khong dat)
FP=$(timeout 8 ros2 topic echo --once /global_costmap/published_footprint geometry_msgs/msg/PolygonStamped 2>/dev/null \
  | python3 -c '
import re, sys, time
t = sys.stdin.read()
xs = [float(v) for v in re.findall(r"^\s*-\s*x:\s*([-+0-9.eE]+)\s*$", t, re.M)]
ys = [float(v) for v in re.findall(r"^\s+y:\s*([-+0-9.eE]+)\s*$", t, re.M)]
sec = re.search(r"^\s+sec:\s*(\d+)", t, re.M)
if len(xs) >= 3 and len(xs) == len(ys) and sec:
    print("x=%.3f m, y=%.3f m, tuoi tin %.1f s" % (sum(xs) / len(xs), sum(ys) / len(ys), time.time() - int(sec.group(1))))
')
echo "GHI LAI    costmap toan cuc: ${FP:-khong doc duoc} (so voi tf o Buoc 7; tuoi tin > 5 s la dau hieu tf cu trong Nav2)"

echo "=========="
if [ "$KQ" -eq 0 ]; then
  if [ -n "$CHUA_SO_SANH" ]; then
    echo "KET QUA: 3 THAM SO DAT; VI TRI CHUA SO SANH (ghi lai x, y o tren)"
  else
    echo "KET QUA: DAT"
  fi
else
  echo "KET QUA: KHONG DAT -> Ctrl+C Nav2 va launch, dat lai xe o vach, khoi dong lai tu buoc 3"
fi
exit $KQ
