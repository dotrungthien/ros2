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

# 10/11 (Thien chot): muc DOC LOI (rong / qua han lenh ros2, KHONG phai gia tri sai) => doc lai rieng muc do toi da 3 lan,
# cach KIEM_DOC_LAI_S (3 s). Tong thoi gian doc lai moi lan chay <= KIEM_DOC_LAI_TONG_S (30 s): lan doc lai chi duoc bat dau khi
# con > KIEM_DOC_LAI_S, han lenh doc lai = min(han thuong, phan con lai); het => "HET_NGAN_SACH_DOC_LAI", muc con lai bao nhu cu.
# Gia tri sai: KHONG DAT ngay, khong doc lai. Khong doi nguong nao.
DOC_LAI_S="${KIEM_DOC_LAI_S:-3}"
NGAN_SACH="${KIEM_DOC_LAI_TONG_S:-30}"
DOC_LAI_DA=0; HET_NS=""; HAN_LAI=""; T_LAI=0
# doc_lai_duoc TEN LAN HAN: 0 = duoc doc lai lan LAN (da cho DOC_LAI_S, dat HAN_LAI = han lenh lan nay); 1 = thoi
doc_lai_duoc() {
  local con
  [ "$2" -le 3 ] || return 1
  con=$((NGAN_SACH - DOC_LAI_DA))
  if [ -n "$HET_NS" ] || [ "$con" -le "$DOC_LAI_S" ]; then
    [ -z "$HET_NS" ] && echo "HET_NGAN_SACH_DOC_LAI: da dung ${DOC_LAI_DA}/${NGAN_SACH} s; muc con lai khong doc lai ($1)"
    HET_NS=1
    return 1
  fi
  HAN_LAI=$(( $3 < con - DOC_LAI_S ? $3 : con - DOC_LAI_S ))
  echo "DOC_LAI    $1: lan $2/3 sau ${DOC_LAI_S} s (han ${HAN_LAI} s, ngan sach con ${con} s)"
  T_LAI=$SECONDS
  sleep "$DOC_LAI_S"
  return 0
}
# goi ngay sau lenh doc lai: cong thoi gian that da dung vao ngan sach (+1 s: SECONDS dem nguyen giay, tinh du de khong vuot)
xong_doc_lai() { DOC_LAI_DA=$((DOC_LAI_DA + SECONDS - T_LAI + 1)); }

# 02/11: moi lenh ros2 co han 10 s (lenh ros2 tren Pi hay treo); qua gio thi in chu HET_GIO thay cho gia tri
lay() {
  local out
  out=$(timeout "${HAN_DOC:-10}" ros2 param get "$1" "$2" 2>/dev/null)
  if [ $? -eq 124 ]; then echo HET_GIO; return; fi
  printf '%s\n' "$out" | sed -n 's/.*value is: *//p' | tail -1
}

kiem_tp() { # $5 = bo_qua: qua 10 s thi in BO_QUA (khong tinh khong dat), dung o Buoc 8
  local ten="$1" node="$2" tp="$3" mong="$4" v lan=1
  v=$(lay "$node" "$tp")
  while { [ "$v" = "HET_GIO" ] || [ -z "$v" ]; } && doc_lai_duoc "$ten" "$lan" 10; do     # 10/11: doc loi => doc lai
    v=$(HAN_DOC=$HAN_LAI lay "$node" "$tp"); xong_doc_lai; lan=$((lan + 1))
  done
  if [ "$v" = "HET_GIO" ]; then
    if [ "$5" = "bo_qua" ]; then
      echo "BO_QUA     $ten: ros2 param get qua 10 s ($node $tp)"
    else
      echo "KHONG DAT  $ten: ros2 param get qua 10 s ($node $tp)"
      KQ=1
    fi
    return
  fi
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
# 113a: ros-humble-tf2 phai >= 0.25.24; chua cai cung la KHONG DAT
V_TF2=$(dpkg-query -W -f='${Version}' ros-humble-tf2 2>/dev/null)
if [ -n "$V_TF2" ] && dpkg --compare-versions "$V_TF2" ge 0.25.24; then
  echo "DAT        ros-humble-tf2 = $V_TF2"
else
  echo "KHONG DAT  ros-humble-tf2 = ${V_TF2:-chua cai} (mong >= 0.25.24)"
  KQ=1
fi

echo "== Buoc 7: vi tri xe tren ban do (doc khoang 8 giay) =="
doc_vi_tri() {   # $1 = han (s); dat TF, POSE
TF=$(timeout "$1" ros2 run tf2_ros tf2_echo map base_link 2>&1)
POSE=$(printf '%s\n' "$TF" | python3 -c '
import re, sys
t = sys.stdin.read()
num = r"([-+0-9.eE]+)"
tr = re.findall(r"Translation: \[" + num + ", " + num + ", " + num + r"\]", t)
dg = re.findall(r"RPY \(degree\) \[" + num + ", " + num + ", " + num + r"\]", t)
if tr and dg:
    print(tr[-1][0], tr[-1][1], dg[-1][2])
')
}
doc_vi_tri 8
lan=1
while [ -z "$POSE" ] && doc_lai_duoc "vi tri" "$lan" 8; do doc_vi_tri "$HAN_LAI"; xong_doc_lai; lan=$((lan + 1)); done   # 10/11

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
  kiem_tp "bt_xml" /bt_navigator default_nav_to_pose_bt_xml "$MONG_BT" bo_qua
else
  BTV=$(lay /bt_navigator default_nav_to_pose_bt_xml)
  if [ "$BTV" = "HET_GIO" ]; then
    echo "BO_QUA     bt_xml: ros2 param get qua 10 s"
  else
    echo "GHI LAI    bt_xml = ${BTV:-'(rong hoac khong doc duoc)'}  (khong khai MONG_BT nen chua so sanh)"
  fi
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
NL=$(timeout 10 ros2 node list 2>/dev/null)
RC_NL=$?
NN=$(printf '%s\n' "$NL" | grep -c '^/')
lan=1
while { [ "$RC_NL" -eq 124 ] || [ "$NN" -lt 3 ]; } && doc_lai_duoc "ros2 node list" "$lan" 10; do     # 10/11
  NL=$(timeout "$HAN_LAI" ros2 node list 2>/dev/null); RC_NL=$?; xong_doc_lai
  NN=$(printf '%s\n' "$NL" | grep -c '^/'); lan=$((lan + 1))
done
DUP=$(printf '%s\n' "$NL" | sort | uniq -d)
if [ "$RC_NL" -eq 124 ]; then
  echo "BO_QUA     ros2 node list qua 10 s: khong kiem node trung ten"
elif [ "$NN" -lt 3 ]; then
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
# 8d. 08/11 bo 1e "mat lai" (~/goc_dang_chay = mat, bat_xe.sh MAT_LAI=true ghi): planner Nav2 cho costmap "current" KHONG han gio
# (Humble 1.1.20 planner_server.cpp:274-281) => thieu luoi /lop_huong_dan la treo. Kieu cu: bo qua buoc nay.
if [ "$(head -1 ~/goc_dang_chay 2>/dev/null)" = "mat" ]; then
  NPUB=$(timeout 10 ros2 topic info /lop_huong_dan 2>/dev/null | sed -n 's/^Publisher count: *//p')
  lan=1
  while [ -z "$NPUB" ] && doc_lai_duoc "/lop_huong_dan" "$lan" 10; do     # 10/11: khong in gi = doc loi ("0" = gia tri, khong doc lai)
    NPUB=$(timeout "$HAN_LAI" ros2 topic info /lop_huong_dan 2>/dev/null | sed -n 's/^Publisher count: *//p'); xong_doc_lai; lan=$((lan + 1))
  done
  if [ "$NPUB" = "1" ]; then
    echo "DAT        /lop_huong_dan: 1 publisher"
  else
    echo "KHONG DAT  /lop_huong_dan: ${NPUB:-0 hoac khong doc duoc} publisher (mong 1: lop_huong_dan.py chua chay => planner treo)"
    KQ=1
  fi
  python3 ~/lop_huong_dan.py --kiem-costmap 10 || KQ=1      # in DAT / KHONG DAT: co tin trong 10 s, cung kich thuoc ban do goc
else
  echo "BO_QUA     buoc 8d (lop_huong_dan): ~/goc_dang_chay khong phai mat"
fi
# 8e. 09/11 nao xe: ~/hanh_vi_xe.yaml khop so trong nav2_params_mat*.yaml + bt_mat.xml, khong vuot than_the, khong trai an_toan
if [ "$(head -1 ~/goc_dang_chay 2>/dev/null)" = "mat" ]; then
  python3 ~/kiem_hanh_vi.py || KQ=1                          # in bang DAT / KHONG DAT tung dong + "KIEM HANH VI: ..."
  python3 ~/gac_vat_la.py --kiem-map 10 || KQ=1               # 09/11 (K13); 20:33: /map_tinh chi de ban do NHAC tuong (LECH); LiDAR van lai
  # 09/11 23:08 sep LiDAR: dong tac thoat cua gui_duong can 3 action cua behavior_server (drive_on_heading moi them)
  AL=$(timeout 10 ros2 action list 2>/dev/null)
  lan=1
  while [ -z "$AL" ] && doc_lai_duoc "ros2 action list" "$lan" 10; do AL=$(timeout "$HAN_LAI" ros2 action list 2>/dev/null); xong_doc_lai; lan=$((lan + 1)); done   # 10/11
  for A in /backup /drive_on_heading /spin; do
    if echo "$AL" | grep -qx "$A"; then echo "DAT        action $A co"; else echo "KHONG DAT  action $A khong co (behavior_plugins?)"; KQ=1; fi
  done
else
  echo "BO_QUA     buoc 8e (kiem_hanh_vi): ~/goc_dang_chay khong phai mat"
fi

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
