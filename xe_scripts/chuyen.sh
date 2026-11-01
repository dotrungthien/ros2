#!/usr/bin/env bash
# chuyen.sh v4 (sua trua 31/10/2026 o ngoai duong, tu ban v3; CHUA CHAY TREN XE): di A->B hoac ve B->A theo BANG TUYEN.
#
# v4 khac v3 o DUNG mot cho: BANG TUYEN TIEP TUYEN (Thien chot 31/10; hinh va tinh toan: THIET_KE_LAI_XE_LIDAR.md muc 9).
#   Moi diem co huong = huong di DEN diem do, moi cho re <= ~36 do (them diem trung gian), huong cuoi o A khong cham.
#   Muc dich: bo cac lan xoay tai cho lon o cho hep (bang cu bat xe xoay 102, 78, 175 do khi ve A).
#   Chon bang bang bien BANG: BANG=cu (bang v3 tro ve truoc, de so sanh) | BANG=tt (tiep tuyen, MAC DINH) | BANG=xs (tinh_tuyen.py tu costmap, 01/11). Khong can sua tep.
#   Cot 2 cua ~/ket_qua_chuyen.csv thanh "ve_A_cu" / "ve_A_tt": lenh dem x/n o Phan 2 tu tach hai bang.
#
# --- noi dung v3 (giu nguyen) ---
# v3 khac v2 o DUNG hai cho (khong doi cach xe chay):
#   (1) KIEM vi tri truoc MOI diem (v2 chi kiem truoc diem dau tien cua chuyen).
#   (2) Ghi them 2 so vao CSV: do lech cua costmap toan cuc (global_costmap) so voi tf, va tuoi tin cua no (giay).
#       Mac dinh CHI GHI, khong dung xe (FP_STOP=0). Dat FP_STOP=1 de dung xe khi costmap lech/cu.
#
# Cach dung (tren Pi, SAU KHI bat_xe.sh bao SAN SANG, cong tac motor ON, tay sat cong tac):
#   bash ~/chuyen.sh di_B          A -> B (B = x 1.60, y 0.82)
#   bash ~/chuyen.sh ve_A          B -> A
#   bash ~/chuyen.sh ve_A 2        nhu tren nhung thu tuyen 2 truoc
# Bien moi truong (tuy chon): BANG (tt | cu | xs, mac dinh tt), TRY (so lan thu moi diem, mac dinh 1: cho ~30 giay roi doi tuyen, theo chinh sach 31/10), CHO (giay dung yen giua hai lan, 5),
#   TO (han chot moi lan gui, 130 giay), LECH_MAX (m, 0.30), BO_QUA_KIEM=1 (bo buoc kiem vi tri),
#   FP_STOP (0 = chi ghi, 1 = dung xe khi costmap lech), TUOI_MAX (giay, 5).
#   Muc B cua sep: SEP_LENH (tep lenh cua sep, ~/sep_lenh), SEP_TUOI_MAX (giay, 3), SEP_CHO_S (giay cho khi sep bao CHO, 10).
#   Khong co tep lenh hoac tep cu hon SEP_TUOI_MAX (sep tat, ra_lenh=false, sep chet) -> chay y nhu cu.
#   Luu y thoi gian cho: moi lan gui, cay hanh vi tu chiu den het so lan thu (15 lan ~ 60 giay; 8 lan ~ 30 giay) roi moi bao hong; voi TRY=2 xe co the
#   cho ~2 phut o mot diem truoc khi doi tuyen. Muon dung 60 giay: TRY=1 bash ~/chuyen.sh ve_A
#
# Hanh vi (khong quay vo dinh, khong lui):
#   0. KIEM (truoc moi diem): bo lap duong cua Nav2 phai thay xe gan cho tf bao (lech <= LECH_MAX). Sai -> DUNG (ma 3).
#   1. Moi tuyen la mot day diem. Gui lan luot tung diem bang /navigate_to_pose.
#   2. Diem hong -> DUNG YEN CHO giay -> thu lai (toi da TRY lan). Van hong -> bo tuyen, thu tuyen kia tu cho xe dang dung.
#   3. Ca hai tuyen hong -> DUNG, in "HET_DUONG" (ma 1).
#   4. Muc B: truoc moi lan gui va sau moi KHONG_DAT hoi sep: CHO -> dung yen SEP_CHO_S giay roi gui lai cung diem
#      (khong tinh TRY, toi da 2 lan moi diem; qua thi chay nhu cu); DUNG -> ghi DUNG_SEP, dung (ma 4).
#   Moi buoc ghi mot dong vao ~/ket_qua_chuyen.csv de dem x/n.
# Ma thoat: 0 den noi; 1 het duong; 2 sai doi so; 3 Nav2 thay sai vi tri xe (khong gui them lenh nao); 4 sep bao DUNG.
# Dung khan: TAT CONG TAC MOTOR (Ctrl+C chua chac huy muc tieu dang chay trong Nav2).
#
# Toa do (map, goc A = tam xe o vach A; doc tu ban do map_xuong_v2, sai so ~ +-5 cm):
#   Hanh lang 1 (duoi, giua K4 va H0): tam y ~ 0.09, rong ~ 0.50 m, x 0.40..1.04
#   Hanh lang 2 (tren H0): tam y ~ 0.76, rong ~ 0.50 m
#   Huong: "z w" cua quaternion: huong +x = "0.0 1.0"; huong -x (ve phia tuong sau) = "1.0 0.0"

HUONG="$1"
TUYEN_DAU="${2:-1}"
BANG="${BANG:-tt}"
case "$BANG" in tt|cu|xs) ;; *) echo "SAI BANG: '$BANG'. Dung BANG=tt, BANG=cu hoac BANG=xs"; exit 2 ;; esac
NHAN="${HUONG}_${BANG}"   # nhan ghi vao cot 2 cua CSV
TRY="${TRY:-1}"
CHO="${CHO:-5}"
TO="${TO:-130}"
LECH_MAX="${LECH_MAX:-0.30}"
FP_STOP="${FP_STOP:-0}"
TUOI_MAX="${TUOI_MAX:-5}"
SEP_LENH="${SEP_LENH:-$HOME/sep_lenh}"
SEP_TUOI_MAX="${SEP_TUOI_MAX:-3}"
SEP_CHO_S="${SEP_CHO_S:-10}"
CSV="$HOME/ket_qua_chuyen.csv"

[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

case "$HUONG" in
  di_B|ve_A) ;;
  *) echo "SAI DOI SO. Dung: bash ~/chuyen.sh di_B | ve_A [tuyen_dau 1|2]"; exit 2 ;;
esac
case "$TUYEN_DAU" in
  1) THU_TU="1 2" ;;
  2) THU_TU="2 1" ;;
  *) echo "SAI DOI SO: tuyen_dau phai la 1 hoac 2"; exit 2 ;;
esac

ghi() { # $1.. = cac cot cua dong CSV (cot 6 = ket_qua, giu nguyen nhu v2)
  [ -f "$CSV" ] || echo "gio,huong,tuyen,diem,lan,ket_qua,giay,lech_kiem_m,lech_costmap_m,tuoi_costmap_s" > "$CSV"
  echo "$(date '+%F %T'),$*" >> "$CSV"
}

# day diem cua mot tuyen: moi phan tu = "x y z w"
lay_tuyen() { # $1 = huong, $2 = tuyen -> dat mang WP
  WP=()
  case "$BANG:$1:$2" in
    # ---- BANG CU (v3): huong moi diem = huong cuoi, khong theo huong di ----
    cu:ve_A:1) WP=("1.45 0.13 1.0 0.0" "0.0 0.0 0.0 1.0") ;;
    cu:ve_A:2) WP=("1.45 0.76 1.0 0.0" "0.30 0.72 1.0 0.0" "0.0 0.0 0.0 1.0") ;;
    cu:di_B:1) WP=("1.60 0.82 0.0 1.0") ;;
    cu:di_B:2) WP=("0.30 0.72 0.0 1.0" "1.60 0.82 0.0 1.0") ;;
    # ---- BANG TIEP TUYEN (v4): "x y z w"; huong moi diem = huong di den diem do; cho re <= ~36 do ----
    # Cach tinh: THIET_KE_LAI_XE_LIDAR.md muc 9. Cho quay dau o B' (>= 120 do) KHONG tranh duoc vi B' la ngo cut.
    tt:ve_A:1) WP=("1.30 0.30 -0.8659 0.5001" "1.00 0.10 -0.9571 0.2898" "0.00 0.00 -0.9988 0.0498") ;;
    tt:ve_A:2) WP=("1.30 0.75 -0.9934 0.1144" "0.45 0.76 1.0 0.0059" "0.15 0.60 -0.9701 0.2425" "0.00 0.30 -0.8507 0.5257" "0.00 0.00 -0.7071 0.7071") ;;
    tt:di_B:1) WP=("1.60 0.82 0.0 1.0") ;;   # khong doi: A->B da 11/11 (29/10); huong cuoi y = 0 do nhu cu
    tt:di_B:2) WP=("0.00 0.30 0.7071 0.7071" "0.15 0.60 0.5257 0.8507" "0.45 0.76 0.2425 0.9701" "1.30 0.75 -0.0059 1.0" "1.60 0.82 0.0 1.0") ;;
    # BANG XS (01/11): tinh_tuyen.py tu costmap_v3_020, gia 1+60/d^3, 6 diem
    xs:di_B:1) WP=("0.33 0.11 0.0000 1.0000" "0.70 0.11 -0.0885 0.9961" "1.05 0.06 0.0000 1.0000" "1.37 0.17 0.3827 0.9239" "1.56 0.46 0.6418 0.7669" "1.60 0.82 0.0000 1.0000") ;;
    xs:di_B:2) WP=("0.06 0.34 0.5787 0.8156" "0.26 0.61 0.3867 0.9222" "0.56 0.76 0.0000 1.0000" "0.92 0.76 0.0000 1.0000" "1.28 0.76 0.0000 1.0000" "1.60 0.82 0.0000 1.0000") ;;
    xs:ve_A:1) WP=("1.56 0.46 -0.7669 0.6418" "1.37 0.17 -0.9239 0.3827" "1.05 0.06 1.0000 0.0000" "0.68 0.06 1.0000 0.0000" "0.33 0.11 0.9964 0.0846" "0.00 0.00 -0.9257 0.3783") ;;
    xs:ve_A:2) WP=("1.28 0.76 1.0000 0.0000" "0.92 0.76 1.0000 0.0000" "0.56 0.76 1.0000 0.0000" "0.26 0.61 -0.9222 0.3867" "0.06 0.34 -0.8156 0.5787" "0.00 0.00 -0.7979 0.6028") ;;
  esac
}

# vi tri xe theo tf: in "x y"
tf_xy() {
  timeout 10 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null \
    | grep -m1 'Translation' | tr -d '[],' | awk '{print $3, $4}'
}

# diem dau cua duong ma bo lap duong tu vach (tu lay vi tri xe) toi (x,y): dat PX PY PN
plan_dau() {
  PX=""; PY=""; PN=0
  timeout 20 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}" \
    > /tmp/chuyen_cp.txt 2>&1
  sed -n '/Result:/,$p' /tmp/chuyen_cp.txt > /tmp/chuyen_cp_kq.txt
  PN=$(grep -c 'position:' /tmp/chuyen_cp_kq.txt)
  read -r PX PY <<< "$(awk '/position:/{getline; x=$2; getline; y=$2; print x, y; exit}' /tmp/chuyen_cp_kq.txt)"
}

# costmap toan cuc nghi xe o dau (tam hinh footprint) va tuoi tin (giay): in "x y tuoi", rong neu khong doc duoc
fp_xy() {
  timeout 8 ros2 topic echo --once /global_costmap/published_footprint geometry_msgs/msg/PolygonStamped 2>/dev/null \
    | python3 -c '
import re, sys, time
t = sys.stdin.read()
xs = [float(v) for v in re.findall(r"^\s*-\s*x:\s*([-+0-9.eE]+)\s*$", t, re.M)]
ys = [float(v) for v in re.findall(r"^\s+y:\s*([-+0-9.eE]+)\s*$", t, re.M)]
sec = re.search(r"^\s+sec:\s*(\d+)", t, re.M)
if len(xs) >= 3 and len(xs) == len(ys) and sec:
    print("%.3f %.3f %.1f" % (sum(xs) / len(xs), sum(ys) / len(ys), time.time() - int(sec.group(1))))
'
}

LECH="-"; LECHFP="-"; TUOI="-"
kiem_vi_tri() { # $1 $2 = diem se gui ngay sau do. Tra 0 neu cho phep chay.
  LECH="-"; LECHFP="-"; TUOI="-"
  if [ -n "$BO_QUA_KIEM" ]; then LECH="bo_qua"; return 0; fi
  local TX TY FX FY
  read -r TX TY <<< "$(tf_xy)"
  plan_dau "$1" "$2"
  read -r FX FY TUOI <<< "$(fp_xy)"
  [ -z "$TUOI" ] && TUOI="-"
  if [ -z "$TX" ] || [ -z "$PX" ] || [ "$PN" -lt 3 ]; then
    echo "KIEM: khong doc duoc vi tri xe (tf='$TX $TY') hoac bo lap duong khong ra duong (so diem=$PN)."
    return 1
  fi
  LECH=$(awk -v a="$PX" -v b="$PY" -v c="$TX" -v d="$TY" 'BEGIN{dx=a-c; dy=b-d; printf "%.2f", sqrt(dx*dx+dy*dy)}')
  if [ -n "$FX" ]; then
    LECHFP=$(awk -v a="$FX" -v b="$FY" -v c="$TX" -v d="$TY" 'BEGIN{dx=a-c; dy=b-d; printf "%.2f", sqrt(dx*dx+dy*dy)}')
  fi
  echo "KIEM: tf bao xe o ($TX; $TY); bo lap duong xuat phat tu ($PX; $PY), lech $LECH m (cho phep $LECH_MAX); duong $PN diem."
  echo "      costmap toan cuc: ($FX; $FY), lech tf $LECHFP m, tuoi tin $TUOI s."
  if ! awk -v l="$LECH" -v m="$LECH_MAX" 'BEGIN{exit (l<=m)?0:1}'; then
    return 1
  fi
  if [ -n "$FX" ] && ! awk -v l="$LECHFP" -v m="$LECH_MAX" -v a="$TUOI" -v am="$TUOI_MAX" 'BEGIN{exit (l<=m && a<=am)?0:1}'; then
    echo "KIEM: costmap toan cuc thay xe lech hoac tin cu (lech $LECHFP m, tuoi $TUOI s)."
    if [ "$FP_STOP" = "1" ]; then
      return 1
    fi
    echo "      (FP_STOP=0: chi ghi lai, van chay.)"
    ghi "$NHAN,$R,$IDX,0,COSTMAP_LECH,0,$LECH,$LECHFP,$TUOI"
  fi
  return 0
}

# gui mot diem, tra ve 0 neu SUCCEEDED
gui() { # $1 x, $2 y, $3 z, $4 w, $5 ten tep log
  timeout "$TO" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {z: $3, w: $4}}}}" \
    --feedback > "$5" 2>&1
  grep -q 'status: SUCCEEDED' "$5"
}

# muc B: doc ~/sep_lenh ("<giay /proc/uptime> DI|CHO|DUNG <ly do>"), dat SEP va SEP_LY.
# SEP rong (chay y nhu cu) khi khong co tep hoac tep cu hon SEP_TUOI_MAX giay.
doc_sep() {
  SEP=""; SEP_LY=""
  [ -f "$SEP_LENH" ] || return 0
  local T C L U
  read -r T C L < "$SEP_LENH" || return 0
  read -r U _ < /proc/uptime
  awk -v u="$U" -v t="$T" -v m="$SEP_TUOI_MAX" 'BEGIN{d = u - t; exit (t + 0 > 0 && d >= -1 && d <= m) ? 0 : 1}' || return 0
  SEP="$C"; SEP_LY="$L"
}

# muc B: lam theo sep. DUNG -> ghi DUNG_SEP, thoat ma 4. CHO -> dung yen SEP_CHO_S giay (toi da 2 lan moi diem,
# khong tinh TRY) roi tra 0 de gui lai cung diem. Con lai (DI, khong co tep, da cho 2 lan) tra 1: chay nhu cu.
theo_sep() {
  doc_sep
  case "$SEP" in
    DUNG)
      echo "DUNG_SEP: sep bao dung ($SEP_LY). Xe dung yen, khong gui them lenh."
      ghi "$NHAN,$R,$IDX,$LAN,DUNG_SEP,0,$LECH,$LECHFP,$TUOI"
      exit 4 ;;
    CHO)
      if [ "$CHO_SEP" -ge 2 ]; then
        echo "   sep van bao CHO ($SEP_LY) nhung da cho 2 lan o diem nay: chay nhu cu."
        return 1
      fi
      CHO_SEP=$((CHO_SEP + 1))
      echo "   sep bao CHO ($SEP_LY): dung yen $SEP_CHO_S giay roi gui lai cung diem (lan cho $CHO_SEP/2)."
      ghi "$NHAN,$R,$IDX,$LAN,CHO_SEP,$SEP_CHO_S,$LECH,$LECHFP,$TUOI"
      sleep "$SEP_CHO_S"
      return 0 ;;
  esac
  return 1
}

# ---------- chay ----------
TT0=$(date +%s)
R=0; IDX=0
for R in $THU_TU; do
  lay_tuyen "$HUONG" "$R"
  echo "== Tuyen $R: ${#WP[@]} diem =="
  TUYEN_HONG=0
  IDX=0
  for P in "${WP[@]}"; do
    IDX=$((IDX + 1))
    set -- $P
    if ! kiem_vi_tri "$1" "$2"; then
      echo "DUNG: Nav2 thay sai vi tri xe (truoc tuyen $R, diem $IDX). KHONG gui them lenh."
      echo "      Ghi lai 3 so tren (tf, bo lap duong, costmap). Tat het (bash ~/bat_xe.sh tat), dat xe ve vach A, bat lai."
      ghi "$NHAN,$R,$IDX,0,SAI_VI_TRI,0,$LECH,$LECHFP,$TUOI"
      exit 3
    fi
    ghi "$NHAN,$R,$IDX,0,KIEM_DAT,0,$LECH,$LECHFP,$TUOI"
    LAN=0
    XONG=0
    CHO_SEP=0
    while [ "$LAN" -lt "$TRY" ]; do
      while theo_sep; do :; done          # muc B: hoi sep truoc khi gui
      LAN=$((LAN + 1))
      T0=$(date +%s)
      echo "-- tuyen $R, diem $IDX/${#WP[@]} ($1; $2), lan $LAN/$TRY, $(date +%T)"
      TEP="$HOME/chuyen_${NHAN}_t${R}_d${IDX}_l${LAN}"
      [ "$CHO_SEP" -gt 0 ] && TEP="${TEP}_s${CHO_SEP}"
      if gui "$1" "$2" "$3" "$4" "$TEP.txt"; then
        ghi "$NHAN,$R,$IDX,$LAN,SUCCEEDED,$(( $(date +%s) - T0 )),$LECH,$LECHFP,$TUOI"
        XONG=1
        break
      fi
      ghi "$NHAN,$R,$IDX,$LAN,KHONG_DAT,$(( $(date +%s) - T0 )),$LECH,$LECHFP,$TUOI"
      doc_sep
      [ -n "$SEP" ] && sleep 0.5          # sep dang ra lenh: cho no kip thay ABORTED (vong 0,1 s) roi moi hoi
      if theo_sep; then                   # muc B: sep bao CHO -> gui lai cung diem, khong tinh TRY
        LAN=$((LAN - 1))
        continue
      fi
      if [ "$LAN" -lt "$TRY" ]; then
        echo "   khong den duoc. DUNG YEN $CHO giay roi thu lai."
        sleep "$CHO"
      fi
    done
    if [ "$XONG" -ne 1 ]; then
      TUYEN_HONG=1
      echo "== Tuyen $R hong o diem $IDX. =="
      break
    fi
  done
  if [ "$TUYEN_HONG" -eq 0 ]; then
    echo "DEN_NOI: $HUONG xong theo tuyen $R."
    ghi "$NHAN,$R,tong,0,DEN_NOI,$(( $(date +%s) - TT0 )),$LECH,$LECHFP,$TUOI"
    exit 0
  fi
done

echo "HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen. Kiem tra vat can, tat cong tac motor."
ghi "$NHAN,-,het,0,HET_DUONG,0,$LECH,$LECHFP,$TUOI"
exit 1
