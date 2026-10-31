#!/usr/bin/env bash
# so_voi_pi.sh: so tep trong xe_scripts voi tep tuong ung tren Pi bang md5sum. CHI DOC:
# khong ghi gi len Pi, khong in noi dung tep. Ca hai phia bo moi dong chua ACCESS truoc khi tinh md5sum.
# Chay:  bash xe_scripts/so_voi_pi.sh
# Moi dong:  ten tep | GIONG / KHAC / KHONG_CO_TREN_PI / CHUA_RO   (them LOI_SSH neu khong noi duoc Pi)
# Bang duong dan lay tu VI_TRI_TREN_PI.md. "?" = chua ro duong dan tren Pi.
# cau_noi_xe.py va chon_b_v4.html so voi ban da cai (~/cau_noi/), la ban dich vu dang chay.
PI="project1@192.168.0.2"
DIR="$(cd "$(dirname "$0")" && pwd)"

while IFS='|' read -r ten duong; do
  [ -z "$ten" ] && continue
  if [ "$duong" = "?" ]; then echo "$ten | CHUA_RO"; continue; fi
  if [ ! -f "$DIR/$ten" ]; then echo "$ten | LOI_THIEU_TEP_TRONG_XE_SCRIPTS"; continue; fi
  local_md5="$(grep -a -v ACCESS "$DIR/$ten" | md5sum | awk '{print $1}')"
  pi_out="$(ssh -n -o BatchMode=yes -o ConnectTimeout=5 "$PI" \
    "if [ -f $duong ]; then grep -a -v ACCESS $duong | md5sum; else echo MISSING; fi")"
  rc=$?
  if [ $rc -ne 0 ]; then echo "$ten | LOI_SSH"; continue; fi
  pi_md5="$(echo "$pi_out" | awk '{print $1}')"
  if [ "$pi_md5" = "MISSING" ]; then echo "$ten | KHONG_CO_TREN_PI"
  elif [ "$pi_md5" = "$local_md5" ]; then echo "$ten | GIONG"
  else echo "$ten | KHAC"
  fi
done <<'BANG'
bat_xe.sh|~/bat_xe.sh
bat_xe.sh.v3|?
kiem_xe.sh|~/kiem_xe.sh
chuyen.sh|~/chuyen.sh
ghi_bag.sh|~/ghi_bag.sh
bt_cho.xml|~/bt_cho.xml
nav2_params_cho.yaml|~/nav2_params_cho.yaml
cai_cau_noi.sh|~/cai_cau_noi.sh
cau_noi_xe.py|~/cau_noi/cau_noi_xe.py
chon_b_v4.html|~/cau_noi/chon_b.html
lai_xe_lidar.py|~/lai_xe_lidar.py
HUONG_DAN_CAU_NOI.md|?
HUONG_DAN_THU_31_10.md|?
THIET_KE_LAI_XE_LIDAR.md|?
BAO_CAO_SU_CO_31_10_CHIEU.md|?
LENH_CLAUDE_CODE.txt|?
mo_phong_he_thong.py|?
so_sanh_tuyen.py|?
test_lai_xe_lidar.py|?
BANG
