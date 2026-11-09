#!/usr/bin/env bash
# so_voi_pi.sh: so tep trong xe_scripts voi tep tuong ung tren Pi bang md5sum. CHI DOC:
# khong ghi gi len Pi, khong in noi dung tep. Ca hai phia bo moi dong chua ACCESS truoc khi tinh md5sum.
# Chay:  bash xe_scripts/so_voi_pi.sh          so voi Pi, cuoi cung in dong tong "GIONG x/y"
#        bash xe_scripts/so_voi_pi.sh --nhac   chi chay phan tu nhac (khong can Pi)
# Moi dong:  ten tep | GIONG / KHAC / KHONG_CO_TREN_PI / CHUA_RO / THIEU_TREN_PI   (them LOI_SSH neu khong noi duoc Pi)
# Bang duong dan lay tu VI_TRI_TREN_PI.md. "?" = chua ro duong dan tren Pi.
# Cot 3 cua bang: "tuychon" = tep chua bat buoc phai co tren Pi; thieu thi in THIEU_TREN_PI va KHONG tinh vao y.
# Dong tong: GIONG x/y, y = so tep co duong dan Pi (tru THIEU_TREN_PI); CHUA_RO khong tinh vao y.
# cau_noi_xe.py, chon_b_v4.html, trang_v10.py, trang_v10.html (111-Q2) so voi ban da cai (~/cau_noi/), la ban dich vu dang chay.
# Tep trong kho (du_lieu_0111/) co ten trong bang la duong dan tuong doi so voi xe_scripts/.
# 08/11 bo 1e: them bt_mat.xml, nav2_params_mat.yaml, lop_huong_dan.py, map_xuong_v3.yaml/.png (map_server /map_tinh);
#   slam_dinh_vi_xuong.yaml nay lay tu xe_scripts/ (vao kho, ban dang chay travel 0.25), khong con tu du_lieu_0111/;
#   slam_dinh_vi_xuong_mat.yaml (travel 0.10 / heading 0.15) cho kieu mat, chon bang bien SLAM cua bat_xe.sh.
#
# TU NHAC: moi .py/.sh/.yaml/.xml trong xe_scripts/ chua co trong bang ma khong thuoc LUAT LOAI TRU
#   in "CHUA_CO_TRONG_DANH_SACH <ten>" (them tep moi vao bang, hoac vao luat loai tru neu co ly do).
# LUAT LOAI TRU (khong nhac): gia_lap/ (gia lap, khong chay tren Pi), du_lieu_*/ (du lieu do dac, chi cac tep
#   ghi ten trong bang moi duoc so), test_* (thu o may, khong len Pi), so_voi_pi.sh (chinh tep nay).
PI="project1@192.168.0.2"
DIR="$(cd "$(dirname "$0")" && pwd)"

BANG='bat_xe.sh|~/bat_xe.sh
bat_xe.sh.v3|?
kiem_xe.sh|~/kiem_xe.sh
chuyen.sh|~/chuyen.sh
ghi_bag.sh|~/ghi_bag.sh
chup_loi.sh|~/chup_loi.sh
bt_cho.xml|~/bt_cho.xml
bt_mat.xml|~/bt_mat.xml
nav2_params_cho.yaml|~/nav2_params_cho.yaml
nav2_params_mat.yaml|~/nav2_params_mat.yaml
nav2_params_mat_rpp.yaml|~/nav2_params_mat_rpp.yaml
cai_cau_noi.sh|~/cai_cau_noi.sh
cau_noi_xe.py|~/cau_noi/cau_noi_xe.py
chon_b_v4.html|~/cau_noi/chon_b.html
trang_v10.py|~/cau_noi/trang_v10.py
trang_v10.html|~/cau_noi/trang_v10.html
lai_xe_lidar.py|~/lai_xe_lidar.py
sep.py|~/sep.py
gui_duong.py|~/gui_duong.py
dung_xe.py|~/dung_xe.py|tuychon
tinh_tuyen.py|~/tinh_tuyen.py
lop_huong_dan.py|~/lop_huong_dan.py
hanh_vi_xe.yaml|~/hanh_vi_xe.yaml
hanh_vi.py|~/hanh_vi.py
kiem_hanh_vi.py|~/kiem_hanh_vi.py
gac_vat_la.py|~/gac_vat_la.py
slam_dinh_vi_xuong.yaml|~/slam_dinh_vi_xuong.yaml
slam_dinh_vi_xuong_mat.yaml|~/slam_dinh_vi_xuong_mat.yaml
du_lieu_0111/map_xuong_v3_pg.posegraph|~/map_xuong_v3_pg.posegraph
du_lieu_0111/map_xuong_v3_pg.data|~/map_xuong_v3_pg.data
du_lieu_0111/map_xuong_v3.yaml|~/map_xuong_v3.yaml
du_lieu_0111/map_xuong_v3.png|~/map_xuong_v3.png
HUONG_DAN_CAU_NOI.md|?
HUONG_DAN_THU_31_10.md|?
THIET_KE_LAI_XE_LIDAR.md|?
THIET_KE_SEP.md|?
VI_TRI_TREN_PI.md|?
BAO_CAO_SU_CO_31_10_CHIEU.md|?
LENH_CLAUDE_CODE.txt|?
mo_phong_he_thong.py|?
so_sanh_tuyen.py|?'

nhac() {
  local f ten
  while IFS= read -r f; do
    ten="${f#"$DIR"/}"
    case "$ten" in gia_lap/*|du_lieu_*/*|test_*|*/test_*|so_voi_pi.sh) continue ;; esac
    printf '%s\n' "$BANG" | cut -d'|' -f1 | grep -qxF -- "$ten" && continue
    echo "CHUA_CO_TRONG_DANH_SACH $ten"
  done < <(find "$DIR" -type f \( -name '*.py' -o -name '*.sh' -o -name '*.yaml' -o -name '*.xml' \) | sort)
}

if [ "$1" = "--nhac" ]; then nhac; exit 0; fi

x=0; y=0; chuaro=0; thieu=0
while IFS='|' read -r ten duong co; do
  [ -z "$ten" ] && continue
  if [ "$duong" = "?" ]; then echo "$ten | CHUA_RO"; chuaro=$((chuaro + 1)); continue; fi
  if [ ! -f "$DIR/$ten" ]; then echo "$ten | LOI_THIEU_TEP_TRONG_XE_SCRIPTS"; y=$((y + 1)); continue; fi
  local_md5="$(grep -a -v ACCESS "$DIR/$ten" | md5sum | awk '{print $1}')"
  pi_out="$(ssh -n -o BatchMode=yes -o ConnectTimeout=5 "$PI" \
    "if [ -f $duong ]; then grep -a -v ACCESS $duong | md5sum; else echo MISSING; fi")"
  rc=$?
  if [ $rc -ne 0 ]; then echo "$ten | LOI_SSH"; y=$((y + 1)); continue; fi
  pi_md5="$(echo "$pi_out" | awk '{print $1}')"
  if [ "$pi_md5" = "MISSING" ]; then
    if [ "$co" = "tuychon" ]; then echo "$ten | THIEU_TREN_PI"; thieu=$((thieu + 1))
    else echo "$ten | KHONG_CO_TREN_PI"; y=$((y + 1)); fi
  elif [ "$pi_md5" = "$local_md5" ]; then echo "$ten | GIONG"; x=$((x + 1)); y=$((y + 1))
  else echo "$ten | KHAC"; y=$((y + 1))
  fi
done <<< "$BANG"

echo "== GIONG $x/$y   (CHUA_RO $chuaro, THIEU_TREN_PI $thieu: khong tinh vao y) =="
nhac
