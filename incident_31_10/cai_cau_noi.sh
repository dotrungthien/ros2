#!/usr/bin/env bash
# cai_cau_noi.sh: cai cau noi dien thoai <-> Pi (viec 92). Chay 1 LAN tren Pi (dau nhac project1@), o thu muc co 4 tep:
#   cau_noi_xe.py  chon_b_v4.html  bat_xe.sh  (va ban do ~/map_xuong_v2.pgm/.yaml)
# Lenh:  bash cai_cau_noi.sh        (can sudo; hoi mat khau 1 lan neu can)
# Go bo: sudo systemctl disable --now cau_noi_xe   (quy trinh ssh cu khong bi anh huong)
# CHUA CHAY TREN Pi THAT.
set -u
DIR="$HOME/cau_noi"
ME="$(id -un)"
loi() { echo "LOI: $*"; exit 1; }

[ -f cau_noi_xe.py ]  || loi "thieu cau_noi_xe.py o thu muc hien tai ($(pwd))"
[ -f chon_b_v4.html ] || loi "thieu chon_b_v4.html o thu muc hien tai"
[ -f "$HOME/bat_xe.sh" ] || loi "thieu ~/bat_xe.sh (chep ban v3 len truoc)"
command -v python3 > /dev/null || loi "khong co python3"

mkdir -p "$DIR"
cp -f cau_noi_xe.py "$DIR/cau_noi_xe.py"
cp -f chon_b_v4.html "$DIR/chon_b.html"
# tep moi truong: doi NAV_PARAMS o day roi `sudo systemctl restart cau_noi_xe`
if [ ! -f "$DIR/cau_noi.env" ]; then
  cat > "$DIR/cau_noi.env" <<EOT
NAV_PARAMS=/home/$ME/nav2_params_cho.yaml
EOT
fi
echo "[1] Da chep vao $DIR. Tep moi truong: $DIR/cau_noi.env ($(cat "$DIR/cau_noi.env" | tr '\n' ' '))"

{ [ -f "$HOME/map_xuong_v2.pgm" ] || [ -f "$HOME/map_xuong_v2.png" ]; } && [ -f "$HOME/map_xuong_v2.yaml" ] \
  && echo "[2] Co ban do ~/map_xuong_v2 (.pgm hoac .png) va .yaml" \
  || echo "[2] CHUA CO ~/map_xuong_v2.png (hoac .pgm) va .yaml: trang se bao 'khong nap duoc ban do'. Chep len roi chay lai, hoac dat CAU_NOI_MAP trong $DIR/cau_noi.env (duong dan khong co duoi)."

if sudo -n true 2>/dev/null; then
  echo "[3] sudo khong can mat khau: chinh gio tu dien thoai se chay."
else
  echo "[3] CANH BAO: sudo can mat khau. bat_xe.sh se loi khi chinh gio (neu Pi lech gio > 30 s). Chinh gio bang tay, hoac them quyen cho timedatectl."
fi

sudo tee /etc/systemd/system/cau_noi_xe.service > /dev/null <<EOT
[Unit]
Description=Cau noi dien thoai - xe (nut ket noi)
After=network-online.target
Wants=network-online.target

[Service]
User=$ME
WorkingDirectory=$DIR
EnvironmentFile=-$DIR/cau_noi.env
ExecStart=/usr/bin/python3 $DIR/cau_noi_xe.py
Restart=always
RestartSec=3
# QUAN TRONG: khi cau noi khoi dong lai, KHONG giet he thong ROS dang chay
KillMode=process

[Install]
WantedBy=multi-user.target
EOT
sudo systemctl daemon-reload
sudo systemctl enable --now cau_noi_xe || loi "khong bat duoc dich vu (xem: journalctl -u cau_noi_xe -n 20)"
sleep 2
echo "[4] Trang thai dich vu: $(systemctl is-active cau_noi_xe)"
echo "[5] Tra loi tu cong 5001: $(curl -s -m 3 localhost:5001/api/b/status | head -c 90)"
echo "Mo tren dien thoai: http://$(hostname -I | awk '{print $1}'):5001/"
echo "PIN (khong bat buoc, nen dat): echo '<so>' > ~/.cau_noi_pin; chmod 600 ~/.cau_noi_pin"
