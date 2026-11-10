#!/usr/bin/env bash
# test_bat_kiem_cham.sh (10/11): kiem_xe.sh doc lai muc doc loi (ngan sach tong) + bat_xe.sh tu tat sau loi.
# Lenh gia trong PATH (ros2 cham / treo / rong, pgrep, pkill, fuser, rm, sudo, sleep), HOME tam: KHONG dung tien trinh that,
# KHONG xoa gi trong /dev/shm. Chay: bash xe_scripts/test_bat_kiem_cham.sh   (ma 0 = dat het; mat ~1 phut vi lenh treo 10 s)
DAY="$(cd "$(dirname "$0")" && pwd)"
T="$(mktemp -d)"
trap '/bin/rm -rf "$T"' EXIT
BIN="$T/bin"; mkdir -p "$BIN"
DAT=0; TONG=0
ok() { TONG=$((TONG + 1)); if eval "$2"; then DAT=$((DAT + 1)); echo "PASS  $1"; else echo "FAIL  $1"; echo "----- out:"; cat "$T/out"; echo "-----"; fi; }

# ---- ros2 gia: kich ban moi muc trong $T/kich/<ten> (moi dong = 1 lan goi: rong | treo | <gia tri>; het dong => dong cuoi) ----
cat > "$BIN/ros2" <<'EOF'
#!/usr/bin/env bash
K="$KICH"
buoc() {   # $1 = ten muc -> in dong thu n cua kich ban
  local f="$K/$1" n c
  [ -f "$f" ] || { echo ok; return; }
  n=$(cat "$f.n" 2>/dev/null || echo 0); n=$((n + 1)); echo "$n" > "$f.n"
  c=$(sed -n "${n}p" "$f"); [ -z "$c" ] && c=$(tail -1 "$f"); echo "$c"
}
case "$1 $2" in
  "param get")
    c=$(buoc "$4")
    case "$c" in treo) /bin/sleep 30 ;; rong) ;; ok) echo "String value is: x" ;; *) echo "Double value is: $c" ;; esac ;;
  "run tf2_ros")
    c=$(buoc tf)
    case "$c" in treo) /bin/sleep 30 ;; rong) ;; *) printf -- '- Translation: [0.000, 0.000, 0.000]\n- Rotation: in RPY (degree) [0.000, 0.000, 0.000]\n' ;; esac ;;
  "node list") printf '/a\n/b\n/c\n' ;;
  "launch my_bot")
    echo "$1 $2 $3" >> "$K/../ros2_launch.log"
    if [ -f "$K/../launch_song" ]; then touch "$K/../song"; fi
    if [ -f "$K/../launch_song_mai" ]; then touch "$K/../song_mai"; fi
    if [ -f "$K/../launch_ok" ]; then echo "보정 완료"; echo "Managed nodes are active"; fi ;;
  *) ;;
esac
EOF
# yaw_nguon tra 'gyro' (chuoi): kich ban rieng
sed -i 's/\*) echo "Double value is: $c" ;;/gyro) echo "String value is: gyro" ;; *) echo "Double value is: $c" ;;/' "$BIN/ros2"
# tien trinh "song" = co tep $T/song; pkill xoa no (tru khi co $T/song_mai = tat that bai)
cat > "$BIN/pgrep" <<'EOF'
#!/usr/bin/env bash
[ -f "$GIA/song" ] || [ -f "$GIA/song_mai" ] || exit 1
case " $* " in *" -a"*|*" -af "*) echo "99999 tien_trinh_gia" ;; *" -f "*) echo 99999 ;; esac
exit 0
EOF
cat > "$BIN/pkill" <<'EOF'
#!/usr/bin/env bash
/bin/rm -f "$GIA/song"; exit 0
EOF
cat > "$BIN/rm" <<'EOF'
#!/usr/bin/env bash
echo "rm $*" >> "$GIA/rm.log"; exit 0
EOF
for c in fuser sudo vcgencmd ps; do printf '#!/usr/bin/env bash\nexit 0\n' > "$BIN/$c"; done
printf '#!/usr/bin/env bash\nexec /bin/sleep 0.05\n' > "$BIN/sleep"      # moi sleep cua bat_xe ngan lai (doi T1 lan x 0,05 s)
chmod +x "$BIN"/*

# ================= kiem_xe.sh =================
kiem() {   # kiem ARGS... (bien KIEM_DOC_LAI_S / KIEM_DOC_LAI_TONG_S tu moi truong goi); ket qua $T/out, ma $T/rc
  local H="$T/h_kiem"; /bin/rm -rf "$H" "$T/kich"; mkdir -p "$H" "$T/kich"
  for f in "$T"/k_*; do [ -f "$f" ] && /bin/cp "$f" "$T/kich/${f##*/k_}"; done
  /bin/rm -f "$T"/k_*
  HOME="$H" KICH="$T/kich" GIA="$T" PATH="$BIN_KIEM:$PATH" bash "$DAY/kiem_xe.sh" 0 0 0 > "$T/out" 2>&1
  echo $? > "$T/rc"
}
# kiem_xe dung sleep that (cach DOC_LAI_S); chi ros2 gia
BIN_KIEM="$T/bin_kiem"; mkdir -p "$BIN_KIEM"; /bin/cp "$BIN/ros2" "$BIN/vcgencmd" "$BIN_KIEM/"
printf '#!/usr/bin/env bash\necho -n 0.25.24\n' > "$BIN_KIEM/dpkg-query"; chmod +x "$BIN_KIEM/dpkg-query"   # PC co the cai tf2 cu hon Pi
export KIEM_DOC_LAI_S=1
c() { grep -c "$1" "$T/out"; }

printf 'gyro\n' > "$T/k_yaw_nguon"; printf 'rong\nrong\n131.0\n' > "$T/k_gyro_scale"; printf -- '-1.0\n' > "$T/k_dau_yaw"
kiem
ok "doc loi 2 lan roi duoc => DAT, 2 dong DOC_LAI" '[ "$(c "^DOC_LAI    gyro_scale")" = 2 ] && grep -q "^DAT        gyro_scale = 131.0" "$T/out" && ! grep -q "KHONG DAT  gyro_scale" "$T/out"'

printf 'gyro\n' > "$T/k_yaw_nguon"; printf 'rong\n' > "$T/k_gyro_scale"; printf -- '-1.0\n' > "$T/k_dau_yaw"
kiem
ok "doc loi mai => 3 lan doc lai roi KHONG DAT khong doc duoc" '[ "$(c "^DOC_LAI    gyro_scale")" = 3 ] && grep -q "^KHONG DAT  gyro_scale: khong doc duoc" "$T/out" && [ "$(cat "$T/rc")" = 1 ]'

printf 'gyro\n' > "$T/k_yaw_nguon"; printf '100.0\n' > "$T/k_gyro_scale"; printf -- '-1.0\n' > "$T/k_dau_yaw"
kiem
ok "gia tri sai => KHONG DAT ngay, khong doc lai" '[ "$(c "^DOC_LAI")" = 0 ] && grep -q "^KHONG DAT  gyro_scale = 100.0 (mong 131.0)" "$T/out"'

printf 'treo\ngyro\n' > "$T/k_yaw_nguon"; printf '131.0\n' > "$T/k_gyro_scale"; printf -- '-1.0\n' > "$T/k_dau_yaw"; printf 'rong\nok\n' > "$T/k_tf"
kiem
ok "treo qua 10 s (HET_GIO) roi duoc; vi tri rong roi duoc => DAT" '[ "$(c "^DOC_LAI    yaw_nguon: lan 1/3")" = 1 ] && grep -q "^DAT        yaw_nguon = gyro" "$T/out" && [ "$(c "^DOC_LAI    vi tri: lan 1/3")" = 1 ] && grep -q "^DAT         vi tri" "$T/out" && [ "$(cat "$T/rc")" = 0 ]'

# ngan sach het giua chung: 5 s, cach 1 s; yaw / gyro / tf doc loi mai
printf 'rong\n' > "$T/k_yaw_nguon"; printf 'rong\n' > "$T/k_gyro_scale"; printf -- '-1.0\n' > "$T/k_dau_yaw"; printf 'rong\n' > "$T/k_tf"
t0=$SECONDS; KIEM_DOC_LAI_TONG_S=5 kiem; dt=$((SECONDS - t0))
ok "ngan sach 5 s het giua chung: HET_NGAN_SACH_DOC_LAI 1 lan, vi tri khong doc lai, van bao khong doc duoc" \
  '[ "$(c "^HET_NGAN_SACH_DOC_LAI")" = 1 ] && [ "$(c "^DOC_LAI    yaw_nguon")" -ge 1 ] && [ "$(c "^DOC_LAI ")" -le 3 ] && [ "$(c "^DOC_LAI    vi tri")" = 0 ] && grep -q "khong doc duoc vi tri" "$T/out" && grep -q "^KHONG DAT  gyro_scale: khong doc duoc" "$T/out" && [ "$dt" -le 5 ]'
# ngan sach voi lenh TREO: han lenh doc lai bi cat theo phan con lai => tong doc lai <= ngan sach
printf 'treo\n' > "$T/k_yaw_nguon"; printf '131.0\n' > "$T/k_gyro_scale"; printf -- '-1.0\n' > "$T/k_dau_yaw"
t0=$SECONDS; KIEM_DOC_LAI_TONG_S=8 kiem; dt=$((SECONDS - t0))
ok "ngan sach 8 s, lenh treo: 1 lan doc lai han 7 s, roi het ngan sach; ca lan chay <= 10 + 8 + 2 s" \
  'grep -q "^DOC_LAI    yaw_nguon: lan 1/3 sau 1 s (han 7 s, ngan sach con 8 s)" "$T/out" && [ "$(c "^HET_NGAN_SACH_DOC_LAI")" = 1 ] && [ "$dt" -le 20 ] && grep -q "^KHONG DAT  yaw_nguon: ros2 param get qua 10 s" "$T/out"'
echo "   (lan chay ngan sach 8 s voi lenh treo mat $dt s)"

# ================= bat_xe.sh =================
bat() {   # bat [ten_tep_dieu_kien...]: HOME tam, ma $T/rc, ra $T/out (khong in dong nao chua ACCESS)
  local H="$T/h_bat"; /bin/rm -rf "$H" "$T/kich" "$T"/song "$T"/song_mai "$T"/launch_* "$T/rm.log" "$T/ros2_launch.log"
  mkdir -p "$H/ros2_ws/install" "$T/kich"; : > "$H/ros2_ws/install/setup.bash"
  for f in "$@"; do touch "$T/$f"; done
  HOME="$H" KICH="$T/kich" GIA="$T" PATH="$BIN:$PATH" T1=40 T2=40 timeout 60 bash "$DAY/bat_xe.sh" 2>&1 | grep -v ACCESS > "$T/out"
  echo "${PIPESTATUS[0]}" > "$T/rc"
}
bat
ok "launch chinh thoat som (ma 1) => TU_TAT_SAU_LOI 1 lan, tat sach, giu ma 1" \
  '[ "$(cat "$T/rc")" = 1 ] && [ "$(c "^TU_TAT_SAU_LOI (ma 1)")" = 1 ] && grep -q "^KIEM_XONG: da tat sach." "$T/out" && grep -q "^LOI: launch chinh da thoat som." "$T/out"'

bat launch_song_mai
ok "tat tu loi (con tien trinh): KHONG lap, TU_TAT_SAU_LOI 1 lan + 'tat chua sach', ket thuc ma 1 trong han 60 s" \
  '[ "$(cat "$T/rc")" = 1 ] && [ "$(c "^TU_TAT_SAU_LOI (ma 1)")" = 1 ] && [ "$(c "^TU_TAT_SAU_LOI: tat chua sach")" = 1 ] && [ "$(c "^CON_CHAY: van con")" = 1 ]'

# Ctrl-C (INT) luc dang doi B1 (launch con song): ma 130, tu tat
H="$T/h_bat"; /bin/rm -rf "$H" "$T"/song "$T"/song_mai "$T"/launch_*; mkdir -p "$H/ros2_ws/install"; : > "$H/ros2_ws/install/setup.bash"; touch "$T/launch_song"
( HOME="$H" KICH="$T/kich" GIA="$T" PATH="$BIN:$PATH" T1=200 exec bash "$DAY/bat_xe.sh" > "$T/out_raw" 2>&1 ) &
P=$!; /bin/sleep 2; kill -INT "$P"; wait "$P"; echo $? > "$T/rc"; grep -v ACCESS "$T/out_raw" > "$T/out"
ok "Ctrl-C (INT) luc dang bat => TU_TAT_SAU_LOI (ma 130), tat sach, thoat 130" \
  '[ "$(cat "$T/rc")" = 130 ] && [ "$(c "^TU_TAT_SAU_LOI (ma 130)")" = 1 ] && grep -q "^KIEM_XONG: da tat sach." "$T/out"'

bat launch_ok
ok "ma 2 (thieu ~/kiem_xe.sh): KHONG tu tat, in CON_CHAY_DE_DOC_TAY" \
  '[ "$(cat "$T/rc")" = 2 ] && [ "$(c "^CON_CHAY_DE_DOC_TAY: tat bang: bash ~/bat_xe.sh tat")" = 1 ] && [ "$(c "TU_TAT_SAU_LOI")" = 0 ] && ! grep -q "goc_dang_chay" "$T/rm.log" 2>/dev/null'

# D(1): chu moi (chi o log BAT cua cau_noi) khong khop DONG_BAT (dong ket bat) => trang / nut giu dong ket cu (LOI..., CHUA SAN SANG...).
# DONG_TAT chi doc log nut TAT (bat_xe.sh tat thoat truoc trap, khong in cac chu nay); DONG_CHUYEN chi doc log chuyen.
MAU=$(python3 -c "
import re; s=open('$DAY/cau_noi_xe.py').read()
print('|'.join(re.search(r'%s = re.compile\\(r\"(.+?)\"\\)' % k, s).group(1) for k in ('DONG_BAT',)))")
printf '%s\n' 'DOC_LAI    gyro_scale: lan 1/3 sau 3 s (han 10 s, ngan sach con 30 s)' 'HET_NGAN_SACH_DOC_LAI: da dung 31/30 s; muc con lai khong doc lai (vi tri)' \
  'TU_TAT_SAU_LOI (ma 1): bash /home/project1/bat_xe.sh tat' 'TU_TAT_SAU_LOI: tat chua sach (ma 1), xem dong CON_CHAY o tren; khong thu lai' \
  'CON_CHAY_DE_DOC_TAY: tat bang: bash ~/bat_xe.sh tat' > "$T/out"
ok "D(1): DOC_LAI / HET_NGAN_SACH_DOC_LAI / TU_TAT_SAU_LOI / CON_CHAY_DE_DOC_TAY khong khop DONG_BAT cua cau_noi" '[ -n "$MAU" ] && ! grep -Eq "$MAU" "$T/out"'

echo "TONG: $DAT/$TONG dat"
[ "$DAT" -eq "$TONG" ]
