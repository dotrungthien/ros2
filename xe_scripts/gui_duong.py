#!/usr/bin/env python3
"""gui_duong (viec 109a, soan 02/11/2026 o truong, CHUA CHAY TREN XE): gui DUONG DAY cua tinh_tuyen.py qua /follow_path.

Tu 02/11 18:09 duong gui di la DUONG DOAN THANG da chia day (tinh_tuyen.doan_thang): dinh re dat cho du cho xoay,
RPP xoay tai cho khi lech > 20 do (rotate_to_heading_min_angle 0.35) thay vi cua rong.
Ly do (xe 02/11 17:03): bang xs gui 6 diem qua /navigate_to_pose; moi diem Nav2 nhan "da toi" khi con cach 14-15 cm
(xy_goal_tolerance 0,15) nen sai so don lai, xe troi 14 cm ve K4 roi ket. Duong day la MOT dich: chi diem cuoi co dung sai.

Mot tien trinh rclpy, KHONG goi ros2 CLI. Chay (chuyen.sh BANG=day goi):  python3 ~/gui_duong.py di_B|ve_A [tuyen_dau 1|2]
  1. KIEM: tf map->base_footprint moi (<= 1 s) va /global_costmap/costmap da co, xe khong dung trong o do. Sai => SAI_VI_TRI (ma 3).
  2. Moi tuyen (thu tu nhu chuyen.sh), toi da LUOT lan (18:09: phu lai XOAY xong => huy duong cu, tinh lai tu cho xe, ghi
     TINH_LAI, KHONG tinh la mot lan hong; toi da LUOT lan tinh lai moi tuyen): hoi sep (~/sep_lenh: DUNG => dung; CHO => cho SEP_CHO_S s, toi da 2 lan),
     cho khi phu lai dang RA / XOAY / THOAT (/rosout cua lai_xe_lidar) hoac sep dang giao thoat (~/sep_thoat moi);
     tinh duong tu vi tri HIEN TAI tren costmap HIEN TAI (tinh_tuyen.tinh_duong), phat /duong_day, gui FollowPath.
     Trong luc chay: ~/sep_lenh DUNG => huy dich, DUNG_SEP (ma 4); /duong_day cap nhat (phan con lai) 2 lan/giay.
     SUCCEEDED ma xe cach dich <= TOI_SAI_SO m => DEN_NOI (ma 0). Hong => cho (>= CHO_HONG s, toi da gap doi, ton trong
     CHO/DUNG va phu lai) roi tinh lai tu vi tri hien tai. Het LUOT lan => tuyen kia. Ca hai hong => HET_DUONG (ma 1).
  CSV: ~/ket_qua_chuyen.csv cung cot va cung chu voi chuyen.sh (KIEM_DAT, SUCCEEDED, KHONG_DAT, DEN_NOI, HET_DUONG,
  SAI_VI_TRI, DUNG_SEP) va NGAT (bi ngat, dong ket thuc chuyen); cot 2 = "<huong>_day".
  Ma thoat: 0 den noi, 1 het duong, 2 sai doi so, 3 sai vi tri, 4 sep bao dung, 130 bi ngat (NGAT).
  06/11 viec 121: 5 = loi khac (LOI_GUI_DUONG, khong bi ngat); van huy dich dang chay truoc khi thoat.
Bien moi truong: LUOT (3), CHO_HONG (30 s), TO (130 s/lan gui), TOI_SAI_SO (0,20 m); cung ten voi chuyen.sh: SEP_CHO_S (10 s),
  SEP_TUOI_MAX (3 s). Phu lai lai mai thi tong thoi gian cho giua hai lan gui toi da 3 x CHO_HONG.
04/11 lop 2: truoc MOI lan gui ghi ~/gui_duong_luot (SAP_GUI); sep tra loi trong ~/sep_thoat: KHONG_CAN => gui; RA_LUI (06/11: hoac RA_TIEN) => cho phu
  lai lui thang va bao RA_XONG (toi da 12 s); sep khong tra loi 2 s / het 12 s => in RA_HET_GIO roi gui nhu cu.
Dung khan: TAT CONG TAC MOTOR. Ctrl+C / SIGTERM: ghi HUY vao ~/gui_duong_luot (phu lai dung lui lop 2),
  huy dich dang chay (in HUY_DICH_KHI_NGAT ...), ghi dong NGAT, thoat 130.
Phan thuan (khong ROS) o nua tren, thu bang test_gui_duong.py.

08/11 bo 1e "mat lai" (thiet ke ban 3, CHUA CHAY TREN XE): ~/goc_dang_chay = "mat" (bat_xe.sh MAT_LAI=true ghi) => KIEU MAT
(chay_mat); khac => kieu cu y het 82ab5d1 (chay). Dong dau in "KIEU gui_duong: mat|cu".
  Kieu mat: dau chuyen /chon_tuyen "0", cho 2 s; KIEM nhu cu (tf <= 1 s, o >= 98) => SAI_VI_TRI; moi tuyen MOT lan:
  hoi sep (DUNG / CHO 10 s x 2; gui lai sau khi bo khe: bo qua CHO vi PLAN_* / ABORT_NHANH) -> /chon_tuyen "r x y" -> cho 2 s
  -> "-- tuyen r, lan 1/1: ..." -> navigate_to_pose. Dong ho DUY NHAT (cay bt_mat.xml khong tu bo cuoc): "tien" = khoang
  cach thang xe->dich (tf) HOAC distance_remaining (bo so 0) giam >= TIEN_MIN so voi gia tri nho nhat tai moc; bat dau tu
  feedback dau tien; CHO_KHE s khong tien => bo khe (CHO_VAT_30s); CHO_KHE s khong co feedback => bo khe (KHONG_FEEDBACK).
  Dung >= 5 s: in "CHO_VAT: n/30 s" moi 5 s; tien lai sau do: in "DI_TIEP"; so recoveries tang: in 1 dong.
  Bo khe: huy dich, 1 dong ~/kinh_nghiem.csv (gio,x,y,huong,tuyen,ly_do), chup ~/chup/, "khong den duoc (CHO_VAT_30s, ...)",
  KHONG_DAT, tuyen kia. Moi loi ra (0, 1, 3, 4, 5, 130): /chon_tuyen "0"; loi ra != 0 chup ~/chup/<HHMMSS>_<huong>.txt;
  noi moi dong da in vao ~/log_gui_duong.txt; CSV + ma thoat nhu cu.
  Bien moi: CHO_KHE (30 s), TIEN_MIN (0,10 m).
  08/11 buoc 2c: truoc khi gui tuyen kia: Nav2 BackUp (/backup) 0,35 m o 0,10 m/s, in "LUI_RA ..."; sep DUNG luc lui => DUNG_SEP.
  behavior_server ra cmd_vel_nav_raw -> velocity_smoother -> collision_monitor (nhu lenh controller).
  09/11 buoc 3 (CHUA CHAY TREN XE): #2 GAN_DICH: xe cach dich < GAN_DICH_M (0,60 m; Thien chot 09/11, truoc 0,40) ma khong tien GAN_DICH_S (10 s) => huy dich,
  dung yen CHO_DICH (60 s, in "CHO_VAT: n/60 s"), gui lai 1 lan ("-- tuyen r, lan 2/2"); 10 s van khong tien => HET_DUONG,
  KHONG doi khe. #3 ~/vi_tri_xe moi 1 s (noi_dung_vi_tri). #5 ~/kinh_nghiem.csv ly_do KET_CO_DUONG (co /plan trong CHO_KHE s
  ma khong tien) | VAT_CHAN (planner khong ra duong). #6 "LUI_RA: bt|gui <m>" khi bat dau lui (bt: cay bt_mat goi /backup,
  thay qua /backup/_action/status; gui: truoc /backup cua minh), "   LUI_XONG: bt|gui => KQ" khi xong. #7 bi ngat: huy ca
  /backup dang chay. (09/11: behavior_server ra thang /cmd_vel_avoid khi mat_lai:=true, buoc 2g.)
09/11 20:33 VUNG QUANH XE (gac_vat_la.py): LiDAR lai, ban do chi nhac (LECH). MEM khi doi.
09/11 23:08 SEP LiDAR MOT NGUOI LAI (Thien chot 21:54-23:08, CHUA CHAY TREN XE; sep_lidar.py): kieu mat thay toan bo luong cu
  (cong sach 2 s, dong ho 30 s, LUI_RA thang, GAN_DICH 60 s, hoi sep CHO / DUNG). gui_duong la nguoi quyet DUY NHAT; sep.py
  (/sep/trang_thai), ban do, ~/kinh_nghiem.csv, thoi_quen chi DUA TIN; KHONG phat toc do (moi dong tac la action Nav2).
  Chan duy nhat = ban nang: vung do (VAT truoc <= 0,10 / hong <= 0,06 => huy dich, dung im; mui 0,07 = collision_monitor;
  duoi 0,06 = BackUp), 정지, cong tac motor. Nhac doi: VAT CHAC (LiDAR VAT, khong LECH) truoc mui <= 0,14 => dung yen,
  sach 2 s => di, toi da cho_vat_s 30 s. Bi chan (khong tien 10 s / het doi 30 s / kep) => THOAT: cho chat tung dong tac
  (drive_on_heading / backup / spin, kiem khe theo footprint suot quang quet, huy theo Q3), cho thoang diem thoat sector
  (navigate_to_pose 15 s). Luot: xa dich 3, gan dich 2 hoac 100 s => CHO_NGUOI; thoat hong 3 lan lien / kep moi huong =>
  CHO_NGUOI. Vi tri khong tin (sep) => van thoat bang LiDAR + odom, chi giao dich khi vi tri tin; qua 60 s => CHO_VI_TRI.
  Dong quyet: "SEP_LIDAR: DI | DOI_VAT | THOAT | KEP | CHO_VI_TRI ..." (trang dong 두뇌).
"""
import math
import os
import re
import signal
import sys
import time

import gac_vat_la as GV  # 09/11 dung im ban 2: gac vat la (phan thuan)
import hanh_vi as HV
import sep_lidar as SL     # 09/11 23:08 sep LiDAR mot nguoi lai (phan thuan)      # 09/11 nao xe viec 2: moi so hanh vi lay tu ~/hanh_vi_xe.yaml (mac dinh = so 2607fee)
import tinh_tuyen as TT

DICH = {'di_B': (TT.B, 0.0), 've_A': (TT.A, None)}     # (diem, goc cuoi): di_B giu 0 do nhu bang tt/xs; ve_A theo tiep tuyen
COT_CSV = 'gio,huong,tuyen,diem,lan,ket_qua,giay,lech_kiem_m,lech_costmap_m,tuoi_costmap_s'
PHU_LAI_BAN = ('RA_TIEN', 'RA_LUI', 'XOAY_P')           # ten trang thai lai_xe_lidar dang tu lai banh (cong moi ten THOAT_*)


def cau_hinh(env=os.environ):
    """09/11 nao xe: mac dinh tu hanh_vi (ten thoi_quen trong hanh_vi_xe.yaml); bien moi truong van ghi de nhu cu."""
    f = lambda k, d: float(env.get(k, d))
    h = HV.so
    return dict(luot=int(env.get('LUOT', h('luot_lan_gui'))), cho_hong=f('CHO_HONG', h('cho_hong_s')), to=f('TO', h('qua_gio_lan_gui_s')),
                cho_sep=f('SEP_CHO_S', h('cho_sep_s')),
                toi_sai_so=f('TOI_SAI_SO', h('toi_sai_so_m')), sep_tuoi=f('SEP_TUOI_MAX', h('sep_lenh_tuoi_max_s')), tf_tuoi=h('tf_tuoi_max_s'),
                lop2_cho=h('lop2_cho_s'), lop2_tra=h('lop2_tra_s'),      # 04/11 lop 2: cho RA_XONG toi da / cho sep tra loi SAP_GUI toi da (s)
                # kieu mat: muc tien toi thieu (m), cho sau /chon_tuyen (s), nhip in CHO_VAT (s)
                tien_min=f('TIEN_MIN', h('tien_toi_thieu_m')), cho_chon=h('cho_chon_tuyen_s'), nhip_cho=h('nhip_in_cho_s'),
                gan_dich_m=f('GAN_DICH_M', h('gan_dich_m')),
                so_luot=int(env.get('SO_LUOT_CHAN', h('so_luot_chan'))),  # 09/11 20:33: so luot bi chan (xa dich) truoc CHO_NGUOI
                cho_lai=f('CHO_LAI', h('cho_lai_s')),    # 09/11 18:47 (i): CHO_NGUOI, tu thu lai 1 luot sau bay lau (<= 0: HET_DUONG, ca thu cu)
                # 09/11 23:08 sep LiDAR (Thien chot 21:54-23:08): khong tien bao lau => bi chan (s); doi vat / vung do toi da (s);
                # gan dich: so luot / han (s); cho vi tri bao lau thi bao nguoi (s); toc do dong tac thoat (m/s); so dong tac nho
                # moi lan thoat; thoat hong lien tiep => CHO_NGUOI; han di toi diem thoat (s)
                khong_tien=h('khong_tien_s'), cho_vat=h('cho_vat_s'), so_luot_gan=h('so_luot_gan_dich'), han_gan=h('han_gan_dich_s'),
                cho_vi_tri=h('cho_vi_tri_s'), v_thoat=h('toc_do_thoat_mps'), thoat_buoc=h('thoat_buoc_max'),
                thoat_hong=h('thoat_hong_max'), thoat_han=h('thoat_han_s'),
                # 10/11 muc 6 + 7: diem dan ve_A, diem vuot (huong dan co loi ra)
                diem_dan=h('ve_A_diem_dan'), diem_dan_han=h('diem_dan_han_s'), diem_vuot=h('diem_vuot'), vuot_han=h('diem_vuot_han_s'))


# ---------- 08/11 bo 1e: kieu mat (phan thuan) ----------
TEP_GOC = os.path.expanduser('~/goc_dang_chay')
COT_KINH_NGHIEM = HV.COT_KINH_NGHIEM     # 09/11 nao xe viec 4: 'gio,chuyen,x,y,huong,su_co,cach,ket_qua,ly_do' (hanh_vi.doc_kinh_nghiem doc ca dong cu)
KET_CO_DUONG, VAT_CHAN = 'KET_CO_DUONG', 'VAT_CHAN'      # 09/11 buoc 3 #5: ly_do ~/kinh_nghiem.csv (lop_huong_dan chi dem KET_CO_DUONG)


def doc_goc(path=TEP_GOC):
    """~/goc_dang_chay (bat_xe.sh ghi MOI lan bat: 'mat' | 'cu'): 'mat' neu dong dau la mat; khac / khong co tep => 'cu'."""
    try:
        with open(path) as f:
            return 'mat' if f.readline().strip() == 'mat' else 'cu'
    except OSError:
        return 'cu'


def ghi_kinh_nghiem(path, dong):
    """Them 1 dong ~/kinh_nghiem.csv. dong: dict theo COT_KINH_NGHIEM (x, y la so hoac None). 09/11 nao xe viec 4: dong dau
    cuoi cung cua tep khac COT_KINH_NGHIEM (tep cu 'gio,x,y,huong,tuyen,ly_do' hoac chua co tep) => ghi them dong dau moi truoc
    (KHONG sua / xoa dong cu; hanh_vi.doc_kinh_nghiem doi cot theo dong dau gan nhat)."""
    dau = None
    try:
        with open(path) as f:
            for l in f:
                if l.startswith('gio,'):
                    dau = l.strip()
    except OSError:
        pass
    so = lambda v: '' if v is None else '%.3f' % v
    with open(path, 'a') as f:
        if dau != COT_KINH_NGHIEM:
            f.write(COT_KINH_NGHIEM + '\n')
        f.write(','.join(so(dong.get(c)) if c in ('x', 'y') else str(dong.get(c, '-')).replace(',', ';')
                         for c in COT_KINH_NGHIEM.split(',')) + '\n')


# ---------- 09/11 gia X (Thien chot 17:50-17:57): X = cho KET_CO_DUONG chua DA_GIAI = cho can tim cach giai, KHONG phai vung ne ----------
DA_GIAI = 'DA_GIAI'      # ly_do (va su_co) dong ~/kinh_nghiem.csv: chuyen DEN_NOI di qua X ma khong co su co gan X


def _xy(d):
    try:
        return float(d['x']), float(d['y'])
    except (KeyError, TypeError, ValueError):
        return None


def diem_x(dong, r, giay_mac_dinh):
    """(b) dong = hanh_vi.doc_kinh_nghiem(...). Dong ly_do KET_CO_DUONG gop lan luot vao X dau tien co tam cach <= r (tam = trung binh);
    giay X = trung binh cho=N (cot cach) cac dong co so, khong co thi giay_mac_dinh. X co dong DA_GIAI cach tam <= r SAU dong
    KET_CO_DUONG cuoi cung cua no => da giai, bo. -> [(x, y, giay)]."""
    cum = []
    for d in dong:
        p = _xy(d)
        if p is None or d.get('ly_do') != KET_CO_DUONG:
            continue
        c = next((c for c in cum if math.hypot(p[0] - c['x'], p[1] - c['y']) <= r), None)
        if c is None:
            c = dict(diem=[], giay=[], cuoi=0)
            cum.append(c)
        c['diem'].append(p)
        c['x'], c['y'] = sum(a for a, _ in c['diem']) / len(c['diem']), sum(b for _, b in c['diem']) / len(c['diem'])
        m = re.match(r'cho=([0-9.]+)$', d.get('cach') or '')
        if m:
            c['giay'].append(float(m.group(1)))
        c['cuoi'] = max(c['cuoi'], d.get('so_dong', 0))
    giai = [(_xy(d), d.get('so_dong', 0)) for d in dong if d.get('ly_do') == DA_GIAI and _xy(d) is not None]
    return [(c['x'], c['y'], sum(c['giay']) / len(c['giay']) if c['giay'] else giay_mac_dinh) for c in cum
            if not any(n > c['cuoi'] and math.hypot(q[0] - c['x'], q[1] - c['y']) <= r for q, n in giai)]


def kc_duong(p, duong):
    """Khoang cach (m) tu diem p toi duong gap khuc duong [(x, y), ...]."""
    if len(duong) == 1:
        return math.hypot(p[0] - duong[0][0], p[1] - duong[0][1])
    best = 1e9
    for (ax, ay), (bx, by) in zip(duong, duong[1:]):
        L2 = (bx - ax) ** 2 + (by - ay) ** 2
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L2))
        best = min(best, math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay)))
    return best


def gia_tuyen(duong, xs, r, v):
    """(c) (dai m, tong giay X cach duong <= r, gia = dai + v x giay)."""
    dai = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(duong, duong[1:]))
    giay = sum(g for x, y, g in xs if kc_duong((x, y), duong) <= r)
    return dai, giay, dai + v * giay




def cach_kinh_nghiem(dong, p, r):
    """09/11 18:44 ②b (kinh nghiem CHI LA HUONG DAN): dong ket_qua DUOC (khong tinh DA_GIAI) gan p nhat trong r m
    -> (su_co, cach, khoang cach) | None. 09/11 23:08: thoat dung cot cach (dong tac) cua dong nay, thu TRUOC neu LiDAR kiem kha thi."""
    if p is None:
        return None
    best = None
    for d in dong:
        q = _xy(d)
        if q is None or d.get('ket_qua') != 'DUOC' or d.get('ly_do') == DA_GIAI:
            continue
        k = math.hypot(q[0] - p[0], q[1] - p[1])
        if k <= r and (best is None or k < best[2]):
            best = (d.get('su_co'), d.get('cach'), k)
    return best


def thu_tu_gia_x(io, thu_tu, pose, ban, dich, yaw_cuoi):
    """(c) Xep tuyen tang dan theo gia (hoa giu thu_tu = tuyen_dau truoc). gia_x_bat false / khong co X => thu_tu, khong log.
    tinh_duong loi hoac rong o mot tuyen => bo gia X chuyen nay, giu thu_tu, log 1 dong."""
    if not HV.so('gia_x_bat'):
        return thu_tu
    r, v = HV.so('gia_x_r_m'), HV.so('gia_x_v_ms')
    xs = diem_x(io.doc_kinh_nghiem(), r, HV.so('gia_x_giay_mac_dinh'))
    if not xs:
        return thu_tu
    gia = {}
    for t in thu_tu:
        try:
            duong = [(x, y) for x, y, _ in TT.tinh_duong(ban, (pose[0], pose[1]), dich, t, yaw_cuoi=yaw_cuoi)]
            ly = 'duong rong' if not duong else ''
        except Exception as e:                   # noqa: BLE001 (KhongCoDuong hoac loi khac: bo gia X, giu thu tu cu)
            duong, ly = [], '%s: %s' % (e.__class__.__name__, str(e)[:80])
        if not duong:
            io.bao('GIA_X: bo (tuyen %d %s); thu tu (%d,%d)' % (t, ly, thu_tu[0], thu_tu[1]))
            return thu_tu
        gia[t] = gia_tuyen(duong, xs, r, v)
    moi = tuple(sorted(thu_tu, key=lambda t: gia[t][2]))
    io.bao('GIA_X: %s; thu tu (%d,%d)' % ('; '.join('t%d %.2fm + %gs' % (t, gia[t][0], gia[t][1]) for t in (1, 2)), moi[0], moi[1]))
    return moi


def ghi_da_giai(io, huong, chuyen):
    """(d) Chuyen DEN_NOI: moi X chua giai ma vet xe that (io.vet_chuyen) di cach X <= r va chuyen nay (cot chuyen) khong co dong
    su co nao cach X <= r => them 1 dong DA_GIAI (khong xoa dong nao). gia_x_bat false => khong lam gi."""
    if not HV.so('gia_x_bat'):
        return
    r = HV.so('gia_x_r_m')
    dong = io.doc_kinh_nghiem()
    vet = io.vet_chuyen()
    su_co = [_xy(d) for d in dong if d.get('chuyen') == chuyen and d.get('ly_do') != DA_GIAI and _xy(d) is not None]
    for x, y, _ in diem_x(dong, r, HV.so('gia_x_giay_mac_dinh')):
        if any(math.hypot(a - x, b - y) <= r for a, b in vet) and not any(math.hypot(a - x, b - y) <= r for a, b in su_co):
            io.ghi_kinh_nghiem(dict(chuyen=chuyen, x=x, y=y, huong=huong, su_co=DA_GIAI, cach='-', ket_qua='DUOC', ly_do=DA_GIAI))
            io.bao('GIA_X: DA_GIAI (%.2f; %.2f)' % (x, y))


class SoKinhNghiem:
    """09/11 nao xe viec 4: ghi ~/kinh_nghiem.csv moi su co cua mot chuyen kieu mat (su_co, cach, ket_qua, ly_do).
    ngay(): ghi ngay. doi(): cho ket qua: trong quan_sat s sau do xe TIEN (DongHo) hoac toi dich => DUOC; het quan_sat s / dich
    hong => HONG. ket(duoc): chot moi dong cho. 09/11 23:08 sep LiDAR: su_co THOAT (cach = dong tac 'tien=0.10' / 'xoay=+60' /
    'diem=0.40'), LUOT_CHAN (cach = ly do bi chan), CHO_VAT_LA (cach 'doi')."""

    def __init__(self, io, huong, chuyen, quan_sat, o_luoi):
        self.io, self.huong, self.chuyen, self.qs, self.o_luoi = io, huong, chuyen, quan_sat, o_luoi
        self.cho = []
        self.mu = False                        # gac dang mu (CANH BAO 1 lan moi dot)

    def _ghi(self, su_co, cach, ket_qua, ly_do, p):
        self.io.ghi_kinh_nghiem(dict(chuyen=self.chuyen, x=None if p is None else p[0], y=None if p is None else p[1],
                                     huong=self.huong, su_co=su_co, cach=cach, ket_qua=ket_qua, ly_do=ly_do))

    def ngay(self, su_co, cach, ket_qua, ly_do, p):
        self._ghi(su_co, cach, ket_qua, ly_do, p)

    def doi(self, su_co, cach, p, ly_do='-'):
        self.cho.append((su_co, cach, ly_do, p, self.io.now()))

    def _chot(self, dk, ket_qua):
        con = []
        for e in self.cho:
            if dk(e):
                self._ghi(e[0], e[1], ket_qua, e[2], e[3])
            else:
                con.append(e)
        self.cho = con

    def tien(self):
        self._chot(lambda e: True, 'DUOC')

    def nhip(self):
        t = self.io.now()
        self._chot(lambda e: t - e[4] >= self.qs, 'HONG')

    def ket(self, duoc):
        self._chot(lambda e: True, 'DUOC' if duoc else 'HONG')

    def het_chuyen(self, duoc):
        """Het chuyen: dong con cho => chot."""
        self.ket(duoc)


def noi_dung_chup(ly, gio, pose, fb, plan, cm):
    """Tep chup ~/chup/<HHMMSS>_<huong>.txt (bai 514): pose, feedback cuoi, /plan cuoi, costmap toan cuc (o >= 50: x y gia_tri).
    pose (x, y, yaw) | None; fb (distance_remaining, number_of_recoveries) | None; plan [(x, y)]; cm (w, h, res, ox, oy, data) | None."""
    d = ['# chup gui_duong kieu mat %s: %s' % (gio, ly),
         'pose %s' % ('khong co tf moi' if pose is None else '%.3f %.3f %.1f' % (pose[0], pose[1], math.degrees(pose[2])))]
    d.append('feedback %s' % ('chua co' if fb is None else 'distance_remaining %.3f number_of_recoveries %d' % (fb[0], fb[1])))
    d.append('plan %d diem' % len(plan or ()))
    d += ['%.3f %.3f' % (x, y) for x, y in (plan or ())]
    if cm is None:
        d.append('costmap chua co')
    else:
        w, h, res, ox, oy, data = cm
        o = [(i, j, v) for j in range(h) for i in range(w) for v in (data[j * w + i],) if v >= 50]
        d.append('costmap %dx%d o %.3f m goc (%.3f; %.3f): %d o >= 50 (x y gia_tri)' % (w, h, res, ox, oy, len(o)))
        d += ['%.3f %.3f %d' % (ox + (i + 0.5) * res, oy + (j + 0.5) * res, v) for i, j, v in o]
    return '\n'.join(d) + '\n'


def ly_do_kinh_nghiem(tuoi_plan, cua_so):
    """09/11 buoc 3 #5: co /plan (cua dich nay) trong `cua_so` s cuoi => KET_CO_DUONG (co duong ma khong tien);
    khong => VAT_CHAN (planner khong ra duong: Nav2 1.1.20 chi phat /plan khi tinh duoc)."""
    return KET_CO_DUONG if tuoi_plan is not None and tuoi_plan <= cua_so else VAT_CHAN


def noi_dung_vi_tri(t_epoch, pose, plan, n_max=100):
    """09/11 buoc 3 #3: ~/vi_tri_xe (gui_duong kieu mat ghi moi 1 s; trang_v10.doc_vi_tri_xe doc).
    Dong 1: '<epoch> <x> <y> <goc_do>' ('-' khi khong co tf moi). Dong 2: 'plan <n> x1 y1 x2 y2 ...' (/plan cua dich dang
    chay, toi da n_max diem cach deu, giu diem cuoi; khong co dich => 'plan 0')."""
    d1 = '%.2f %s' % (t_epoch, '- - -' if pose is None else '%.3f %.3f %.1f' % (pose[0], pose[1], math.degrees(pose[2])))
    plan = list(plan or ())
    if len(plan) > n_max:
        b = (len(plan) - 1) / (n_max - 1)
        plan = [plan[int(round(k * b))] for k in range(n_max)]
    return d1 + '\n' + ' '.join(['plan', str(len(plan))] + ['%.3f %.3f' % (x, y) for x, y in plan]) + '\n'


class DongHo:
    """Dong ho duy nhat cua kieu mat (thiet ke ban 3 muc 3b #2). cap_nhat(t, d_tf, d_fb) -> (trang_thai, ly_do, giay_dung):
    'CHUA' (chua co feedback), 'TIEN' (vua tien >= tien_min: dat lai moc), 'DI' (chua tien du), 'BO_KHE' (ly do CHO_VAT_30s |
    KHONG_FEEDBACK). Moc so sanh = gia tri nho nhat luc dat moc (tf va distance_remaining rieng); dich lui / tinh lai duong dai
    hon khong lam lui moc; distance_remaining = 0 (chua co duong) bo qua."""

    def __init__(self, t_gui, cho_khe, tien_min):
        self.t_gui, self.cho_khe, self.tien_min = t_gui, cho_khe, tien_min
        self.t_moc, self.moc_tf, self.moc_fb = None, None, None

    def cap_nhat(self, t, d_tf, d_fb):
        if self.t_moc is None:
            if d_fb is None:
                da = t - self.t_gui
                return ('BO_KHE', 'KHONG_FEEDBACK', da) if da >= self.cho_khe else ('CHUA', None, da)
            self.t_moc = t
        tien = False
        if d_tf is not None:
            if self.moc_tf is None:
                self.moc_tf = d_tf
            elif d_tf <= self.moc_tf - self.tien_min:
                tien = True
        if d_fb is not None and d_fb > 0:
            if self.moc_fb is None:
                self.moc_fb = d_fb
            elif d_fb <= self.moc_fb - self.tien_min:
                tien = True
        if tien:
            self.t_moc = t
            if d_tf is not None:
                self.moc_tf = min(self.moc_tf, d_tf)
            if d_fb is not None and d_fb > 0:
                self.moc_fb = min(self.moc_fb, d_fb)
            return 'TIEN', None, 0.0
        dung = t - self.t_moc
        if dung >= self.cho_khe:
            return 'BO_KHE', 'CHO_VAT_30s', dung
        return 'DI', None, dung


def doc_sep_lenh(dong, t_now, tuoi_max=HV.so('sep_lenh_tuoi_max_s')):
    """'<giay BOOTTIME> DI|CHO|DUNG <ly do>' (sep.py ghi_lenh) -> (chu, ly do) neu con moi; None neu khong co / cu / hong.
    Cung luat tuoi voi doc_sep cua chuyen.sh (-1 s <= tuoi <= tuoi_max)."""
    o = (dong or '').split(None, 2)
    if len(o) < 2 or o[1] not in ('DI', 'CHO', 'DUNG'):
        return None
    try:
        t = float(o[0])
    except ValueError:
        return None
    if t <= 0 or not -1.0 <= t_now - t <= tuoi_max:
        return None
    return o[1], (o[2].strip() if len(o) > 2 else '')


def cat_con_lai(duong, pose):
    """Phan duong (x, y, yaw) tu diem gan xe nhat tro di (cho /duong_day: diem dau = canh xe)."""
    if not duong or pose is None:
        return duong
    k = min(range(len(duong)), key=lambda i: math.hypot(duong[i][0] - pose[0], duong[i][1] - pose[1]))
    return duong[k:]


def cho_tf_costmap(co_cm, lay_ban, lay_pose, now, ngu, giay=HV.so('cho_san_sang_s')):
    """02/11 18:2x (xe 18:18, 18:20: lan 1 luon 'khong co tf hoac costmap moi', cho 30 s moi thu lan 2): doi toi `giay` s.
    Costmap TRUOC (doi luoi ra Ban mat vai giay tren Pi va khong spin, tf cu di), roi spin toi khi co tf moi. Tra (pose, ban)."""
    t_het = now() + giay
    while not co_cm() and now() < t_het:
        ngu(0.1)
    ban = lay_ban() if co_cm() else None
    pose = lay_pose()
    while pose is None and now() < t_het:
        ngu(0.05)
        pose = lay_pose()
    return pose, ban


def thieu_gi(pose, ban):
    return ' va '.join(t for t, ok in (('tf moi', pose is not None), ('costmap', ban is not None)) if not ok)


def phu_lai_ban(l1):
    l1 = l1 or ''
    return l1 in PHU_LAI_BAN or l1.startswith('THOAT')


def chay(io, huong, tuyen_dau, cfg):
    """Mot chuyen. io: nut ROS (GuiDuongNode) hoac gia trong ca thu. Tra ma thoat."""
    nhan = '%s_day' % huong
    thu_tu = (1, 2) if tuyen_dau == 1 else (2, 1)
    (dx, dy), yaw_cuoi = DICH[huong]
    t_chuyen = io.now()

    pose, ban = io.cho_san_sang()
    if pose is None or ban is None or ban.vat(ban.o(pose[0], pose[1])):      # 04/11: o CO VAT (khong phai dai cam lop 1)
        ly = ('khong co %s sau 15 s' % thieu_gi(pose, ban)) if pose is None or ban is None else 'xe nam trong o do cua costmap'
        io.bao('DUNG: SAI_VI_TRI (%s). KHONG gui lenh nao.' % ly)
        io.ghi(nhan, '-', 1, 0, 'SAI_VI_TRI', 0)
        return 3
    io.ghi(nhan, thu_tu[0], 1, 0, 'KIEM_DAT', 0)
    for r in thu_tu:
        lan = tinh_lai = 0
        while lan < cfg['luot']:
            lan += 1
            if cho_truoc_khi_gui(io, cfg) == 'DUNG' or cho_lop2(io, cfg, huong, r, lan) == 'DUNG':
                return dung_sep(io, nhan, r, lan)
            pose, ban = io.cho_san_sang()                 # 18:2x: doi toi 15 s (khong tinh la hong), chi bao khi qua han
            if pose is None or ban is None:
                io.bao('   lan %d: khong co %s sau 15 s' % (lan, thieu_gi(pose, ban)))
                io.ghi(nhan, r, 1, lan, 'KHONG_DAT', 0)
                if cho_sau_hong(io, cfg) == 'DUNG':
                    return dung_sep(io, nhan, r, lan)
                continue
            canh_bao = []
            try:
                duong = TT.tinh_duong(ban, (pose[0], pose[1]), (dx, dy), r, yaw_cuoi=yaw_cuoi, canh_bao=canh_bao)
            except TT.KhongCoDuong as e:
                io.bao('   tuyen %d: %s => doi tuyen' % (r, e))
                io.ghi(nhan, r, 1, lan, 'KHONG_DAT', 0)
                break
            for x, y, c in canh_bao:
                io.bao('   CANH BAO: dinh re (%.2f; %.2f) chi cach o do %.2f m (< %.2f), khong doi duoc' % (x, y, c, TT.R_XOAY_CAN))
            t0 = io.now()
            io.bao('-- tuyen %d, lan %d/%d: %d diem tu (%.2f; %.2f) toi (%.2f; %.2f)' % (r, lan, cfg['luot'], len(duong), pose[0], pose[1], dx, dy))
            kq = theo_doi(io, duong, cfg)
            giay = int(round(io.now() - t0))
            if kq == 'DUNG':
                return dung_sep(io, nhan, r, lan)
            if kq == 'TINH_LAI' and tinh_lai < cfg['luot']:   # 18:09: phu lai vua RA/XOAY xong: tinh lai tu cho xe, khong tinh la hong
                tinh_lai += 1
                io.bao('   phu lai vua XOAY xong: tinh lai duong tu cho xe (lan %d/%d).' % (tinh_lai, cfg['luot']))
                io.ghi(nhan, r, 1, lan, 'TINH_LAI', giay)
                lan -= 1
                continue
            pose = io.pose()
            lech = None if pose is None else math.hypot(pose[0] - dx, pose[1] - dy)
            if kq == 'SUCCEEDED' and lech is not None and lech <= cfg['toi_sai_so']:
                io.ghi(nhan, r, 1, lan, 'SUCCEEDED', giay, '%.2f' % lech)
                io.ghi(nhan, r, 'tong', 0, 'DEN_NOI', int(round(io.now() - t_chuyen)))
                io.bao('DEN_NOI: %s xong theo tuyen %d (cach dich %.2f m).' % (huong, r, lech))
                return 0
            io.bao('   khong den duoc (%s%s).' % (kq, '' if lech is None else ', cach dich %.2f m' % lech))
            io.ghi(nhan, r, 1, lan, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
            if lan < cfg['luot'] and cho_sau_hong(io, cfg) == 'DUNG':
                return dung_sep(io, nhan, r, lan)
        io.bao('== Tuyen %d hong. ==' % r)
    io.bao('HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.')
    io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
    return 1


def chay_mat(io, huong, tuyen_dau, cfg):
    """08/11 bo 1e: mot chuyen kieu mat. Cung CSV, cung chu log ket thuc, cung ma thoat voi chay().
    09/11 nao xe viec 4: ghi ~/kinh_nghiem.csv (SoKinhNghiem); het chuyen binh thuong: dong con cho => DUOC neu den noi."""
    kn = SoKinhNghiem(io, huong, io.ma_chuyen(), HV.so('kinh_nghiem_quan_sat_s'), HV.than_the('o_luoi_m'))
    ma = _chay_mat(io, huong, tuyen_dau, cfg, kn)
    kn.het_chuyen(ma == 0)
    if ma == 0:
        ghi_da_giai(io, huong, kn.chuyen)                # 09/11 gia X (d)
    return ma


class Luot:
    """09/11 23:07 (Thien): luot bi chan = mot lan giao dich (xe da that su di thu) khong toi, ke ca sau khi thoat thanh cong.
    Xa dich: so_luot luot => CHO_NGUOI. Gan dich (cach dich <= gan_dich_m luc bi chan): so_luot_gan luot HOAC han_gan s tinh tu
    lan bi chan dau tien gan dich (cai nao toi truoc) => CHO_NGUOI. hong_lien: so lan thoat hong lien tiep (thoat_hong => CHO_NGUOI)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.xa, self.gan, self.t_gan0, self.hong_lien = 0, 0, None, 0

    def danh_dau_gan(self, t):
        if self.t_gan0 is None:
            self.t_gan0 = t

    def het_han_gan(self, t):
        return self.t_gan0 is not None and t - self.t_gan0 >= self.cfg['han_gan'] - 1e-9

    def sau_cho_nguoi(self):
        """CHO_NGUOI xong: tu thu MOT luot (bi chan nua => CHO_NGUOI lai); han gan tinh lai tu lan bi chan sau."""
        self.xa = min(self.xa, self.cfg['so_luot'] - 1)
        self.gan = min(self.gan, self.cfg['so_luot_gan'] - 1)
        self.t_gan0, self.hong_lien = None, 0


def _cot_tu_do(ban, x, y0, buoc=0.01, ymax=1.5):
    """Tu (x, y0) o trong: di xuong / len toi o tuong dau tien -> (y mep duoi, y mep tren) | None neu (x, y0) la tuong."""
    if ban.la_tuong(x, y0):
        return None
    lo, hi = y0, y0
    while lo > y0 - ymax and not ban.la_tuong(x, lo - buoc):
        lo -= buoc
    while hi < y0 + ymax and not ban.la_tuong(x, hi + buoc):
        hi += buoc
    return lo, hi


def diem_dan_ve_A(ban, cach=0.15, x0=0.45, x1=0.90, y_doan=0.08, dich=None):
    """10/11 muc 6: diem dan ve_A tinh tu ban do goc: doan x0..x1, khe K4 (duoi) - H0 (tren) quanh y_doan. Mep K4 = mep duoi cao
    nhat, mep H0 = mep tren thap nhat tren doan; tam giua khe, cach K4 >= cach (neu khe hep thi giua). -> (x, y, yaw, kc_K4, kc_H0) | None."""
    dich = dich or TT.A
    cot = [_cot_tu_do(ban, x0 + k * (x1 - x0) / 9, y_doan) for k in range(10)]
    if any(c is None for c in cot):
        return None
    yk, yh = max(c[0] for c in cot), min(c[1] for c in cot)
    y = (yk + yh) / 2
    if y - yk < cach and yh - yk >= 2 * cach:
        y = yk + cach
    x = (x0 + x1) / 2
    return x, y, math.atan2(dich[1] - y, dich[0] - x), y - yk, yh - y


def _ve_than(p, tu_the):
    """Diem ban do -> khung than xe dat o tu_the (x, y, yaw)."""
    dx, dy, c, s_ = p[0] - tu_the[0], p[1] - tu_the[1], math.cos(tu_the[2]), math.sin(tu_the[2])
    return dx * c + dy * s_, -dx * s_ + dy * c


def khe_than(tu_the, vat, tuong):
    """(khe THAN xe 25 x 16 toi VAT gan nhat, toi TUONG gan nhat) tai tu_the; diem khung ban do."""
    kv = min((SL.kc_than(_ve_than(p, tu_the))[1] for p in vat), default=math.inf)
    kt = min((SL.kc_than(_ve_than(p, tu_the))[1] for p in tuong), default=math.inf)
    return kv, kt


def diem_phan_loai(ds, pts, pose, ban, t):
    """LiDAR luc nay -> (diem VAT, diem TUONG) khung ban do. Cum TUONG (ba dau hieu) => tuong; ong / vat chac => vat; diem ngoai cum
    danh gia: gan o tuong ban do <= 0,08 => tuong, khac => vat. Them tam o tuong ban do trong 1 m (ban do cung la tuong)."""
    c, s_ = math.cos(pose[2]), math.sin(pose[2])
    to_map = lambda q: (pose[0] + q[0] * c - q[1] * s_, pose[1] + q[0] * s_ + q[1] * c)
    nhan = {}
    for d in ds or ():
        for k in d.get('ks', ()):
            nhan[k] = 'tuong' if d.get('tuong_k') and not d.get('chac') else 'vat'
    vat, tuong = [], []
    for k, q in enumerate(pts or ()):
        if q is None:
            continue
        m = to_map(q)
        n = nhan.get(k) or ('tuong' if ban is not None and ban.gan_tuong(m[0], m[1], 0.08) else 'vat')
        (tuong if n == 'tuong' else vat).append(m)
    if ban is not None:
        i0, j0 = int((pose[0] - ban.ox) / ban.res), int((pose[1] - ban.oy) / ban.res)
        r = int(1.0 / ban.res)
        for j in range(j0 - r, j0 + r + 1):
            for i in range(i0 - r, i0 + r + 1):
                g = ban.gia(i, j)
                if g is not None and g >= GV.O_TUONG_MIN:
                    tuong.append((ban.ox + (i + 0.5) * ban.res, ban.oy + (j + 0.5) * ban.res))
    return vat, tuong


def chon_diem_vuot(P, dich, pose, vat, tuong, ban, h=HV.so):
    """10/11 muc 7 (+K1): diem vuot V phia TRUOC P (huong P -> dich), lech +-0/20/40/60 do, cach P d_min..d_max. Dieu kien do theo
    THAN xe (tai V quay ve dich): bac 1 VAT >= vuot_vat1, TUONG >= vuot_tuong1; khong co thi bac 2 (vuot_vat2 / vuot_tuong2). V o o
    trong ban do. Chon: bac nho, roi khe duong thang xe -> V lon (chi XEP HANG, khong loai: Nav2 tu vach duong), roi gan dich.
    -> dict(V, yaw, bac, phia, khe, ly) | None."""
    phi = math.atan2(dich[1] - P[1], dich[0] - P[0])
    ds_d = [h('diem_vuot_d_min_m'), (h('diem_vuot_d_min_m') + h('diem_vuot_d_max_m')) / 2, h('diem_vuot_d_max_m')]
    ung = []
    for dg in (0, 20, -20, 40, -40, 60, -60):
        a = phi + math.radians(dg)
        for d in ds_d:
            V = (P[0] + d * math.cos(a), P[1] + d * math.sin(a))
            if ban is not None and (ban.gia(int((V[0] - ban.ox) / ban.res), int((V[1] - ban.oy) / ban.res)) is None
                                    or ban.la_tuong(*V)):
                continue
            yaw = math.atan2(dich[1] - V[1], dich[0] - V[0])
            kv, kt = khe_than((V[0], V[1], yaw), vat, tuong)
            bac = (1 if kv >= h('vuot_vat1_m') - 1e-9 and kt >= h('vuot_tuong1_m') - 1e-9 else
                   2 if kv >= h('vuot_vat2_m') - 1e-9 and kt >= h('vuot_tuong2_m') - 1e-9 else None)
            if bac is None:
                continue
            L = math.hypot(V[0] - pose[0], V[1] - pose[1])
            yd = math.atan2(V[1] - pose[1], V[0] - pose[0])
            duong = min((min(khe_than((pose[0] + (V[0] - pose[0]) * u / 10, pose[1] + (V[1] - pose[1]) * u / 10, yd), vat, tuong)[0] - 0.06,
                             khe_than((pose[0] + (V[0] - pose[0]) * u / 10, pose[1] + (V[1] - pose[1]) * u / 10, yd), vat, tuong)[1] - 0.02)
                         for u in range(1, 11)), default=0.0) if L > 0.02 else 0.0
            ung.append((bac, -round(duong, 3), round(math.hypot(dich[0] - V[0], dich[1] - V[1]), 3), V, yaw, dg, kv, kt))
    if not ung:
        return None
    ung.sort(key=lambda u: u[:3])
    bac, nd, _, V, yaw, dg, kv, kt = ung[0]
    return dict(V=V, yaw=yaw, bac=bac, phia='thang' if dg == 0 else 'trai' if dg > 0 else 'phai',
                ly='bac %d, than cach vat %s / tuong %s, duong cach them %.2f' % (bac, '-' if kv == math.inf else '%.2f' % kv,
                                                                            '-' if kt == math.inf else '%.2f' % kt, -nd))


def huong_toi(p, dich):
    """Goc toi dich trong khung xe (rad)."""
    return math.atan2(dich[1] - p[1], dich[0] - p[0]) - p[2]


def tin_sep(io, kn):
    """Muc 6 / Q1: tin cua sep.py (/sep/trang_thai) la TIN, khong phai lenh. Thien 00:5x: vi tri khong tin CHI khi muc_tin KHONG_TIN
    (tin_cay < 0,50 lien tuc >= 5 s) hoac BAO (>= 15 s); NGHI (0,50-0,70) van tin (khong huy dich). In 1 dong khi doi: 'sep: KHONG_TIN (ghi, khong chan)',
    'sep: DUNG (ly) (ghi, khong chan)'. -> True neu vi tri tin (khong co tin sep => tin)."""
    s = io.tin_sep()
    moi = None if s is None else (s['muc_tin'] if not s['tin'] else None, s['di'] if s['di'] in ('DUNG', 'CHO') else None, s['ly_do'])
    if moi != getattr(kn, 'sep_cu', None):
        kn.sep_cu = moi
        if moi is not None and moi[0]:
            io.bao('sep: %s (ghi, khong chan)' % moi[0])
        elif moi is not None and moi[1]:
            io.bao('sep: %s (%s) (ghi, khong chan)' % (moi[1], moi[2] or '-'))
    return s is None or s['tin']


def cho_vi_tri(io, cfg, nhan, kn):
    """Muc 4 + Q2: cho tf + costmap + sep khong bao vi tri khong tin. Chua co: dung yen cho (KHONG thoat tien trinh); qua
    cho_vi_tri s => 'CHO_VI_TRI: ... bao nguoi' + 1 dong CSV CHO_VI_TRI (trang: 도움이 필요해요), van cho, vi tri tin lai => di tiep.
    cho_lai <= 0 (ca thu cu) => SAI_VI_TRI ma 3 nhu cu. -> (pose, ban) | (None, None)."""
    pose, ban = io.pose(), io.ban()
    if pose is None or ban is None:
        pose, ban = io.cho_san_sang()
    t0, da_cho, da_bao = io.now(), False, False
    while True:
        tin = tin_sep(io, kn)
        if pose is not None and ban is not None and tin:
            if da_cho:
                io.bao('CHO_VI_TRI: vi tri tin lai sau %d s, di tiep.' % int(io.now() - t0))
            return pose, ban
        ly = ('khong co %s' % thieu_gi(pose, ban)) if pose is None or ban is None else 'sep bao vi tri khong tin'
        if cfg.get('cho_lai', 0) <= 0:
            io.bao('DUNG: SAI_VI_TRI (%s). KHONG gui lenh nao.' % ly)
            io.ghi(nhan, '-', 1, 0, 'SAI_VI_TRI', 0)
            return None, None
        if not da_cho:
            da_cho = True
            io.bao('SEP_LIDAR: CHO_VI_TRI (%s): dung yen, chi giao dich khi vi tri tin lai.' % ly)
        if not da_bao and io.now() - t0 >= cfg['cho_vi_tri']:
            da_bao = True
            io.bao('CHO_VI_TRI: %s qua %d s, bao nguoi; van cho, vi tri tin lai thi tu di tiep (bam 정지 de dung).' % (ly, int(cfg['cho_vi_tri'])))
            io.ghi(nhan, '-', 'het', 0, 'CHO_VI_TRI', 0)
        io.ngu(0.5)
        pose, ban = io.pose(), io.ban()


def _chay_mat(io, huong, tuyen_dau, cfg, kn):
    """09/11 23:08 SEP LiDAR mot nguoi lai (Thien chot 21:54-23:08). gui_duong la nguoi quyet DUY NHAT (thay -> hieu -> quyet ->
    kiem lai): di / doi vat / thoat / giao dich; moi phan khac chi dua tin (sep.py, ban do, kinh nghiem, thoi_quen).
    Moi vong: truoc khi giao (LiDAR: kep / vung do / nhac doi) -> cho vi tri tin (muc 4) -> giao dich (luot 1 tuyen goi y, roi Nav2 tu vach)
    Thien 00:5x: Nav2 tu choi dich = 1 luot bi chan (thoat truoc khi giao lai; CHO_NGUOI chi theo luat dem luot).
    -> bi chan (10 s khong tien / doi het 30 s) => dem luot, THOAT (sep_lidar) roi giao lai. Het luot / thoat hong 3 lan / kep moi
    huong => CHO_NGUOI (dung yen, bao nguoi, tu thu lai sau cho_lai s; cho_lai <= 0 => HET_DUONG nhu ca thu cu)."""
    nhan = '%s_day' % huong
    thu_tu = (1, 2) if tuyen_dau == 1 else (2, 1)
    (dx, dy), yaw_cuoi = DICH[huong]
    dich = (dx, dy)
    t_chuyen = io.now()
    io.chon_tuyen('0')                                   # 3b #3: bo o cam cu (kill -9 / loi ra 130, 5 cua chuyen truoc)
    io.ngu(cfg['cho_chon'])
    pose, ban = io.cho_san_sang()                        # tf + costmap (<= 15 s); sep chua xet (thoat bang LiDAR khong can vi tri)
    if pose is None or ban is None:
        pose, ban = cho_vi_tri(io, cfg, nhan, kn)
        if pose is None:
            return 3
    thu_tu = thu_tu_gia_x(io, thu_tu, pose, ban, dich, yaw_cuoi)     # 09/11 gia X (c): chi goi y cho luot 1
    io.ghi(nhan, thu_tu[0], 1, 0, 'KIEM_DAT', 0)
    luot, lan, can_thoat, nhac = Luot(cfg), 0, None, True
    diem_chan, da_dan = None, False                      # 10/11 muc 7: P (diem bi chan) cho diem vuot; muc 6: diem dan da dung

    def cho_nguoi_luot(ly):
        if cfg.get('cho_lai', 0) <= 0:
            io.bao('HET_DUONG: %s. Xe dung yen.' % ly)
            io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
            return 1
        cho_nguoi(io, cfg, nhan, ly)
        luot.sau_cho_nguoi()
        return None

    while True:
        if can_thoat is not None:
            kq = thoat(io, cfg, kn, luot, dich, can_thoat)
            nhac = not can_thoat.startswith('CHO_VAT_')     # vua het 30 s doi vat: lan giao sau khong nhac doi lai
            can_thoat = None
            ma = None
            if kq == 'KEP':
                ma = cho_nguoi_luot('kep moi huong, khong dong tac nao dat')
            elif kq == 'HAN_GAN':
                ma = cho_nguoi_luot('gan dich qua %d s' % int(cfg['han_gan']))
            elif kq == 'HONG':
                luot.hong_lien += 1
                if luot.hong_lien >= cfg['thoat_hong']:
                    ma = cho_nguoi_luot('thoat hong %d lan lien tiep' % luot.hong_lien)
            else:
                luot.hong_lien = 0
            if ma is not None:
                return ma
            if kq not in ('KEP', 'HAN_GAN') and diem_chan is not None and cfg.get('diem_vuot'):
                di_vuot(io, cfg, kn, luot, diem_chan, dich)        # 10/11 muc 7: huong dan, loi ra = bo V
            diem_chan = None
        kq, ly = truoc_khi_giao(io, cfg, kn, luot, nhac)  # LiDAR truoc: kep / vung do / nhac doi (muc 4: khong can vi tri tin)
        if kq == 'HAN_GAN':
            ma = cho_nguoi_luot('gan dich qua %d s' % int(cfg['han_gan']))
            if ma is not None:
                return ma
            can_thoat = 'SAU_CHO_NGUOI'
            continue
        if kq == 'THOAT':
            can_thoat = ly
            continue
        pose, ban = cho_vi_tri(io, cfg, nhan, kn)        # chi giao dich ban do khi vi tri tin
        if pose is None:
            return 3
        r = thu_tu[0] if lan == 0 else 0
        lan += 1
        io.chon_tuyen('0' if r == 0 else '%d %.3f %.3f' % (r, pose[0], pose[1]))
        io.ngu(cfg['cho_chon'])
        if huong == 've_A' and r == 1 and cfg.get('diem_dan') and not da_dan:     # 10/11 muc 6: diem dan giua khe K4-H0
            da_dan = True
            dd = diem_dan_ve_A(io.gac_ban(), HV.so('diem_dan_cach_k4_m')) if io.gac_ban() is not None else None
            p0 = io.pose()
            if dd is not None and p0 is not None and p0[0] > dd[0] + 0.10:
                io.bao('DIEM_DAN: (%.2f; %.2f) huong %+d do, cach K4 %.2f m, cach H0 %.2f m (ban do).' % (
                    dd[0], dd[1], int(round(math.degrees(dd[2]))), dd[3], dd[4]))
                if di_phu(io, cfg, kn, luot, (dd[0], dd[1]), dd[2], cfg['diem_dan_han'], 'DIEM_DAN') != 'TOI':
                    io.bao('DIEM_DAN: bo, di thang A (Nav2 tu vach, khong tinh luot).')
                    r = 0
                    io.chon_tuyen('0')
                    io.ngu(cfg['cho_chon'])
        p = io.pose() or pose
        t0 = io.now()
        io.bao('SEP_LIDAR: DI dich (%.2f; %.2f), %s' % (dx, dy, 'tuyen %d' % r if r else 'Nav2 tu vach'))
        io.bao('-- tuyen %d, lan %d/%d: navigate_to_pose tu (%.2f; %.2f) toi (%.2f; %.2f)%s' % (
            r, lan, max(lan, cfg['so_luot']), p[0], p[1], dx, dy, ' (Nav2 tu vach)' if r == 0 else ''))
        kq = theo_doi_mat(io, dich, yaw_cuoi, cfg, kn, luot, nhac)
        nhac = True
        giay = int(round(io.now() - t0))
        p = io.pose()
        lech = None if p is None else math.hypot(p[0] - dx, p[1] - dy)
        if kq == 'SUCCEEDED' and lech is not None and lech <= cfg['toi_sai_so']:
            io.ghi(nhan, r, 1, lan, 'SUCCEEDED', giay, '%.2f' % lech)
            io.ghi(nhan, r, 'tong', 0, 'DEN_NOI', int(round(io.now() - t_chuyen)))
            io.bao('DEN_NOI: %s xong theo %s (cach dich %.2f m).' % (huong, 'tuyen %d' % r if r else 'duong Nav2 tu vach', lech))
            return 0
        if kq == 'VI_TRI':                               # muc 4: vi tri khong tin giua chung => da huy dich, cho vi tri (khong tinh luot)
            io.bao('   khong den duoc (VI_TRI_KHONG_TIN%s): cho vi tri.' % ('' if lech is None else ', cach dich %.2f m' % lech))
            lan -= 1
            continue
        gan = lech is not None and lech <= cfg['gan_dich_m']
        if gan:
            luot.gan += 1
            luot.danh_dau_gan(io.now())
        else:
            luot.xa += 1
        kn.ngay('LUOT_CHAN', kq, 'HONG', ly_do_kinh_nghiem(io.tuoi_plan(), cfg['khong_tien']), p)
        diem_chan = None if p is None else (p[0], p[1])  # 10/11 muc 7
        io.chup(huong, 'luot bi chan (tuyen %d, %s)' % (r, kq))
        io.bao('   khong den duoc (%s%s).' % (kq, '' if lech is None else ', cach dich %.2f m' % lech))
        io.ghi(nhan, r, 1, lan, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
        io.bao('== Luot %d bi chan (%s). ==' % (luot.xa + luot.gan, 'gan dich %d/%d' % (luot.gan, cfg['so_luot_gan']) if gan
                                                 else 'xa dich %d/%d' % (luot.xa, cfg['so_luot'])))
        if kq == 'HAN_GAN' or luot.het_han_gan(io.now()) or (gan and luot.gan >= cfg['so_luot_gan']) or (not gan and luot.xa >= cfg['so_luot']):
            ma = cho_nguoi_luot('khong tim thay duong (%s)' % ('gan dich %d luot / %d s' % (luot.gan, int(cfg['han_gan'])) if gan
                                                               else '%d luot bi chan' % luot.xa))
            if ma is not None:
                return ma
        can_thoat = kq


def cho_nguoi(io, cfg, nhan, ly):
    """09/11 18:47 (i) (Thien: dung han roi bao nguoi KHONG phai loi ra): xe dung yen (khong co dich Nav2), 1 dong CSV
    CHO_NGUOI (trang: 도움이 필요해요, tien trinh van song), in 'CHO_NGUOI: n/N s' moi nhip_cho s, het cho_lai s thi tra ve de thu lai
    1 luot. Nguoi bam 정지 (SIGINT trong io.ngu) => thoat nhu cu (NGAT 130)."""
    io.bao('CHO_NGUOI: %s, xe dung yen cho nguoi; tu thu lai sau %d s (bam 정지 de dung).' % (ly, int(cfg['cho_lai'])))
    io.ghi(nhan, '-', 'het', 0, 'CHO_NGUOI', 0)
    t0, lan_in = io.now(), 0
    while cfg['cho_lai'] - (io.now() - t0) > 1e-3:      # con < 1 ms thi xong (tranh buoc qua nho bi lam tron mat => lap mai)
        io.ngu(min(cfg['nhip_cho'], cfg['cho_lai'] - (io.now() - t0)))
        da = round(io.now() - t0, 6)                     # 20:33: tranh 24,9999 => mat dong 25/60 (dong ho cong don 0,1 s)
        if int(da // cfg['nhip_cho']) > lan_in and da < cfg['cho_lai']:
            lan_in = int(da // cfg['nhip_cho'])
            io.bao('CHO_NGUOI: %d/%d s' % (int(da), int(cfg['cho_lai'])))
    io.bao('CHO_NGUOI: het %d s, thu lai.' % int(cfg['cho_lai']))


class Mat:
    """20:33 vung quanh xe: moi lan quet moi -> danh gia (gac_vat_la.danh_gia) + in MEM khi doi (GV.Mem). Mot Mat moi chuyen (kn.mat)."""

    def __init__(self, io, kn):
        self.io, self.kn, self.t, self.mem = io, kn, GV.tham_so(), GV.Mem()
        self.theo = GV.TheoDoiOng()                      # 10/11 A++: on dinh cung qua cac lan quet (ong_on_dinh)

    def quet(self):
        """-> ('OK', t_quet, ds, diem khung xe) | ('MU', None, None, None) (CANH BAO 1 lan moi dot, kn.mu) | ('CHUA', ...)."""
        q = self.io.gac_quet()
        if q[0] == 'MU':
            if not self.kn.mu:
                self.io.bao('CANH BAO gac: mu (%s)' % q[1])
            self.kn.mu = True
            return 'MU', None, None, None
        if q[0] != 'OK':
            return 'CHUA', None, None, None
        self.kn.mu = False
        t_q, pxe, pmap = q[1]
        ds = GV.danh_gia(pxe, pmap, self.io.gac_ban(), self.t, self.theo, t_q)
        for c in self.mem.cap_nhat(ds):
            self.io.bao(c)
        return 'OK', t_q, ds, pxe


def mat_cua(io, kn):
    if getattr(kn, 'mat', None) is None:
        kn.mat = Mat(io, kn)
    return kn.mat


def quet_moi(io, kn, giay=1.0):
    """Lan quet LiDAR moi trong `giay` s -> (ds, diem khung xe) | (None, None) (mu / khong co)."""
    mat, t0 = mat_cua(io, kn), io.now()
    while True:
        tt, _, ds, pts = mat.quet()
        if tt == 'OK':
            return ds, pts
        if tt == 'MU' or io.now() - t0 >= giay:
            return None, None
        io.ngu(0.05)


def _mo_ta(d):
    return '%s %s cach %.2f m' % (d['vung'], 'vat chac' if d['lidar'] == 'VAT' and not d['lech'] else 'vat', d['kc'])


def kep(pts, pl, t, tuong=None):
    """Kep (thay LUI_NGAN bo / huy lui cu): sau duoi (hanh lang than, gac_vat_la.khoang_sau) <= duoi_do (khong lui duoc) VA
    (hong <= 0,06 hoac truoc <= 0,10) - moi loai, ke ca tuong (khe vung sep_lidar.khe).
    10/11 A+ (tuong: co cum TUONG cua ba dau hieu): hong = VAT <= 0,06 hoac TUONG <= khong_cham_m 0,02."""
    sau, k = GV.khoang_sau(pts, t['gac']), SL.khe(pl, t, tuong=tuong)
    hong = (k['hong'] <= t['hong_do'] + 1e-9 if tuong is None
            else k['hong_v'] <= t['hong_do'] + 1e-9 or k['hong_t'] <= t['khong_cham'] + 1e-9)
    return sau is not None and sau <= HV.an_toan('duoi_do_m') + 1e-9 and (hong or k['hl'] <= t['truoc_do'] + 1e-9)


def truoc_khi_giao(io, cfg, kn, luot, nhac=True):
    """Truoc moi lan giao dich: kep => ('THOAT', 'KEP'); vung do (an_toan) hoac nhac doi (VAT CHAC truoc mui <= 0,14) => dung yen doi
    (sach sach_s => di; het cho_vat_s => ('THOAT', 'CHO_VAT_30s' | 'VUNG_DO_30s')); con lai => ('DI', None) NGAY (khong cong 2 s).
    Mu => ('DI', None) (Nav2 + collision_monitor van chay). Han gan dich => ('HAN_GAN', None).
    nhac False (vua thoat vi doi vat het 30 s): khong nhac doi lai cung vat, chi vung do; Nav2 tu vach vong qua vat."""
    ds, pts = quet_moi(io, kn)
    if ds is None:
        return 'DI', None
    t = SL.tham_so()
    pl, fl = SL.diem_loc_nhan(pts, t['gac'], ds)
    if kep(pts, pl, t, fl):
        return 'THOAT', 'KEP'
    c, d = SL.vat_chac(ds, t) if nhac else None, SL.vung_do(ds, t)
    if c is None and d is None:
        return 'DI', None
    io.bao('SEP_LIDAR: DOI_VAT %s (%s), dung yen toi da %d s.' % (_mo_ta(c or d), 'nhac doi' if c else 'vung do', int(cfg['cho_vat'])))
    kq = doi_vat(io, cfg, kn, luot, None)
    kn.ngay('CHO_VAT_LA', 'doi', 'DUOC' if kq == 'DI' else 'HONG', '-', io.pose())
    if kq == 'DI':
        return 'DI', None
    if kq == 'HAN_GAN':
        return 'HAN_GAN', None
    return 'THOAT', 'CHO_VAT_%ds' % int(cfg['cho_vat']) if c else 'VUNG_DO_%ds' % int(cfg['cho_vat'])


def doi_vat(io, cfg, kn, luot, dich):
    """Dung yen doi (khong co dich Nav2): sach (khong vung do, khong vat chac) lien tuc sach_s => 'DI'; cho_vat_s => 'HET_GIO';
    han gan => 'HAN_GAN'. In 'CHO_VAT: n/30 s' moi nhip_cho s. Mu => 'DI'."""
    t, mat = SL.tham_so(), mat_cua(io, kn)
    t0, t_sach, lan_in = io.now(), None, 0
    while True:
        tin_sep(io, kn)
        tt, t_q, ds, _ = mat.quet()
        if tt == 'MU':
            return 'DI'
        if tt == 'OK':
            if SL.vung_do(ds, t) is not None or SL.vat_chac(ds, t) is not None:
                t_sach = None
            elif t_sach is None:
                t_sach = t_q
            if t_sach is not None and io.now() - t_sach >= HV.so('sach_s') - 1e-9:
                io.bao('GAC: het chan sau %d s' % int(io.now() - t0))
                return 'DI'
        if luot.het_han_gan(io.now()):
            return 'HAN_GAN'
        da = io.now() - t0
        if da >= cfg['cho_vat'] - 1e-9:
            return 'HET_GIO'
        if da >= cfg['nhip_cho'] * (lan_in + 1):
            lan_in = int(da // cfg['nhip_cho'])
            p = io.pose()
            io.bao('CHO_VAT: %d/%d s%s' % (int(da), int(cfg['cho_vat']), '' if p is None or dich is None else
                                            ' (cach dich %.2f m)' % math.hypot(p[0] - dich[0], p[1] - dich[1])))
        kn.nhip()
        io.ngu(0.1)


def theo_doi_mat(io, dich, yaw_cuoi, cfg, kn, luot, nhac=True):
    """Giao dich navigate_to_pose, theo doi toi khi xong. Tra 'SUCCEEDED' | 'ABORTED' | 'CANCELED' (Nav2) | 'TU_CHOI' | 'VI_TRI'
    (vi tri khong tin: da huy dich) | 'KHONG_TIEN_10s' | 'KHONG_FEEDBACK' | 'CHO_VAT_30s' | 'VUNG_DO_30s' | 'HAN_GAN' (da huy dich).
    Moi lan quet: vung do (an_toan: VAT truoc <= 0,10 / hong <= 0,06) hoac nhac doi (VAT CHAC truoc mui <= 0,14) => TU huy dich,
    DUNG IM ('GAC: dung im, ...'), sach sach_s => gui LAI dich; dung cho_vat_s => bi chan. khong_tien s khong tien >= tien_min =>
    bi chan. Bi chan / dung khi cach dich <= gan_dich_m => danh dau lan bi chan dau tien gan dich (han_gan).
    nhac False (vua thoat vi doi vat het 30 s): chua nhac doi cho toi khi xe tien lan dau (khong doi lai cung vat; vung do van giu)."""
    if not io.gui_nav(dich, yaw_cuoi):
        kn.ket(False)
        return 'TU_CHOI'
    t, mat = SL.tham_so(), mat_cua(io, kn)
    dh = DongHo(io.now(), cfg['khong_tien'], cfg['tien_min'])
    n_rec, lan_in, da_in = 0, 0, False
    dung, t_dung, t_sach = None, None, None              # dung: 'CHO_VAT' | 'VUNG_DO' (dich da TU huy, dang dung im)
    kq = None
    while True:
        if dung is None:
            kq = io.ket_qua()
            if kq is not None:
                break
        if not tin_sep(io, kn):
            if dung is None:
                io.huy()
            kq = 'VI_TRI'
            break
        p = io.pose()
        d_tf = None if p is None else math.hypot(p[0] - dich[0], p[1] - dich[1])
        gan = d_tf is not None and d_tf <= cfg['gan_dich_m']
        if dung is None:
            fb = io.feedback()
            tt, ly, _ = dh.cap_nhat(io.now(), d_tf, None if fb is None else fb[0])
            if fb is not None and fb[1] > n_rec:
                n_rec = fb[1]
                io.bao('   Nav2 hoi phuc: number_of_recoveries = %d.' % n_rec)
            if tt == 'TIEN':
                nhac = True
                kn.tien()
                if da_in:
                    io.bao('   DI_TIEP: xe tien lai.')
                lan_in, da_in = 0, False
            elif tt == 'BO_KHE':
                if gan:
                    luot.danh_dau_gan(dh.t_moc if dh.t_moc is not None else io.now())
                io.huy()
                kq = 'KHONG_FEEDBACK' if ly == 'KHONG_FEEDBACK' else 'KHONG_TIEN_%ds' % int(cfg['khong_tien'])
                break
        tq, t_q, ds, _ = mat.quet()
        if tq == 'OK':
            c, d = SL.vat_chac(ds, t) if nhac or dung is not None else None, SL.vung_do(ds, t)
            if dung is None and (c is not None or d is not None):
                io.huy()
                e = d or c
                dung, t_dung, t_sach = ('CHO_VAT' if c is not None else 'VUNG_DO'), io.now(), None
                io.bao('GAC: dung im, %s vat cach %.2f m%s' % (e['vung'], e['kc'], '' if e['xy'] is None else ' (%.2f; %.2f)' % e['xy']))
                io.bao('SEP_LIDAR: DOI_VAT %s (%s), dung yen toi da %d s.' % (_mo_ta(e), 'nhac doi' if c is not None else 'vung do',
                                                                            int(cfg['cho_vat'])))
                if gan:
                    luot.danh_dau_gan(io.now())
            elif dung is not None:
                if c is not None or d is not None:
                    t_sach = None
                elif t_sach is None:
                    t_sach = t_q
                if t_sach is not None and io.now() - t_sach >= HV.so('sach_s') - 1e-9:
                    io.bao('GAC: het chan sau %d s' % int(io.now() - t_dung))
                    kn.ngay('CHO_VAT_LA', 'doi', 'DUOC', '-', p)
                    if not io.gui_nav(dich, yaw_cuoi):
                        kq = 'TU_CHOI'
                        break
                    dung, lan_in, da_in = None, 0, False
                    dh = DongHo(io.now(), cfg['khong_tien'], cfg['tien_min'])
        if dung is not None and io.now() - t_dung >= cfg['cho_vat'] - 1e-9:
            kn.ngay('CHO_VAT_LA', 'doi', 'HONG', '-', p)
            kq = '%s_%ds' % (dung, int(cfg['cho_vat']))
            break
        if luot.het_han_gan(io.now()):
            if dung is None:
                io.huy()
            kq = 'HAN_GAN'
            break
        da = io.now() - t_dung if dung is not None else None
        if da is not None and da >= cfg['nhip_cho'] * (lan_in + 1):
            lan_in = int(da // cfg['nhip_cho'])
            da_in = True
            io.bao('CHO_VAT: %d/%d s%s' % (int(da), int(cfg['cho_vat']), '' if d_tf is None else ' (cach dich %.2f m)' % d_tf))
        kn.nhip()
        io.ngu(0.1)
    kn.ket(kq == 'SUCCEEDED')                            # nao xe viec 4: dong con cho chot theo ket qua dich
    return kq


def _huy_dong_tac(io, kn, k0, t):
    """Q3: moi lan quet moi trong luc behavior chay: ly do huy (sep_lidar.ly_do_huy) | None."""
    tt, _, ds, _ = mat_cua(io, kn).quet()
    if tt != 'OK':
        return None
    ly = SL.ly_do_huy(SL.tom_tat(ds), k0, t)
    return None if ly is None else 'HUY (%s)' % ly


def _khe_chu(k):
    f = lambda v: '-' if v == math.inf else '%.2f' % v
    return 'truoc %s, hong %s, sau %s' % (f(k['hl']), f(min(k['hong'], k['cheo'])), f(k['sau']))


def thoat(io, cfg, kn, luot, dich, ly):
    """Muc 3 THOAT (mot chu: sep LiDAR). Cho chat (co diem cach than < cho_chat_m): TUNG dong tac nho (sep_lidar.chon_dong_tac:
    drive_on_heading / backup / spin qua Nav2, toi da thoat_buoc buoc), moi dong tac theo doi moi lan quet va huy theo Q3. Cach
    DUOC gan nhat trong ~/kinh_nghiem.csv thu truoc neu LiDAR kiem kha thi. Khong dong tac nao dat: khe >= 0,06 => giao dich;
    khong => 'KEP'. Cho thoang + vi tri tin: sector gan huong dich => diem thoat (navigate_to_pose, han thoat_han_s).
    Vi tri khong tin: van thoat bang LiDAR + odom (#19), khong dung huong dich / diem thoat ban do. -> 'XONG' | 'HONG' | 'KEP' | 'HAN_GAN'."""
    t = SL.tham_so()
    pl, k, p, tin = None, None, None, True
    for buoc in range(int(cfg['thoat_buoc']) + 1):
        if luot.het_han_gan(io.now()):
            return 'HAN_GAN'
        ds, pts = quet_moi(io, kn)
        if ds is None:
            io.bao('SEP_LIDAR: THOAT bo (khong co lan quet moi: mu).')
            return 'HONG'
        pl, fl = SL.diem_loc_nhan(pts, t['gac'], ds)      # 10/11 A+: fl = co TUONG tung diem (None: phan_loai_cum cu)
        k = SL.khe(pl, t, tuong=fl)
        tin = tin_sep(io, kn)
        p = io.pose()
        if k['nho'] >= t['cho_chat'] or buoc == int(cfg['thoat_buoc']):
            break
        hd = None if p is None or not tin else huong_toi(p, dich)
        uu = None
        kc = cach_kinh_nghiem([d for d in io.doc_kinh_nghiem() if SL.doc_ten(d.get('cach'))], p, HV.so('gia_x_r_m'))
        if kc is not None:
            uu = SL.doc_ten(kc[1])
        dt, c, bo = SL.chon_dong_tac(pl, t, hd, uu, fl)
        if uu is not None:
            ly_bo = next((l for n, l in bo if n == SL.ten_dong_tac(uu)), None)
            io.bao('KINH_NGHIEM: %s DUOC cach %.2f m => %s' % (kc[1], kc[2], 'thu truoc' if dt == uu else 'bo (LiDAR: %s)' % (ly_bo or 'khong tot bang')))
        if dt is None:
            if SL.thieu(k, t) <= 0.0:
                break                                    # da ra khoi vung do, khong lam tot hon duoc: giao dich
            io.bao('SEP_LIDAR: KEP moi huong (%s): khong dong tac nao dat (%s).' % (
                _khe_chu(k), '; '.join('%s %s' % (n, l) for n, l in bo[:4])))
            return 'KEP'
        ten = SL.ten_dong_tac(dt)
        io.bao('SEP_LIDAR: THOAT %s (%s; cho chat: %s => cuoi %.2f).' % (ten, ly, _khe_chu(k), c['cuoi']['nho']))
        k0 = SL.tom_tat(ds)
        kq, quang = io.dong_tac(dt, cfg['v_thoat'], lambda: _huy_dong_tac(io, kn, k0, t))
        io.bao('   THOAT_XONG: %s => %s%s.' % (ten, kq, '' if quang is None else ' (xe xoay %d do)' % round(quang) if dt[0] == 'xoay'
                                                    else ' (xe dich %.2f m)' % quang))
        if kq != 'SUCCEEDED':
            kn.ngay('THOAT', ten, 'HONG', kq, p)
            return 'HONG'
        kn.doi('THOAT', ten, p, ly)
    if SL.thieu(k, t) > 0.0:
        io.bao('SEP_LIDAR: THOAT chua ra vung do (%s) sau %d dong tac.' % (_khe_chu(k), int(cfg['thoat_buoc'])))
        return 'HONG'
    if k['nho'] < t['cho_chat'] or p is None or not tin:
        return 'XONG'
    th = p[2]
    to_map = lambda x, y: (p[0] + x * math.cos(th) - y * math.sin(th), p[1] + x * math.sin(th) + y * math.cos(th))
    sec = SL.chon_sector(pl, t, huong_toi(p, dich), lambda x, y: io.o_costmap(*to_map(x, y)) is not None
                         and 0 <= io.o_costmap(*to_map(x, y)) < 99)
    if sec is None:
        return 'XONG'
    phi, r, (bx, by) = sec
    mx, my = to_map(bx, by)
    io.bao('SEP_LIDAR: THOAT diem (%.2f; %.2f): sector %+d do, %.2f m (%s).' % (mx, my, int(round(math.degrees(phi))), r, ly))
    k0 = SL.tom_tat(quet_moi(io, kn)[0] or [])
    kq = di_diem(io, cfg, kn, luot, (mx, my), th + phi, k0)
    io.bao('   THOAT_XONG: diem => %s.' % kq)
    if kq == 'HAN_GAN':
        return 'HAN_GAN'
    if kq != 'TOI':
        kn.ngay('THOAT', 'diem=%.2f' % r, 'HONG', kq, p)
        return 'HONG'
    kn.doi('THOAT', 'diem=%.2f' % r, p, ly)
    return 'XONG'


def di_diem(io, cfg, kn, luot, diem, yaw, k0=None, han=None, phu=False):
    """navigate_to_pose toi diem thoat, han thoat_han_s; vung do TE HON luc bat dau (k0, Q3 khong xet khe) => huy.
    -> 'TOI' (SUCCEEDED / cach diem <= tien_min) | 'HONG_<ly>' | 'HAN_GAN'."""
    if not io.gui_nav(diem, yaw):
        return 'HONG_TU_CHOI'
    t, mat, t0 = SL.tham_so(), mat_cua(io, kn), io.now()
    han = cfg['thoat_han'] if han is None else han       # 10/11 muc 6/7: diem dan / diem vuot co han rieng; phu: khong xet han gan (K3)
    if not t['ra_xa']:
        k0 = None                                        # 629a17b: k0 truyen vao bi bo, lay o lan quet dau khi xe da chay
    while True:
        kq = io.ket_qua()
        p = io.pose()
        toi = p is not None and math.hypot(p[0] - diem[0], p[1] - diem[1]) <= cfg['tien_min']
        if kq is not None:
            return 'TOI' if kq == 'SUCCEEDED' or toi else 'HONG_%s' % kq
        tq, _, ds, _ = mat.quet()
        if tq == 'OK':
            k0 = k0 or SL.tom_tat(ds)
            ly = SL.ly_do_huy(SL.tom_tat(ds), k0, t, xet_khe=False)      # vung do te hon luc bat dau => huy
            if ly is not None:
                io.huy()
                return 'TOI' if toi else 'HONG (%s)' % ly
        if not phu and luot.het_han_gan(io.now()):
            io.huy()
            return 'HAN_GAN'
        if io.now() - t0 >= han - 1e-9:
            io.huy()
            return 'TOI' if toi else 'HONG_HET_%ds' % int(han)
        kn.nhip()
        io.ngu(0.1)


def di_vuot(io, cfg, kn, luot, P, dich):
    """10/11 muc 7: sau luot bi chan tai P (va THOAT xong): chon V (chon_diem_vuot, LiDAR tai cho + ban do), di toi V (han vuot_han)
    roi nguoi goi gui dich. Khong co V / hong => bo (Nav2 tu vach nhu cu). -> 'TOI' | 'BO'."""
    ds, pts = quet_moi(io, kn)
    p = io.pose()
    if p is None or pts is None:
        io.bao('DIEM_VUOT: bo (khong co vi tri / LiDAR).')
        return 'BO'
    vat, tuong = diem_phan_loai(ds, pts, p, io.gac_ban(), SL.tham_so())
    c = chon_diem_vuot(P, dich, p, vat, tuong, io.gac_ban())
    if c is None:
        io.bao('DIEM_VUOT: P (%.2f; %.2f) => khong co V hop le, bo (Nav2 tu vach).' % P)
        return 'BO'
    io.bao('DIEM_VUOT: P (%.2f; %.2f) => V (%.2f; %.2f), phia %s, ly do %s.' % (P[0], P[1], c['V'][0], c['V'][1], c['phia'], c['ly']))
    return di_phu(io, cfg, kn, luot, c['V'], c['yaw'], cfg['vuot_han'], 'DIEM_VUOT')


def di_phu(io, cfg, kn, luot, diem, yaw, han, ten):
    """10/11 muc 6 / 7 + K2 + K3: di toi diem phu (dan / vuot). Nav2 tu choi / huy / vung do / qua han => 'BO' (KHONG thoat, KHONG tinh
    luot). Thoi gian di khong tinh vao han gan dich. -> 'TOI' | 'BO'."""
    t0 = io.now()
    k0 = SL.tom_tat(quet_moi(io, kn)[0] or [])
    kq = di_diem(io, cfg, kn, luot, diem, yaw, k0, han=han, phu=True)
    if luot.t_gan0 is not None:
        luot.t_gan0 += io.now() - t0                     # K3: khong tinh vao han gan dich
    io.bao('   %s_XONG: (%.2f; %.2f) => %s.' % (ten, diem[0], diem[1], kq))
    return 'TOI' if kq == 'TOI' else 'BO'


def don_sau_chuyen(io, ma, huong):
    """Kieu mat, MOI loi ra (0, 1, 3, 4, 5, 130): (1) /chon_tuyen "0"; (2) ma != 0: chup; (3) noi log; (4) CSV + ma thoat da
    ghi trong chay_mat / chay_co_ngat. Loi o mot viec khong chan viec sau. Tra danh sach (viec, ket qua)."""
    out = []
    for ten, lam in (('mask0', lambda: io.chon_tuyen('0')),
                     ('chup', (lambda: io.chup(huong, 'ket thuc ma %d' % ma)) if ma != 0 else None),
                     ('log', lambda: io.noi_log(huong, ma))):
        if lam is None:
            out.append((ten, 'khong can'))
            continue
        try:
            lam()
            out.append((ten, 'ok'))
        except Exception as e:                       # noqa: BLE001 (don dep: khong de mot loi chan viec sau)
            out.append((ten, 'LOI %s: %s' % (e.__class__.__name__, str(e)[:80])))
    return out


def dung_sep(io, nhan, r, lan):
    io.huy()
    io.bao('DUNG_SEP: sep bao dung. Xe dung yen, khong gui them lenh.')
    io.ghi(nhan, r, 1, lan, 'DUNG_SEP', 0)
    return 4


MA_LOI = 5                 # 06/11 viec 121: loi khac khi khong bi ngat (khong trung 0, 1, 2, 3, 4, 130)
DA_NGAT = [False]          # 06/11 viec 121: ngat() dat co TRUOC khi raise


def ngat(signum, frame):
    """Bat SIGINT/SIGTERM (main dang ky). 06/11 viec 121: dat co roi moi raise: neu KeyboardInterrupt roi vao luc rclpy
    dang lay tin (take_message), pybind doi no thanh RuntimeError; co nay van cho biet la bi ngat."""
    DA_NGAT[0] = True
    raise KeyboardInterrupt


def chay_co_ngat(io, huong, tuyen_dau, cfg, bo_qua_ngat=lambda: None, da_ngat=lambda: DA_NGAT[0], chay_fn=None):
    """chay(); bi ngat (SIGINT/SIGTERM => KeyboardInterrupt) thi huy dich dang chay, ghi dong NGAT (dong ket thuc chuyen,
    sep.py dong chuyen), tra 130. bo_qua_ngat: tat bat tin hieu lan 2 trong luc huy (main dat SIG_IGN).
    06/11 viec 121 (gia lap 03:56: SIGINT thanh RuntimeError, khong huy dich, xe di tiep 26 s): bat MOI loi (BaseException).
    Bi ngat (KeyboardInterrupt hoac co da_ngat) => nhu cu (HUY, huy dich, NGAT, 130). Loi khac => van huy dich dang chay
    (con dich thi ghi HUY truoc), in + ghi LOI_GUI_DUONG, tra MA_LOI.
    08/11 bo 1e: chay_fn = chay_mat cho kieu mat (None = chay, kieu cu y het)."""
    try:
        return (chay_fn or chay)(io, huong, tuyen_dau, cfg)
    except BaseException as e:             # noqa: B036 (co y: ca KeyboardInterrupt lan RuntimeError cua rclpy)
        bi_ngat = isinstance(e, KeyboardInterrupt) or da_ngat()
        bo_qua_ngat()
        if bi_ngat or io.co_dich():
            io.huy_luot()                  # 05/11 lop 2: ghi HUY vao ~/gui_duong_luot truoc (phu lai dung lui lop 2 <= 1 s)
        kq = io.huy_khi_ngat()
        if bi_ngat:
            io.bao('HUY_DICH_KHI_NGAT %s' % kq)
            io.ghi('%s_day' % huong, '-', 'ngat', 0, 'NGAT', 0)
            return 130
        io.bao('LOI_GUI_DUONG %s: %s; HUY_DICH %s' % (e.__class__.__name__, str(e)[:160], kq))
        io.ghi('%s_day' % huong, '-', 'loi', 0, 'LOI_GUI_DUONG', 0)
        return MA_LOI


def cho_truoc_khi_gui(io, cfg):
    """Truoc moi lan gui: DUNG => 'DUNG'; CHO => cho cho_sep s (toi da 2 lan); phu lai dang lai / sep dang giao thoat => cho
    toi khi xong (toi da cho_hong s). Tra 'DUNG' hoac None."""
    lan_cho = 0
    t_het = io.now() + cfg['cho_hong']
    while True:
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        if lenh and lenh[0] == 'CHO' and lan_cho < 2:
            lan_cho += 1
            io.bao('   sep bao CHO (%s): dung yen %d s (lan %d/2).' % (lenh[1], cfg['cho_sep'], lan_cho))
            if cho(io, cfg['cho_sep']) == 'DUNG':
                return 'DUNG'
            continue
        if (io.phu_lai_ban() or io.sep_dang_giao_thoat()) and io.now() < t_het:
            if cho(io, 0.5) == 'DUNG':
                return 'DUNG'
            continue
        return None


def dong_sap_gui(t, huong, tuyen, lan, ma):
    """04/11 lop 2: dong ~/gui_duong_luot (sep.doc_sap_gui doc lai dung khuon nay)."""
    return '%.2f SAP_GUI %s %d %d %d' % (t, huong, tuyen, lan, ma)


def dong_huy(t):
    """05/11 lop 2: dong ~/gui_duong_luot khi bi ngat (lai_xe_lidar.luot_huy doc; sep.doc_sap_gui bo qua)."""
    return '%.2f HUY' % t


def cho_lop2(io, cfg, huong, r, lan):
    """04/11 lop 2, truoc moi lan gui: ghi SAP_GUI, cho sep tra loi (~/sep_thoat). KHONG_CAN => gui ngay. RA_LUI / RA_TIEN => cho phu lai
    bao RA_XONG (toi da lop2_cho s tinh tu SAP_GUI). RA_XONG moi sau SAP_GUI cung tinh (phu lai co the xong truoc khi doc RA_LUI).
    Sep khong tra loi sau lop2_tra s hoac het lop2_cho s => in RA_HET_GIO, gui nhu cu. ~/sep_lenh DUNG => 'DUNG'.
    Tra 'KHONG_CAN' | 'RA_XONG' | 'RA_HET_GIO' | 'DUNG'."""
    ma = io.gui_sap_gui(huong, r, lan)
    n0, t0, ra = io.so_lan_ra_xong(), io.now(), False
    while True:
        tra = io.tra_lop2()
        if tra is not None and tra['ma_luot'] == ma:
            if tra['chu'] == 'KHONG_CAN':
                return 'KHONG_CAN'
            if not ra:
                ra = True
                chu, lam = ('RA_TIEN', 'tien ra') if tra['chu'] == 'RA_TIEN' else ('RA_LUI (vat sat mui)', 'lui ra')
                io.bao('   lop 2: sep bao %s: cho phu lai %s, toi da %d s.' % (chu, lam, cfg['lop2_cho']))
        if io.so_lan_ra_xong() != n0:
            io.bao('   lop 2: RA_XONG sau %.1f s.' % (io.now() - t0))
            return 'RA_XONG'
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        da = io.now() - t0
        if (not ra and da >= cfg['lop2_tra']) or da >= cfg['lop2_cho']:
            io.bao('   RA_HET_GIO: %s; gui nhu cu.' % (('sep khong tra loi sau %d s' % cfg['lop2_tra']) if not ra else
                                                     ('khong thay RA_XONG sau %d s' % cfg['lop2_cho'])))
            return 'RA_HET_GIO'
        io.ngu(0.1)


def cho_sau_hong(io, cfg):
    """Sau mot lan hong: cho it nhat cho_hong s; het thi cho them toi khi phu lai xong (toi da gap doi). DUNG => 'DUNG'."""
    t0 = io.now()
    while io.now() - t0 < 2 * cfg['cho_hong']:
        if io.now() - t0 >= cfg['cho_hong'] and not io.phu_lai_ban() and not io.sep_dang_giao_thoat():
            return None
        if cho(io, 0.5) == 'DUNG':
            return 'DUNG'
    return None


def cho(io, giay):
    """Cho `giay` s; giua chung ~/sep_lenh bao DUNG thi tra 'DUNG' ngay."""
    t0 = io.now()
    while io.now() - t0 < giay:
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        io.ngu(min(0.1, giay))
    return None


def theo_doi(io, duong, cfg):
    """Gui duong, theo doi toi khi xong. Tra 'SUCCEEDED' | 'ABORTED' | 'CANCELED' | 'TU_CHOI' | 'QUA_GIO' | 'DUNG'."""
    io.phat_duong(duong)
    n_xong = io.so_lan_phu_lai_xong()
    if not io.gui(duong):
        return 'TU_CHOI'
    t0 = t_phat = io.now()
    while True:
        if io.so_lan_phu_lai_xong() != n_xong:        # 18:09: phu lai XOAY xong (xe da doi cho, doi huong): bo duong cu
            io.huy()
            io.da_xu_ly_thoat()
            return 'TINH_LAI'
        kq = io.ket_qua()
        if kq is not None:
            return kq
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            io.huy()
            return 'DUNG'
        if io.now() - t0 > cfg['to']:
            io.huy()
            return 'QUA_GIO'
        if io.now() - t_phat >= HV.so('nhip_duong_day_s'):
            io.phat_duong(cat_con_lai(duong, io.pose()))
            t_phat = io.now()
        io.ngu(0.1)


# ---------- nut ROS 2 ----------

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from nav2_msgs.action import BackUp, DriveOnHeading, FollowPath, NavigateToPose, Spin
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    import tf2_ros
except ImportError:
    rclpy = None

if rclpy is not None:
    import sep as S          # chi dung doc_rosout (mot khuon cau log lai_xe_lidar cho ca sep va gui_duong)
    import lai_xe_lidar as L  # chi dung doc_sep_thoat

    class GuiDuongNode(Node):
        def __init__(self, cfg, kieu='cu'):
            super().__init__('gui_duong')
            self.cfg = cfg
            self.kieu = kieu                           # 08/11 bo 1e: 'mat' | 'cu' (doc_goc)
            self.dong = []                             # moi dong da in (kieu mat noi vao ~/log_gui_duong.txt)
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            # costmap cua Nav2: KeepLast(1), TRANSIENT_LOCAL, RELIABLE => nghe cung QoS de nhan ngay ban dang giu
            giu = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_costmap, giu)
            self.create_subscription(Log, '/rosout', self._on_rosout, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
            self.pub = self.create_publisher(Path, '/duong_day', giu)
            self.ac = ActionClient(self, FollowPath, 'follow_path')
            self.cm, self.cm_ban, self.l1 = None, None, ''
            self.n_xong, self.ma_xu_ly = 0, None       # 18:09: so lan phu lai XOAY xong; ma ~/sep_thoat da xu ly
            self.n_ra_xong = 0                         # 04/11 lop 2: so lan phu lai vao RA_XONG
            self.tep_luot = os.path.expanduser('~/gui_duong_luot')
            self.goal, self.kq = None, None
            self.tep_lenh = os.path.expanduser('~/sep_lenh')
            self.tep_thoat = os.path.expanduser('~/sep_thoat')
            self.csv = os.path.expanduser('~/ket_qua_chuyen.csv')
            if kieu == 'mat':                          # 08/11 bo 1e: chi kieu mat moi them pub / sub / client moi
                self.pub_chon = self.create_publisher(String, '/chon_tuyen', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
                self.create_subscription(Path, '/plan', self._on_plan, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
                self.ac_nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
                # 09/11 23:08 sep LiDAR: dong tac thoat = behavior_server (ra /cmd_vel_avoid, KHONG qua collision_monitor => tu kiem khe)
                self.ac_dt = {'lui': ActionClient(self, BackUp, 'backup'), 'tien': ActionClient(self, DriveOnHeading, 'drive_on_heading'),
                              'xoay': ActionClient(self, Spin, 'spin')}
                self.fb, self.plan_cuoi = None, []
                self.goal_dt, self.fut_dt, self.kq_dt = None, None, None     # dong tac dang chay (#7 bi ngat thi huy)
                # muc 6 / Q1: tin cua sep.py (JSON /sep/trang_thai), chi la TIN
                self.sep_tt, self.t_sep_tt = None, None
                self.create_subscription(String, '/sep/trang_thai', self._on_sep_tt, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
                self.t_plan, self.t_vi_tri = None, 0.0
                self.tep_vi_tri = os.path.expanduser('~/vi_tri_xe')
                # 09/11 dung im ban 2 (A1, K7, K10, K13): gac doc /scan + /map_tinh (ban do goc, TRANSIENT_LOCAL), TF luc quet
                self.scan, self.t_scan, self.scan_da_xu_ly, self.ban_tinh = None, None, None, None
                self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
                self.create_subscription(OccupancyGrid, '/map_tinh', self._on_map_tinh, giu)
                self.tep_kinh_nghiem = os.path.expanduser('~/kinh_nghiem.csv')
                self.vet_xe = []                         # 09/11 gia X (d): (x, y) moi nhip_vi_tri_s trong chuyen
                self.thu_muc_chup = os.path.expanduser('~/chup')
                self.tep_log = os.path.expanduser('~/log_gui_duong.txt')

        # --- nghe ---
        def _on_costmap(self, msg):
            self.cm, self.cm_ban = msg, None

        def _on_rosout(self, msg):
            if msg.name == 'lai_xe_lidar':
                e = S.doc_rosout(msg.name, msg.msg)
                if e and e[0] == 'L1':
                    if e[1] == 'XOAY_XONG' and self.l1 != 'XOAY_XONG':
                        self.n_xong += 1
                    if e[1] == 'RA_XONG' and self.l1 != 'RA_XONG':
                        self.n_ra_xong += 1
                    self.l1 = e[1]

        # --- io cho chay() ---
        def now(self):
            return time.monotonic()

        def ngu(self, giay):
            t_het = time.monotonic() + giay
            while True:
                if self.kieu == 'mat' and time.monotonic() - self.t_vi_tri >= HV.so('nhip_vi_tri_s'):     # 09/11 buoc 3 #3
                    self._ghi_vi_tri()
                con = t_het - time.monotonic()
                if con <= 0:
                    return
                rclpy.spin_once(self, timeout_sec=min(con, 0.05))

        def _ghi_vi_tri(self):
            """09/11 buoc 3 #3: ~/vi_tri_xe (tep tam roi doi ten): vi tri tf + /plan cua dich navigate_to_pose dang chay."""
            self.t_vi_tri = time.monotonic()
            dang = self.goal is not None and self.kq is None
            p = self.pose()
            if p is not None:
                self.vet_xe.append((p[0], p[1]))         # 09/11 gia X (d): vet that cua chuyen
            try:
                with open(self.tep_vi_tri + '.tam', 'w') as f:
                    f.write(noi_dung_vi_tri(time.time(), p, self.plan_cuoi if dang else []))
                os.replace(self.tep_vi_tri + '.tam', self.tep_vi_tri)
            except OSError:
                pass

        def bao(self, cau):
            self.dong.append(cau)
            print(cau, flush=True)

        def ghi(self, nhan, tuyen, diem, lan, ket_qua, giay, lech='-', lech_fp='-', tuoi='-'):
            moi = not os.path.exists(self.csv)
            with open(self.csv, 'a') as f:
                if moi:
                    f.write(COT_CSV + '\n')
                f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), nhan, tuyen, diem, lan, ket_qua, giay, lech, lech_fp, tuoi))

        def pose(self):
            """(x, y, yaw) tu tf map->base_footprint, None neu khong co hoac cu hon tf_tuoi."""
            try:
                tf = self.tf_buf.lookup_transform('map', 'base_footprint', Time())
            except Exception:
                return None
            st = tf.header.stamp
            if self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9) > self.cfg['tf_tuoi']:
                return None
            q = tf.transform.rotation
            return (tf.transform.translation.x, tf.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

        def ban(self):
            if self.cm is None:
                return None
            if self.cm_ban is None:
                i = self.cm.info
                self.cm_ban = TT.Ban.tu_luoi(i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, list(self.cm.data))
            return self.cm_ban

        def cho_san_sang(self, giay=HV.so('cho_san_sang_s')):
            return cho_tf_costmap(lambda: self.cm is not None, self.ban, self.pose, time.monotonic, self.ngu, giay)

        def lenh_sep(self):
            try:
                with open(self.tep_lenh) as f:
                    return doc_sep_lenh(f.readline(), time.clock_gettime(time.CLOCK_BOOTTIME), self.cfg['sep_tuoi'])
            except OSError:
                return None

        def _thoat_moi(self):
            try:
                with open(self.tep_thoat) as f:
                    return L.doc_sep_thoat(f.readline(), time.clock_gettime(time.CLOCK_BOOTTIME))
            except OSError:
                return None

        def sep_dang_giao_thoat(self):
            """Sep dang giao mot dot thoat CHUA xu ly (ma moi): cho phu lai lam truoc roi moi gui."""
            t = self._thoat_moi()
            return t is not None and t['ma'] != self.ma_xu_ly

        def da_xu_ly_thoat(self):
            t = self._thoat_moi()
            if t is not None:
                self.ma_xu_ly = t['ma']

        def so_lan_phu_lai_xong(self):
            return self.n_xong

        def gui_sap_gui(self, huong, r, lan):
            """04/11 lop 2: ghi ~/gui_duong_luot (tep tam roi doi ten, nhu sep.ghi_lenh). Tra ma."""
            t = time.clock_gettime(time.CLOCK_BOOTTIME)
            ma = int(t * 10)
            with open(self.tep_luot + '.tam', 'w') as f:
                f.write(dong_sap_gui(t, huong, r, lan, ma) + '\n')
            os.replace(self.tep_luot + '.tam', self.tep_luot)
            return ma

        def huy_luot(self):
            """05/11 lop 2: bi ngat => ghi HUY vao ~/gui_duong_luot (phu lai dang lui lop 2 thi dung). Kieu mat: khong dung lop 2."""
            if self.kieu == 'mat':
                return
            try:
                with open(self.tep_luot + '.tam', 'w') as f:
                    f.write(dong_huy(time.clock_gettime(time.CLOCK_BOOTTIME)) + '\n')
                os.replace(self.tep_luot + '.tam', self.tep_luot)
            except OSError:
                pass

        def tra_lop2(self):
            try:
                with open(self.tep_thoat) as f:
                    return L.doc_sep_ra2(f.readline(), time.clock_gettime(time.CLOCK_BOOTTIME))
            except OSError:
                return None

        def so_lan_ra_xong(self):
            return self.n_ra_xong

        def phu_lai_ban(self):
            return phu_lai_ban(self.l1)

        def phat_duong(self, duong):
            self.pub.publish(self._path(duong))

        def _path(self, duong):
            p = Path()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            for x, y, yaw in duong:
                q = PoseStamped()
                q.header = p.header
                q.pose.position.x, q.pose.position.y = float(x), float(y)
                q.pose.orientation.z, q.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                p.poses.append(q)
            return p

        def gui(self, duong):
            if not self.ac.wait_for_server(timeout_sec=HV.so('cho_may_chu_s')):
                self.bao('   khong thay may chu /follow_path (controller_server)')
                return False
            g = FollowPath.Goal()
            g.path = self._path(duong)
            g.controller_id = 'FollowPath'          # nhu bt_cho.xml; goal_checker_id rong = plugin duy nhat (general_goal_checker)
            self.goal, self.kq = None, None
            fut = self.ac.send_goal_async(g)
            t_het = time.monotonic() + HV.so('cho_may_chu_s')
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                self.bao('   /follow_path tu choi dich')
                return False
            self.goal = fut.result()
            self.goal.get_result_async().add_done_callback(self._on_result)
            return True

        def _on_result(self, fut):
            st = fut.result().status
            self.kq = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, 'ABORTED')

        def ket_qua(self):
            return self.kq

        def huy(self):
            if self.goal is not None and self.kq is None:
                fut = self.goal.cancel_goal_async()
                t_het = time.monotonic() + HV.so('cho_huy_s')
                while not fut.done() and time.monotonic() < t_het:
                    self.ngu(0.05)

        def co_dich(self):
            """06/11 viec 121: con dich /follow_path dang chay (da gui, chua co ket qua). 09/11 #7: kieu mat tinh ca /backup cua gui."""
            return (self.goal is not None and self.kq is None) or (self.kieu == 'mat' and self.dong_tac_dang_chay())

        # --- 08/11 bo 1e: io kieu mat ---
        def chon_tuyen(self, s):
            # 08/11 buoc 2 (gia lap): publisher vua tao phat ngay thi DDS chua ghep voi lop_huong_dan => mat "0" dau chuyen.
            # Cho toi da 3 s cho co nguoi nghe roi moi phat; van khong co thi in canh bao (lop_huong_dan chua chay?).
            t_het = time.monotonic() + HV.so('cho_nghe_chon_tuyen_s')
            while self.pub_chon.get_subscription_count() == 0 and time.monotonic() < t_het:
                self.ngu(0.05)
            if self.pub_chon.get_subscription_count() == 0:
                self.bao('   CANH BAO: /chon_tuyen chua co ai nghe (lop_huong_dan?): "%s" co the mat' % s)
            m = String()
            m.data = s
            self.pub_chon.publish(m)

        def _on_plan(self, msg):
            self.plan_cuoi = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.t_plan = time.monotonic()

        def _on_scan(self, msg):
            self.scan, self.t_scan = msg, time.monotonic()

        def _on_map_tinh(self, msg):
            i = msg.info
            self.ban_tinh = GV.BanDo(i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, list(msg.data))

        @staticmethod
        def _tu_the(tf):
            q = tf.transform.rotation
            return (tf.transform.translation.x, tf.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

        def gac_quet(self):
            """20:33 vung quanh xe: lan quet /scan MOI -> ('OK', (t, diem khung xe, diem khung ban do | None)), diem theo THU TU TIA
            (None = tia khong ve), TF TAI THOI DIEM QUET (K7: map <- frame LiDAR theo stamp; base_footprint <- frame LiDAR, tinh).
            TF map luc quet chua toi / chua co /map_tinh => diem ban do None (ban do chi nhac). ('CHUA', None) khi chua co lan quet
            moi hoac TF base_footprint chua co; ('MU', ly) khi /scan cu > lidar_cu_s."""
            if self.scan is None or time.monotonic() - self.t_scan > HV.so('lidar_cu_s'):
                return 'MU', '/scan cu > %g s' % HV.so('lidar_cu_s')
            m = self.scan
            if m is self.scan_da_xu_ly:
                return 'CHUA', None
            try:
                t_xe = self.tf_buf.lookup_transform('base_footprint', m.header.frame_id, Time())
            except Exception:                        # noqa: BLE001 (TF chua toi: vong sau thu lai)
                return 'CHUA', None
            try:
                t_map = self.tf_buf.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp)) if self.ban_tinh else None
            except Exception:                        # noqa: BLE001 (ban do chi nhac: thieu thi bo)
                t_map = None
            self.scan_da_xu_ly = m
            pts = GV.diem_quet(m.ranges, m.angle_min, m.angle_increment, max(m.range_min, HV.than_the('lidar_range_min_m')), m.range_max)
            return 'OK', (time.monotonic(), GV.bien_doi(pts, self._tu_the(t_xe)),
                          None if t_map is None else GV.bien_doi(pts, self._tu_the(t_map)))

        def gac_ban(self):
            return self.ban_tinh

        def tuoi_plan(self):
            """09/11 #5: giay tu /plan cuoi CUA DICH DANG CHAY (gui_nav xoa moc); None neu chua co."""
            return None if self.t_plan is None else time.monotonic() - self.t_plan

        def _on_sep_tt(self, msg):
            self.sep_tt, self.t_sep_tt = msg.data, time.monotonic()

        def tin_sep(self):
            """Q1: /sep/trang_thai -> sep_lidar.doc_tin_sep (cu hon sep_lenh_tuoi_max_s => None = khong co tin)."""
            return SL.doc_tin_sep(self.sep_tt, None if self.t_sep_tt is None else time.monotonic() - self.t_sep_tt,
                                  HV.so('sep_lenh_tuoi_max_s'))

        def o_costmap(self, x, y):
            """Gia tri o /global_costmap/costmap (OccupancyGrid 0..100, -1) tai diem ban do (x, y); ngoai / chua co => None."""
            if self.cm is None:
                return None
            i = self.cm.info
            c, r = int(math.floor((x - i.origin.position.x) / i.resolution)), int(math.floor((y - i.origin.position.y) / i.resolution))
            if not (0 <= c < i.width and 0 <= r < i.height):
                return None
            return self.cm.data[r * i.width + c]

        def _on_fb(self, msg):
            f = msg.feedback
            self.fb = (float(f.distance_remaining), int(f.number_of_recoveries))

        def feedback(self):
            return self.fb

        def gui_nav(self, dich, yaw_cuoi):
            if not self.ac_nav.wait_for_server(timeout_sec=HV.so('cho_may_chu_s')):
                self.bao('   khong thay may chu /navigate_to_pose (bt_navigator)')
                return False
            if yaw_cuoi is None:                     # ve_A: goc cuoi theo huong xe -> dich luc gui
                p = self.pose()
                yaw_cuoi = 0.0 if p is None else math.atan2(dich[1] - p[1], dich[0] - p[0])
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = 'map'
            g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = float(dich[0]), float(dich[1])
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw_cuoi / 2), math.cos(yaw_cuoi / 2)
            self.goal, self.kq, self.fb = None, None, None
            self.plan_cuoi, self.t_plan = [], None          # 09/11 #3 #5: /plan cua dich truoc khong tinh
            fut = self.ac_nav.send_goal_async(g, feedback_callback=self._on_fb)
            t_het = time.monotonic() + HV.so('cho_may_chu_s')
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                self.bao('   /navigate_to_pose tu choi dich')
                return False
            self.goal = fut.result()
            self.goal.get_result_async().add_done_callback(self._on_result)
            return True

        def dong_tac(self, dt, v, dung_khi=None):
            """09/11 23:08 sep LiDAR: mot dong tac Nav2 behavior: ('tien', m) DriveOnHeading, ('lui', m) BackUp (m, v > 0; Humble tu lui),
            ('xoay', rad) Spin. Moi 0,1 s: dung_khi() (lan quet moi, Q3) tra ly do => huy. Tra (ket qua, quang xe dich theo tf | None)."""
            loai, g = dt
            ac = self.ac_dt[loai]
            if not ac.wait_for_server(timeout_sec=HV.so('cho_may_chu_s')):
                self.bao('   khong thay may chu %s (behavior_server)' % {'lui': '/backup', 'tien': '/drive_on_heading', 'xoay': '/spin'}[loai])
                return 'TU_CHOI', None
            p0 = self.pose()
            if loai == 'xoay':
                gl = Spin.Goal()
                gl.target_yaw = float(g)
                han = abs(g) / HV.so('xoay_thoat_rad_s') * 2.0 + 5.0
            else:
                gl = BackUp.Goal() if loai == 'lui' else DriveOnHeading.Goal()
                gl.target.x, gl.speed = float(g), float(v)
                han = g / v * 2.0 + 5.0
            gl.time_allowance.sec = int(math.ceil(han))
            self.goal_dt, self.kq_dt = None, None
            fut = self.fut_dt = ac.send_goal_async(gl)
            t_het = time.monotonic() + HV.so('cho_may_chu_s')
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                return 'TU_CHOI', None
            self.goal_dt = fut.result()
            self.goal_dt.get_result_async().add_done_callback(self._on_ket_dt)
            t_het = time.monotonic() + han + 5.0
            kq = None
            while kq is None:
                kq = self.kq_dt
                ly_dung = None if kq is not None or dung_khi is None else dung_khi()
                if ly_dung:
                    self._huy_dt()
                    kq = ly_dung
                elif kq is None and time.monotonic() > t_het:
                    self._huy_dt()
                    kq = 'QUA_GIO'
                if kq is None:
                    self.ngu(0.1)
            p1 = self.pose()
            if p0 is None or p1 is None:
                return kq, None
            return kq, (abs(math.degrees((p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi)) if loai == 'xoay'
                        else math.hypot(p1[0] - p0[0], p1[1] - p0[1]))

        def _on_ket_dt(self, fut):
            st = fut.result().status
            self.kq_dt = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, 'ABORTED')

        def dong_tac_dang_chay(self):
            """09/11 #7: dong tac cua gui dang chay (da chap nhan chua xong) hoac dang cho chap nhan."""
            if self.fut_dt is None:
                return False
            if not self.fut_dt.done():
                return True
            return self.goal_dt is not None and self.kq_dt is None

        def _huy_dt(self):
            """Huy dong tac cua gui (cho chap nhan toi 2 s neu chua). Tra 'ok' | 'khong_tra_loi' | 'khong_co_lui'."""
            if not self.dong_tac_dang_chay():
                return 'khong_co_lui'
            t_het = time.monotonic() + HV.so('cho_huy_s')
            while not self.fut_dt.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not self.fut_dt.done():
                return 'khong_tra_loi'
            r = self.fut_dt.result()
            if not r.accepted or self.kq_dt is not None:
                return 'khong_co_lui'
            fut = r.cancel_goal_async()
            while not fut.done() and time.monotonic() < t_het + HV.so('cho_huy_s'):
                self.ngu(0.05)
            return 'ok' if fut.done() else 'khong_tra_loi'

        def ghi_kinh_nghiem(self, dong):
            ghi_kinh_nghiem(self.tep_kinh_nghiem, dict(dong, gio=time.strftime('%Y-%m-%d %H:%M:%S')))

        def doc_kinh_nghiem(self):
            return HV.doc_kinh_nghiem(self.tep_kinh_nghiem)

        def vet_chuyen(self):
            return list(self.vet_xe)

        def ma_chuyen(self):
            """09/11 nao xe viec 4: cot chuyen cua ~/kinh_nghiem.csv = gio bat dau chuyen."""
            return time.strftime('%m%d_%H%M%S')

        def chup(self, huong, ly):
            os.makedirs(self.thu_muc_chup, exist_ok=True)
            goc = os.path.join(self.thu_muc_chup, '%s_%s' % (time.strftime('%H%M%S'), huong))
            p, k = goc + '.txt', 1
            while os.path.exists(p):
                k += 1
                p = '%s_%d.txt' % (goc, k)
            cm = None
            if self.cm is not None:
                i = self.cm.info
                cm = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, list(self.cm.data))
            with open(p, 'w') as f:
                f.write(noi_dung_chup(ly, time.strftime('%Y-%m-%d %H:%M:%S'), self.pose(), self.fb, self.plan_cuoi, cm))
            return p

        def noi_log(self, huong, ma):
            with open(self.tep_log, 'a') as f:
                f.write('== %s gui_duong kieu %s %s: ma %d ==\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), self.kieu, huong, ma))
                for c in self.dong:
                    f.write(c + '\n')

        def huy_khi_ngat(self):
            """Bi ngat (SIGINT/SIGTERM): huy dich /follow_path dang chay (handle hien co) khi context con song, cho <= 2 s.
            09/11 buoc 3 #7 (kieu mat): huy ca dong tac dang chay (in '   HUY dong tac dang chay: ...'). Tra chung:
            'ok' | 'khong_tra_loi' (mot trong hai khong tra loi) | 'khong_co_dich' (khong co gi dang chay)."""
            kq = 'khong_co_dich'
            if self.goal is not None and self.kq is None:
                fut = self.goal.cancel_goal_async()
                t_het = time.monotonic() + HV.so('cho_huy_s')
                while not fut.done() and time.monotonic() < t_het:
                    self.ngu(0.05)
                kq = 'ok' if fut.done() else 'khong_tra_loi'
            if self.kieu == 'mat':
                kl = self._huy_dt()
                if kl != 'khong_co_lui':
                    self.bao('   HUY dong tac dang chay: %s' % kl)
                    kq = kl if kq != 'khong_tra_loi' else kq
            return kq

    def main():
        a = sys.argv[1:]
        if not a or a[0] not in DICH or (len(a) > 1 and a[1] not in ('1', '2')):
            print('SAI DOI SO. Dung: python3 ~/gui_duong.py di_B | ve_A [tuyen_dau 1|2]')
            sys.exit(2)
        # 03/11 (viec 5): rclpy KHONG cai bo bat tin hieu (no tat context truoc, huy dich khong di duoc);
        # SIGINT/SIGTERM => KeyboardInterrupt, huy dich khi context con song, roi thoat 130
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGINT, ngat)            # 06/11 viec 121: ngat() o phan thuan (dat co DA_NGAT roi raise)
        signal.signal(signal.SIGTERM, ngat)
        kieu = doc_goc()                                     # 08/11 bo 1e: 'mat' | 'cu' (~/goc_dang_chay, bat_xe.sh ghi moi lan bat)
        node = GuiDuongNode(cau_hinh(), kieu)
        node.bao('KIEU gui_duong: %s (~/goc_dang_chay)' % kieu)

        def bo_qua_ngat():                                   # khong ngat lan 2 trong luc huy
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)

        ma = 1
        try:
            ma = chay_co_ngat(node, a[0], int(a[1]) if len(a) > 1 else 1, node.cfg, bo_qua_ngat,
                              chay_fn=chay_mat if kieu == 'mat' else None)
            if kieu == 'mat':                                # moi loi ra (0, 1, 3, 4, 5, 130): don sau chuyen
                bo_qua_ngat()
                for ten, kq in don_sau_chuyen(node, ma, a[0]):
                    if kq != 'ok' and kq != 'khong can':
                        print('   DON %s: %s' % (ten, kq), flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        sys.exit(ma)
else:
    def main():
        raise SystemExit('can ROS 2 (rclpy, nav2_msgs) de chay nut; chi import duoc phan ham thuan')


if __name__ == '__main__':
    main()
