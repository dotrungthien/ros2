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
"""
import math
import os
import re
import signal
import sys
import time

import gac_vat_la as GV  # 09/11 dung im ban 2: gac vat la (phan thuan)
import hanh_vi as HV      # 09/11 nao xe viec 2: moi so hanh vi lay tu ~/hanh_vi_xe.yaml (mac dinh = so 2607fee)
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
                # 08/11 bo 1e kieu mat: dong ho bo khe (s), muc tien toi thieu (m), cho sau /chon_tuyen (s), nhip in CHO_VAT (s)
                cho_khe=f('CHO_KHE', h('cho_khe_s')), tien_min=f('TIEN_MIN', h('tien_toi_thieu_m')), cho_chon=h('cho_chon_tuyen_s'),
                nhip_cho=h('nhip_in_cho_s'),
                # 08/11 buoc 2c: truoc khi gui tuyen kia lui thang bang Nav2 BackUp (m, m/s): gia lap xe khong tu quay dau trong khe
                lui_m=h('lui_ra_khe_m'), lui_v=h('toc_do_lui_mps'),
                # 09/11 buoc 3 #2: gan dich (m), khong tien bao lau thi huy (s), dung yen bao lau roi gui lai 1 lan (s)
                gan_dich_m=f('GAN_DICH_M', h('gan_dich_m')), gan_dich_s=f('GAN_DICH_S', h('gan_dich_s')), cho_dich=f('CHO_DICH', h('cho_dich_s')),
                gac=h('gac_vat_la'),                     # 09/11 dung im ban 2: gac vat la (false = hanh vi 48995c2)
                so_vong=int(env.get('SO_VONG_KHE', h('so_vong_khe'))),     # 09/11 18:44 ②a: luan phien khe bay nhieu vong
                cho_lai=f('CHO_LAI', h('cho_lai_s')))    # 09/11 18:47 (i): het vong => CHO_NGUOI, tu thu lai 1 vong sau bay lau (<= 0: HET_DUONG nhu cu)


# ---------- 08/11 bo 1e: kieu mat (phan thuan) ----------
TEP_GOC = os.path.expanduser('~/goc_dang_chay')
COT_KINH_NGHIEM = HV.COT_KINH_NGHIEM     # 09/11 nao xe viec 4: 'gio,chuyen,x,y,huong,su_co,cach,ket_qua,ly_do' (hanh_vi.doc_kinh_nghiem doc ca dong cu)
BO_QUA_KHI_GUI_LAI = ('PLAN_', 'ABORT_NHANH')           # ly do CHO cua sep bo qua khi gui lai sau khi bo khe (sep.py:370)
LUI_BT_M = HV.so('lui_thoat_goc_m')   # 09/11 buoc 3 #6: backup_dist cua nhanh FollowPath trong bt_mat.xml (test kiem khop)
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


CACH_LUI = ('BT_LUI', 'GUI_LUI')     # su_co ~/kinh_nghiem.csv: lui ra roi di tiep da DUOC


def cach_kinh_nghiem(dong, p, r):
    """09/11 18:44 ②b (kinh nghiem CHI LA HUONG DAN): dong ket_qua DUOC (khong tinh DA_GIAI) gan p nhat trong r m
    -> (su_co, cach, khoang cach) | None (khong co / p None => thu tu cu). Bo khe: BT_LUI => lui roi gui lai cung khe truoc;
    GAN_DICH: BT_LUI | GUI_LUI => lui truoc khi cho. Su co khac => thu tu cu (chi in)."""
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
    """09/11 nao xe viec 4: ghi ~/kinh_nghiem.csv moi su co cua mot chuyen kieu mat (su_co, cach = so thoi quen da dung).
    ngay(): ghi ngay (DOI_KHE: HONG luc bo khe, nhu cu; DONG_BANG: lui ma xe dich < o_luoi_m). doi(): cho ket qua: trong
    quan_sat s sau do xe TIEN (DongHo) hoac toi dich => DUOC; het quan_sat s / dich hong => HONG. ket(duoc): chot moi dong cho."""

    def __init__(self, io, huong, chuyen, quan_sat, o_luoi):
        self.io, self.huong, self.chuyen, self.qs, self.o_luoi = io, huong, chuyen, quan_sat, o_luoi
        self.cho = []
        self.gac, self.thay_lui = None, None   # 09/11 dung im: su co gac dang mo; (su_co, cach, ly_do, p) thay cho GUI_LUI
        self.mu = False                        # gac dang mu (CANH BAO 1 lan moi dot, chung cong + vong theo doi)

    def mo_gac(self, p, t):
        """Su co gac (CHO_VAT_LA) bat dau: 1 su co = 1 dong, du gac bat / tat bao nhieu lan."""
        self.gac = dict(p=p, t=t)

    def dong_gac(self, ket_qua, t):
        """Xe TIEN lai trong su co gac (vung sach, gui lai) => 1 dong CHO_VAT_LA (cach mac dinh lui_doi_khe)."""
        if self.gac is not None:
            self.ngay('CHO_VAT_LA', 'lui_doi_khe', ket_qua, '-', self.gac['p'])
            self.gac = None

    def gac_sang_lui(self, ly_do):
        """Het cho_khe_s trong su co gac => KHONG ghi DOI_KHE; dong CHO_VAT_LA ghi o buoc lui (thay GUI_LUI) va doi ket qua."""
        self.thay_lui = ('CHO_VAT_LA', 'lui_doi_khe', ly_do, self.gac['p'])
        self.gac = None

    def _ghi(self, su_co, cach, ket_qua, ly_do, p):
        self.io.ghi_kinh_nghiem(dict(chuyen=self.chuyen, x=None if p is None else p[0], y=None if p is None else p[1],
                                     huong=self.huong, su_co=su_co, cach=cach, ket_qua=ket_qua, ly_do=ly_do))

    def ngay(self, su_co, cach, ket_qua, ly_do, p):
        self._ghi(su_co, cach, ket_qua, ly_do, p)

    def doi(self, su_co, cach, p, ly_do='-'):
        self.cho.append((su_co, cach, ly_do, p, self.io.now()))

    def xong_lui(self, cach, p0, p1, su_co='BT_LUI'):
        """Mot lan lui xong: xe dich < o_luoi_m (biet ca hai vi tri) => DONG_BANG HONG; khac => cho ket qua su_co."""
        if p0 is not None and p1 is not None and math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < self.o_luoi:
            self.ngay('DONG_BANG', cach, 'HONG', su_co, p1)
        else:
            self.doi(su_co, cach, p1)

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
        """Het chuyen: dong con cho + su co gac con mo / cho lui ma khong lui duoc => chot."""
        self.ket(duoc)
        if self.thay_lui is not None:
            su_co, cach, ly_do, p = self.thay_lui
            self.ngay(su_co, cach, 'DUOC' if duoc else 'HONG', ly_do, p)
            self.thay_lui = None
        self.dong_gac('DUOC' if duoc else 'HONG', None)


class _KhongGhi(SoKinhNghiem):
    def __init__(self):
        self.cho = []
        self.gac, self.thay_lui, self.mu = None, None, False

    def _ghi(self, *a):
        pass

    def nhip(self):
        pass


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
    """08/11 bo 1e: mot chuyen kieu mat (thiet ke ban 3 muc 1 + 3b). Cung CSV, cung chu log ket thuc, cung ma thoat voi chay().
    09/11 nao xe viec 4: ghi ~/kinh_nghiem.csv (SoKinhNghiem); het chuyen binh thuong: dong con cho => DUOC neu den noi."""
    kn = SoKinhNghiem(io, huong, io.ma_chuyen(), HV.so('kinh_nghiem_quan_sat_s'), HV.than_the('o_luoi_m'))
    ma = _chay_mat(io, huong, tuyen_dau, cfg, kn)
    kn.het_chuyen(ma == 0)
    if ma == 0:
        ghi_da_giai(io, huong, kn.chuyen)                # 09/11 gia X (d)
    return ma


def _chay_mat(io, huong, tuyen_dau, cfg, kn):
    nhan = '%s_day' % huong
    thu_tu = (1, 2) if tuyen_dau == 1 else (2, 1)
    (dx, dy), yaw_cuoi = DICH[huong]
    t_chuyen = io.now()
    io.chon_tuyen('0')                                   # 3b #3: bo o cam cu (kill -9 / loi ra 130, 5 cua chuyen truoc)
    io.ngu(cfg['cho_chon'])
    pose, ban = io.cho_san_sang()
    while pose is None or ban is None or ban.vat(ban.o(pose[0], pose[1])):
        ly = ('khong co %s sau 15 s' % thieu_gi(pose, ban)) if pose is None or ban is None else 'xe nam trong o do cua costmap'
        if cfg.get('cho_lai', 0) <= 0:
            io.bao('DUNG: SAI_VI_TRI (%s). KHONG gui lenh nao.' % ly)
            io.ghi(nhan, '-', 1, 0, 'SAI_VI_TRI', 0)
            return 3
        cho_nguoi(io, cfg, nhan, 'SAI_VI_TRI (%s), KHONG gui lenh' % ly)     # 09/11 19:05: khong thoat, cho nguoi roi kiem lai
        pose, ban = io.cho_san_sang()
    thu_tu = thu_tu_gia_x(io, thu_tu, pose, ban, (dx, dy), yaw_cuoi)     # 09/11 gia X (c)
    io.ghi(nhan, thu_tu[0], 1, 0, 'KIEM_DAT', 0)
    gui_lai = False
    hang = [thu_tu[k % 2] for k in range(2 * max(1, cfg.get('so_vong', 1)))]    # 09/11 18:44 ②a: luan phien khe so_vong vong
    i = -1
    while True:
        if i + 1 >= len(hang):                           # 09/11 18:47 (i): het vong KHONG thoat: dung yen cho nguoi, roi thu lai 1 vong
            if cfg.get('cho_lai', 0) <= 0:
                break
            cho_nguoi(io, cfg, nhan, 'da thu %d lan khe' % len(hang))
            hang += list(thu_tu)
        i += 1
        r = hang[i]
        if not cfg.get('gac') and hoi_sep_mat(io, cfg, gui_lai) == 'DUNG':     # gac bat: hoi sep SAU gac cong (duoi)
            return dung_sep(io, nhan, r, 1)
        if gui_lai:                                      # 08/11 buoc 2c: lui ra khoi khe vua bo truoc khi /chon_tuyen tuyen kia
            io.bao('LUI_RA: gui %.2f (truoc tuyen %d, /backup %.2f m/s)' % (cfg['lui_m'], r, cfg['lui_v']))     # 09/11 #6
            cach_vat = HV.an_toan('cach_vat_toi_thieu_m')
            kq_lui, quang = io.lui(cfg['lui_m'], cfg['lui_v'], (lambda: gac_hong_lui(io, cach_vat)) if cfg.get('gac') else None)
            if kq_lui == 'GAC_HONG':                     # 09/11: vat la cach hong < 0,06 luc lui => da huy /backup, dung im
                io.bao('GAC: huy lui, hong < %.2f' % cach_vat)
            io.bao('   LUI_XONG: gui => %s%s.' % (kq_lui, '' if quang is None else ' (xe lui %.2f m)' % quang))
            if kq_lui == 'DUNG':
                return dung_sep(io, nhan, r, 1)
            p1 = io.pose()                               # nao xe viec 4: GUI_LUI (cho ket qua) | DONG_BANG (xe dich < o_luoi_m)
            su_co, cach, ly_lui = kn.thay_lui[:3] if kn.thay_lui else ('GUI_LUI', 'lui=%g' % cfg['lui_m'], '-')
            kn.thay_lui = None                           # 09/11 dung im: su co gac het 30 s => 1 dong CHO_VAT_LA (khong GUI_LUI)
            if quang is not None and quang < kn.o_luoi:
                kn.ngay('DONG_BANG', cach, 'HONG', su_co, p1)
            else:
                kn.doi(su_co, cach, p1, ly_lui)
        if cfg.get('gac'):                               # 09/11 gac cong: vung RA sach 2 s roi moi hoi sep va gui
            t_c = io.now()
            c = cho_cong(io, cfg, kn, True)
            if c == 'DUNG':
                return dung_sep(io, nhan, r, 1)
            if c == 'HET_GIO':                           # nhu tuyen hong (bo khe): con tuyen kia thi lui + cong; het => HET_DUONG
                if kn.gac is not None:
                    kn.gac_sang_lui('CHO_CONG')
                p = io.pose()
                lech = None if p is None else math.hypot(p[0] - dx, p[1] - dy)
                io.bao('   khong den duoc (CHO_CONG_%ds%s).' % (int(cfg['cho_khe']), '' if lech is None else ', cach dich %.2f m' % lech))
                io.ghi(nhan, r, 1, 1, 'KHONG_DAT', int(round(io.now() - t_c)), '-' if lech is None else '%.2f' % lech)
                io.bao('== Tuyen %d hong. ==' % r)
                gui_lai = True
                continue
            if hoi_sep_mat(io, cfg, gui_lai) == 'DUNG':
                return dung_sep(io, nhan, r, 1)
        gui_lai = True
        p = io.pose()
        if p is None:
            p, _ = io.cho_san_sang()
        if p is None:
            io.bao('   tuyen %d: khong co tf moi sau 15 s => doi tuyen' % r)
            io.ghi(nhan, r, 1, 1, 'KHONG_DAT', 0)
            io.bao('== Tuyen %d hong. ==' % r)
            continue
        io.chon_tuyen('%d %.3f %.3f' % (r, p[0], p[1]))
        io.ngu(cfg['cho_chon'])
        t0 = io.now()
        io.bao('-- tuyen %d, lan 1/1: navigate_to_pose tu (%.2f; %.2f) toi (%.2f; %.2f)' % (r, p[0], p[1], dx, dy))
        kq = theo_doi_mat(io, (dx, dy), yaw_cuoi, cfg, kn=kn)
        giay = int(round(io.now() - t0))
        if kq == 'DUNG':
            return dung_sep(io, nhan, r, 1)
        p = io.pose()
        lech = None if p is None else math.hypot(p[0] - dx, p[1] - dy)
        if kq == 'SUCCEEDED' and lech is not None and lech <= cfg['toi_sai_so']:
            io.ghi(nhan, r, 1, 1, 'SUCCEEDED', giay, '%.2f' % lech)
            io.ghi(nhan, r, 'tong', 0, 'DEN_NOI', int(round(io.now() - t_chuyen)))
            io.bao('DEN_NOI: %s xong theo tuyen %d (cach dich %.2f m).' % (huong, r, lech))
            return 0
        if kq == 'GAN_DICH':                               # 09/11 buoc 3 #2: gan dich ma ket => cho, gui lai 1 lan, KHONG doi khe
            io.ghi(nhan, r, 1, 1, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
            return gan_dich(io, cfg, nhan, huong, r, (dx, dy), yaw_cuoi, lech, t_chuyen, kn)
        if kq in ('CHO_VAT_30s', 'KHONG_FEEDBACK'):        # bo khe: nho cho ket + chup bang chung
            ly = ly_do_kinh_nghiem(io.tuoi_plan(), cfg['cho_khe'])        # 09/11 buoc 3 #5
            if kn.gac is not None:                       # 09/11 dung im: su co gac het 30 s => dong CHO_VAT_LA o buoc lui
                kn.gac_sang_lui(ly)
            else:
                kn.ngay('DOI_KHE', 'cho=%g' % cfg['cho_khe'], 'HONG', ly, p)  # nao xe viec 4: cho cho_khe_s khong giai duoc => doi khe
            io.chup(huong, 'bo khe tuyen %d (%s, %s)' % (r, kq, ly))
            c = cach_kinh_nghiem(io.doc_kinh_nghiem(), p, HV.so('gia_x_r_m'))       # 09/11 18:44 ②b: kinh nghiem = huong dan
            if c is not None:
                lai = c[0] == 'BT_LUI' and (i == 0 or hang[i - 1] != r)     # chen 1 lan thu cung khe (khong chen lien tiep)
                io.bao('KINH_NGHIEM: %s DUOC cach %.2f m => %s' % (c[0], c[2], 'lui roi gui lai tuyen %d truoc' % r if lai else 'thu tu cu'))
                if lai:
                    hang.insert(i + 1, r)
        io.bao('   khong den duoc (%s%s).' % (kq, '' if lech is None else ', cach dich %.2f m' % lech))
        io.ghi(nhan, r, 1, 1, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
        io.bao('== Tuyen %d hong. ==' % r)
    io.bao('HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.')
    io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
    return 1


def cho_nguoi(io, cfg, nhan, ly):
    """09/11 18:47 (i) (Thien: dung han roi bao nguoi KHONG phai loi ra): het vong khe => xe dung yen (khong co dich Nav2), 1 dong CSV
    CHO_NGUOI (trang: 도움이 필요해요, tien trinh van song), in 'CHO_NGUOI: n/N s' moi nhip_cho s, het cho_lai s thi tra ve de thu lai
    1 vong khe. Nguoi bam 정지 (SIGINT trong io.ngu) => thoat nhu cu (NGAT 130)."""
    io.bao('CHO_NGUOI: %s, xe dung yen cho nguoi; tu thu lai sau %d s (bam 정지 de dung).' % (ly, int(cfg['cho_lai'])))
    io.ghi(nhan, '-', 'het', 0, 'CHO_NGUOI', 0)
    t0, lan_in = io.now(), 0
    while cfg['cho_lai'] - (io.now() - t0) > 1e-3:      # con < 1 ms thi xong (tranh buoc qua nho bi lam tron mat => lap mai)
        io.ngu(min(cfg['nhip_cho'], cfg['cho_lai'] - (io.now() - t0)))
        da = io.now() - t0
        if int(da // cfg['nhip_cho']) > lan_in and da < cfg['cho_lai']:
            lan_in = int(da // cfg['nhip_cho'])
            io.bao('CHO_NGUOI: %d/%d s' % (int(da), int(cfg['cho_lai'])))
    io.bao('CHO_NGUOI: het %d s, thu lai.' % int(cfg['cho_lai']))


def gan_dich(io, cfg, nhan, huong, r, dich, yaw_cuoi, lech, t_chuyen, kn=None):
    """09/11 buoc 3 #2. Dich DA huy (theo_doi_mat). Dung yen cho_dich s (in 'CHO_VAT: n/60 s' moi nhip_cho s; Nav2 khong co
    dich => cay BT khong chay, khong lui), sep DUNG => DUNG_SEP; roi hoi sep NHU gui lai sau doi khe (hoi_sep_mat gui_lai=True:
    nghe CHO ly do vi tri, bo qua PLAN_* / ABORT_NHANH, luon nghe DUNG) va gui lai 1 lan cung tuyen (khong /chon_tuyen moi, khong
    lui). Lan 2: gan_dich_s s khong tien (o dau cung vay) hoac hong => HET_DUONG (ma 1), KHONG doi khe."""
    io.bao('   GAN_DICH: cach dich %s, khong tien %d s => huy dich, dung yen %d s roi gui lai 1 lan (khong doi khe).' % (
        '?' if lech is None else '%.2f m' % lech, int(cfg['gan_dich_s']), int(cfg['cho_dich'])))
    c = cach_kinh_nghiem(io.doc_kinh_nghiem(), io.pose(), HV.so('gia_x_r_m'))      # 09/11 18:44 ②b: kinh nghiem = huong dan
    if c is not None:
        lui = c[0] in CACH_LUI
        io.bao('KINH_NGHIEM: %s DUOC cach %.2f m => %s' % (c[0], c[2], 'lui %.2f truoc khi cho' % cfg['lui_m'] if lui else 'thu tu cu'))
        if lui:
            cach_vat = HV.an_toan('cach_vat_toi_thieu_m')
            kq_lui, _ = io.lui(cfg['lui_m'], cfg['lui_v'], (lambda: gac_hong_lui(io, cach_vat)) if cfg.get('gac') else None)
            if kq_lui == 'DUNG':
                return dung_sep(io, nhan, r, 2)
    lan_gan = 0                                          # 09/11 19:05: GAN_DICH hong => CHO_NGUOI roi gui lai (khong cho_dich lan nua)
    while True:
        t0, lan_in = io.now(), 0
        while lan_gan == 0 and io.now() - t0 < cfg['cho_dich']:
            lenh = io.lenh_sep()
            if lenh and lenh[0] == 'DUNG':
                return dung_sep(io, nhan, r, 2)
            da = io.now() - t0
            if da >= cfg['nhip_cho'] * (lan_in + 1):
                lan_in = int(da // cfg['nhip_cho'])
                p = io.pose()
                io.bao('CHO_VAT: %d/%d s%s' % (int(da), int(cfg['cho_dich']),
                                               '' if p is None else ' (cach dich %.2f m)' % math.hypot(p[0] - dich[0], p[1] - dich[1])))
            io.ngu(0.1)
        kn = kn or _KhongGhi()
        lan_gan += 1
        if cfg.get('gac'):                                   # 09/11 gac cong truoc khi gui lai (chay TRUOC hoi sep)
            c = cho_cong(io, cfg, kn, False)
            if c == 'DUNG':
                return dung_sep(io, nhan, r, 2)
            if c == 'HET_GIO':
                kn.ngay('CHO_DICH', 'cho=%g' % cfg['cho_dich'], 'HONG', 'CHO_CONG', io.pose())
                p = io.pose()
                lech = None if p is None else math.hypot(p[0] - dich[0], p[1] - dich[1])
                io.bao('   khong den duoc (CHO_CONG_%ds%s).' % (int(cfg['cho_khe']), '' if lech is None else ', cach dich %.2f m' % lech))
                io.ghi(nhan, r, 1, 2, 'KHONG_DAT', 0, '-' if lech is None else '%.2f' % lech)
                if cfg.get('cho_lai', 0) <= 0:
                    io.bao('HET_DUONG: gan dich ma khong toi (GAN_DICH, tuyen %d), khong doi khe. Xe dung yen.' % r)
                    io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
                    return 1
                cho_nguoi(io, cfg, nhan, 'GAN_DICH tuyen %d khong toi' % r)        # 09/11 19:05: khong thoat
                continue
        if hoi_sep_mat(io, cfg, True) == 'DUNG':             # Thien chot 09/11: chung ham hoi sep voi gui lai sau doi khe
            return dung_sep(io, nhan, r, 2)
        kn.doi('CHO_DICH', 'cho=%g' % cfg['cho_dich'], io.pose())        # nao xe viec 4: ket qua theo lan gui lai
        p = io.pose()
        t1 = io.now()
        io.bao('-- tuyen %d, lan 2/2: navigate_to_pose tu %s toi (%.2f; %.2f) (GAN_DICH gui lai)' % (
            r, '(?)' if p is None else '(%.2f; %.2f)' % (p[0], p[1]), dich[0], dich[1]))
        kq = theo_doi_mat(io, dich, yaw_cuoi, cfg, gui_lai_gan=True, kn=kn)
        giay = int(round(io.now() - t1))
        if kq == 'DUNG':
            return dung_sep(io, nhan, r, 2)
        p = io.pose()
        lech = None if p is None else math.hypot(p[0] - dich[0], p[1] - dich[1])
        if kq == 'SUCCEEDED' and lech is not None and lech <= cfg['toi_sai_so']:
            io.ghi(nhan, r, 1, 2, 'SUCCEEDED', giay, '%.2f' % lech)
            io.ghi(nhan, r, 'tong', 0, 'DEN_NOI', int(round(io.now() - t_chuyen)))
            io.bao('DEN_NOI: %s xong theo tuyen %d (cach dich %.2f m).' % (huong, r, lech))
            return 0
        io.bao('   khong den duoc (%s%s).' % (kq, '' if lech is None else ', cach dich %.2f m' % lech))
        io.ghi(nhan, r, 1, 2, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
        if cfg.get('cho_lai', 0) <= 0:
            io.bao('HET_DUONG: gan dich ma khong toi (GAN_DICH, tuyen %d), khong doi khe. Xe dung yen.' % r)
            io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
            return 1
        cho_nguoi(io, cfg, nhan, 'GAN_DICH tuyen %d khong toi' % r)                # 09/11 19:05: khong thoat, cho nguoi roi gui lai


def hoi_sep_mat(io, cfg, gui_lai):
    """Kieu mat, truoc moi lan gui: DUNG => 'DUNG'; CHO => cho cho_sep s (toi da 2 lan). gui_lai (sau khi bo khe): bo qua CHO
    vi PLAN_* / ABORT_NHANH (khe vua bi chan la chuyen da biet); van nghe ly do khac (NHAY_VI_TRI, KHONG_TIN, TF_CU, MAT_SCAN, ...).
    Khong cho phu lai / sep giao thoat (kieu mat: phu lai chi nhin, sep ra_thoat false)."""
    lan_cho = 0
    while True:
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        if lenh and lenh[0] == 'CHO' and lan_cho < 2:
            if gui_lai and lenh[1].startswith(BO_QUA_KHI_GUI_LAI):
                io.bao('   sep bao CHO (%s): bo qua (khe vua bo).' % lenh[1])
                return None
            lan_cho += 1
            io.bao('   sep bao CHO (%s): dung yen %d s (lan %d/2).' % (lenh[1], cfg['cho_sep'], lan_cho))
            if cho(io, cfg['cho_sep']) == 'DUNG':
                return 'DUNG'
            continue
        return None


def theo_doi_mat(io, dich, yaw_cuoi, cfg, gui_lai_gan=False, kn=None):
    """Gui navigate_to_pose, theo doi toi khi xong. Tra 'SUCCEEDED' | 'ABORTED' | 'CANCELED' | 'TU_CHOI' | 'DUNG' |
    'CHO_VAT_30s' | 'KHONG_FEEDBACK' (hai ly do bo khe: da huy dich) | 'GAN_DICH' (09/11 #2: cach dich < gan_dich_m, khong tien
    gan_dich_s s; da huy dich) | 'GAN_DICH_HET' (gui_lai_gan: khong tien gan_dich_s s o bat ky dau; da huy dich).
    09/11 #6: cay BT goi /backup => in 'LUI_RA: bt <m>' / '   LUI_XONG: bt => KQ' (io.su_kien_lui)."""
    kn = kn or _KhongGhi()
    io.su_kien_lui()                                     # bo su kien /backup cu (vd CANCELED cua dich vua huy)
    if not io.gui_nav(dich, yaw_cuoi):
        kn.ket(False)
        return 'TU_CHOI'
    kq, p_ra = _theo_doi_mat(io, dich, yaw_cuoi, cfg, gui_lai_gan, kn)
    if p_ra is not None:                                 # dich xong / bi huy giua luc cay BT dang lui: cay BT huy BackUp
        io.bao('   LUI_XONG: bt => HUY (dich %s).' % kq)
        kn.xong_lui('lui=%g' % LUI_BT_M, p_ra[0], io.pose())
    kn.ket(kq == 'SUCCEEDED')                            # nao xe viec 4: dong con cho chot theo ket qua dich
    return kq


def cho_cong(io, cfg, kn, mo_su_co):
    """09/11 gac cong (Thien): truoc MOI lan gui dich (lan dau, tuyen kia, gui lai GAN) chi gui khi vung RA (0,40 truoc / 0,15 hong)
    sach LIEN TUC gac_sach_s 2 s. Chay TRUOC hoi_sep_mat. Luc cho: dung im (chua co dich), "GAC: cho cong, vung=..., (x; y)" 1 lan,
    "CHO_VAT: n/30 s" moi nhip (n tinh tu luc vao cong). Qua cho_khe_s => 'HET_GIO' (nguoi goi coi nhu tuyen hong; khong vong moi).
    Gac mu => "CANH BAO gac: mu (...)" (1 lan moi dot, chung co kn.mu voi vong theo doi) va cho qua (gac tat). Tra 'QUA' | 'HET_GIO' | 'DUNG'.
    mo_su_co: lan cho nay mo su co CHO_VAT_LA (khong mo khi gui lai GAN: da co CHO_DICH)."""
    t0, t_sach, da_bao, lan_in = io.now(), None, False, 0
    tham = GV.tham_so()
    while True:
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        q = io.gac_quet()
        if q[0] == 'MU':
            if not kn.mu:
                io.bao('CANH BAO gac: mu (%s)' % q[1])
            kn.mu = True
            return 'QUA'
        if q[0] == 'OK':
            kn.mu = False
            t_q, pxe, pmap = q[1]
            d = GV.danh_gia(pxe, pmap, io.gac_ban(), tham, ra=True)
            if d is not None:
                t_sach = None
                if not da_bao:
                    da_bao = True
                    io.bao('GAC: cho cong, vung=%s, (%.2f; %.2f)' % (d[0], d[2][0], d[2][1]))
                    if mo_su_co and kn.gac is None:
                        kn.mo_gac(io.pose(), io.now())
            elif t_sach is None:
                t_sach = t_q
            if t_sach is not None and io.now() - t_sach >= tham['sach_s']:
                return 'QUA'
        da = io.now() - t0
        if da >= cfg['cho_khe']:
            return 'HET_GIO'
        if da_bao and da >= cfg['nhip_cho'] * (lan_in + 1):
            lan_in = int(da // cfg['nhip_cho'])
            io.bao('CHO_VAT: %d/%d s' % (int(da), int(cfg['cho_khe'])))
        kn.nhip()
        io.ngu(0.1)


def bi_chan(r_gac, d_tf, cfg):
    """09/11 dung im ban 2 (K8): gac dang chan => (vung, gan dich?) quyet 30 s (cho_khe_s) hay luat GAN_DICH (60 s); khong => None."""
    if not r_gac or not r_gac['chan']:
        return None
    return r_gac['vung'], d_tf is not None and d_tf < cfg['gan_dich_m']


def gac_hong_lui(io, cach_vat):
    """09/11 (Thien chot sau A1): luc BackUp lui thang, gac KHONG huy theo vung truoc / hong, TRU vat la cach hong < cach_vat
    => 'GAC_HONG'. Mu / chua co lan quet moi => None."""
    q = io.gac_quet()
    if q[0] != 'OK':
        return None
    _, pxe, pmap = q[1]
    g = GV.hong_gan_nhat(pxe, pmap, io.gac_ban(), GV.tham_so())
    return 'GAC_HONG' if g is not None and g < cach_vat else None


def _theo_doi_mat(io, dich, yaw_cuoi, cfg, gui_lai_gan, kn):
    """Vong theo doi cua theo_doi_mat. Tra (ket qua, None | (vi tri luc cay BT bat dau lui,) neu BT con dang lui).
    09/11 dung im ban 2 (cfg['gac']): gac vat la (10 Hz, TF luc quet) bat => TU huy dich (K4: CANCELED sau do KHONG doc, khong
    thanh KHONG_DAT), DUNG IM (khong co dich => khong lenh xoay), log "GAC: dung im, vung=..., (x; y)". Cho bang CHINH dong ho
    "khong tien" nay (K1, K11: khong tao dong ho moi), van in "CHO_VAT: n/30 s" (K12). Vung RA sach 2 s => "GAC: het chan sau N s",
    hoi sep nhu gui lai sau doi khe (hoi_sep_mat gui_lai=True), gui LAI dich cung tuyen (khong tinh tuyen moi; nhieu lan duoc).
    Het cho_khe_s => tra ly do bo khe nhu cu (chay_mat lui 0,35 + doi khe, toi da 1 lan). Gac bat ma cach dich < gan_dich_m (K8)
    => tra 'GAN_DICH' (luat 60 s). Cay BT dang lui: gac KHONG dung theo vung (K3), TRU vat la cach hong < cach_vat_toi_thieu_m
    => huy, dung im, "GAC: huy lui, hong < 0.06" (cung dong ho). Gac mu (khong /map_tinh, /scan cu) => "CANH BAO gac: mu (...)", tat.
    Su co gac (khong gan dich, khong gui lai GAN): kn.gac mo luc dung dau tien; TIEN => 1 dong CHO_VAT_LA DUOC."""
    han = cfg['gan_dich_s'] if gui_lai_gan else cfg['cho_khe']
    dh = DongHo(io.now(), han, cfg['tien_min'])
    n_rec, lan_in, da_in, p_ra = 0, 0, False, None
    gac = GV.Gac(GV.tham_so()) if cfg.get('gac') else None
    cach_vat = HV.an_toan('cach_vat_toi_thieu_m')
    cho_gac, t_chan = False, None                        # cho_gac: dich da TU huy vi gac, dang dung im
    while True:
        for loai, chi_tiet in io.su_kien_lui():
            io.bao('LUI_RA: bt %.2f' % LUI_BT_M if loai == 'RA' else '   LUI_XONG: bt => %s.' % chi_tiet)
            if loai == 'RA':
                p_ra = (io.pose(),)
            else:
                kn.xong_lui('lui=%g' % LUI_BT_M, None if p_ra is None else p_ra[0], io.pose())
                p_ra = None
        kq = None if cho_gac else io.ket_qua()          # K4: CANCELED do gac tu huy: khong doc
        if kq is not None:
            return kq, p_ra
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            io.huy()
            return 'DUNG', p_ra
        fb = None if cho_gac else io.feedback()
        p = io.pose()
        d_tf = None if p is None else math.hypot(p[0] - dich[0], p[1] - dich[1])
        tt, ly, dung = dh.cap_nhat(io.now(), d_tf, None if fb is None else fb[0])
        if fb is not None and fb[1] > n_rec:
            n_rec = fb[1]
            io.bao('   Nav2 hoi phuc: number_of_recoveries = %d.' % n_rec)
        if tt == 'BO_KHE':
            if not cho_gac:
                io.huy()
            return ('GAN_DICH_HET' if gui_lai_gan else ly), p_ra
        if gac is not None:                              # 09/11 dung im ban 2
            q = io.gac_quet()
            if q[0] == 'MU':
                if not kn.mu:
                    io.bao('CANH BAO gac: mu (%s)' % q[1])
                kn.mu = True
            elif q[0] == 'OK':
                kn.mu = False
                t_q, pxe, pmap = q[1]
                r_gac = gac.cap_nhat(t_q, pxe, pmap, io.gac_ban())
                if not cho_gac and p_ra is not None:     # cay BT dang lui thang: chi huy khi hong < cach_vat (K3 + ngoai le)
                    g = GV.hong_gan_nhat(pxe, pmap, io.gac_ban(), gac.t)
                    if g is not None and g < cach_vat:
                        io.huy()
                        cho_gac, t_chan = True, io.now()
                        io.bao('GAC: huy lui, hong < %.2f' % cach_vat)
                elif not cho_gac and bi_chan(r_gac, d_tf, cfg):
                    vung, gan = bi_chan(r_gac, d_tf, cfg)
                    io.huy()
                    cho_gac, t_chan = True, io.now()
                    xy = r_gac['xy']
                    io.bao('GAC: dung im, vung=%s, (%.2f; %.2f)' % (vung, xy[0], xy[1]))
                    if gan and not gui_lai_gan:
                        return 'GAN_DICH', p_ra
                    if not gui_lai_gan and kn.gac is None:
                        kn.mo_gac(p, io.now())
                elif cho_gac and not r_gac['chan']:
                    io.bao('GAC: het chan sau %d s' % int(io.now() - t_chan))
                    if hoi_sep_mat(io, cfg, True) == 'DUNG':
                        return 'DUNG', p_ra
                    if not io.gui_nav(dich, yaw_cuoi):
                        return 'TU_CHOI', p_ra
                    cho_gac = False
        if (tt == 'DI' and not gui_lai_gan and d_tf is not None and d_tf < cfg['gan_dich_m']
                and dung >= cfg['gan_dich_s']):
            io.huy()
            return 'GAN_DICH', p_ra
        if tt == 'TIEN':
            kn.tien()
            kn.dong_gac('DUOC', io.now())
            if da_in:
                io.bao('   DI_TIEP: xe tien lai.')
            lan_in, da_in = 0, False
        elif tt == 'DI' and dung >= cfg['nhip_cho'] * (lan_in + 1):
            lan_in = int(dung // cfg['nhip_cho'])
            da_in = True
            io.bao('CHO_VAT: %d/%d s%s' % (int(dung), int(han), '' if d_tf is None else ' (cach dich %.2f m)' % d_tf))
        kn.nhip()
        io.ngu(0.1)


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
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from nav2_msgs.action import BackUp, FollowPath, NavigateToPose
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
                self.ac_lui = ActionClient(self, BackUp, 'backup')      # 08/11 buoc 2c: behavior_server
                self.fb, self.plan_cuoi = None, []
                # 09/11 buoc 3: /backup cua gui (#7 bi ngat thi huy), su kien /backup cua cay BT (#6), ~/vi_tri_xe (#3), tuoi /plan (#5)
                self.goal_lui, self.fut_lui, self.kq_lui = None, None, None
                self.id_lui_gui, self.lui_bt, self.su_kien = set(), {}, []
                self.create_subscription(GoalStatusArray, '/backup/_action/status', self._on_lui_status,
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
            return (self.goal is not None and self.kq is None) or (self.kieu == 'mat' and self.lui_dang_chay())

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
            """09/11 dung im ban 2 (A1): lan quet /scan MOI -> ('OK', (t, diem khung xe, diem khung ban do)) dung TF TAI THOI
            DIEM QUET (K7: map <- frame LiDAR theo stamp; base_footprint <- frame LiDAR, tinh); ('CHUA', None) khi chua co lan
            quet moi hoac TF luc quet chua toi (thu lai vong sau); ('MU', ly) khi chua co /map_tinh hoac /scan cu > gac_scan_cu_s (K13)."""
            if self.ban_tinh is None:
                return 'MU', 'chua co /map_tinh'
            if self.scan is None or time.monotonic() - self.t_scan > HV.so('gac_scan_cu_s'):
                return 'MU', '/scan cu > %g s' % HV.so('gac_scan_cu_s')
            m = self.scan
            if m is self.scan_da_xu_ly:
                return 'CHUA', None
            try:
                t_map = self.tf_buf.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp))
                t_xe = self.tf_buf.lookup_transform('base_footprint', m.header.frame_id, Time())
            except Exception:                        # noqa: BLE001 (TF luc quet chua toi / ngoai suy: vong sau thu lai)
                return 'CHUA', None
            self.scan_da_xu_ly = m
            pts = GV.diem_quet(m.ranges, m.angle_min, m.angle_increment, max(m.range_min, HV.than_the('lidar_range_min_m')), m.range_max)
            return 'OK', (time.monotonic(), GV.bien_doi(pts, self._tu_the(t_xe)), GV.bien_doi(pts, self._tu_the(t_map)))

        def gac_ban(self):
            return self.ban_tinh

        def tuoi_plan(self):
            """09/11 #5: giay tu /plan cuoi CUA DICH DANG CHAY (gui_nav xoa moc); None neu chua co."""
            return None if self.t_plan is None else time.monotonic() - self.t_plan

        def _on_lui_status(self, msg):
            """09/11 #6: goal /backup KHONG do gui gui (cay BT): vao EXECUTING => su kien RA; ket thuc => XONG <ket qua>."""
            ten = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}
            for st in msg.status_list:
                k = bytes(st.goal_info.goal_id.uuid)
                if k in self.id_lui_gui:
                    continue
                cu = self.lui_bt.get(k)
                if st.status == GoalStatus.STATUS_EXECUTING and cu is None:
                    self.lui_bt[k] = 'RA'
                    self.su_kien.append(('RA', None))
                elif st.status in ten and cu == 'RA':
                    self.lui_bt[k] = 'XONG'
                    self.su_kien.append(('XONG', ten[st.status]))

        def su_kien_lui(self):
            out, self.su_kien = self.su_kien, []
            return out

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

        def lui(self, m, v, dung_khi=None):
            """08/11 buoc 2c: Nav2 BackUp m met o v m/s. Tra (ket qua, quang xe lui theo tf | None). Sep DUNG giua chung => huy, 'DUNG'."""
            if not self.ac_lui.wait_for_server(timeout_sec=HV.so('cho_may_chu_s')):
                self.bao('   khong thay may chu /backup (behavior_server)')
                return 'TU_CHOI', None
            p0 = self.pose()
            g = BackUp.Goal()
            g.target.x, g.speed = float(m), float(v)
            han = m / v * 2.0 + 5.0
            g.time_allowance.sec = int(math.ceil(han))
            # 09/11 buoc 3 #7: handle /backup rieng (self.goal_lui), bi ngat thi huy_khi_ngat huy ca no (ke ca luc cho chap nhan)
            self.goal_lui, self.kq_lui = None, None
            fut = self.fut_lui = self.ac_lui.send_goal_async(g)
            t_het = time.monotonic() + HV.so('cho_may_chu_s')
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                return 'TU_CHOI', None
            self.goal_lui = fut.result()
            self.id_lui_gui.add(bytes(self.goal_lui.goal_id.uuid))       # #6: khong bao la LUI_RA bt
            self.goal_lui.get_result_async().add_done_callback(self._on_ket_lui)
            t_het = time.monotonic() + han + 5.0
            kq = None
            while kq is None:
                kq = self.kq_lui
                lenh = self.lenh_sep()
                ly_dung = None if kq is not None or dung_khi is None else dung_khi()     # 09/11: gac hong < 0,06
                if kq is None and lenh and lenh[0] == 'DUNG':
                    self._huy_lui()
                    kq = 'DUNG'
                elif ly_dung:
                    self._huy_lui()
                    kq = ly_dung
                elif kq is None and time.monotonic() > t_het:
                    self._huy_lui()
                    kq = 'QUA_GIO'
                if kq is None:
                    self.ngu(0.1)
            p1 = self.pose()
            return kq, (None if p0 is None or p1 is None else math.hypot(p1[0] - p0[0], p1[1] - p0[1]))

        def _on_ket_lui(self, fut):
            st = fut.result().status
            self.kq_lui = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, 'ABORTED')

        def lui_dang_chay(self):
            """09/11 #7: /backup cua gui dang chay (da chap nhan chua xong) hoac dang cho chap nhan."""
            if self.fut_lui is None:
                return False
            if not self.fut_lui.done():
                return True
            return self.goal_lui is not None and self.kq_lui is None

        def _huy_lui(self):
            """Huy /backup cua gui (cho chap nhan toi 2 s neu chua). Tra 'ok' | 'khong_tra_loi' | 'khong_co_lui'."""
            if not self.lui_dang_chay():
                return 'khong_co_lui'
            t_het = time.monotonic() + HV.so('cho_huy_s')
            while not self.fut_lui.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not self.fut_lui.done():
                return 'khong_tra_loi'
            r = self.fut_lui.result()
            if not r.accepted or self.kq_lui is not None:
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
            09/11 buoc 3 #7 (kieu mat): huy ca /backup cua gui dang chay (in '   HUY /backup dang chay: ...'). Tra chung:
            'ok' | 'khong_tra_loi' (mot trong hai khong tra loi) | 'khong_co_dich' (khong co gi dang chay)."""
            kq = 'khong_co_dich'
            if self.goal is not None and self.kq is None:
                fut = self.goal.cancel_goal_async()
                t_het = time.monotonic() + HV.so('cho_huy_s')
                while not fut.done() and time.monotonic() < t_het:
                    self.ngu(0.05)
                kq = 'ok' if fut.done() else 'khong_tra_loi'
            if self.kieu == 'mat':
                kl = self._huy_lui()
                if kl != 'khong_co_lui':
                    self.bao('   HUY /backup dang chay: %s' % kl)
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
