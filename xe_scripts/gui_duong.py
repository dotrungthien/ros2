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
"""
import math
import os
import signal
import sys
import time

import tinh_tuyen as TT

DICH = {'di_B': (TT.B, 0.0), 've_A': (TT.A, None)}     # (diem, goc cuoi): di_B giu 0 do nhu bang tt/xs; ve_A theo tiep tuyen
COT_CSV = 'gio,huong,tuyen,diem,lan,ket_qua,giay,lech_kiem_m,lech_costmap_m,tuoi_costmap_s'
PHU_LAI_BAN = ('RA_TIEN', 'RA_LUI', 'XOAY_P')           # ten trang thai lai_xe_lidar dang tu lai banh (cong moi ten THOAT_*)


def cau_hinh(env=os.environ):
    f = lambda k, d: float(env.get(k, d))
    return dict(luot=int(env.get('LUOT', 3)), cho_hong=f('CHO_HONG', 30.0), to=f('TO', 130.0), cho_sep=f('SEP_CHO_S', 10.0),
                toi_sai_so=f('TOI_SAI_SO', 0.20), sep_tuoi=f('SEP_TUOI_MAX', 3.0), tf_tuoi=1.0,
                lop2_cho=12.0, lop2_tra=2.0,      # 04/11 lop 2: cho RA_XONG toi da / cho sep tra loi SAP_GUI toi da (s)
                # 08/11 bo 1e kieu mat: dong ho bo khe (s), muc tien toi thieu (m), cho sau /chon_tuyen (s), nhip in CHO_VAT (s)
                cho_khe=f('CHO_KHE', 30.0), tien_min=f('TIEN_MIN', 0.10), cho_chon=2.0, nhip_cho=5.0,
                # 08/11 buoc 2c: truoc khi gui tuyen kia lui thang bang Nav2 BackUp (m, m/s): gia lap xe khong tu quay dau trong khe
                lui_m=0.35, lui_v=0.10)


# ---------- 08/11 bo 1e: kieu mat (phan thuan) ----------
TEP_GOC = os.path.expanduser('~/goc_dang_chay')
COT_KINH_NGHIEM = 'gio,x,y,huong,tuyen,ly_do'           # lop_huong_dan.doc_kinh_nghiem doc theo ten cot
BO_QUA_KHI_GUI_LAI = ('PLAN_', 'ABORT_NHANH')           # ly do CHO cua sep bo qua khi gui lai sau khi bo khe (sep.py:370)


def doc_goc(path=TEP_GOC):
    """~/goc_dang_chay (bat_xe.sh ghi MOI lan bat: 'mat' | 'cu'): 'mat' neu dong dau la mat; khac / khong co tep => 'cu'."""
    try:
        with open(path) as f:
            return 'mat' if f.readline().strip() == 'mat' else 'cu'
    except OSError:
        return 'cu'


def ghi_kinh_nghiem(path, gio, x, y, huong, tuyen, ly_do):
    """Them 1 dong ~/kinh_nghiem.csv (tao kem dong dau neu chua co tep)."""
    moi = not os.path.exists(path)
    with open(path, 'a') as f:
        if moi:
            f.write(COT_KINH_NGHIEM + '\n')
        f.write('%s,%.3f,%.3f,%s,%s,%s\n' % (gio, x, y, huong, tuyen, ly_do))


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


def doc_sep_lenh(dong, t_now, tuoi_max=3.0):
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


def cho_tf_costmap(co_cm, lay_ban, lay_pose, now, ngu, giay=15.0):
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
    """08/11 bo 1e: mot chuyen kieu mat (thiet ke ban 3 muc 1 + 3b). Cung CSV, cung chu log ket thuc, cung ma thoat voi chay()."""
    nhan = '%s_day' % huong
    thu_tu = (1, 2) if tuyen_dau == 1 else (2, 1)
    (dx, dy), yaw_cuoi = DICH[huong]
    t_chuyen = io.now()
    io.chon_tuyen('0')                                   # 3b #3: bo o cam cu (kill -9 / loi ra 130, 5 cua chuyen truoc)
    io.ngu(cfg['cho_chon'])
    pose, ban = io.cho_san_sang()
    if pose is None or ban is None or ban.vat(ban.o(pose[0], pose[1])):
        ly = ('khong co %s sau 15 s' % thieu_gi(pose, ban)) if pose is None or ban is None else 'xe nam trong o do cua costmap'
        io.bao('DUNG: SAI_VI_TRI (%s). KHONG gui lenh nao.' % ly)
        io.ghi(nhan, '-', 1, 0, 'SAI_VI_TRI', 0)
        return 3
    io.ghi(nhan, thu_tu[0], 1, 0, 'KIEM_DAT', 0)
    gui_lai = False
    for r in thu_tu:
        if hoi_sep_mat(io, cfg, gui_lai) == 'DUNG':
            return dung_sep(io, nhan, r, 1)
        if gui_lai:                                      # 08/11 buoc 2c: lui ra khoi khe vua bo truoc khi /chon_tuyen tuyen kia
            kq_lui, quang = io.lui(cfg['lui_m'], cfg['lui_v'])
            io.bao('   LUI_RA truoc tuyen %d: /backup %.2f m, %.2f m/s => %s%s.' % (
                r, cfg['lui_m'], cfg['lui_v'], kq_lui, '' if quang is None else ' (xe lui %.2f m)' % quang))
            if kq_lui == 'DUNG':
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
        kq = theo_doi_mat(io, (dx, dy), yaw_cuoi, cfg)
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
        if kq in ('CHO_VAT_30s', 'KHONG_FEEDBACK'):        # bo khe: nho cho ket + chup bang chung
            if p is not None:
                io.ghi_kinh_nghiem(p[0], p[1], huong, r, kq)
            io.chup(huong, 'bo khe tuyen %d (%s)' % (r, kq))
        io.bao('   khong den duoc (%s%s).' % (kq, '' if lech is None else ', cach dich %.2f m' % lech))
        io.ghi(nhan, r, 1, 1, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
        io.bao('== Tuyen %d hong. ==' % r)
    io.bao('HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.')
    io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
    return 1


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


def theo_doi_mat(io, dich, yaw_cuoi, cfg):
    """Gui navigate_to_pose, theo doi toi khi xong. Tra 'SUCCEEDED' | 'ABORTED' | 'CANCELED' | 'TU_CHOI' | 'DUNG' |
    'CHO_VAT_30s' | 'KHONG_FEEDBACK' (hai ly do bo khe: da huy dich)."""
    if not io.gui_nav(dich, yaw_cuoi):
        return 'TU_CHOI'
    dh = DongHo(io.now(), cfg['cho_khe'], cfg['tien_min'])
    n_rec, lan_in, da_in = 0, 0, False
    while True:
        kq = io.ket_qua()
        if kq is not None:
            return kq
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            io.huy()
            return 'DUNG'
        fb = io.feedback()
        p = io.pose()
        d_tf = None if p is None else math.hypot(p[0] - dich[0], p[1] - dich[1])
        tt, ly, dung = dh.cap_nhat(io.now(), d_tf, None if fb is None else fb[0])
        if fb is not None and fb[1] > n_rec:
            n_rec = fb[1]
            io.bao('   Nav2 hoi phuc: number_of_recoveries = %d.' % n_rec)
        if tt == 'BO_KHE':
            io.huy()
            return ly
        if tt == 'TIEN':
            if da_in:
                io.bao('   DI_TIEP: xe tien lai.')
            lan_in, da_in = 0, False
        elif tt == 'DI' and dung >= cfg['nhip_cho'] * (lan_in + 1):
            lan_in = int(dung // cfg['nhip_cho'])
            da_in = True
            io.bao('CHO_VAT: %d/%d s%s' % (int(dung), int(cfg['cho_khe']), '' if d_tf is None else ' (cach dich %.2f m)' % d_tf))
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
        if io.now() - t_phat >= 0.5:
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
    from nav2_msgs.action import BackUp, FollowPath, NavigateToPose
    from rcl_interfaces.msg import Log
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
                self.tep_kinh_nghiem = os.path.expanduser('~/kinh_nghiem.csv')
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
                con = t_het - time.monotonic()
                if con <= 0:
                    return
                rclpy.spin_once(self, timeout_sec=min(con, 0.05))

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

        def cho_san_sang(self, giay=15.0):
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
            if not self.ac.wait_for_server(timeout_sec=5.0):
                self.bao('   khong thay may chu /follow_path (controller_server)')
                return False
            g = FollowPath.Goal()
            g.path = self._path(duong)
            g.controller_id = 'FollowPath'          # nhu bt_cho.xml; goal_checker_id rong = plugin duy nhat (general_goal_checker)
            self.goal, self.kq = None, None
            fut = self.ac.send_goal_async(g)
            t_het = time.monotonic() + 5.0
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
                t_het = time.monotonic() + 2.0
                while not fut.done() and time.monotonic() < t_het:
                    self.ngu(0.05)

        def co_dich(self):
            """06/11 viec 121: con dich /follow_path dang chay (da gui, chua co ket qua)."""
            return self.goal is not None and self.kq is None

        # --- 08/11 bo 1e: io kieu mat ---
        def chon_tuyen(self, s):
            # 08/11 buoc 2 (gia lap): publisher vua tao phat ngay thi DDS chua ghep voi lop_huong_dan => mat "0" dau chuyen.
            # Cho toi da 3 s cho co nguoi nghe roi moi phat; van khong co thi in canh bao (lop_huong_dan chua chay?).
            t_het = time.monotonic() + 3.0
            while self.pub_chon.get_subscription_count() == 0 and time.monotonic() < t_het:
                self.ngu(0.05)
            if self.pub_chon.get_subscription_count() == 0:
                self.bao('   CANH BAO: /chon_tuyen chua co ai nghe (lop_huong_dan?): "%s" co the mat' % s)
            m = String()
            m.data = s
            self.pub_chon.publish(m)

        def _on_plan(self, msg):
            self.plan_cuoi = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        def _on_fb(self, msg):
            f = msg.feedback
            self.fb = (float(f.distance_remaining), int(f.number_of_recoveries))

        def feedback(self):
            return self.fb

        def gui_nav(self, dich, yaw_cuoi):
            if not self.ac_nav.wait_for_server(timeout_sec=5.0):
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
            fut = self.ac_nav.send_goal_async(g, feedback_callback=self._on_fb)
            t_het = time.monotonic() + 5.0
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                self.bao('   /navigate_to_pose tu choi dich')
                return False
            self.goal = fut.result()
            self.goal.get_result_async().add_done_callback(self._on_result)
            return True

        def lui(self, m, v):
            """08/11 buoc 2c: Nav2 BackUp m met o v m/s. Tra (ket qua, quang xe lui theo tf | None). Sep DUNG giua chung => huy, 'DUNG'."""
            if not self.ac_lui.wait_for_server(timeout_sec=5.0):
                self.bao('   khong thay may chu /backup (behavior_server)')
                return 'TU_CHOI', None
            p0 = self.pose()
            g = BackUp.Goal()
            g.target.x, g.speed = float(m), float(v)
            han = m / v * 2.0 + 5.0
            g.time_allowance.sec = int(math.ceil(han))
            self.goal, self.kq = None, None
            fut = self.ac_lui.send_goal_async(g)
            t_het = time.monotonic() + 5.0
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                return 'TU_CHOI', None
            self.goal = fut.result()
            self.goal.get_result_async().add_done_callback(self._on_result)
            t_het = time.monotonic() + han + 5.0
            kq = None
            while kq is None:
                kq = self.kq
                lenh = self.lenh_sep()
                if kq is None and lenh and lenh[0] == 'DUNG':
                    self.huy()
                    kq = 'DUNG'
                elif kq is None and time.monotonic() > t_het:
                    self.huy()
                    kq = 'QUA_GIO'
                if kq is None:
                    self.ngu(0.1)
            p1 = self.pose()
            return kq, (None if p0 is None or p1 is None else math.hypot(p1[0] - p0[0], p1[1] - p0[1]))

        def ghi_kinh_nghiem(self, x, y, huong, tuyen, ly_do):
            ghi_kinh_nghiem(self.tep_kinh_nghiem, time.strftime('%Y-%m-%d %H:%M:%S'), x, y, huong, tuyen, ly_do)

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
            """Bi ngat (SIGINT/SIGTERM): huy dich /follow_path dang chay (handle hien co) khi context con song, cho <= 2 s."""
            if self.goal is None or self.kq is not None:
                return 'khong_co_dich'
            fut = self.goal.cancel_goal_async()
            t_het = time.monotonic() + 2.0
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            return 'ok' if fut.done() else 'khong_tra_loi'

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
