#!/usr/bin/env python3
"""sep (buoc 1 cua THIET_KE_SEP.md ban 1.4, soan 01/11/2026 o nha, CHUA THU TREN XE): sep CHI NHIN.

Sep nghe cap duoi, tong hop buc tranh toan canh, chon tu the va ghi nhat ky "se lam gi" + truy vet (muc 3, 4, 5).
Buoc 1 KHONG co ma ra lenh: khong phat toc do, khong huy dich, khong ghi tep lenh, khong goi lenh ngoai.
Mot node, nghe mot lan; moi hen gio bang time.monotonic(); vong 10 lan/giay.
Phat duy nhat /sep/trang_thai (std_msgs/String, JSON). Nhat ky: ~/sep_nhat_ky.csv (moi dau hieu mot dong).
Chay tren Pi: python3 ~/sep.py   (can ~/lai_xe_lidar.py cung thu muc; ham thuan o nua tren thu bang test_sep.py)

Thien chot 01/11 (duyet ke hoach buoc 1):
  dich = diem cuoi cua /plan cuoi cung truoc SUCCEEDED; chuyen theo duoi ~/ket_qua_chuyen.csv (KIEM_DAT bat dau,
  DEN_NOI / HET_DUONG ket thuc; them SAI_VI_TRI vi chuyen.sh cung thoat o do); do tin cay tren moi diem <= 3 m;
  nhay = do doi cua map->odom; trang thai lai_xe_lidar doc tu /rosout; /scan cu > 0,5 s thi ghi MAT_SCAN;
  "0 poses": ghi ca /plan rong va ABORTED < 2 s.

CAN KIEM TREN PI (chua kiem; chi suy tu ma trong kho va tai lieu):
  1. /bond co duoc phat khong; truong id co dung ten node Nav2 khong; ben lifecycle_manager co phat nhip tim rieng
     (instance_id khac) khong. Neu khong co /bond thi nav2_song chi dua vao dong CRITICAL FAILURE trong /rosout.
  2. Cau chu that cua lifecycle_manager_navigation khi Nav2 tu tat (sep do "CRITICAL FAILURE") va khi len lai
     ("Managed nodes are active", bat_xe.sh cung do cau nay).
  3. Ten logger cua lai_xe_lidar trong /rosout la "lai_xe_lidar" (test_sep.py kiem cau log trong ma, khong kiem tren Pi).
  4. QoS cua /rosout, /plan, /map, /navigate_to_pose/_action/status tren Pi: sai thi sep khong nhan, nhat ky se thieu.
  5. Planner co phat /plan rong khi "0 poses" khong, hay khong phat gi (khi do chi con dau hieu ABORTED < 2 s).
  6. Planner co phat /plan cho yeu cau compute_path_to_pose cua chuyen.sh (buoc KIEM) khong.
  7. /ket (ket_detector trong launch chinh) co dang phat khong.
  8. tf map->odom co (slam_toolbox phat) va map->base_footprint co du moi (< 5 s).
  9. Tuoi tf tinh bang gio ROS (gio he thong) tru stamp: cung may nen dung; bat_xe.sh chinh gio giua chung thi sai mot lan.
 10. Tai CPU: truong vong_ms cua /sep/trang_thai phai nho hon 100 ms (vong 10 lan/giay).
"""
import json
import math
import os
import re
import time
from collections import deque

import lai_xe_lidar as L

INF = 1e9

LECH_MAX = 0.30                  # m, diem dau /plan lech xe (muc 5 dong 1) [uoc]
TF_CU = 5.0                      # s, tf map->base_footprint cu tu so nay = dinh vi hong (muc 5 dong 2)
SCAN_CU = 0.5                    # s, /scan cu hon so nay = MAT_SCAN
R_TIN_CAY = 3.0                  # m, chi tinh do tin cay tren diem LiDAR trong ban kinh nay
TC_TIN, TC_NGHI = 0.70, 0.50     # do tin cay vi tri (muc 3) [de xuat]
T_KHONG_TIN, T_BAO = 5.0, 15.0   # s, thap lien tuc tu luc bat dau thap: khong tin / dung va bao (#16)
CUA_SO_KET = 2.0                 # s, cua so so sanh LiDAR va vi tri khi nghi ket (giong cua_so cua ket_detector)
LIDAR_DOI, POSE_DOI = 0.02, 0.02 # m, duoi so nay la "khong doi" [de xuat]
SUC_LECH, SUC_LECH_MAX, SUC_NHANH = 0.15, 0.20, 0.05   # m, m, s (#15, muc 5 dong 4)
NHAY_M, NHAY_DEG, NHAY_DT = 0.10, 5.0, 1.0             # nhay vi tri giua chuyen (muc 4) [de xuat]
BOND_TIMEOUT = 4.0               # s, khong nhan nhip tim lau hon so nay = mat nhip [de xuat, chua kiem tren Pi]
BOND_QUEN = 30.0                 # s, nhip tim cu hon so nay thi bo (node da tat han hoac da bat lai)
NAV2_LAN_MAX = 2                 # lan khoi dong lai Nav2 moi chuyen (#8)
LUOT_THOAT_MAX = 3               # luot thoat moi chuyen (muc 7)
KHE_TANG, KHE_CUA_SO = 0.02, 3.0 # m, s: khe phia truoc rong them it nhat bay nhieu trong cua so (#12) [de xuat]
NHIN_XA = 0.6                    # m dau cua /plan dat len LiDAR (muc 4 buoc D) [de xuat]
XA_TRUOC = 0.15                  # m tu mep mui: vat trong dai nay la "vat phia truoc" (vung vang cua lai_xe_lidar, #20)
T_CHO, T_CHO_THEM, T_CHO_B = 30.0, 10.0, 60.0          # s (#9, #12; vat tai B: 2 x 30 s)
B_XY, R_B = (1.60, 0.82), 0.20   # B (chuyen.sh) va ban kinh coi la "dich tai B"
TREO = 1.0                       # s, mot vong cham hon so nay = sep treo (muc 9.9) [de xuat]
ABORT_NHANH = 2.0                # s, ABORTED som hon so nay = nghi "0 poses"
L1_KET = ('KET_HINH_HOC', 'XOAY_BI_CHAN_QUA_LAU')     # ly do thoat cua lai_xe_lidar = no bao ket

COT_NHAT_KY = 'gio_mono,gio,loai,dau_hieu,nguon_nghi,phep_kiem,ket_qua,se_lam'
COT_CSV = ('gio', 'huong', 'tuyen', 'diem', 'lan', 'ket_qua', 'giay', 'lech_kiem_m', 'lech_costmap_m', 'tuoi_costmap_s')

_L1 = L.NguoiLai()               # chi dung diem_chan_truoc (khong giu trang thai)


# ---------- ham thuan ----------

def do_tin_cay(pts, pose, grid, tol=1, r_max=R_TIN_CAY):
    """Ti le diem LiDAR (cach LiDAR <= r_max) roi trung o tuong cua ban do, cho lech tol o. Tra (ti_le, so_diem);
    (None, so_diem) neu thieu ban do, vi tri hoac diem."""
    if grid is None or pose is None:
        return None, 0
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    n = trung = 0
    for x, y in pts:
        if math.hypot(x - L.LIDAR_X, y) > r_max:
            continue
        n += 1
        trung += L.o_chiem(grid, px + x * c - y * s, py + x * s + y * c, tol)
    return (trung / n if n else None), n


def muc_tin(tc, t_thap_tu, now):
    """TIN (>= 0,70); NGHI (0,50-0,70, hoac < 0,50 chua du 5 s); KHONG_TIN (< 0,50 tu 5 s); BAO (< 0,50 tu 15 s).
    tc None (thieu ban do / vi tri) tinh nhu thap. t_thap_tu: luc bat dau thap lien tuc."""
    if tc is not None and tc >= TC_TIN:
        return 'TIN'
    if (tc is not None and tc >= TC_NGHI) or t_thap_tu is None:
        return 'NGHI'
    d = now - t_thap_tu
    if d >= T_BAO:
        return 'BAO'
    if d >= T_KHONG_TIN:
        return 'KHONG_TIN'
    return 'NGHI'


def lech_dau_plan(p0, pose, nguong=LECH_MAX):
    """Khoang cach tu diem dau /plan toi xe va co vuot nguong khong: (m, bool); (None, False) neu thieu."""
    if p0 is None or pose is None:
        return None, False
    d = math.hypot(p0[0] - pose[0], p0[1] - pose[1])
    return d, d > nguong


def tf_cu(tuoi, nguong=TF_CU):
    """tf map->base_footprint cu (hoac khong co) = dinh vi hong (khac 1 s cua lai_xe_lidar: khac muc dich)."""
    return tuoi is None or tuoi >= nguong


def kiem_ket(l1_ket, ket_bool, lidar_doi, pose_doi):
    """Muc 5 dong 3. Co bao ket (lai_xe_lidar hoac /ket) thi: LiDAR khong doi = KET_THAT; LiDAR doi ma vi tri
    khong doi = VI_TRI_SAI; con lai None."""
    if not (l1_ket or ket_bool):
        return None
    if lidar_doi is not None and lidar_doi < LIDAR_DOI:
        return 'KET_THAT'
    if pose_doi is not None and pose_doi < POSE_DOI:
        return 'VI_TRI_SAI'
    return None


def xet_succeeded(dich, pose, t_chay, lan):
    """Muc 5 dong 4 (#15). Tra (ket_qua, lech_cm). Lan 1 lech > 0,15 m: GUI_LAI. Lan 2: <= 0,15 DAT,
    <= 0,20 COI_LA_TOI, > 0,20 NAV2_GIA. SUCCEEDED sau < 0,05 s: NAV2_GIA."""
    cm = None
    if dich is not None and pose is not None:
        cm = round(math.hypot(dich[0] - pose[0], dich[1] - pose[1]) * 100)
    if t_chay is not None and t_chay < SUC_NHANH:
        return 'NAV2_GIA', cm
    if cm is None:
        return 'KHONG_BIET', None
    d = cm / 100.0
    if d <= SUC_LECH:
        return 'DAT', cm
    if lan <= 1:
        return 'GUI_LAI', cm
    if d <= SUC_LECH_MAX:
        return 'COI_LA_TOI', cm
    return 'NAV2_GIA', cm


def nhay(mo_truoc, mo_sau, dt, p_odom=(0.0, 0.0)):
    """Do doi cua map->odom (x, y, yaw) trong dt giay, tinh tai diem p_odom (vi tri xe trong khung odom).
    Tra (m, do, la_nhay); la_nhay khi dt < 1 s va (>= 0,10 m hoac >= 5 do)."""
    px, py = p_odom
    a = [None, None]
    for k, (ox, oy, oth) in enumerate((mo_truoc, mo_sau)):
        c, s = math.cos(oth), math.sin(oth)
        a[k] = (ox + px * c - py * s, oy + px * s + py * c)
    d_m = math.hypot(a[1][0] - a[0][0], a[1][1] - a[0][1])
    d_deg = abs(math.degrees(L.wrap(mo_sau[2] - mo_truoc[2])))
    return d_m, d_deg, dt < NHAY_DT and (d_m >= NHAY_M or d_deg >= NHAY_DEG)


def nav2_song(bond_tuoi, critical):
    """bond_tuoi: {khoa: giay tu nhip tim cuoi}. Tra (SONG | MAT_NHIP | KHONG_BIET, ghi chu)."""
    if critical:
        return 'MAT_NHIP', 'CRITICAL_FAILURE'
    if not bond_tuoi:
        return 'KHONG_BIET', ''
    cu = sorted(k for k, a in bond_tuoi.items() if a > BOND_TIMEOUT)
    if cu:
        return 'MAT_NHIP', ' '.join(cu)
    return 'SONG', ''


def vat_truoc(pts, pose, grid, xa=XA_TRUOC):
    """Vat dang chan mui (cach mep mui <= xa): VAT_LA (khong co tren ban do), DIA_HINH (co), None (khong co vat
    hoac khong ket luan duoc: thieu ban do / vi tri). Can >= 2 diem (loc nhieu nhu lai_xe_lidar)."""
    diem = _L1.diem_chan_truoc(pts, xa)
    if len(diem) < L.DEFAULT['dem_diem_do']:
        return None
    k = L.la_dia_hinh(diem, pose, grid)
    if k is None:
        return None
    return 'DIA_HINH' if k else 'VAT_LA'


def khe_rong_dan(ls_ff):
    """ls_ff: [(t, khoang thoang phia truoc)] theo thu tu thoi gian. True neu trong KHE_CUA_SO giay cuoi khe
    rong them >= KHE_TANG (can it nhat nua cua so du lieu)."""
    if len(ls_ff) < 2:
        return False
    t1, f1 = ls_ff[-1]
    t0, f0 = next((t, f) for t, f in ls_ff if t >= t1 - KHE_CUA_SO)
    return t1 - t0 >= KHE_CUA_SO * 0.5 and f1 - f0 >= KHE_TANG


def nhin_xa(plan, pose, pts, dai=NHIN_XA):
    """Khoang cach nho nhat tu duong tam cua `dai` m dau /plan (toa do ban do) toi diem LiDAR; None neu thieu."""
    if not plan or pose is None or not pts:
        return None
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    best, di, truoc = INF, 0.0, None
    for wx, wy in plan:
        if truoc is not None:
            di += math.hypot(wx - truoc[0], wy - truoc[1])
            if di > dai:
                break
        truoc = (wx, wy)
        dx, dy = wx - px, wy - py
        bx, by = dx * c + dy * s, -dx * s + dy * c
        for x, y in pts:
            best = min(best, math.hypot(x - bx, y - by))
    return best


def chon_tu_the(bt):
    """Tu the va ly do (chi de ghi "se lam gi"), theo thu tu uu tien muc 4: (0) mat /scan; (1) phan xa cua
    lai_xe_lidar (#10); (2) vi tri co tin duoc khong (#16, #19); (3) Nav2 con song khong (#8); (4) vat can, duong di.
    Doi hanh lang va thoat (dua vao ban do) chi khi vi tri TIN: muc NGHI cung chi CHO (vi du muc 4: tin cay 0,60)."""
    g = bt.get
    if g('mat_scan'):
        return 'CHO', 'MAT_SCAN'
    l1 = g('l1') or ''
    if l1.startswith('THOAT') or l1.startswith('CHAN_'):
        return 'CHO', 'PHAN_XA_L1 ' + l1
    if g('tf_cu'):
        return 'CHO', 'TF_CU'
    mt = g('muc_tin', 'TIN')
    ket = g('ket')
    if mt == 'BAO':
        return 'DUNG_VA_BAO', 'KHONG_TIN_QUA_%ds' % T_BAO
    if mt == 'KHONG_TIN':
        return 'CHO', 'KHONG_TIN' + (' (L1 duoc tu lui/cung theo LiDAR, #19)' if ket == 'KET_THAT' else '')
    if g('nhay'):
        return 'CHO', 'NHAY_VI_TRI: doi chieu lai'
    if g('nav2') == 'MAT_NHIP':
        n = g('nav2_lan', 1)
        if n > NAV2_LAN_MAX:
            return 'DUNG_VA_BAO', 'NAV2_CHET_LAN_%d' % n
        return 'CHO', 'NAV2_MAT_NHIP lan %d: khoi dong lai qua bat_xe.sh' % n
    if g('plan_lech'):
        return 'KHONG_DI', 'PLAN_SAI'
    tin = mt == 'TIN'
    if ket == 'VI_TRI_SAI':
        return 'CHO', 'VI_TRI_SAI: kiem dinh vi'
    if ket == 'KET_THAT':
        if not tin:
            return 'CHO', 'KET_THAT, vi tri chua TIN'
        if g('luot_thoat', 0) >= LUOT_THOAT_MAX:
            return 'DOI_HANH_LANG', 'HET_%d_LUOT_THOAT' % LUOT_THOAT_MAX
        return 'THOAT', 'KET_THAT luot %d' % (g('luot_thoat', 0) + 1)
    if g('vat_truoc') == 'VAT_LA':
        t = g('t_cho', 0.0)
        if g('tai_B'):
            if t >= T_CHO_B:
                return 'DUNG_VA_BAO', 'VAT_CHAN_TAI_B %ds' % t
            return 'CHO', 'VAT_LA_TAI_B %ds' % t
        if t < T_CHO:
            return 'CHO', 'VAT_LA %ds' % t
        if t < T_CHO + T_CHO_THEM and g('khe_rong'):
            return 'CHO', 'VAT_LA %ds, khe rong dan' % t
        if not tin:
            return 'CHO', 'VAT_LA %ds, vi tri chua TIN' % t
        return 'DOI_HANH_LANG', 'VAT_LA %ds' % t
    if not g('goal'):
        return 'CHO', 'KHONG_CO_DICH'
    if mt == 'NGHI':
        tc = g('tin_cay')
        return 'THAN_TRONG', 'TIN_CAY %s' % ('-' if tc is None else '%.2f' % tc)
    return 'DI_THUONG', ''


def kiem_treo(t_truoc, now, nguong=TREO):
    """So giay giua hai vong neu cham hon nguong (sep treo), nguoc lai None."""
    if t_truoc is None:
        return None
    d = now - t_truoc
    return d if d > nguong else None


_RE_L1_DOI = re.compile(r'^(\S+) -> (\S+) \(([^)]*)\)')
_RE_L1_DINH_KY = re.compile(r'trang_thai=(\S+)')


def doc_rosout(name, msg):
    """Mot dong /rosout -> ('NAV2_CRITICAL', cau) | ('NAV2_ACTIVE', '') | ('L1', trang_thai, ly_do) | None.
    Cau log cua lai_xe_lidar: '%s -> %s (%s)%s | %s' (doi trang thai) va '... trang_thai=%s' (dinh ky)."""
    if 'lifecycle_manager' in name:
        if 'CRITICAL FAILURE' in msg:
            return ('NAV2_CRITICAL', msg[:120])
        if 'Managed nodes are active' in msg:
            return ('NAV2_ACTIVE', '')
        return None
    if name == 'lai_xe_lidar':
        m = _RE_L1_DOI.match(msg)
        if m:
            return ('L1', m.group(2), m.group(3))
        m = _RE_L1_DINH_KY.search(msg)
        if m:
            return ('L1', m.group(1), '')
    return None


def doc_csv(dong):
    """Mot dong cua ~/ket_qua_chuyen.csv -> dict theo COT_CSV; None neu la dong tieu de hoac hong."""
    o = dong.strip().split(',')
    if len(o) < 6 or o[0] == 'gio':
        return None
    return dict(zip(COT_CSV, o))


def dong_nhat_ky(gio_mono, gio, loai, dau_hieu, nguon='', phep_kiem='', ket_qua='', se_lam=''):
    """Mot dong nhat ky theo COT_NHAT_KY (dau phay trong noi dung doi thanh ';')."""
    o = ['%.2f' % gio_mono, gio, loai, dau_hieu, nguon, phep_kiem, ket_qua, se_lam]
    return ','.join(str(v).replace(',', ';') for v in o)


def _r(v, n=2):
    return None if v is None else round(v, n)


class Sep:
    """Bo nho cua sep (khong ROS). vong(now, v) nhan thong tin moi nhat, tra (trang_thai, cac dong nhat ky).
    v: pts, scan_tuoi, pose, tf_tuoi, mo (map->odom), grid, plans [[(x, y)]], goals [[(id, status)]], ket,
    rosout [(name, msg)], csv [dong], bond {khoa: tuoi}, gio (chuoi gio de nguoi doc)."""

    def __init__(self):
        self.t_vong = None
        # su that (giu khi sep treo)
        self.chuyen = None
        self.diem_txt = ''
        self.nav2_lan = 0
        self.luot_thoat = 0
        self.nav2_crit = False
        self.l1 = ''
        self.l1_ket = False
        self.goals = {}
        self.xong = deque(maxlen=200)
        self.status_dau = True
        self.goal_active = False
        self.plan_cuoi = None
        self.dich = None
        self.lech_plan = None
        self.plan_sai = False
        self.suc_truoc = None
        self._moi()

    def _moi(self):
        """Bo quyet dinh cu, bat dau lai tu buoc A (#14)."""
        self.t_thap_tu = None
        self.t_vat = None
        self.ls_ket = deque()
        self.ls_ff = deque()
        self.ls_mo = deque()
        self.co = {}
        self.tu_the = None

    def vong(self, now, v):
        gio = v.get('gio', '')
        log = []

        def ghi(loai, dau_hieu, nguon='', kiem='', kq='', se=''):
            log.append(dong_nhat_ky(now, gio, loai, dau_hieu, nguon, kiem, kq, se))

        d = kiem_treo(self.t_vong, now)
        self.t_vong = now
        if d is not None:
            ghi('TREO', 'sep treo %.1f s' % d, se='bo quyet dinh cu, bat dau lai tu buoc A')
            self._moi()

        # ---- A. thu thap: su kien ----
        for dong in v.get('csv', ()):
            self._csv(doc_csv(dong), ghi)
        for name, msg in v.get('rosout', ()):
            self._rosout(doc_rosout(name, msg), ghi)
        pose = v.get('pose')
        for plan in v.get('plans', ()):
            self._plan(plan, pose, ghi)
        for ds in v.get('goals', ()):
            self._goals(ds, now, pose, ghi)

        # ---- B. tong hop ----
        pts, st = v.get('pts'), v.get('scan_tuoi')
        mat_scan = pts is None or st is None or st > SCAN_CU
        self._co('MAT_SCAN', mat_scan, ghi, '/scan hoac LiDAR', 'tuoi %s s' % _r(st, 1))

        tfc = tf_cu(v.get('tf_tuoi'))
        self._co('TF_CU', tfc, ghi, 'dinh vi hoac duong truyen tf', 'tuoi %s s' % _r(v.get('tf_tuoi'), 1))
        pose_ok = None if tfc else pose
        grid = v.get('grid')

        tc, n = (None, 0) if mat_scan else do_tin_cay(pts, pose_ok, grid)
        if tc is None or tc < TC_NGHI:
            if self.t_thap_tu is None:
                self.t_thap_tu = now
        else:
            self.t_thap_tu = None
        mt = muc_tin(tc, self.t_thap_tu, now)
        self._co('KHONG_TIN', mt in ('KHONG_TIN', 'BAO'), ghi, 'dinh vi', 'tin cay %s tren %d diem' % (_r(tc), n))

        mo = v.get('mo')
        nh, nh_txt = False, ''
        if mo is not None and pose is not None:
            self.ls_mo.append((now, mo))
            while now - self.ls_mo[0][0] > NHAY_DT:
                self.ls_mo.popleft()
            t0, mo0 = self.ls_mo[0]
            c, s = math.cos(mo[2]), math.sin(mo[2])
            dx, dy = pose[0] - mo[0], pose[1] - mo[1]
            dm, dd, nh = nhay(mo0, mo, now - t0, (dx * c + dy * s, -dx * s + dy * c))
            nh_txt = '%.2f m %.1f do trong %.1f s' % (dm, dd, now - t0)
        self._co('NHAY_VI_TRI', nh, ghi, 'SLAM sua hoac truot banh', nh_txt)

        ket, ff = None, None
        if not mat_scan:
            ff, fb = L.thoang(pts, 0.0), L.thoang(pts, math.pi)
            self.ls_ket.append((now, ff, fb, pose))
            while now - self.ls_ket[0][0] > CUA_SO_KET:
                self.ls_ket.popleft()
            t0, ff0, fb0, p0 = self.ls_ket[0]
            if now - t0 >= CUA_SO_KET * 0.8:
                lidar_doi = max(abs(ff - ff0), abs(fb - fb0))
                pose_doi = None if pose is None or p0 is None else math.hypot(pose[0] - p0[0], pose[1] - p0[1])
                ket = kiem_ket(self.l1_ket, v.get('ket', False), lidar_doi, pose_doi)
            self.ls_ff.append((now, ff))
            while now - self.ls_ff[0][0] > KHE_CUA_SO:
                self.ls_ff.popleft()
        self._co('KET_THAT', ket == 'KET_THAT', ghi, 'banh bi chan', 'LiDAR khong doi trong %ds' % CUA_SO_KET)
        self._co('VI_TRI_SAI', ket == 'VI_TRI_SAI', ghi, 'odom hoac dinh vi', 'LiDAR doi; vi tri khong doi')

        vat = None if mat_scan else vat_truoc(pts, pose_ok, grid)
        if vat == 'VAT_LA':
            if self.t_vat is None:
                self.t_vat = now
        else:
            self.t_vat = None
        t_cho = 0.0 if self.t_vat is None else now - self.t_vat
        self._co('VAT_LA_TRUOC', vat == 'VAT_LA', ghi, 'vat la (nguoi; hop)', 'ban do: o trong')
        khe = khe_rong_dan(self.ls_ff)

        n2, n2_txt = nav2_song(v.get('bond') or {}, self.nav2_crit)
        if n2 == 'MAT_NHIP' and not self.co.get('NAV2_MAT_NHIP'):
            self.nav2_lan += 1
        self._co('NAV2_MAT_NHIP', n2 == 'MAT_NHIP', ghi, 'Nav2 hoac mang ROS', '%s; lan %d trong chuyen' % (n2_txt, self.nav2_lan))

        tai_b = self.dich is not None and math.hypot(self.dich[0] - B_XY[0], self.dich[1] - B_XY[1]) <= R_B

        # ---- C. chien luoc (chi ghi) ----
        tt, ly = chon_tu_the(dict(
            mat_scan=mat_scan, l1=self.l1, tf_cu=tfc, muc_tin=mt, tin_cay=tc, ket=ket, nhay=nh, nav2=n2,
            nav2_lan=self.nav2_lan, plan_lech=self.plan_sai, luot_thoat=self.luot_thoat, vat_truoc=vat,
            t_cho=t_cho, khe_rong=khe, tai_B=tai_b, goal=self.goal_active))
        if tt != self.tu_the:
            ghi('TU_THE', tt, kq=ly, se='SE_LAM=%s (chi nhin)' % tt)
            self.tu_the = tt

        nx = None if mat_scan else nhin_xa(self.plan_cuoi, pose_ok, pts)
        trang_thai = dict(
            ra_lenh=False, tu_the=tt, ly_do=ly, tin_cay=_r(tc), muc_tin=mt,
            xe=None if pose is None else [round(pose[0], 2), round(pose[1], 2), round(math.degrees(pose[2]))],
            tf_tuoi=_r(v.get('tf_tuoi'), 1), lech_plan=_r(self.lech_plan), dich=self.dich, nav2_dich=self.goal_active,
            nav2_song=n2, nav2_lan=self.nav2_lan, l1=self.l1, luot_thoat=self.luot_thoat, ket=ket, nhay=nh,
            vat_truoc=vat, t_cho=round(t_cho), nhin_xa=_r(nx), chuyen=self.diem_txt)
        return trang_thai, log

    def _co(self, ten, bat, ghi, nguon='', kiem=''):
        """Ghi mot dong khi dau hieu bat len va khi het (khong ghi lap moi vong)."""
        if bat and not self.co.get(ten):
            ghi('TRUY_VET', ten, nguon, kiem, 'BAT')
        elif not bat and self.co.get(ten):
            ghi('TRUY_VET', ten, kq='HET')
        self.co[ten] = bat

    def _csv(self, r, ghi):
        if r is None:
            return
        kq = r['ket_qua']
        self.diem_txt = '%s t%s d%s %s' % (r['huong'], r['tuyen'], r['diem'], kq)
        ghi('CSV', kq, kiem=self.diem_txt)
        if kq == 'KIEM_DAT' and self.chuyen is None:
            self.chuyen = r['huong']
            self.nav2_lan = self.luot_thoat = 0
            ghi('CHUYEN', 'BAT_DAU', kiem=r['huong'], se='dem lai: Nav2 0/%d; thoat 0/%d' % (NAV2_LAN_MAX, LUOT_THOAT_MAX))
        elif kq in ('DEN_NOI', 'HET_DUONG', 'SAI_VI_TRI') and self.chuyen is not None:
            ghi('CHUYEN', 'KET_THUC', kiem=self.chuyen, kq=kq)
            self.chuyen = None

    def _rosout(self, e, ghi):
        if e is None:
            return
        if e[0] == 'NAV2_CRITICAL':
            self.nav2_crit = True
            ghi('ROSOUT', 'NAV2_CRITICAL', 'lifecycle_manager', e[1])
        elif e[0] == 'NAV2_ACTIVE':
            if self.nav2_crit:
                ghi('ROSOUT', 'NAV2_LEN_LAI', 'lifecycle_manager')
            self.nav2_crit = False
        else:
            ten, ly = e[1], e[2]
            if ten.startswith('THOAT'):
                if not self.l1.startswith('THOAT'):
                    self.luot_thoat += 1
                    ghi('L1', 'BAT_DAU_THOAT', kiem=ly, kq='luot %d' % self.luot_thoat)
                if ly:
                    self.l1_ket = ly in L1_KET
            else:
                self.l1_ket = False
            self.l1 = ten

    def _plan(self, plan, pose, ghi):
        if not plan:
            ghi('TRUY_VET', 'PLAN_RONG', 'bo lap duong', '0 poses', 'BAT', 'KHONG_DI')
            self.plan_sai = True
            self.lech_plan = None
            return
        self.plan_cuoi = plan
        self.dich = plan[-1]
        d, vuot = lech_dau_plan(plan[0], pose)
        self.lech_plan = d
        if vuot and not self.plan_sai:
            ghi('TRUY_VET', 'PLAN_LECH', 'bo lap duong nhan sai vi tri xe',
                'dau plan (%.2f;%.2f) xe (%.2f;%.2f) lech %.2f m' % (plan[0][0], plan[0][1], pose[0], pose[1], d), 'BAT', 'KHONG_DI')
        elif not vuot and self.plan_sai:
            ghi('TRUY_VET', 'PLAN_SAI', kq='HET', kiem='lech %s m' % _r(d))
        self.plan_sai = vuot

    def _goals(self, ds, now, pose, ghi):
        self.goal_active = any(s in (1, 2) for _, s in ds)
        for gid, s in ds:
            if gid in self.xong:
                continue
            if s in (4, 5, 6) and self.status_dau:
                self.xong.append(gid)        # dich da xong truoc khi sep chay: bo qua
                continue
            t0 = self.goals.setdefault(gid, now)
            if s not in (4, 5, 6):
                continue
            self.xong.append(gid)
            del self.goals[gid]
            t_chay = now - t0
            if s == 4:
                self._succeeded(t_chay, pose, ghi)
            elif s == 6:
                ghi('NAV2', 'ABORTED', kiem='sau %.1f s' % t_chay)
                if t_chay < ABORT_NHANH:
                    ghi('TRUY_VET', 'ABORT_NHANH', 'bo lap duong (0 poses?)', 'ABORTED sau %.2f s' % t_chay, 'BAT', 'KHONG_DI')
            else:
                ghi('NAV2', 'CANCELED', kiem='sau %.1f s' % t_chay)
        self.status_dau = False

    def _succeeded(self, t_chay, pose, ghi):
        lan = 1
        if self.suc_truoc is not None and self.dich is not None and self.suc_truoc[0] is not None:
            (x0, y0), kq0 = self.suc_truoc
            if kq0 == 'GUI_LAI' and math.hypot(self.dich[0] - x0, self.dich[1] - y0) < 0.05:
                lan = 2
        kq, cm = xet_succeeded(self.dich, pose, t_chay, lan)
        self.suc_truoc = (self.dich, kq)
        se = {'GUI_LAI': 'gui lai dich 1 lan', 'COI_LA_TOI': 'coi la toi; hien "lech %s cm"' % cm,
              'NAV2_GIA': 'khoi dong lai Nav2 qua bat_xe.sh'}.get(kq, '')
        if kq in ('DAT', 'KHONG_BIET'):
            ghi('NAV2', 'SUCCEEDED', kiem='sau %.2f s; lech %s cm' % (t_chay, cm), kq=kq)
        else:
            ghi('TRUY_VET', 'SUCCEEDED_' + kq, 'trang thai Nav2', 'sau %.2f s; lech %s cm; lan %d' % (t_chay, cm, lan), 'BAT', se)


# ---------- nut ROS 2 (chi dung khi co ROS) ----------

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from std_msgs.msg import Bool, String
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid, Path
    from action_msgs.msg import GoalStatusArray
    from rcl_interfaces.msg import Log
    import tf2_ros
except ImportError:      # may khong co ROS: van import duoc phan ham thuan de thu
    rclpy = None
try:
    from bond.msg import Status as BondStatus
except ImportError:
    BondStatus = None

if rclpy is not None:
    class SepNode(Node):
        def __init__(self):
            super().__init__('sep')
            self.declare_parameter('nhat_ky', os.path.expanduser('~/sep_nhat_ky.csv'))
            self.declare_parameter('csv_chuyen', os.path.expanduser('~/ket_qua_chuyen.csv'))
            self.declare_parameter('lidar_angle_offset_deg', 0.0)
            self.sep = Sep()
            self.scan, self.scan_t = None, None
            self.grid = None
            self.plans, self.goals, self.rosout = [], [], []
            self.ket, self.ket_t = False, None
            self.bond = {}
            self.csv_pos = None
            self.tt_truoc, self.t_pub = None, -INF
            p = self.get_parameter('nhat_ky').value
            moi = not os.path.exists(p)
            self.f_log = open(p, 'a')
            if moi:
                self.f_log.write(COT_NHAT_KY + '\n')
            self.pub = self.create_publisher(String, '/sep/trang_thai', 10)
            self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
            self.create_subscription(Path, '/plan', self._on_plan, 10)
            st_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._on_status, st_qos)
            self.create_subscription(Bool, '/ket', self._on_ket, 10)
            self.create_subscription(Log, '/rosout', self._on_rosout, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
            if BondStatus is not None:
                self.create_subscription(BondStatus, '/bond', self._on_bond, 10)
            else:
                self.get_logger().warning('khong co bond.msg: nhip tim Nav2 chi dua vao /rosout')
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_timer(0.1, self._tick)
            self.get_logger().info('sep san sang: CHI NHIN (buoc 1). Nhat ky: %s' % p)

        def _on_scan(self, msg):
            self.scan, self.scan_t = msg, time.monotonic()

        def _on_map(self, msg):
            i = msg.info
            self.grid = dict(res=i.resolution, w=i.width, h=i.height, ox=i.origin.position.x, oy=i.origin.position.y, data=list(msg.data))

        def _on_plan(self, msg):
            self.plans.append([(q.pose.position.x, q.pose.position.y) for q in msg.poses])

        def _on_status(self, msg):
            self.goals.append([(bytes(s.goal_info.goal_id.uuid), s.status) for s in msg.status_list])

        def _on_ket(self, msg):
            self.ket, self.ket_t = msg.data, time.monotonic()

        def _on_rosout(self, msg):
            if msg.name == 'lai_xe_lidar' or 'lifecycle_manager' in msg.name:
                self.rosout.append((msg.name, msg.msg))

        def _on_bond(self, msg):
            k = '%s#%s' % (msg.id, msg.instance_id[:8])
            if msg.active:
                self.bond[k] = time.monotonic()
            else:
                self.bond.pop(k, None)

        def _tf(self, cha, con):
            """((x, y, yaw), tuoi giay) cua con trong cha, hoac (None, None)."""
            try:
                tf = self.tf_buf.lookup_transform(cha, con, Time())
            except Exception:
                return None, None
            st = tf.header.stamp
            tuoi = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, yaw), tuoi

        def _csv_moi(self):
            """Cac dong moi cua ~/ket_qua_chuyen.csv tu lan doc truoc (lan dau: bo qua dong cu)."""
            p = self.get_parameter('csv_chuyen').value
            try:
                size = os.path.getsize(p)
            except OSError:
                self.csv_pos = 0
                return []
            if self.csv_pos is None:
                self.csv_pos = size
            if size < self.csv_pos:          # tep bi xoa roi tao lai
                self.csv_pos = 0
            if size == self.csv_pos:
                return []
            with open(p, 'rb') as f:
                f.seek(self.csv_pos)
                b = f.read(size - self.csv_pos)
            i = b.rfind(b'\n')
            if i < 0:
                return []
            self.csv_pos += i + 1
            return b[:i + 1].decode('utf-8', 'replace').splitlines()

        def _tick(self):
            now = time.monotonic()
            pts = st = None
            if self.scan is not None:
                st = now - self.scan_t
                pts = L.scan_to_xy(self.scan, math.radians(self.get_parameter('lidar_angle_offset_deg').value))
            pose, tf_tuoi = self._tf('map', 'base_footprint')
            mo, _ = self._tf('map', 'odom')
            for k in [k for k, t in self.bond.items() if now - t > BOND_QUEN]:
                del self.bond[k]
            v = dict(pts=pts, scan_tuoi=st, pose=pose, tf_tuoi=tf_tuoi, mo=mo, grid=self.grid,
                     plans=self.plans, goals=self.goals, rosout=self.rosout, csv=self._csv_moi(),
                     ket=self.ket and self.ket_t is not None and now - self.ket_t < 1.0,
                     bond={k: now - t for k, t in self.bond.items()}, gio=time.strftime('%Y-%m-%d %H:%M:%S'))
            self.plans, self.goals, self.rosout = [], [], []
            tt, log = self.sep.vong(now, v)
            tt['vong_ms'] = round((time.monotonic() - now) * 1000)
            for d in log:
                self.f_log.write(d + '\n')
                self.get_logger().info(d)
            if log:
                self.f_log.flush()
            if tt['tu_the'] != self.tt_truoc or now - self.t_pub >= 1.0:
                self.pub.publish(String(data=json.dumps(tt)))
                self.tt_truoc, self.t_pub = tt['tu_the'], now

    def main():
        rclpy.init()
        node = SepNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.f_log.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
else:
    def main():
        raise SystemExit('can ROS 2 de chay nut; chi import duoc phan ham thuan')


if __name__ == '__main__':
    main()
