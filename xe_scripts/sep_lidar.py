#!/usr/bin/env python3
"""sep_lidar.py - "SEP LiDAR mot nguoi lai" (Thien chot 09/11 21:54-23:08, CHUA THU TREN XE). Phan thuan (khong ROS).

gui_duong.py (kieu mat) la nguoi quyet DUY NHAT; tep nay la phan nhin + chon cua no (thay -> hieu -> quyet). KHONG phat toc do:
moi dong tac giao cho Nav2 (drive_on_heading / backup / spin / navigate_to_pose).
  - Than xe: footprint x -0,105..0,145, y +-0,08 (gac_vat_la). Khe = khoang tu diem LiDAR toi DA GIAC than (cheo: toi goc).
    Vung: truoc (x > mui), sau (x < duoi), hong (con lai) - nhu gac_vat_la.vung_diem.
  - Khe VUNG (nhu MEM): khe nho nhat cua diem trong VANG cua vung do (truoc <= 0,30, hong <= 0,12, sau <= 0,10); khong co => vo cung.
  - Cho chat: co diem cach than < cho_chat_m (0,12). Cho thoang: moi diem >= 0,12.
  - Dong tac nho (cho chat): tien / lui thang 0,05..0,20, xoay +-30..180 do (chi khi MOI diem cach tam > ban_kinh_xoay_m 0,186).
    Cham tren SUOT quang quet (buoc 5 mm / 1 do): khe hong+duoi (nho nhat CHUNG hai vung, trong vang: diem doi vung khi xe di,
    vd 21:50 diem hong thanh diem duoi khi xe tien) KHONG nho hon luc dau; khe vung truoc khong vao <= 0,10
    (coi moi diem la vat; da <= 0,10 luc dau thi khong nho hon); dong tac phai lam khe nho nhat (moi diem) tot len. "Tu the cuoi
    moi khe >= 0,06" ap cho CUOI LAN THOAT (chuoi dong tac; gui_duong.thoat kiem): ca 21:50 tren ban do hong 0,008, khong dong tac
    don nao tu dat 0,06. Muc tieu (theo thu tu): (1) giam THIEU vung do (truoc phai > 0,10, hong+duoi >= 0,06; moi diem, ke ca tuong),
    (2) khe nho nhat lon hon. Chon: cach kinh nghiem DUOC (kha thi, lam tot len) truoc; roi thieu nho nhat, khe cuoi lon nhat (tran
    0,12), gan huong dich, dong tac nho. Khong dong tac nao lam tot len ma con thieu => KEP (gui_duong: CHO_NGUOI).
  - Sector (cho thoang): 36 x 10 do, trong(phi) = khoang truoc mui neu xe quay ve phi (diem trong dai |lech| <= 0,14);
    diem thoat r = min(0,40, trong - 0,10) >= 0,30 m; gan huong dich nhat; o costmap < 253 (nguoi goi kiem).
  - Huy dong tac dang chay (moi lan quet; Thien Q3): truoc <= 0,07 moi vat; truoc <= 0,10 VAT; hong <= 0,06 VAT; khe hong+duoi
    (nho nhat chung, trong vang) nho hon luc bat dau. Dieu kien da dung ngay luc bat dau thi chi huy khi khe NHO HON luc bat dau (khong thi ca 21:50 khong ra duoc).
  - Nhac doi: VAT CHAC (LiDAR VAT: cum ho hai ben, KHONG LECH) trong hanh lang than truoc mui <= nhac_doi_m 0,14.
"""
import json
import math

import gac_vat_la as GV
import hanh_vi as HV

X_MUI, X_DUOI, Y_HONG = GV.X_MUI, GV.X_DUOI, GV.Y_HONG
SAI = 0.002                     # m: nhieu LiDAR / lam tron khi so "nho hon luc dau"
BUOC_M, BUOC_RAD = 0.005, math.radians(1.0)
DONG_TAC = ([('tien', d) for d in (0.05, 0.10, 0.15, 0.20)] + [('lui', d) for d in (0.05, 0.10, 0.15, 0.20)]
            + [('xoay', math.radians(a)) for a in (30, -30, 60, -60, 90, -90, 135, -135, 180)])


def tham_so():
    h, at = HV.so, HV.an_toan
    return dict(hong_do=at('cach_vat_toi_thieu_m'), truoc_do=at('truoc_vat_do_m'), mui_do=h('vung_phanh_m'),
                cho_chat=h('cho_chat_m'), r_xoay=HV.than_the('ban_kinh_xoay_m'), nhac_doi=h('nhac_doi_m'),
                sector=math.radians(h('thoat_sector_deg')), trong=h('thoat_trong_m'), r_min=h('thoat_diem_min_m'),
                r_max=h('thoat_diem_max_m'), gac=GV.tham_so())


def ten_dong_tac(dt):
    """('tien', 0.1) -> 'tien=0.10'; ('xoay', rad) -> 'xoay=+60' (cot cach cua ~/kinh_nghiem.csv, dong log)."""
    if dt[0] == 'xoay':
        return 'xoay=%+d' % int(round(math.degrees(dt[1])))
    return '%s=%.2f' % dt


def doc_ten(s):
    """Nguoc ten_dong_tac; khong phai dong tac => None."""
    try:
        loai, g = (s or '').split('=')
        if loai == 'xoay':
            return 'xoay', math.radians(float(g))
        if loai in ('tien', 'lui'):
            return loai, float(g)
    except ValueError:
        pass
    return None


def diem_loc(pts, t):
    """Diem LiDAR (khung xe, theo tia) thuoc cum >= so_diem_toi_thieu (bo nhieu nhu gac_vat_la). -> [(x, y)]."""
    out = []
    for a, b in GV.cum(pts, t):
        ks = [(a + i) % len(pts) for i in range((b - a) % len(pts) + 1)]
        if len(ks) >= t['so_diem']:
            out += [pts[k] for k in ks]
    return out


def _ve(p, tt):
    """Diem p (khung xe luc dau) -> khung than xe dat o tu the tt = (dx, dy, dth) (khung xe luc dau)."""
    dx, dy, th = tt
    c, s = math.cos(th), math.sin(th)
    x0, y0 = p[0] - dx, p[1] - dy
    return x0 * c + y0 * s, -x0 * s + y0 * c


def kc_than(p):
    """Diem khung than -> (vung, khe m toi da giac than; trong than = 0)."""
    x, y = p
    vung = 'truoc' if x > X_MUI else 'sau' if x < X_DUOI else 'hong'
    return vung, math.hypot(max(x - X_MUI, X_DUOI - x, 0.0), max(abs(y) - Y_HONG, 0.0))


def khe(pts, t, tt=(0.0, 0.0, 0.0)):
    """-> dict(truoc, hong, sau: khe VUNG (chi diem trong vang; vo cung neu khong co); hl: truoc mui trong hanh lang than |y| <= 0,08
    (luat truoc 0,10 cua vung do); hs: khe hong+duoi = min(hong, sau, diem CHEO truoc mui) (luat 0,06); nho: khe nho nhat MOI diem)."""
    out = dict(truoc=math.inf, hong=math.inf, sau=math.inf, nho=math.inf, hl=math.inf, cheo=math.inf)
    g = t['gac']
    for p in pts:
        q = _ve(p, tt)
        v, kc = kc_than(q)
        out['nho'] = min(out['nho'], kc)
        _, _, vang, _ = GV.vung_diem(q, g)
        if vang:
            out[v] = min(out[v], kc)
            if v == 'truoc':
                out['hl' if abs(q[1]) <= Y_HONG else 'cheo'] = min(out['hl' if abs(q[1]) <= Y_HONG else 'cheo'], kc)
    out['hs'] = min(out['hong'], out['sau'], out['cheo'])
    return out


def tu_the_luc(dt, s):
    loai, g = dt
    if loai == 'tien':
        return s * g, 0.0, 0.0
    if loai == 'lui':
        return -s * g, 0.0, 0.0
    return 0.0, 0.0, s * g


def cham(pts, dt, t):
    """Cham mot dong tac tren SUOT quang quet. -> dict(kha_thi, ly, cuoi (khe() o tu the cuoi))."""
    if dt[0] == 'xoay':
        gan = min((math.hypot(*p) for p in pts), default=math.inf)
        if gan <= t['r_xoay']:
            return dict(kha_thi=False, ly='xoay: diem cach tam %.3f <= %.3f' % (gan, t['r_xoay']), cuoi=None)
    k0 = khe(pts, t)
    n = max(1, int(math.ceil(abs(dt[1]) / (BUOC_RAD if dt[0] == 'xoay' else BUOC_M))))
    for i in range(1, n + 1):
        k = khe(pts, t, tu_the_luc(dt, i / n))
        if k['hs'] < k0['hs'] - SAI:
            return dict(kha_thi=False, ly='khe hong+duoi %.3f < luc dau %s' % (k['hs'], _m(k0['hs'])), cuoi=None)
        if k['hl'] <= t['truoc_do'] and k['hl'] < k0['hl'] - SAI:
            return dict(kha_thi=False, ly='truoc %.3f <= %.2f' % (k['hl'], t['truoc_do']), cuoi=None)
    return dict(kha_thi=True, ly='', cuoi=k)


def _m(v):
    return '-' if v == math.inf else '%.3f' % v


def _lech_goc(a):
    return abs((a + math.pi) % (2 * math.pi) - math.pi)


def thieu(k, t):
    """Phan thieu so voi vung do (m): truoc phai > truoc_do (moi diem, coi nhu vat), hong+duoi phai >= hong_do. 0 = ra khoi do."""
    return max(0.0, t['truoc_do'] + SAI - k['hl']) + max(0.0, t['hong_do'] - k['hs'])


def huong_dong_tac(dt):
    return {'tien': 0.0, 'lui': math.pi}.get(dt[0], dt[1])


def chon_dong_tac(pts, t, huong_dich=None, uu_tien=None):
    """Cho chat: chon MOT dong tac nho. huong_dich: goc toi dich trong khung xe (None: khong biet vi tri => uu tien thang truoc).
    uu_tien: dong tac tu kinh nghiem DUOC (thu truoc neu kha thi va lam khe tot len). -> (dt | None, cham, danh sach ly do bo)."""
    k0 = khe(pts, t)
    nho0, th0 = k0['nho'], thieu(k0, t)
    hd = 0.0 if huong_dich is None else huong_dich
    ung, bo = [], []
    for dt in ([uu_tien] if uu_tien else []) + [d for d in DONG_TAC if d != uu_tien]:
        c = cham(pts, dt, t)
        if not c['kha_thi']:
            bo.append((ten_dong_tac(dt), c['ly']))
            continue
        tot, th = min(c['cuoi']['nho'], t['cho_chat']), thieu(c['cuoi'], t)
        if not (th < th0 - SAI or (th <= th0 + SAI and tot > min(nho0, t['cho_chat']) + SAI)):
            bo.append((ten_dong_tac(dt), 'khong lam tot len (khe %.3f, thieu do %.3f)' % (c['cuoi']['nho'], th)))
            continue
        if dt == uu_tien:
            return dt, c, bo
        ung.append(((-round(th, 3), round(tot, 2), round(math.cos(_lech_goc(huong_dong_tac(dt) - hd)), 2), -abs(dt[1])), dt, c))
    if not ung:
        return None, None, bo
    ung.sort(key=lambda u: u[0], reverse=True)
    return ung[0][1], ung[0][2], bo


def chon_sector(pts, t, huong_dich, kiem_o=lambda x, y: True):
    """Cho thoang: sector 10 do gan huong dich nhat co diem thoat. kiem_o(x, y): diem (khung xe) o costmap < 253.
    -> (phi, r, (x, y)) | None."""
    n = int(round(2 * math.pi / t['sector']))
    phis = sorted((k * t['sector'] - math.pi for k in range(n)), key=lambda a: (_lech_goc(a - huong_dich), a))
    for phi in phis:
        c, s = math.cos(phi), math.sin(phi)
        trong = math.inf
        for x, y in pts:
            doc, ngang = x * c + y * s, abs(-x * s + y * c)
            if doc > 0 and ngang <= Y_HONG + t['hong_do']:
                trong = min(trong, doc - X_MUI)
        if trong < t['trong']:
            continue
        r = min(t['r_max'], trong - t['truoc_do'])
        while r >= t['r_min'] - 1e-9:
            if kiem_o(r * c, r * s):
                return phi, r, (r * c, r * s)
            r -= 0.05
    return None


def tom_tat(ds):
    """danh_gia -> dict khe dung khi huy: truoc_moi (moi loai), truoc_vat (hanh lang than truoc mui), hong_vat, hong, sau (moi loai,
    trong vang). Cum vung truoc ma diem gan nhat nam CHEO (ngoai hanh lang) => tinh vao hong (nhu khe())."""
    out = dict(truoc_moi=math.inf, truoc_vat=math.inf, hong_vat=math.inf, hong=math.inf, sau=math.inf)
    for d in ds:
        v, kc = d['vung'], d['kc']
        if v == 'truoc':
            if d.get('hl') is not None:
                out['truoc_moi'] = min(out['truoc_moi'], d['hl'])
                if d['loai'] == 'VAT':
                    out['truoc_vat'] = min(out['truoc_vat'], d['hl'])
            if d.get('hl') is None or kc < d['hl'] - 1e-9:
                out['hong'] = min(out['hong'], kc)
                if d['loai'] == 'VAT':
                    out['hong_vat'] = min(out['hong_vat'], kc)
        elif v == 'hong':
            out['hong'] = min(out['hong'], kc)
            if d['loai'] == 'VAT':
                out['hong_vat'] = min(out['hong_vat'], kc)
        else:
            out['sau'] = min(out['sau'], kc)
    return out


def _vi_pham(g, g0, nguong):
    return g <= nguong + 1e-9 and (g0 > nguong + 1e-9 or g < g0 - SAI)


def ly_do_huy(k, k0, t, xet_khe=True):
    """Thien Q3: k, k0 = tom_tat luc nay / luc bat dau dong tac. -> chuoi ly do huy | None. xet_khe False (Nav2 di toi diem thoat):
    chi ba dieu kien vung do (khe hong+duoi doi lien tuc khi xe di theo duong)."""
    if _vi_pham(k['truoc_moi'], k0['truoc_moi'], t['mui_do']):
        return 'truoc %.2f <= %.2f (moi vat)' % (k['truoc_moi'], t['mui_do'])
    if _vi_pham(k['truoc_vat'], k0['truoc_vat'], t['truoc_do']):
        return 'truoc vat %.2f <= %.2f' % (k['truoc_vat'], t['truoc_do'])
    if _vi_pham(k['hong_vat'], k0['hong_vat'], t['hong_do']):
        return 'hong vat %.2f <= %.2f' % (k['hong_vat'], t['hong_do'])
    hs, hs0 = min(k['hong'], k['sau']), min(k0['hong'], k0['sau'])
    if xet_khe and hs < hs0 - SAI:
        return 'khe hong+duoi %.2f < luc bat dau %s' % (hs, _m(hs0))
    return None


def vat_chac(ds, t):
    """Nhac doi: VAT CHAC (lidar VAT, khong LECH) trong hanh lang than truoc mui <= nhac_doi_m. -> dict danh_gia | None."""
    return next((d for d in ds if d['vung'] == 'truoc' and d['lidar'] == 'VAT' and not d['lech']
                 and d.get('hl') is not None and d['hl'] <= t['nhac_doi'] + 1e-9), None)


def vung_do(ds, t):
    """Vung do (an_toan, nhu 95d0f82: khong chac => VAT): VAT truoc <= 0,10 (hanh lang) / hong <= 0,06 => dict | None."""
    return GV.do_vat(ds)


def doc_tin_sep(s, tuoi, tuoi_max):
    """/sep/trang_thai (JSON cua sep.py) -> dict(tin, muc_tin, di, ly_do, nav2_song) la TIN (khong phai lenh). Cu / hong => None.
    tin False CHI khi muc_tin KHONG_TIN (tin_cay < 0,50 >= 5 s) / BAO (>= 15 s); NGHI van tin (Thien 00:5x)."""
    if s is None or tuoi is None or tuoi > tuoi_max:
        return None
    try:
        j = json.loads(s)
    except ValueError:
        return None
    if not isinstance(j, dict):
        return None
    mt = j.get('muc_tin')
    return dict(tin=mt not in ('KHONG_TIN', 'BAO'), muc_tin=mt, di=j.get('di'), ly_do=j.get('di_ly') or j.get('ly_do'),
                nav2_song=j.get('nav2_song'))
