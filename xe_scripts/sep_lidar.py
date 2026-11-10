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
  - 10/11 A+ (Thien duyet 15:34). phan_loai_cum ba_dau_hieu: diem cua cum TUONG (theo luat hong: gac_vat_la.ba_dau_hieu) mang co
    `tuong`; khe() them hs_v / hs_t (hong+duoi cua VAT / cua TUONG), thieu() doi VAT >= 0,06 nhung TUONG chi >= khong_cham_m 0,02
    (hong + tuong khong phai vung do). VAT CHAC = cung ong / ngoai ban do. thoat_ra_xa: dong tac duoc ke ca khi dang trong vung do
    neu khe nho nhat (moi diem) KHONG GIAM (trong cho_chat) suot quang quet; xoay khong con cam truoc theo ban_kinh_xoay_m (quet
    kiem). Duong lui: phan_loai_cum cu + thoat_ra_xa false = 629a17b.
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
                r_max=h('thoat_diem_max_m'), gac=GV.tham_so(), ra_xa=h('thoat_ra_xa'), khong_cham=h('khong_cham_m'))


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


def diem_loc_nhan(pts, t, ds):
    """10/11 A+: nhu diem_loc + co tung diem: True = diem cua cum TUONG theo luat hong (danh_gia ba dau hieu, khoa 'ks' / 'tuong_k').
    ds khong co nhan (phan_loai_cum cu) => (diem_loc, None) y het cu."""
    tuong = {k for d in ds or () if d.get('tuong_k') for k in d.get('ks', ())}
    if not any('ks' in d for d in ds or ()):
        return diem_loc(pts, t), None
    out, fl = [], []
    for a, b in GV.cum(pts, t):
        ks = [(a + i) % len(pts) for i in range((b - a) % len(pts) + 1)]
        if len(ks) >= t['so_diem']:
            out += [pts[k] for k in ks]
            fl += [k in tuong for k in ks]
    return out, fl


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


def khe(pts, t, tt=(0.0, 0.0, 0.0), tuong=None):
    """-> dict(truoc, hong, sau: khe VUNG (chi diem trong vang; vo cung neu khong co); hl: truoc mui trong hanh lang than |y| <= 0,08
    (luat truoc 0,10 cua vung do); hs: khe hong+duoi = min(hong, sau, diem CHEO truoc mui) (luat 0,06); nho: khe nho nhat MOI diem)."""
    out = dict(truoc=math.inf, hong=math.inf, sau=math.inf, nho=math.inf, hl=math.inf, cheo=math.inf)
    if tuong is not None:                                # 10/11 A+: hong+duoi tach VAT (_v) / TUONG (_t)
        out.update(hs_v=math.inf, hs_t=math.inf, hong_v=math.inf, hong_t=math.inf, nho_v=math.inf, nho_t=math.inf)
    g = t['gac']
    for i, p in enumerate(pts):
        q = _ve(p, tt)
        v, kc = kc_than(q)
        out['nho'] = min(out['nho'], kc)
        if tuong is not None:                            # khe nho nhat MOI diem (ca ngoai vang) cua VAT / TUONG
            out['nho_t' if tuong[i] else 'nho_v'] = min(out['nho_t' if tuong[i] else 'nho_v'], kc)
        _, _, vang, _ = GV.vung_diem(q, g)
        if vang:
            out[v] = min(out[v], kc)
            if v == 'truoc':
                out['hl' if abs(q[1]) <= Y_HONG else 'cheo'] = min(out['hl' if abs(q[1]) <= Y_HONG else 'cheo'], kc)
            if tuong is not None and not (v == 'truoc' and abs(q[1]) <= Y_HONG):
                h = '_t' if tuong[i] else '_v'
                out['hs' + h] = min(out['hs' + h], kc)
                if v == 'hong':
                    out['hong' + h] = min(out['hong' + h], kc)
    out['hs'] = min(out['hong'], out['sau'], out['cheo'])
    return out


def tu_the_luc(dt, s):
    loai, g = dt
    if loai == 'tien':
        return s * g, 0.0, 0.0
    if loai == 'lui':
        return -s * g, 0.0, 0.0
    return 0.0, 0.0, s * g


def cham(pts, dt, t, tuong=None):
    """Cham mot dong tac tren SUOT quang quet (moi BUOC_RAD 1 do / BUOC_M 5 mm, moi diem so voi DA GIAC than 25 x 16 o tu the do).
    -> dict(kha_thi, ly, cuoi (khe() o tu the cuoi)).
    10/11 A+ thoat_ra_xa (Thien 16:xx): bo cam xoay theo ban_kinh_xoay_m va bo hai luat cu (khe hong+duoi khong giam, truoc <= 0,10);
    thay bang KHONG CHAM o MOI mau: VAT >= hong_do 0,06, TUONG >= khong_cham_m 0,02 (khong co nhan: moi diem la VAT); luc dau da
    duoi nguong thi khoang cach do khong duoc giam o mau nao; them mui: truoc hanh lang than (moi vat) khong vao <= vung_phanh 0,07
    (da <= thi khong giam)."""
    ra_xa = t.get('ra_xa', False)
    if dt[0] == 'xoay' and not ra_xa:
        gan = min((math.hypot(*p) for p in pts), default=math.inf)
        if gan <= t['r_xoay']:
            return dict(kha_thi=False, ly='xoay: diem cach tam %.3f <= %.3f' % (gan, t['r_xoay']), cuoi=None)
    k0 = khe(pts, t, tuong=tuong)
    n = max(1, int(math.ceil(abs(dt[1]) / (BUOC_RAD if dt[0] == 'xoay' else BUOC_M))))
    for i in range(1, n + 1):
        k = khe(pts, t, tu_the_luc(dt, i / n), tuong)
        if ra_xa:
            for ten, khoa, nguong in (('vat', 'nho_v', t['hong_do']), ('tuong', 'nho_t', t['khong_cham'])):
                g, g0 = k.get(khoa, k['nho'] if khoa == 'nho_v' else math.inf), k0.get(khoa, k0['nho'] if khoa == 'nho_v' else math.inf)
                if g < min(nguong, g0) - SAI:
                    return dict(kha_thi=False, ly='cham %s: %.3f < %s (nguong %.2f / luc dau)' % (ten, g, _m(min(nguong, g0)), nguong), cuoi=None)
            if k['hl'] <= t['mui_do'] and k['hl'] < k0['hl'] - SAI:
                return dict(kha_thi=False, ly='mui %.3f <= %.2f' % (k['hl'], t['mui_do']), cuoi=None)
            continue
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
    if 'hs_v' in k:                                      # 10/11 A+: TUONG o hong / duoi chi can khong cham
        return (max(0.0, t['truoc_do'] + SAI - k['hl']) + max(0.0, t['hong_do'] - k['hs_v'])
                + max(0.0, t['khong_cham'] - k['hs_t']))
    return max(0.0, t['truoc_do'] + SAI - k['hl']) + max(0.0, t['hong_do'] - k['hs'])


def huong_dong_tac(dt):
    return {'tien': 0.0, 'lui': math.pi}.get(dt[0], dt[1])


def chon_dong_tac(pts, t, huong_dich=None, uu_tien=None, tuong=None):
    """Cho chat: chon MOT dong tac nho. huong_dich: goc toi dich trong khung xe (None: khong biet vi tri => uu tien thang truoc).
    uu_tien: dong tac tu kinh nghiem DUOC (thu truoc neu kha thi va lam khe tot len). -> (dt | None, cham, danh sach ly do bo)."""
    k0 = khe(pts, t, tuong=tuong)
    nho0, th0 = k0['nho'], thieu(k0, t)
    hd = 0.0 if huong_dich is None else huong_dich
    ung, bo = [], []
    for dt in ([uu_tien] if uu_tien else []) + [d for d in DONG_TAC if d != uu_tien]:
        c = cham(pts, dt, t, tuong)
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
    return next((d for d in ds if d['vung'] == 'truoc' and (d['chac'] if 'chac' in d else d['lidar'] == 'VAT' and not d['lech'])
                 and d.get('hl') is not None and d['hl'] <= t['nhac_doi'] + 1e-9), None)       # 10/11 A+: chac = ong / ngoai ban do


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
