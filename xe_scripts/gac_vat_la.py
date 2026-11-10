#!/usr/bin/env python3
"""gac_vat_la.py — "vung quanh xe" (Thien chot 09/11 19:57-20:33, CHUA THU TREN XE). Thay gac ban do 09/11 (dung im ban 2).

Phan thuan (khong ROS), gui_duong.py goi moi lan quet /scan moi (10 Hz). LiDAR LAI, ban do chi NHAC.
  1. Cum: tia LiDAR theo thu tu goc (vong 360), hai diem ke nhau cach <= cum_cach_m thi chung cum; tia khong ve (None) cat cum.
     Cum < so_diem_toi_thieu diem = nhieu, bo.
  2. LiDAR quyet: dai (dau-cuoi) <= vat_dai_max_m va HO HAI BEN (tia ke ngoai cum khong ve, hoac xa hon dau cum >= ho_hai_ben_m)
     => VAT; dai >= tuong_dai_min_m => TUONG; con lai (giua hai so, hoac mot ben bi che) => KHONG CHAC => VAT.
     ponytail: chua dung do cong cua cum; them neu VAT/TUONG nham tren xe.
  3. Ban do (/map_tinh, tuong >= 65; ban v3 tuong 99): >= nua so diem cua cum cach o tuong <= gan_tuong_ban_do_m => nhac TUONG,
     khac => nhac VAT. Lech voi ket qua => LECH (chi ghi log / trang, KHONG doi ket qua). Khong co ban do => khong nhac.
  4. Vung theo TAM XE (khung base_footprint; than x -0,105..0,145, y +-0,08):
       truoc (x > mui): kc = toi mui (cheo: toi goc); vang: kc <= vang_truoc_m va |y| <= 0,08 + vang_hong_m; DO: kc <= truoc_vat_do_m
         va |y| <= 0,08 (hanh lang than)
       sau (x < duoi): kc = toi duoi (cheo: toi goc); vang: kc <= vang_sau_m va |y| <= 0,08 + vang_hong_m; DO: kc <= duoi_do_m, |y| <= 0,08
       hong (doc than): kc = |y| - 0,08; vang: kc <= vang_hong_m;                              DO: kc <= cach_vat_toi_thieu_m
     DO lam xe dung CHI KHI la VAT (truoc, hong); duoi do moi vat la viec cua BackUp (gui_duong chi rut quang lui: khoang_sau).
  5. 10/11 A+ (Thien duyet 15:34; phan_loai_cum = ba_dau_hieu, mac dinh; "cu" = 629a17b): BA DAU HIEU moi cum
     a. hinh cung: doan ngan lien tiep (Douglas-Peucker 0,005) >= cung_so_diem diem, khop tron binh phuong nho nhat r 0,06-0,12,
        RMS <= 0,01, loi ve phia LiDAR => "ong";
     b. ban do: diem cach o tuong /map_tinh > ngoai_ban_do_m 0,08 la "ngoai"; >= ngoai_so_diem 2 diem => co phan ngoai ban do;
     c. thang / gay goc: Douglas-Peucker thang_rms_m 0,01, moi khop gay 90 (hoac 180) +- goc_gay_lech_deg.
     Quyet: cung ong => VAT (ke ca sat tuong); co phan ngoai ban do => VAT; tren ban do + thang/gay goc + khong cung => TUONG;
     con lai KHONG_CHAC: mui / duoi coi VAT, hong theo ban do. Khong co ban do (khong /map_tinh / TF): cum khong phai cung => luat
     LiDAR cu (muc 2) cho cum do, MEM "(... : khong ban do, lidar vat|tuong|khong_chac)". MEM ghi nhan + ly do: "(tuong: tren ban do, goc 88 do)",
     "(ong: cung r=0.09 rms=0.004)". Hong + TUONG: KHONG do; chi giu khong cham (kc <= khong_cham_m 0,02 => 'cham' = do).
Chay rieng (kiem_xe.sh 8e): python3 ~/gac_vat_la.py --kiem-map 10  => "DAT gac: /map_tinh ..." | "KHONG DAT gac: ..." (ma 0 / 1).
"""
import math
import sys

import hanh_vi as HV

X_MUI, X_DUOI, Y_HONG = 0.145, -0.105, 0.08       # footprint (nav2_params_mat*.yaml), nhu lai_xe_lidar.py
X_LIDAR = 0.025                                    # LiDAR trong khung base_footprint (y 0)
O_TUONG_MIN = 65                                   # /map_tinh: o >= so nay la tuong (v3 tinh: 99)


def tham_so():
    """So lay tu hanh_vi (~/hanh_vi_xe.yaml): do = ban_nang.an_toan, vang + phan biet = thoi_quen."""
    h = HV.so
    return dict(truoc_do=HV.an_toan('truoc_vat_do_m'), hong_do=HV.an_toan('cach_vat_toi_thieu_m'), duoi_do=HV.an_toan('duoi_do_m'),
                vang_truoc=h('vang_truoc_m'), vang_hong=h('vang_hong_m'), vang_sau=h('vang_sau_m'),
                vat_dai=h('vat_dai_max_m'), tuong_dai=h('tuong_dai_min_m'), cum_cach=h('cum_cach_m'), ho=h('ho_hai_ben_m'),
                gan_tuong=h('gan_tuong_ban_do_m'), so_diem=h('so_diem_toi_thieu'), sach_s=h('sach_s'), cu_s=h('lidar_cu_s'),
                phan_loai=h('phan_loai_cum'), cung_r=(h('cung_r_min_m'), h('cung_r_max_m')), cung_rms=h('cung_rms_m'),
                cung_n=h('cung_so_diem'), ngoai_m=h('ngoai_ban_do_m'), ngoai_n=h('ngoai_so_diem'), thang_rms=h('thang_rms_m'),
                goc_lech=math.radians(h('goc_gay_lech_deg')), khong_cham=h('khong_cham_m'))


class BanDo:
    """/map_tinh: (w, h, res, ox, oy, data) hang tu duoi len (OccupancyGrid)."""

    def __init__(self, w, h, res, ox, oy, data):
        self.w, self.h, self.res, self.ox, self.oy, self.data = w, h, res, ox, oy, data

    def gia(self, i, j):
        return self.data[j * self.w + i] if 0 <= i < self.w and 0 <= j < self.h else None

    def gan_tuong(self, x, y, r):
        """Co o tuong (>= O_TUONG_MIN) cach diem (x, y) <= r (khoang toi canh o)."""
        i, j = int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))
        k = int(math.ceil(r / self.res)) + 1
        for jj in range(j - k, j + k + 1):
            for ii in range(i - k, i + k + 1):
                g = self.gia(ii, jj)
                if g is None or g < O_TUONG_MIN:
                    continue
                x0, y0 = self.ox + ii * self.res, self.oy + jj * self.res
                if math.hypot(max(x0 - x, 0.0, x - (x0 + self.res)), max(y0 - y, 0.0, y - (y0 + self.res))) <= r:
                    return True
        return False


def diem_quet(ranges, a_min, a_inc, r_min, r_max):
    """LaserScan -> [(x, y) | None] khung LiDAR, GIU thu tu tia (None = tia ngoai [r_min, r_max] / khong huu han)."""
    out = []
    for k, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < r_min or r > r_max:
            out.append(None)
            continue
        a = a_min + k * a_inc
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


def bien_doi(pts, tu_the):
    """Diem trong khung A (None giu nguyen), tu_the (x, y, yaw) cua A trong khung B -> diem trong khung B."""
    x0, y0, th = tu_the
    c, s = math.cos(th), math.sin(th)
    return [None if p is None else (x0 + p[0] * c - p[1] * s, y0 + p[0] * s + p[1] * c) for p in pts]


def _r(p):
    return math.hypot(p[0] - X_LIDAR, p[1])


def cum(pts, t):
    """[(x, y) | None] khung xe, theo thu tu tia (vong) -> [(chi so dau, chi so cuoi)] (cuoi co the < dau khi vat qua tia 0)."""
    n = len(pts)
    noi = [pts[k] is not None and pts[(k + 1) % n] is not None and math.dist(pts[k], pts[(k + 1) % n]) <= t['cum_cach']
           for k in range(n)]                                 # noi[k]: tia k va k+1 chung cum
    if n and all(noi):
        return [(0, n - 1)]
    dau = [k for k in range(n) if pts[k] is not None and not noi[(k - 1) % n]]
    out = []
    for a in dau:
        b = a
        while noi[b]:
            b = (b + 1) % n
        out.append((a, b))
    return out


def lidar_noi(pts, a, b, t):
    """-> 'VAT' | 'TUONG' | 'KHONG_CHAC' cho cum tia a..b (vong)."""
    n = len(pts)
    dai = math.dist(pts[a], pts[b])
    if dai >= t['tuong_dai']:
        return 'TUONG'

    def ho(k, dau):
        q = pts[k % n]
        return q is None or _r(q) >= _r(dau) + t['ho']
    if dai <= t['vat_dai'] and (b - a) % n + 1 < n and ho(a - 1, pts[a]) and ho(b + 1, pts[b]):
        return 'VAT'
    return 'KHONG_CHAC'


def vung_diem(p, t):
    """Diem khung xe -> (vung 'truoc'|'hong'|'sau', kc m, trong vang?, trong do?)."""
    x, y = p
    e = 1e-9                                             # so m chinh xac (vd 0,445 - 0,145) khong roi ra ngoai vi lam tron
    ngang = max(0.0, abs(y) - Y_HONG)                    # cheo goc: kc = toi goc than
    if x > X_MUI:
        kc = math.hypot(x - X_MUI, ngang)
        return 'truoc', kc, kc <= t['vang_truoc'] + e and ngang <= t['vang_hong'] + e, kc <= t['truoc_do'] + e and ngang == 0.0
    if x < X_DUOI:
        kc = math.hypot(X_DUOI - x, ngang)
        return 'sau', kc, kc <= t['vang_sau'] + e and ngang <= t['vang_hong'] + e, kc <= t['duoi_do'] + e and ngang == 0.0
    kc = abs(y) - Y_HONG
    return 'hong', kc, kc <= t['vang_hong'] + e, kc <= t['hong_do'] + e


def _dp(P, tol):
    """Douglas-Peucker tren day diem P (thu tu tia) -> chi so dinh gay [0, ..., len(P) - 1]."""
    def tach(i, j):
        (x0, y0), (x1, y1) = P[i], P[j]
        L = math.hypot(x1 - x0, y1 - y0) or 1e-12
        k, dm = None, tol
        for m in range(i + 1, j):
            d = abs((x1 - x0) * (y0 - P[m][1]) - (x0 - P[m][0]) * (y1 - y0)) / L
            if d > dm:
                k, dm = m, d
        return [i, j] if k is None else tach(i, k)[:-1] + tach(k, j)
    return tach(0, len(P) - 1) if len(P) > 1 else [0]


def khop_tron(P):
    """Khop duong tron binh phuong nho nhat (Kasa) -> (cx, cy, r, rms) | None."""
    n = len(P)
    if n < 3:
        return None
    mx, my = sum(p[0] for p in P) / n, sum(p[1] for p in P) / n
    u = [(p[0] - mx, p[1] - my) for p in P]
    suu = sum(a * a for a, _ in u); svv = sum(b * b for _, b in u); suv = sum(a * b for a, b in u)
    suuu = sum(a ** 3 for a, _ in u); svvv = sum(b ** 3 for _, b in u)
    suvv = sum(a * b * b for a, b in u); svuu = sum(b * a * a for a, b in u)
    det = suu * svv - suv * suv
    if abs(det) < 1e-12:
        return None
    r1, r2 = (suuu + suvv) / 2, (svvv + svuu) / 2
    uc, vc = (r1 * svv - r2 * suv) / det, (r2 * suu - r1 * suv) / det
    r = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    cx, cy = uc + mx, vc + my
    rms = math.sqrt(sum((math.hypot(p[0] - cx, p[1] - cy) - r) ** 2 for p in P) / n)
    return cx, cy, r, rms


def _goc(a, b, c):
    """Goc tai b (do) giua ba -> bc."""
    v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
    n = math.hypot(*v1) * math.hypot(*v2) or 1e-12
    return math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / n))))


def ba_dau_hieu(P, M, ban, t):
    """10/11 A+: P = diem cum khung xe theo tia, M = diem khung ban do (None tung diem / ca cum khi khong co TF ban do).
    -> (nhan 'ong' | 'ngoai' | 'tuong' | 'khong_chac', ly do, tren_ban_do True | False | None (khong co ban do))."""
    # a. cung: chuoi doan NGAN (<= 0,06) lien tiep cua Douglas-Peucker 0,005
    dinh = _dp(P, 0.005)
    ung = []
    chay = []
    for i, j in zip(dinh, dinh[1:]):
        if math.dist(P[i], P[j]) <= 0.06:
            chay = chay or [i]
            chay.append(j)
        else:
            if chay:
                ung.append((chay[0], chay[-1]))
            chay = []
    if chay:
        ung.append((chay[0], chay[-1]))
    lid = (X_LIDAR, 0.0)
    for a, b in ung:
        Q = P[a:b + 1]
        if len(Q) < t['cung_n']:
            continue
        f = khop_tron(Q)
        if f is None:
            continue
        cx, cy, r, rms = f
        loi = math.dist(lid, (cx, cy)) > sum(math.dist(lid, q) for q in Q) / len(Q)      # tam o sau mat cung (loi ve LiDAR)
        if t['cung_r'][0] <= r <= t['cung_r'][1] and rms <= t['cung_rms'] and loi:
            return 'ong', 'cung r=%.2f rms=%.3f' % (r, rms), None
    # b. ban do
    tren = None
    if ban is not None and M is not None:
        Mm = [m for m in M if m is not None]
        if Mm:
            ngoai = sum(not ban.gan_tuong(x, y, t['ngoai_m']) for x, y in Mm)
            if ngoai >= t['ngoai_n']:
                return 'ngoai', 'ngoai ban do %d diem' % ngoai, False
            tren = True
    # c. thang / gay goc
    dinh = _dp(P, t['thang_rms'])
    goc = [_goc(P[i], P[j], P[k]) for i, j, k in zip(dinh, dinh[1:], dinh[2:])]
    thang_gay = all(abs(g - 90) <= math.degrees(t['goc_lech']) or abs(g - 180) <= math.degrees(t['goc_lech']) for g in goc)
    hinh = 'thang' if not goc else 'goc %d do' % round(next((g for g in goc if abs(g - 90) <= math.degrees(t['goc_lech'])), goc[0]))
    if tren and thang_gay:
        return 'tuong', 'tren ban do, %s' % hinh, True
    return 'khong_chac', '%s, %s' % ('tren ban do' if tren else 'khong ban do' if tren is None else '-', hinh if thang_gay else 'khong thang'), tren


def danh_gia(pts_xe, pts_map, ban, t):
    """-> [dict(vung, loai 'VAT'|'TUONG', lidar, bd 'VAT'|'TUONG'|None, lech, n, kc, do, xy, hl)] moi (cum, vung) co diem trong VANG,
    gan truoc. do = co diem trong DO cua vung do (chi co nghia dung xe khi loai VAT, vung truoc / hong). xy = diem ban do gan nhat.
    hl (vung truoc): kc diem gan nhat trong hanh lang than |y| <= 0,08 (None neu khong co; sep_lidar nhac doi)."""
    out = []
    for a, b in cum(pts_xe, t):
        ks = [(a + i) % len(pts_xe) for i in range((b - a) % len(pts_xe) + 1)]
        if len(ks) < t['so_diem']:
            continue
        theo_vung, hl = {}, None
        for k in ks:
            v, kc, vang, do = vung_diem(pts_xe[k], t)
            if vang:
                n, kc0, do0, k0 = theo_vung.get(v, (0, None, False, None))
                theo_vung[v] = (n + 1, kc if kc0 is None or kc < kc0 else kc0, do0 or do, k if kc0 is None or kc < kc0 else k0)
                if v == 'truoc' and abs(pts_xe[k][1]) <= Y_HONG and (hl is None or kc < hl):
                    hl = kc                                      # 09/11 23:08 nhac doi: diem gan nhat trong hanh lang than truoc mui
        if not theo_vung:
            continue
        lidar = lidar_noi(pts_xe, a, b, t)
        loai = 'TUONG' if lidar == 'TUONG' else 'VAT'           # khong chac => VAT
        bd = None
        if ban is not None and pts_map is not None:
            pm = [pts_map[k] for k in ks if pts_map[k] is not None]
            if pm:
                bd = 'TUONG' if 2 * sum(ban.gan_tuong(x, y, t['gan_tuong']) for x, y in pm) >= len(pm) else 'VAT'
        moi = t.get('phan_loai', 'cu') == 'ba_dau_hieu'
        if moi:                                                  # 10/11 A+: ba dau hieu (cu: y het 629a17b o tren)
            nhan, ly, tren = ba_dau_hieu([pts_xe[k] for k in ks], None if pts_map is None else [pts_map[k] for k in ks], ban, t)
        for v, (n, kc, do, k0) in theo_vung.items():
            d = dict(vung=v, loai=loai, lidar=lidar, bd=bd, lech=bd is not None and bd != loai, n=n, kc=kc, do=do,
                     xy=None if pts_map is None else pts_map[k0], hl=hl if v == 'truoc' else None)
            if moi and nhan != 'ong' and tren is None:         # khong co ban do (/map_tinh hay TF ban do): luat LiDAR cu cho cum nay
                d.update(nhan='lidar', ly='khong ban do, lidar %s' % lidar.lower(), chac=lidar == 'VAT' and not d['lech'], ks=ks,
                         tuong_k=loai == 'TUONG', cham=loai == 'TUONG' and v == 'hong' and kc <= t['khong_cham'] + 1e-9)
            elif moi:
                lo = ('VAT' if nhan in ('ong', 'ngoai') else 'TUONG' if nhan == 'tuong'
                      else 'VAT' if v in ('truoc', 'sau') or not tren else 'TUONG')          # khong chac: mui / duoi VAT, hong theo ban do
                d.update(loai=lo, nhan=nhan, ly=ly, chac=nhan in ('ong', 'ngoai'), lech=bd is not None and bd != lo, ks=ks,
                         tuong_k=nhan == 'tuong' or (nhan == 'khong_chac' and bool(tren)),
                         cham=lo == 'TUONG' and v == 'hong' and kc <= t['khong_cham'] + 1e-9)
            out.append(d)
    return sorted(out, key=lambda d: d['kc'])


def do_vat(ds, vung=('truoc', 'hong')):
    """VAT dau tien (gan nhat) dang trong DO cua mot vung trong `vung` (luat cung: dung xe); khong co => None."""
    return next((d for d in ds if d['vung'] in vung and ((d['loai'] == 'VAT' and d['do']) or d.get('cham'))), None)   # cham: 10/11 A+ hong tuong <= 0,02


def vang_vat(ds, vung=('truoc', 'hong')):
    """VAT gan nhat trong VANG truoc / hong (cong sach / het chan: tuong bo qua; VAT sau duoi khong giu xe tien); khong co => None."""
    return next((d for d in ds if d['loai'] == 'VAT' and d['vung'] in vung), None)


def khoang_sau(pts_xe, t):
    """Khoang (m) tu duoi toi MOI VAT (ca tuong) sau duoi trong hanh lang than (|y| <= 0,08 + 0,02), diem thu so_diem gan nhat
    (loc nhieu); khong co => None. Dung rut quang lui (luat do duoi)."""
    kc = sorted(X_DUOI - x for x, y in (p for p in pts_xe if p is not None) if x < X_DUOI and abs(y) <= Y_HONG + 0.02)
    return kc[t['so_diem'] - 1] if len(kc) >= t['so_diem'] else None


def dong_mem(d):
    """'MEM: truoc|hong|sau n diem cach x m (tuong|vat)' (+ ' LECH ban do=...')."""
    if 'nhan' in d:                                      # 10/11 A+ ba dau hieu: nhan + ly do
        nhan = 'ong' if d['nhan'] == 'ong' else d['loai'].lower()        # nhan 'lidar' (khong ban do): ly = luat LiDAR cu
        return 'MEM: %s %d diem cach %.2f m (%s: %s%s)%s' % (d['vung'], d['n'], d['kc'], nhan, 'khong chac, ' if d['nhan'] == 'khong_chac' else '',
                                                           d['ly'], ' LECH ban do=%s' % d['bd'].lower() if d['lech'] else '')
    return 'MEM: %s %d diem cach %.2f m (%s)%s' % (d['vung'], d['n'], d['kc'], d['loai'].lower(),
                                                   ' LECH ban do=%s' % d['bd'].lower() if d['lech'] else '')


class Mem:
    """Nho vung dang co gi de chi in MEM khi doi: vung moi, doi loai / LECH, kc doi >= 0,03 m; het moi thu => 'MEM: sach' 1 lan."""

    def __init__(self):
        self.cu = {}

    def cap_nhat(self, ds):
        moi, out = {}, []
        for d in ds:
            moi.setdefault(d['vung'], d)                     # ds sap gan truoc: lay gan nhat moi vung
        for v, d in sorted(moi.items()):
            c = self.cu.get(v)
            if c is None or c['loai'] != d['loai'] or c['lech'] != d['lech'] or abs(c['kc'] - d['kc']) >= 0.03 - 1e-9:
                out.append(dong_mem(d))
            else:
                moi[v] = c                                   # giu moc da in (troi cham khong in lai)
        if self.cu and not moi:
            out.append('MEM: sach')
        self.cu = moi
        return out


def kiem_map(giay):
    """kiem_xe.sh: co /map_tinh (TRANSIENT_LOCAL) trong `giay` s, kich thuoc > 0 va co o tuong. Ma 0 DAT, 1 KHONG DAT."""
    import time
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    rclpy.init()
    n = rclpy.create_node('kiem_gac_map')
    nhan = []
    n.create_subscription(OccupancyGrid, '/map_tinh', nhan.append,
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    t_het = time.monotonic() + giay
    while not nhan and time.monotonic() < t_het:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()
    if not nhan:
        print('KHONG DAT  gac: khong nhan /map_tinh trong %d s (ban do khong nhac duoc tuong: map_server / mat_lai?)' % giay)
        return 1
    m = nhan[-1]
    tuong = sum(1 for v in m.data if v >= O_TUONG_MIN)
    ok = m.info.width > 0 and m.info.height > 0 and tuong > 0
    print('%s gac: /map_tinh %dx%d o %.3f m, %d o tuong' % ('DAT       ' if ok else 'KHONG DAT ', m.info.width, m.info.height,
                                                              m.info.resolution, tuong))
    return 0 if ok else 1


if __name__ == '__main__':
    a = sys.argv[1:]
    if a[:1] == ['--kiem-map']:
        sys.exit(kiem_map(float(a[1]) if len(a) > 1 else 10.0))
    print('Dung: python3 ~/gac_vat_la.py --kiem-map [giay]')
    sys.exit(2)
