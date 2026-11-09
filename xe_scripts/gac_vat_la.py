#!/usr/bin/env python3
"""gac_vat_la.py (thiet ke "dung im khi bi chan" ban 2, phan A1, 09/11, CHUA THU TREN XE).

Phan thuan (khong ROS), gui_duong.py goi o vong theo doi 10 Hz:
  - Vat la = diem LiDAR roi vao o TRONG cua /map_tinh (ban do goc, KHONG phai costmap co lop huong dan: K10) VA cach o
    tuong gan nhat > gac_le_tuong_m (lech dinh vi 7-8 cm). Can >= gac_so_diem diem trong MOT vung moi tinh (loc nhieu).
  - Vung gac (khung xe, footprint x -0,105..0,145, y +-0,08):
      truoc: tu mui toi gac_truoc_m, nua rong = 0,08 + cach_vat_toi_thieu_m (rong 0,28);
      trai / phai: doc than (x -0,105..0,145), ra ngoai gac_hong_m; sau: khong (BackUp tu nhin 0,19 m).
  - Tre hai nguong (K2): VAO khi vung (gac_truoc_m, gac_hong_m) co vat la; RA khi vung rong hon (gac_ra_truoc_m,
    gac_ra_hong_m; nua rong truoc them gac_ra_hong_m - gac_hong_m) sach LIEN TUC gac_sach_s giay.
  - Tu the: diem doi sang khung ban do bang TF TAI THOI DIEM QUET (K7; gui_duong lay), khong dung ~/vi_tri_xe.
Chay rieng (kiem_xe.sh): python3 ~/gac_vat_la.py --kiem-map 10  => "DAT gac: /map_tinh ..." | "KHONG DAT gac: ..." (ma 0 / 1).
"""
import math
import sys

import hanh_vi as HV

X_MUI, X_DUOI, Y_HONG = 0.145, -0.105, 0.08       # footprint (nav2_params_mat*.yaml), nhu lai_xe_lidar.py
O_TRONG_MAX, O_TUONG_MIN = 49, 65                  # /map_tinh (trinary): 0 trong, 100 tuong, -1 chua biet


def tham_so():
    """So cua gac lay tu hanh_vi (~/hanh_vi_xe.yaml)."""
    h = HV.so
    return dict(bat=h('gac_vat_la'), truoc=h('gac_truoc_m'), hong=h('gac_hong_m'), ra_truoc=h('gac_ra_truoc_m'),
                ra_hong=h('gac_ra_hong_m'), le_tuong=h('gac_le_tuong_m'), so_diem=h('gac_so_diem'), sach_s=h('gac_sach_s'),
                scan_cu_s=h('gac_scan_cu_s'), le_rong=HV.an_toan('cach_vat_toi_thieu_m'))


class BanDo:
    """/map_tinh: (w, h, res, ox, oy, data) hang tu duoi len (OccupancyGrid)."""

    def __init__(self, w, h, res, ox, oy, data):
        self.w, self.h, self.res, self.ox, self.oy, self.data = w, h, res, ox, oy, data

    def gia(self, i, j):
        return self.data[j * self.w + i] if 0 <= i < self.w and 0 <= j < self.h else None

    def la_vat_la(self, x, y, le):
        """O chua (x, y) la o TRONG va moi o tuong trong ban kinh le deu cach diem > le."""
        i, j = int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))
        v = self.gia(i, j)
        if v is None or v < 0 or v > O_TRONG_MAX:
            return False
        k = int(math.ceil(le / self.res)) + 1
        for jj in range(j - k, j + k + 1):
            for ii in range(i - k, i + k + 1):
                g = self.gia(ii, jj)
                if g is None or g < O_TUONG_MIN:
                    continue
                x0, y0 = self.ox + ii * self.res, self.oy + jj * self.res
                dx = max(x0 - x, 0.0, x - (x0 + self.res))
                dy = max(y0 - y, 0.0, y - (y0 + self.res))
                if math.hypot(dx, dy) <= le:
                    return False
        return True


def diem_quet(ranges, a_min, a_inc, r_min, r_max):
    """LaserScan -> [(x, y)] khung LiDAR (bo tia ngoai [r_min, r_max] hoac khong huu han)."""
    out = []
    for k, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < r_min or r > r_max:
            continue
        a = a_min + k * a_inc
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


def bien_doi(pts, tu_the):
    """Diem trong khung A, tu_the (x, y, yaw) cua A trong khung B -> diem trong khung B."""
    x0, y0, th = tu_the
    c, s = math.cos(th), math.sin(th)
    return [(x0 + x * c - y * s, y0 + x * s + y * c) for x, y in pts]


def vung_cua(px, py, t, ra=False):
    """Diem (khung xe) -> 'truoc' | 'trai' | 'phai' | None. ra=True: vung RA (rong hon) cua tre hai nguong."""
    truoc = t['ra_truoc'] if ra else t['truoc']
    hong = t['ra_hong'] if ra else t['hong']
    nua = Y_HONG + t['le_rong'] + ((t['ra_hong'] - t['hong']) if ra else 0.0)
    if X_MUI < px <= X_MUI + truoc and abs(py) <= nua:
        return 'truoc'
    if X_DUOI <= px <= X_MUI and Y_HONG < abs(py) <= Y_HONG + hong:
        return 'trai' if py > 0 else 'phai'
    return None


def danh_gia(pts_xe, pts_map, ban, t, ra=False):
    """-> None | (vung, so diem, (x, y) ban do cua diem vat la gan xe nhat trong vung do). Vung co >= so_diem diem vat la;
    nhieu vung thi lay vung co diem gan xe nhat."""
    dem = {}
    for (px, py), (mx, my) in zip(pts_xe, pts_map):
        v = vung_cua(px, py, t, ra)
        if v is None or not ban.la_vat_la(mx, my, t['le_tuong']):
            continue
        d = math.hypot(px, py)
        n, gan = dem.get(v, (0, None))
        dem[v] = (n + 1, gan if gan is not None and gan[0] <= d else (d, (mx, my)))
    du = [(gan[0], v, n, gan[1]) for v, (n, gan) in dem.items() if n >= t['so_diem']]
    if not du:
        return None
    _, v, n, xy = min(du)
    return v, n, xy


def hong_gan_nhat(pts_xe, pts_map, ban, t):
    """Khoang ho (m) tu canh than xe toi vat la O HAI HONG (x trong than, |y| > 0,08), lay diem thu gac_so_diem gan nhat
    (loc nhieu nhu danh_gia); khong du diem => None. Dung khi BackUp lui thang (ngoai le hong < cach_vat_toi_thieu_m)."""
    ho = sorted(abs(py) - Y_HONG for (px, py), (mx, my) in zip(pts_xe, pts_map)
                if X_DUOI <= px <= X_MUI and abs(py) > Y_HONG and ban.la_vat_la(mx, my, t['le_tuong']))
    return ho[t['so_diem'] - 1] if len(ho) >= t['so_diem'] else None


class Gac:
    """Trang thai gac (tre hai nguong, K2). cap_nhat(t, pts_xe, pts_map, ban) -> dict(chan, moi, het, vung, n, xy, sach_s)."""

    def __init__(self, t):
        self.t, self.chan, self.t_sach, self.cuoi = t, False, None, None

    def cap_nhat(self, now, pts_xe, pts_map, ban):
        moi = het = False
        if not self.chan:
            d = danh_gia(pts_xe, pts_map, ban, self.t)
            if d is not None:
                self.chan, moi, self.t_sach, self.cuoi = True, True, None, d
        else:
            d = danh_gia(pts_xe, pts_map, ban, self.t, ra=True)
            if d is not None:
                self.t_sach, self.cuoi = None, d
            else:
                self.t_sach = now if self.t_sach is None else self.t_sach
                if now - self.t_sach >= self.t['sach_s']:
                    self.chan, het = False, True
        v, n, xy = self.cuoi if self.cuoi else (None, 0, None)
        return dict(chan=self.chan, moi=moi, het=het, vung=v, n=n, xy=xy,
                    sach_s=0.0 if self.t_sach is None or not self.chan else now - self.t_sach)


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
        print('KHONG DAT  gac: khong nhan /map_tinh trong %d s (gac se mu: map_server / mat_lai?)' % giay)
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
