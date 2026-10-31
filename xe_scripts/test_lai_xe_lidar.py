#!/usr/bin/env python3
"""Thu lai_xe_lidar.py bang xe mo phong vong kin (khong can ROS). Chay: python3 -I test_lai_xe_lidar.py
Mo phong: the gioi la cac hinh chu nhat, LiDAR 410 tia 360 do (giong that), xe vi sai ly tuong (khong tre, khong sai lech).
KHONG chung minh chay tot tren xe that: chi kiem logic va khong cham vat trong mo phong."""
import math
import sys
from types import SimpleNamespace

import lai_xe_lidar as L

BEAMS = 410
RANGE_MIN, RANGE_MAX = 0.10, 8.0


def ray_rect(ox, oy, dx, dy, r):
    xmin, xmax, ymin, ymax = r
    tmin, tmax = 0.0, 1e9
    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                return None
    return tmin


def make_scan(world, pose):
    x, y, th = pose
    lx, ly = x + L.LIDAR_X * math.cos(th), y + L.LIDAR_X * math.sin(th)
    inc = 2 * math.pi / BEAMS
    ranges = []
    for i in range(BEAMS):
        a = th + (-math.pi + i * inc)
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for r in world:
            t = ray_rect(lx, ly, dx, dy, r)
            if t is not None and t > 0 and t < best:
                best = t
        ranges.append(best if RANGE_MIN <= best <= RANGE_MAX else math.inf)   # vung mu < range_min nhu LiDAR that
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=inc, range_min=RANGE_MIN, range_max=RANGE_MAX)


def collides(world, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    ix = L.X_DUOI
    while ix <= L.X_MUI + 1e-9:
        iy = -L.Y_HONG
        while iy <= L.Y_HONG + 1e-9:
            wx, wy = x + ix * c - iy * s, y + ix * s + iy * c
            for (x0, x1, y0, y1) in world:
                if x0 + 1e-6 < wx < x1 - 1e-6 and y0 + 1e-6 < wy < y1 - 1e-6:
                    return True
            iy += 0.01
        ix += 0.01
    return False


def min_edge(world, pose):
    pts = L.scan_to_xy(make_scan(world, pose))
    return min((L.khoang_cach_mep(a, b) for a, b in pts), default=math.inf)


def simulate(world, pose, secs, goal_active=True, dt=0.1, grid=None, pose_err=(0.0, 0.0, 0.0), **kw):
    brain = L.NguoiLai(**kw)
    t = 0.0
    first_cmd = None
    names = []
    path_len = 0.0
    start = pose
    while t < secs:
        pts = L.scan_to_xy(make_scan(world, pose))
        pl = None
        if grid is not None:
            est = (pose[0] + pose_err[0], pose[1] + pose_err[1], pose[2] + pose_err[2])
            pl = lambda d, est=est: L.la_dia_hinh(d, est, grid)
        v, w, ten = brain.step(pts, t, goal_active, pl)
        if not names or names[-1] != ten:
            names.append(ten)
        moved = v is not None and (abs(v) > 1e-9 or abs(w) > 1e-9)
        if moved and first_cmd is None:
            first_cmd = t
        if v is not None:
            x, y, th = pose
            pose = (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt)
            path_len += abs(v) * dt
        if collides(world, pose):
            return dict(ok=False, why='CHAM VAT luc t=%.1f' % t, names=names, pose=pose, t_first=first_cmd, dmin=0.0)
        t += dt
    return dict(ok=True, names=names, pose=pose, t_first=first_cmd, dmin=min_edge(world, pose), dist=path_len,
                turned=abs(math.degrees(L.wrap(pose[2] - start[2]))))


# the gioi: hanh lang doc truc x, tuong o y = +-0.26 (rong 0,52 m), dai tu x = -1 toi 2
def hanh_lang(extra=()):
    return [(-1.0, 2.0, 0.26, 0.60), (-1.0, 2.0, -0.60, -0.26)] + list(extra)


results = []


def check(name, cond, detail=''):
    results.append((name, cond))
    print(('PASS  ' if cond else 'FAIL  ') + name + ('  | ' + detail if detail else ''))


# --- ham hinh hoc ---
check('khoang cach mep: diem sat mui 0,06 m', abs(L.khoang_cach_mep(0.205, 0.0) - 0.06) < 1e-9)
check('khoang cach mep: diem canh hong 0,05 m', abs(L.khoang_cach_mep(0.0, 0.13) - 0.05) < 1e-9)
check('vung: truoc, trai, goc truoc-phai', L.vung(0.3, 0) == (1, 0) and L.vung(0, 0.3) == (0, 1) and L.vung(0.3, -0.3) == (1, -1))
check('thoang ve phia truoc trong hanh lang trong', abs(L.thoang(L.scan_to_xy(make_scan(hanh_lang([(0.5, 0.7, -0.1, 0.1)]), (0, 0, 0))), 0.0) - (0.5 - 0.145)) < 0.03)

# 1. thoang hoan toan: khong lam gi
r = simulate(hanh_lang(), (0.0, 0.0, 0.0), 10)
check('1 thoang: khong phat lenh nao', r['ok'] and r['t_first'] is None, str(r['names']))

def tt_cuoi(world, pose):
    nl = L.NguoiLai()
    return nl.phan_tich(L.scan_to_xy(make_scan(world, pose)))[0]


# 2. vat chan truoc mui 6 cm (nguoi, rong 20 cm: rong hon xe nen cham ca hai goc truoc): cho 30 s roi xoay ve huong thoang
world = hanh_lang([(0.205, 0.40, -0.10, 0.10)])
r = simulate(world, (0.0, 0.0, 0.0), 80)
check('2 chan truoc: khong cham, cho >= 40 s moi hanh dong', r['ok'] and r['t_first'] is not None and r['t_first'] >= 39.0,
      'bat dau thoat luc t=%s; %s' % (r['t_first'], r['names']))
check('2 chan truoc: da xoay >= 120 do ve huong thoang, khong cham', r['ok'] and r.get('turned', 0) >= 120 and tt_cuoi(world, r['pose'])['ff'] >= 0.15,
      'ff=%.2f quay %.0f do' % (tt_cuoi(world, r['pose'])['ff'], r.get('turned', 0)))

# 3. sat tuong ben trai 5 cm, mui thoang: di cung ra xa
world = [(-1.0, 3.0, 0.13, 0.60)]
r = simulate(world, (0.0, 0.0, 0.0), 15)
check('3 sat hong trai 5 cm: khong cham', r['ok'], r.get('why', '') + str(r['names']))
t3 = tt_cuoi(world, r['pose']) if r['ok'] else {'dl': 0}
check('3 sat hong trai 5 cm: hong cach tuong >= 0,115 m', r['ok'] and t3['dl'] >= 0.115, 'dl=%.3f %s pose=%s' % (t3['dl'], r['names'], tuple(round(v, 3) for v in r['pose'])))

# 4. tuong dung truoc mui 6 cm, hanh lang hep (hong 8 cm, khong xoay duoc), phia sau thoang: lui
world = [(0.205, 0.50, -0.60, 0.60), (-1.0, 3.0, 0.16, 0.60), (-1.0, 3.0, -0.60, -0.16)]
r = simulate(world, (0.0, 0.0, 0.0), 15)
check('4 hem + chan truoc: khong cham', r['ok'], r.get('why', '') + str(r['names']))
check('4 hem + chan truoc: lui du ra khoi tuong (>= 6 cm) va khong lui qua 30 cm', r['ok'] and -0.30 < r['pose'][0] < -0.05,
      'x=%.3f %s' % (r['pose'][0], r['names']))

# 5. bi bit kin 4 phia (5 cm): khong co duong, khong duoc nhuc nhich
world = [(0.195, 0.60, -0.60, 0.60), (-0.60, -0.155, -0.60, 0.60), (-0.60, 0.60, 0.13, 0.60), (-0.60, 0.60, -0.60, -0.13)]
r = simulate(world, (0.0, 0.0, 0.0), 15)
check('5 bit kin: bo cuoc, khong nhuc nhich, khong cham', r['ok'] and r['t_first'] is None and any('BO_CUOC' in n for n in r['names']), str(r['names']))

# 6. dang di vao vung vang (15 cm), khong do: khong can thiep
world = [(-1.0, 3.0, 0.23, 0.60)]
r = simulate(world, (0.0, 0.0, 0.0), 10)
check('6 vung vang (hong 15 cm): khong can thiep', r['ok'] and r['t_first'] is None, str(r['names']))

# 7. khong co dich dang chay: khong lam gi du sat tuong
world = [(-1.0, 3.0, 0.13, 0.60)]
r = simulate(world, (0.0, 0.0, 0.0), 10, goal_active=False)
check('7 khong co dich: khong can thiep', r['ok'] and r['t_first'] is None, str(r['names']))

# 8. goc trong: tuong truoc 6 cm + tuong trai 5 cm, phia sau thoang (khong xoay duoc): lui roi di cung ra xa tuong trai
world = [(0.205, 0.50, -0.60, 0.60), (-1.0, 3.0, 0.13, 0.60)]
r = simulate(world, (0.0, 0.0, 0.0), 30)
t8 = tt_cuoi(world, r['pose']) if r['ok'] else {'ff': 0, 'dl': 0}
check('8 goc trong: khong cham', r['ok'], r.get('why', '') + str(r['names']))
check('8 goc trong: thoat duoc, mui cach tuong truoc >= 0,10 m', r['ok'] and t8['ff'] >= 0.10, 'ff=%.2f dl=%.2f pose=%s %s' % (t8['ff'], t8['dl'], tuple(round(v, 3) for v in r['pose']), r['names']))

# 9. nhieu: 1 diem la trong vung do khong du de coi la do
pts = [(0.205, 0.0)]
nl = L.NguoiLai()
z, red = nl.phan_tich(pts)
check('9 loc nhieu: 1 diem trong vung do khong tinh la do', not red)

# 10. hanh lang hep deu hai ben (hong 8 cm moi ben), phia truoc va sau thoang: khong lam gi (khong co cach cai thien)
world = [(-1.0, 3.0, 0.16, 0.60), (-1.0, 3.0, -0.60, -0.16)]
r = simulate(world, (0.0, 0.0, 0.0), 10)
check('10 hep hai ben, thoang truoc/sau: khong phat lenh', r['ok'] and r['t_first'] is None, str(r['names']))

# 11. mat quet bi nhieu 1 diem: khong dung xe
pts = [(0.205, 0.0), (1.0, 0.0), (0.0, 1.0)]
nl = L.NguoiLai()
out = [nl.step(pts, 0.1 * i, True)[2] for i in range(5)]
check('11 1 diem nhieu truoc mui: khong cho, khong thoat', all(n in ('XANH_VANG',) for n in out), str(set(out)))

def grid_tu_the_gioi(world, res=0.05, x0=-1.5, y0=-1.0, x1=3.0, y1=1.0):
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    data = []
    for j in range(h):
        for i in range(w):
            cx, cy = x0 + (i + 0.5) * res, y0 + (j + 0.5) * res
            data.append(100 if any(a < cx < b and c < cy < d for a, b, c, d in world) else 0)
    return dict(res=res, w=w, h=h, ox=x0, oy=y0, data=data)


# 13. Huong A: phan biet dia hinh (co trong ban do) voi vat la (khong co)
# tuong mong nhu foam/ban do that (5 cm = 1 o)
tuong = [(-1.0, 2.0, 0.26, 0.31), (-1.0, 2.0, -0.31, -0.26)]
tuong_truoc = (0.205, 0.255, -0.60, 0.60)
nguoi = (0.205, 0.40, -0.10, 0.10)
g_map = grid_tu_the_gioi(tuong + [tuong_truoc])                    # ban do: hanh lang + tuong truoc
r = simulate(tuong + [tuong_truoc], (0.0, 0.0, 0.0), 12, grid=g_map)
check('13a tuong truoc co trong ban do: thoat NGAY (khong cho 30 s), khong cham', r['ok'] and r['t_first'] is not None and r['t_first'] < 3.0 and not any(n.startswith('CHO_') for n in r['names']),
      'bat dau luc t=%s %s' % (r['t_first'], r['names'][:6]))
g_khong_nguoi = grid_tu_the_gioi(tuong)                              # ban do khong co nguoi
r = simulate(tuong + [nguoi], (0.0, 0.0, 0.0), 60, grid=g_khong_nguoi)
check('13b nguoi KHONG co trong ban do: van cho >= 40 s', r['ok'] and r['t_first'] is not None and r['t_first'] >= 39.0, 'bat dau luc t=%s' % r['t_first'])
r = simulate(tuong + [tuong_truoc], (0.0, 0.0, 0.0), 60, grid=g_map, pose_err=(0.15, 0.0, 0.0))
check('13c vi tri xe sai 15 cm: coi la vat la (an toan), van cho >= 40 s', r['ok'] and r['t_first'] is not None and r['t_first'] >= 39.0, 'bat dau luc t=%s' % r['t_first'])
r = simulate(tuong + [tuong_truoc], (0.0, 0.0, 0.0), 60, grid=None)
check('13d khong co ban do: coi la vat la, cho >= 40 s', r['ok'] and r['t_first'] is not None and r['t_first'] >= 39.0, 'bat dau luc t=%s' % r['t_first'])
check('13e o_chiem: sat o chiem trong 1 o thi tinh, xa 2 o thi khong',
      L.o_chiem(g_map, 0.0, 0.27) and not L.o_chiem(g_map, 0.0, 0.10),)
check('13f la_dia_hinh: khong co diem hoac thieu vi tri thi None', L.la_dia_hinh([], (0, 0, 0), g_map) is None and L.la_dia_hinh([(0.2, 0)], None, g_map) is None)

# 12. thu ngau nhien (an toan): the gioi ngau nhien sat quanh xe, xe khoi dau khong cham; trong 12 s khong duoc cham vat
import random
random.seed(31102026)
tong = cham = 0
da_nhuc = bo_cuoc = 0
lan_thu = 0
while tong < 40 and lan_thu < 2000:
    lan_thu += 1
    world = []
    for _k in range(random.randint(1, 4)):
        ang = random.uniform(-math.pi, math.pi)
        rad = random.uniform(0.18, 0.45)
        cx, cy = rad * math.cos(ang), rad * math.sin(ang)
        hw, hh = random.uniform(0.04, 0.5), random.uniform(0.04, 0.5)
        world.append((cx - hw, cx + hw, cy - hh, cy + hh))
    pose = (0.0, 0.0, random.uniform(-math.pi, math.pi))
    if collides(world, pose) or min_edge(world, pose) < 0.015 or min_edge(world, pose) > 0.14:
        continue
    tong += 1
    r = simulate(world, pose, 12)
    if not r['ok']:
        cham += 1
        print('   cham o ca ngau nhien:', world, tuple(round(v, 3) for v in pose), r['why'], r['names'][:8])
    else:
        da_nhuc += r['t_first'] is not None
        bo_cuoc += any('BO_CUOC' in n for n in r['names'])
check('12 ngau nhien: %d ca sat vat, %d ca cham vat' % (tong, cham), tong >= 30 and cham == 0, 'xe da chu dong di chuyen %d ca, bo cuoc %d ca' % (da_nhuc, bo_cuoc))

fails = [n for n, ok in results if not ok]
print('\nTONG: %d/%d dat' % (len(results) - len(fails), len(results)))
sys.exit(1 if fails else 0)
