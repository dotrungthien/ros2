#!/usr/bin/env python3
"""Thu lai_xe_lidar.py bang xe mo phong vong kin (khong can ROS). Chay: python3 -I test_lai_xe_lidar.py
Mo phong: the gioi la cac hinh chu nhat, LiDAR 410 tia 360 do (giong that), xe vi sai ly tuong (khong tre, khong sai lech).
KHONG chung minh chay tot tren xe that: chi kiem logic va khong cham vat trong mo phong."""
import math
import os
import re
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

# --- viec 109c-c1: doc ~/sep_thoat, chi in "SE LAM", khong phat toc do ---
ok_t = L.doc_sep_thoat('100.00 THOAT 1.05 0.06 2 777', 101.5)
check('c1 doc ~/sep_thoat hop le', ok_t == dict(luot=2, ma=777, p=(1.05, 0.06)), str(ok_t))
check('c1 tep cu hon 3 s hoac tuoi am qua 1 s: bo', L.doc_sep_thoat('100.00 THOAT 1 2 2 7', 103.5) is None and L.doc_sep_thoat('100.00 THOAT 1 2 2 7', 98.0) is None)
check('c1 tep hong hoac sai chu: bo', all(L.doc_sep_thoat(d, 101.0) is None for d in ('', 'x', '100.00 CHO 1 2 3 4', '100.00 THOAT 1 2 3', '100.00 THOAT a 2 3 4', '100.00 THOAT 1 2 3 x')))
check('c1 P = "-" thanh p None', L.doc_sep_thoat('100.00 THOAT - - 1 5', 101.0) == dict(luot=1, ma=5, p=None))
check('c1 goc toi P trong khung than xe', abs(L.goc_toi_diem((0, 0, 0), (1, 0))) < 1e-9 and abs(L.goc_toi_diem((0, 0, 0), (0, 1)) - math.pi / 2) < 1e-9
      and abs(L.goc_toi_diem((0, 0, math.pi / 2), (1, 0)) + math.pi / 2) < 1e-9 and L.goc_toi_diem(None, (1, 0)) is None and L.goc_toi_diem((0, 0, 0), None) is None)
_t = dict(luot=1, ma=9, p=(1.05, 0.06))
_pts_truoc = L.scan_to_xy(make_scan(hanh_lang([(0.205, 0.30, -0.10, 0.10)]), (0, 0, 0)))      # vat sat mui, phia sau thoang
_pts_sau = L.scan_to_xy(make_scan(hanh_lang([(-0.20, -0.125, -0.10, 0.10)]), (0, 0, 0)))      # vat sat duoi, phia truoc thoang
_pts_kin = L.scan_to_xy(make_scan(hanh_lang([(0.17, 0.30, -0.10, 0.10), (-0.20, -0.125, -0.10, 0.10)]), (0, 0, 0)))
_c1 = L.cau_se_lam(_t, _pts_truoc, 0.3)
_c2 = L.cau_se_lam(_t, _pts_sau, None)
_c3 = L.cau_se_lam(_t, _pts_kin, -0.5)
check('c1 SE LAM: vat truoc => ra phia sau (LUI), co P', _c1.startswith('SE LAM') and 'ra phia LUI' in _c1 and 'P=(1.05;0.06)' in _c1 and '17 do' in _c1, _c1)
check('c1 SE LAM: vat sau => ra phia truoc (TIEN), khong co huong P', 'ra phia TIEN' in _c2 and 'huong toi P khong co' in _c2, _c2)
check('c1 SE LAM: ke hai dau => khong ra duoc', 'khong ra duoc' in _c3, _c3)
check('c1 cau SE LAM khong giong cau may trang thai', '->' not in _c1 and 'trang_thai=' not in _c1)
_src = open(L.__file__ if L.__file__.endswith('.py') else L.__file__[:-1]).read()
_khoi = _src[_src.index('def _doc_thoat'):_src.index('def _on_nav')]
check('c1 _doc_thoat khong phat toc do (khong publish / Twist / self.pub / phat_lenh)', not any(k in _khoi for k in ('publish', 'Twist', 'self.pub', 'phat_lenh')))

# --- viec 109c-c2: pha RA (di thang ve phia thoang hon cho du cho xoay) ---
def ra_chay(world, pose, giay=12.0, doi_the_gioi=None, goal_active=True):
    """Bo nao + xe ly tuong, 0,1 s/vong. Tra dict: bat_dau, cau, ten (cac ten), xong (cau RA XONG), pose, cham, v (cac van toc)."""
    b = L.NguoiLai()
    pts = L.scan_to_xy(make_scan(world, pose))
    ok, cau = b.ra_bat_dau(pts, 0.0, goal_active)
    r = dict(bat_dau=ok, cau=cau, ten=[], xong='', v=[], cham=False)
    t = 0.0
    while ok and t < giay:
        t += 0.1
        if doi_the_gioi and t >= doi_the_gioi[0]:
            world = doi_the_gioi[1]
        v, w, ten = b.step(L.scan_to_xy(make_scan(world, pose)), t, goal_active)
        r['ten'].append(ten)
        if v is None:
            break
        r['v'].append((v, w))
        pose = (pose[0] + v * math.cos(pose[2]) * 0.1, pose[1] + v * math.sin(pose[2]) * 0.1, pose[2] + w * 0.1)
        r['cham'] = r['cham'] or collides(world, pose)
    r['xong'], r['pose'], r['state'] = b.ra_xong, pose, b.state
    return r


_TRUOC_2CM = hanh_lang([(0.165, 0.40, -0.10, 0.10)])       # vat cach mui 2 cm (mui o x = 0,145), phia sau thoang
_r1 = ra_chay(_TRUOC_2CM, (0.0, 0.0, 0.0))
check('c2 vat sat mui 2 cm: LUI toi khi du cho xoay, khong cham', _r1['bat_dau'] and set(_r1['ten'][:-1]) == {'RA_LUI'} and _r1['xong'].startswith('RA XONG DU_CHO_XOAY')
      and all(v < 0 and w == 0 for v, w in _r1['v']) and _r1['pose'][0] < 0 and not _r1['cham'], '%s %s' % (_r1['cau'], _r1['xong']))
_k4 = ra_chay(hanh_lang([(0.20, 0.40, -0.10, 0.10)]), (0.0, 0.0, 0.0))
check('c2 vat cach mui 5,5 cm (K4 01/11): da du cho xoay (cach tam 20 cm > 18,56), khong di', not _k4['bat_dau'] and _k4['cau'] == 'RA XONG DU_CHO_XOAY 0 cm', _k4['cau'])
_hep = [(-1.0, 2.0, 0.13, 0.60), (-1.0, 2.0, -0.60, -0.13), (0.165, 0.40, -0.10, 0.10)]       # hanh lang hep: tuong ben hong 5 cm, khong bao gio du cho xoay
_r2 = ra_chay(_hep, (0.0, 0.0, 0.0))
check('c2 hanh lang hep: lui dung 30 cm roi thoi, khong cham', _r2['xong'].startswith('RA XONG DA_DI_30CM') and 30 <= int(_r2['xong'].split()[3]) <= 33 and len(_r2['v']) <= 33 and not _r2['cham'], _r2['xong'])
_hai = hanh_lang([(0.165, 0.40, -0.10, 0.10), (-0.40, -0.125, -0.10, 0.10)])                  # truoc 2 cm, sau 2 cm
_r3 = ra_chay(_hai, (0.0, 0.0, 0.0))
check('c2 hai dau deu chan: khong di, co log', not _r3['bat_dau'] and _r3['cau'].startswith('RA KHONG DI: hai dau deu chan') and not _r3['v'], _r3['cau'])
_r4 = ra_chay(_hep, (0.0, 0.0, 0.0), doi_the_gioi=(0.5, [(-1.0, 2.0, 0.13, 0.60), (-1.0, 2.0, -0.60, -0.13), (0.165, 0.40, -0.10, 0.10), (-0.50, -0.20, -0.10, 0.10)]))
check('c2 do phia dang di: dung ngay, ngung phat', _r4['xong'].startswith('RA XONG DO_PHIA_DI') and _r4['ten'][-1] == 'RA_XONG' and len(_r4['v']) <= 5 and not _r4['cham'], _r4['xong'])
_b5 = L.NguoiLai()
_b5.ra_bat_dau(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.0, True)
_b5.step(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 10.5, True)
check('c2 qua 10 s: dung', _b5.ra_xong.startswith('RA XONG QUA_10S'), _b5.ra_xong)
_b6 = L.NguoiLai()
check('c2 khong co dich: khong bat dau; dang tu THOAT: NHUONG cho RA cua sep (luat 02/11 17:33)', _b6.ra_bat_dau(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.0, False) == (False, 'RA BO QUA: khong co dich')
      and (setattr(_b6, 'state', 'THOAT') or _b6.ra_bat_dau(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.0, True))[1].startswith('NHUONG THOAT: RA BAT DAU') and _b6.state == 'RA')
_r7 = ra_chay(_hep, (0.0, 0.0, 0.0), goal_active=True)
_b7 = L.NguoiLai()
_b7.ra_bat_dau(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.0, True)
_b7.step(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.1, False)
check('c2 mat dich giua chung: dung', _b7.ra_xong.startswith('RA XONG MAT_DICH'), _b7.ra_xong)
_src2 = _src
_khoi2 = _src2[_src2.index('def _doc_thoat'):_src2.index('def _on_nav')]
check('c2 chi giao pha RA khi phat_lenh bat; khong phat toc do trong _doc_thoat', 'if phat:' in _khoi2 and _khoi2.index('if phat:') < _khoi2.index('self.brain.ra_bat_dau')
      and not any(k in _khoi2 for k in ('publish', 'Twist', 'self.pub')))
_tick = _src2[_src2.index('def _tick'):_src2.index('def main')]
check('c2 toc do chi phat o _tick khi phat_lenh', "if v is not None and phat:" in _tick and _tick.count('self.pub.publish(t)') == 1)
# luat G: /cmd_vel_avoid, uu tien 60, timeout 0,2 s cua twist_mux; nut phat 10 lan/giay
_mux = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dev_ws', 'src', 'my_bot', 'config', 'twist_mux.yaml')).read()
_av = re.search(r'avoid:\s*\n\s*topic: (\S+)\s*\n\s*timeout: ([0-9.]+)\s*\n\s*priority: (\d+)', _mux)
check('luat G: lai_xe_lidar phat /cmd_vel_avoid, twist_mux uu tien 60 > Nav2 20, timeout 0,2 s > chu ky phat 0,1 s',
      _av is not None and _av.group(1) == '/cmd_vel_avoid' and int(_av.group(3)) == 60 and float(_av.group(2)) == 0.2
      and "declare_parameter('cmd_topic', '/cmd_vel_avoid')" in _src and 'self.create_timer(0.1, self._tick)' in _src, str(_av.groups() if _av else None))

# --- viec 109c-c3: pha XOAY ve huong thoang nhat trong +-45 do quanh huong toi P ---
_LECH_ODOM = 0.7          # /odom (con quay) khac khung ban do mot hang so: pha XOAY chi duoc dung goc da xoay, khong dung yaw tuyet doi


def xoay_chay(world, pose, p, giay=15.0, doi_the_gioi=None, tf_ok=True, odom_ok=True, truoc_ra=True):
    """Bo nao + xe ly tuong (0,1 s/vong). goc_p lay tu vi tri ban do that (nhu tf); yaw la /odom = yaw that + hang so. Tra dict."""
    b = L.NguoiLai()
    if truoc_ra:
        b.ra_bat_dau(L.scan_to_xy(make_scan(world, pose)), 0.0, True)
    else:
        b.xoay_cho = True
    r = dict(ten=[], w=[], v=[], ra='', xoay='', cham=False)
    t = 0.0
    while t < giay:
        t += 0.1
        if doi_the_gioi and t >= doi_the_gioi[0]:
            world = doi_the_gioi[1](pose) if callable(doi_the_gioi[1]) else doi_the_gioi[1]
        pts = L.scan_to_xy(make_scan(world, pose))
        v, w, ten = b.step(pts, t, True, None, None, pose[2] + _LECH_ODOM if odom_ok else None, L.goc_toi_diem(pose, p) if tf_ok else None)
        r['ten'].append(ten)
        r['ra'] = r['ra'] or b.ra_xong
        r['xoay'] = r['xoay'] or b.xoay_xong
        b.ra_xong = b.xoay_xong = ''
        if v is not None:
            r['v'].append(v)
            r['w'].append(w)
            pose = (pose[0] + v * math.cos(pose[2]) * 0.1, pose[1] + v * math.sin(pose[2]) * 0.1, pose[2] + w * 0.1)
            r['cham'] = r['cham'] or collides(world, pose)
        if r['xoay'] and b.state == 'BINH_THUONG' and not b.xoay_cho:
            break
    r['pose'], r['state'] = pose, b.state
    return r


def lech_do(a, b_):
    return abs(math.degrees(L.wrap(a - b_)))


_MO = [(0.20, 0.40, -0.10, 0.10)]                              # troi trong, chi co vat truoc mui 5,5 cm (K4)
_P40 = (math.cos(math.radians(40)), math.sin(math.radians(40)))
_x1 = xoay_chay(_MO, (0.0, 0.0, 0.0), _P40)
_phi1 = L.chon_huong_quanh(L.scan_to_xy(make_scan(_MO, (0.0, 0.0, 0.0))), math.radians(40))
check('c3 xoay dung huong P (+40 do): quay trai, toi huong trong 20 do, khong cham', _x1['xoay'].startswith('XOAY XONG DEN_HUONG') and all(w > 0 for w in _x1['w'])
      and lech_do(_x1['pose'][2], _phi1) <= 26 and not _x1['cham'], '%s | huong chon %.0f do, xe %.0f do' % (_x1['xoay'], math.degrees(_phi1), math.degrees(_x1['pose'][2])))
_xm = xoay_chay(_MO, (0.0, 0.0, 0.0), (math.cos(math.radians(-100)), math.sin(math.radians(-100))))
check('c3 xoay dung huong P (-100 do): quay phai, cham lai khi gan', _xm['xoay'].startswith('XOAY XONG DEN_HUONG') and all(w < 0 for w in _xm['w']) and min(abs(w) for w in _xm['w']) == 0.6
      and max(abs(w) for w in _xm['w']) == 1.0 and 55 <= -math.degrees(_xm['pose'][2]) <= 150, '%s | xe %.0f do' % (_xm['xoay'], math.degrees(_xm['pose'][2])))
_TUONG = _MO + [(0.0, 0.40, 0.30, 0.35)]                      # tuong cach tam 30 cm, chan huong +90 do (P sau tuong) nhung de ngo ben trai
_pts_t = L.scan_to_xy(make_scan(_TUONG, (0.0, 0.0, 0.0)))
_phi2 = L.chon_huong_quanh(_pts_t, math.pi / 2)
_tot = max(L.thoang(_pts_t, math.radians(a), cap=0.5) for a in range(45, 136))
check('c3 P sau tuong: chon huong thoang nhat trong +-45 do (khong phai huong P)', _phi2 is not None and 45 - 1e-9 <= math.degrees(_phi2) <= 135 + 1e-9
      and L.thoang(_pts_t, _phi2, cap=0.5) >= _tot - 1e-9 and abs(math.degrees(_phi2) - 90) >= 5 and L.thoang(_pts_t, math.pi / 2, cap=0.5) < L.thoang(_pts_t, _phi2, cap=0.5),
      'chon %.0f do, thoang %.2f m (huong P: %.2f m)' % (math.degrees(_phi2), L.thoang(_pts_t, _phi2, cap=0.5), L.thoang(_pts_t, math.pi / 2, cap=0.5)))
_x2 = xoay_chay(_TUONG, (0.0, 0.0, 0.0), (0.0, 1.0))
check('c3 P sau tuong (xe xoay that): toi huong thoang da chon, truoc mui >= 12 cm, khong cham', _x2['xoay'].startswith('XOAY XONG DEN_HUONG') and lech_do(_x2['pose'][2], _phi2) <= 26 and not _x2['cham']
      and L.thoang(L.scan_to_xy(make_scan(_TUONG, _x2['pose'])), 0.0) >= 0.12, '%s | xe %.0f do' % (_x2['xoay'], math.degrees(_x2['pose'][2])))
_x3 = xoay_chay(_MO, (0.0, 0.0, 0.0), (0.0, 1.0), doi_the_gioi=(0.8, _MO + [(-0.05, 0.05, 0.14, 0.17)]))
check('c3 co diem vao R_XOAY giua chung: dung xoay, ngung phat', _x3['xoay'].startswith('XOAY XONG DIEM_TRONG_R_XOAY') and _x3['ten'][-1] == 'XOAY_XONG' and len(_x3['w']) <= 9 and not _x3['cham'], _x3['xoay'])
_x4 = xoay_chay(_MO, (0.0, 0.0, 0.0), (0.0, 1.0), tf_ok=False)
check('c3 tf cu hoac P "-": khong xoay, co log', not _x4['w'] and _x4['xoay'].startswith('XOAY KHONG: khong co huong toi P'), _x4['xoay'])
_x5 = xoay_chay(_MO, (0.0, 0.0, 0.0), (0.0, 1.0), odom_ok=False)
check('c3 khong co /odom moi: khong xoay, co log', not _x5['w'] and _x5['xoay'].startswith('XOAY KHONG: khong co /odom'), _x5['xoay'])
_x6 = xoay_chay(hanh_lang([(0.17, 0.40, -0.10, 0.10), (-0.40, -0.125, -0.10, 0.10)]), (0.0, 0.0, 0.0), (0.0, 1.0))
check('c3 hai dau chan (RA khong di): khong xoay vi chua du cho', not _x6['w'] and not _x6['v'] and _x6['state'] == 'BINH_THUONG', str(_x6['xoay']))
# phia truoc < 12 cm khi dang xoay (da tung >= 12 cm)
_b8 = L.NguoiLai()
_b8.xoay_cho = True
_pose8 = (0.0, 0.0, 0.0)
_open = []
for _i in range(1, 6):
    _v8, _w8, _t8 = _b8.step(L.scan_to_xy(make_scan(_open, _pose8)), 0.1 * _i, True, None, None, _pose8[2], math.pi / 2)
    _pose8 = (0.0, 0.0, _pose8[2] + _w8 * 0.1)
_cx, _cy = (0.145 + 0.09 + 0.03) * math.cos(_pose8[2]), (0.145 + 0.09 + 0.03) * math.sin(_pose8[2])
_v8, _w8, _t8 = _b8.step(L.scan_to_xy(make_scan([(_cx - 0.03, _cx + 0.03, _cy - 0.03, _cy + 0.03)], _pose8)), 0.6, True, None, None, _pose8[2], math.pi / 2)
check('c3 toi huong ma phia truoc < 12 cm: dung', _t8 == 'XOAY_XONG' and _b8.xoay_xong.startswith('XOAY XONG TRUOC_DUOI_12CM') and _v8 is None, _b8.xoay_xong)
_b9 = L.NguoiLai()
_b9.xoay_cho = True
_b9.step(L.scan_to_xy(make_scan([], (0, 0, 0))), 0.0, True, None, None, 0.0, math.pi / 2)
_b9.step(L.scan_to_xy(make_scan([], (0, 0, 0))), 10.6, True, None, None, 0.0, math.pi / 2)
check('c3 qua 10 s: dung', _b9.xoay_xong.startswith('XOAY XONG QUA_10S'), _b9.xoay_xong)
_ring = [(-0.30, 0.30, 0.17, 0.30), (-0.30, 0.30, -0.30, -0.17), (0.17, 0.30, -0.30, 0.30), (-0.30, -0.17, -0.30, 0.30)]
check('c3 chon_huong_quanh: None khi moi huong deu < 12 cm', L.chon_huong_quanh(L.scan_to_xy(make_scan(_ring, (0, 0, 0))), 0.0) is None)
_odom_py = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'dev_ws', 'src', 'my_bot', 'scripts', 'odom_tu_lenh.py')).read()
check("luat G: lai_xe_lidar nghe '/odom' (Odometry) do odom_tu_lenh.py phat; yaw tuong doi, khung bat ky", "create_publisher(Odometry, '/odom'" in _odom_py and "create_subscription(Odometry, '/odom'" in _src)

# --- viec 109b: "co dich" gop hai kenh trang thai ---
_tick109 = _src[_src.index('def _tick'):_src.index('def main')]
check('109b co dich = /navigate_to_pose HOAC /follow_path (tat duoc bang nghe_follow_path)',
      "self.goal_kenh['nav'] or (self.goal_kenh['fp'] and self.get_parameter('nghe_follow_path').value)" in _tick109
      and "'/follow_path/_action/status', self._on_status_fp" in _src and "self.goal_kenh['fp'] = any(s.status in (1, 2)" in _src
      and _tick109.index('self.goal_active =') < _tick109.index('self._doc_thoat('))

# --- 02/11 17:33: tu THOAT toi da 3 lan moi dich, roi dung yen cho sep; ~/sep_thoat moi thi nhuong ---
_sat_trai = L.scan_to_xy(make_scan([(-0.30, 0.40, 0.13, 0.30), (0.60, 0.90, -0.30, 0.30)], (0, 0, 0)))   # tuong trai cach hong 5 cm
_bt = L.NguoiLai()
_ten_ds, _t = [], 0.0
for _k in range(2000):                                               # 200 s xe dung yen (THOAT_CUNG khong lam xe dich trong ca nay)
    _t += 0.1
    _ten_ds.append(_bt.step(_sat_trai, _t, True, None, (0.0, 0.0)))
_lan = sum(1 for k in range(1, len(_ten_ds)) if _ten_ds[k][2].startswith('THOAT') and not _ten_ds[k - 1][2].startswith('THOAT'))
check('17:33 tu THOAT toi da 3 lan moi dich roi CHO_SEP (dung yen v = w = 0)', _lan == 3 and _ten_ds[-1] == (0.0, 0.0, 'CHO_SEP') and _bt.state == 'CHO_SEP'
      and sum(1 for x in _ten_ds if x[2] == 'BO_CUOC_HET_GIO') == 3 and _bt.ly_do.startswith('HET_3_LUOT_TU_THOAT'), 'so lan %d, cuoi %s' % (_lan, _ten_ds[-1]))
_bt.step(_sat_trai, _t + 0.1, False, None, (0.0, 0.0))                 # het dich (chuyen.sh / gui_duong gui dich moi)
_moi = [_bt.step(_sat_trai, _t + 0.2 + 0.1 * k, True, None, (0.0, 0.0))[2] for k in range(40)]
check('17:33 dich moi: dem lai, duoc tu THOAT lai', _bt.n_tu_thoat == 1 and any(n.startswith('THOAT') for n in _moi) and 'CHO_SEP' not in _moi, str(_moi[-3:]))
_bn = L.NguoiLai()
_bn.state, _bn.n_tu_thoat = 'CHO_SEP', 3
_ok, _cau = _bn.ra_bat_dau(_sat_trai, 0.0, True)
check('17:33 dang CHO_SEP ma co ~/sep_thoat moi: NHUONG cho pha RA', _ok and _cau.startswith('NHUONG CHO_SEP: RA BAT DAU') and _bn.state == 'RA', _cau)
_bk = L.NguoiLai()
_bk.state = 'CHO_SEP'
check('17:33 CHO_SEP ma khong con dich: tro ve binh thuong, khong phat', _bk.step(_sat_trai, 1.0, False)[:2] == (None, None) and _bk.state == 'BINH_THUONG')

# --- 18:09: sep ra_thoat => khong tu THOAT_CUNG, CHO_SEP kem ly do; XOAY theo huong doan ---
_bs = L.NguoiLai(sep_ra_thoat=True)
_ten18 = [_bs.step(_sat_trai, 0.1 * (k + 1), True, None, (0.0, 0.0)) for k in range(60)]
check('18:09 sep_ra_thoat: ket hinh hoc => CHO_SEP (ly do KET_HINH_HOC), khong THOAT_CUNG, khong ton luot tu thoat',
      _bs.state == 'CHO_SEP' and _bs.ly_do == 'KET_HINH_HOC' and not any(t[2].startswith('THOAT') for t in _ten18) and _bs.n_tu_thoat == 0
      and _ten18[-1] == (0.0, 0.0, 'CHO_SEP'))
check('18:09 doc ~/sep_thoat 7 cot (huong doan), 6 cot nhu cu', L.doc_sep_thoat('100.00 THOAT 1.26 0.06 1 16 0.0982', 100.5) == dict(luot=1, ma=16, p=(1.26, 0.06), yaw=0.0982)
      and L.doc_sep_thoat('100.00 THOAT 1.26 0.06 1 16', 100.5) == dict(luot=1, ma=16, p=(1.26, 0.06))
      and L.doc_sep_thoat('100.00 THOAT 1.26 0.06 1 16 x', 100.5) is None)
check('18:09 XOAY ve huong doan (co yaw) hoac ve P (khong yaw)', abs(L.goc_xoay_ve((0, 0, -math.pi / 2), (1, 0), 0.0) - math.pi / 2) < 1e-9
      and abs(L.goc_xoay_ve((0, 0, 0), (0, 1), None) - math.pi / 2) < 1e-9 and L.goc_xoay_ve(None, (1, 0), 0.0) is None)
_tick18 = _src[_src.index('def _tick'):_src.index('def main')]
check("18:09 cau doi trang thai in ly do ca khi CHO_SEP (sep bat qua /rosout)", "self.brain.state in ('THOAT', 'CHO_SEP')" in _tick18)


# --- 04/11 lop 2: vung kich hoat va lui thang (khong can dich) ---
def ra2_chay(world, pose, giay=8.0, chi_nhin=False, dt=0.1, tien=False, **kw):
    """Giao RA2 luc t = 0 (khong co dich), chay toi RA_XONG. Tra (ten cac buoc, cau ra_xong, tu the cuoi, cham?).
    tien=True: sep giao RA_TIEN (06/11 viec 120)."""
    b = L.NguoiLai(**kw)
    ok, _ = b.ra2_bat_dau(L.scan_to_xy(make_scan(world, pose)), 0.0, chi_nhin, tien=tien)
    t, ten = 0.0, []
    while t < giay:
        v, w, n = b.step(L.scan_to_xy(make_scan(world, pose)), t, False)
        ten.append(n)
        if v is not None:
            x, y, th = pose
            pose = (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt)
        if collides(world, pose):
            return ten, b.ra_xong, pose, True
        if n == 'RA_XONG':
            return ten, b.ra_xong, pose, False
        t += dt
    return ten, b.ra_xong, pose, False


_ong8 = [(0.225, 0.405, -0.09, 0.09)]                                  # vat vuong 18 cm, mat cach mui 8 cm (tam xe o 0)
_pv = L.scan_to_xy(make_scan(hanh_lang(_ong8), (0, 0, 0)))
check('lop2 vung_lui: vat 8 cm truoc mui => co; hanh lang trong => khong', L.vung_lui(_pv) and not L.vung_lui(L.scan_to_xy(make_scan(hanh_lang(), (0, 0, 0)))))
check('lop2 cach_truoc: than cach mat vat 0,08 m (05/11 viec 119: do tu than; cu 0,225 m tu tam)', abs(L.cach_truoc(_pv) - 0.08) < 0.02, '%.3f' % L.cach_truoc(_pv))
_t2, _x2, _p2, _c2 = ra2_chay(hanh_lang(_ong8), (0, 0, 0))
check('lop2 lui thang toi than cach vat >= 0,15 m (khong can dich), khong cham, khong xoay (cu: tam >= 0,30 m)',
      _x2.startswith('RA XONG CACH_15CM') and not _c2 and abs(_p2[1]) < 1e-9 and abs(_p2[2]) < 1e-9 and 0.06 <= -_p2[0] <= 0.12
      and set(_t2[:-1]) == {'RA_LUI'}, '%s %s' % (_x2, _p2))
_t3, _x3, _p3, _c3 = ra2_chay(hanh_lang(_ong8 + [(-0.20, -0.14, -0.10, 0.10)]), (0, 0, 0))    # sau con 3,5 cm
check('lop2 sau < 5 cm: xong ngay, khong lui', _x3 == 'RA XONG SAU_5CM 0 cm' and _p3 == (0, 0, 0) and not _c3, _x3)
_t4, _x4, _p4, _c4 = ra2_chay(hanh_lang([(0.20, 0.40, -0.09, 0.09)]), (0, 0, 0), ra2_cach_than=0.60)   # muc tieu xa: dung o 0,30 m
check('lop2 lui toi da 0,30 m', _x4.startswith('RA XONG DA_LUI_30CM') and 0.29 <= -_p4[0] <= 0.32 and not _c4, '%s %s' % (_x4, _p4))
_t5, _x5, _p5, _ = ra2_chay(hanh_lang(_ong8), (0, 0, 0), ra2_cach_than=0.60, ra2_di_max=9.0, v_esc=0.02)
check('lop2 qua 6 s thi dung', _x5.startswith('RA XONG QUA_6S'), _x5)
_t6, _x6, _p6, _ = ra2_chay(hanh_lang(_ong8), (0, 0, 0), chi_nhin=True)
check('lop2 chi nhin (phat_lenh tat): xong ngay 0 cm', _x6 == 'RA XONG CHI_NHIN 0 cm' and _p6 == (0, 0, 0) and _t6 == ['RA_XONG'], _x6)
_b7 = L.NguoiLai()
_b7.state = 'THOAT'
check('lop2 dang THOAT thi bo qua RA2', _b7.ra2_bat_dau(_pv, 0.0) == (False, 'RA2 BO QUA: dang THOAT'))
check('lop2 doc_sep_ra2: RA_LUI / KHONG_CAN / cu / sai', L.doc_sep_ra2('100.00 RA_LUI 55 900', 100.5) == dict(chu='RA_LUI', ma_luot=55, ma=900)
      and L.doc_sep_ra2('100.00 KHONG_CAN 55 vung_trong', 100.5) == dict(chu='KHONG_CAN', ma_luot=55, ly='vung_trong')
      and L.doc_sep_ra2('100.00 RA_LUI 55 900', 104.0) is None and L.doc_sep_ra2('100.00 THOAT 1.26 0.06 1 16', 100.5) is None
      and L.doc_sep_thoat('100.00 RA_LUI 55 900', 100.5) is None)
_b8 = L.NguoiLai()
_b8.ra2_bat_dau(_pv, 0.0)
_r8 = [_b8.step(_pv, 0.1 * i, False) for i in range(1, 6)]                      # dang lui 0,5 s
_b8.ra2_huy()
_h8 = _b8.step(_pv, 0.6, False)
_s8 = [_b8.step(_pv, 0.6 + 0.1 * i, False)[0] for i in range(1, 40)]            # 4 s sau, cung vat sat mui
check('lop2 HUY (gui_duong bi ngat): buoc ke tiep gui 0 va RA_XONG, sau do khong tu lui lai',
      all(r[2] == 'RA_LUI' and r[0] < 0 for r in _r8) and _h8 == (0.0, 0.0, 'RA_XONG') and _b8.ra_xong.startswith('RA XONG HUY')
      and _b8.state == 'BINH_THUONG' and all(v is None or v >= 0 for v in _s8), '%s %s' % (_h8, _b8.ra_xong))
_b9 = L.NguoiLai()
_b9.ra2_huy()
check('lop2 ra2_huy ngoai RA2 khong lam gi; luot_huy: HUY / mat tep => co, SAP_GUI => khong',
      _b9.state == 'BINH_THUONG' and _b9.ra2 is None and L.luot_huy('12.30 HUY') and L.luot_huy(None)
      and not L.luot_huy('12.30 SAP_GUI di_B 1 1 123') and not L.luot_huy(''))
_khoi2 = _src[_src.index('def _doc_ra2'):_src.index('def _on_odom')]
check('lop2 _doc_ra2 khong phat toc do', not any(k in _khoi2 for k in ('publish', 'Twist', 'self.pub', 'phat_lenh')))


# --- 05/11 viec 119 (loi B, ket K2 19:33): vung lop 2 theo hinh than, vat o goc truoc ---
def vung_cu(pts):
    """Vung lop 2 truoc 05/11: >= 2 diem trong chu nhat X_MUI <= x <= 0,25, |y| <= 0,10."""
    return sum(1 for x, y in pts if L.X_MUI <= x <= 0.25 and abs(y) <= L.LOP2_W_CU) >= 2


_g = 0.055                                                                   # vat cach goc than 5,5 cm (Thien do 19:33)
_goc8 = []
for _s in (1, -1):
    for _ten, (_x0, _y0) in (('ngang hong', (L.X_MUI - 0.01, L.Y_HONG + _g)), ('cheo 45', (L.X_MUI + _g / math.sqrt(2), L.Y_HONG + _g / math.sqrt(2))),
                             ('truoc mui', (L.X_MUI + _g, L.Y_HONG - 0.01)), ('hong sau goc 4 cm', (L.X_MUI - 0.05, L.Y_HONG + _g))):
        _y1 = _y0 + 0.02
        _goc8.append(('%s %s' % ('trai' if _s > 0 else 'phai', _ten), (_x0, _x0 + 0.02, min(_s * _y0, _s * _y1), max(_s * _y0, _s * _y1))))
_kq8 = []
for _ten, _hop in _goc8:
    _w = hanh_lang([_hop])
    _pp = L.scan_to_xy(make_scan(_w, (0, 0, 0)))
    _t, _x, _p, _c = ra2_chay(_w, (0, 0, 0))
    _kq8.append((_ten, L.vung_lui(_pp), vung_cu(_pp), round(min(L.khoang_cach_mep(*q) for q in _pp), 3), _x, round(-_p[0], 3), _c))
check('119 vat cach goc than 5,5 cm (8 ca): vung moi kich, lui that (CACH_15CM, >= 5 cm, khong cham, khong xoay)',
      all(k[1] and k[4].startswith('RA XONG CACH_15CM') and k[5] >= 0.05 and not k[6] for k in _kq8), str(_kq8))
check('119 vung cu bo sot ca ngang hong / cheo / hong sau goc (6/8), chi bat truoc mui', [k[0] for k in _kq8 if k[2]] == ['trai truoc mui', 'phai truoc mui'],
      str([(k[0], k[2]) for k in _kq8]))
_ong21 = hanh_lang([(0.21, 0.228, -0.009, 0.009)])                          # ong O18 thang truoc mui, mat cach tam 0,21 m (6,5 cm truoc mui)
_po = L.scan_to_xy(make_scan(_ong21, (0, 0, 0)))
_to, _xo, _pq, _co = ra2_chay(_ong21, (0, 0, 0))
check('119 ong thang truoc mui 0,21 m: vung moi va vung cu deu kich, lui toi than cach >= 0,15 m', L.vung_lui(_po) and vung_cu(_po)
      and _xo.startswith('RA XONG CACH_15CM') and not _co, '%s %.3f' % (_xo, -_pq[0]))
_khe = {}
for _rong in (0.52, 0.56):
    for _e in (0.0, 0.05, 0.07, 0.09):                                       # xe lech _e ve ben trai trong khe
        _w = [(-1.0, 2.0, _rong / 2 - _e, 0.60), (-1.0, 2.0, -0.60, -_rong / 2 - _e)]
        _pp = L.scan_to_xy(make_scan(_w, (0, 0, 0)))
        _khe[(_rong, _e)] = (L.vung_lui(_pp), vung_cu(_pp))
print('      khe (rong, lech): (vung moi, vung cu) = %s' % _khe)
check('119 khe 52 cm: lech 0/5/7 cm khong kich, 9 cm kich (tuong cach hong 9 cm <= 10,5); khe 56 cm: 0..9 cm khong kich; vung cu khong kich',
      [_khe[(0.52, e)][0] for e in (0.0, 0.05, 0.07, 0.09)] == [False, False, False, True]
      and not any(_khe[(0.56, e)][0] for e in (0.0, 0.05, 0.07, 0.09)) and not any(v[1] for v in _khe.values()), str(_khe))
_b10 = L.NguoiLai()
_b10.ra2_bat_dau([], 0.0)
_b10.step([], 0.1, False)
check('119 RA2 ma phia truoc khong co diem nao: ly do TRUOC_TRONG ngay buoc dau (khong ghi CACH_15CM)', _b10.ra_xong.startswith('RA XONG TRUOC_TRONG'), str(_b10.ra_xong))
check('119 hang ten co gia tri cu canh ben, DEFAULT dung hang', L.DEFAULT['lop2_d_than'] == L.LOP2_D_THAN == 0.105
      and abs(L.DEFAULT['lop2_x_tu'] - 0.095) < 1e-12 and L.DEFAULT['ra2_cach_than'] == L.RA2_CACH_THAN == 0.15
      and not any(k in L.DEFAULT for k in ('lop2_x_xa', 'lop2_w', 'ra2_cach')))

# --- 06/11 viec 120: lop 2 nhanh RA_TIEN (vat sau than, ket K1 21:04-21:06) ---
def doc_scan_tep(ten):
    """LaserScan tu tep `ros2 topic echo --full-length /scan` (du_lieu_0511_toi) -> diem khung than (scan_to_xy)."""
    d = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'du_lieu_0511_toi', ten)).read().split('\n')
    g = lambda k: float([l for l in d if l.startswith(k)][0].split()[1])
    r = [float(l[2:]) for l in d[d.index('ranges:') + 1:d.index('intensities:')] if l.startswith('- ')]
    sc = SimpleNamespace(ranges=r, angle_min=g('angle_min'), angle_increment=g('angle_increment'), range_min=g('range_min'), range_max=g('range_max'))
    return L.scan_to_xy(sc)


_k1 = doc_scan_tep('scan_ket_k1_2106.txt')
check('120 scan K1 21:06: vat sau than (vung_sau), KHONG o vung truoc (vung_lui); than-sau ~6,3 cm, mui-truoc ~24,4 cm',
      L.vung_sau(_k1) and not L.vung_lui(_k1) and abs(L.cach_sau(_k1) - 0.063) < 0.005 and abs(L.mui_truoc(_k1) - 0.244) < 0.005,
      'sau %.3f truoc %.3f' % (L.cach_sau(_k1), L.mui_truoc(_k1)))


def tien_tren_scan(pts, dt=0.05, giay=5.0, **kw):
    """RA_TIEN tren scan that: xe tien d thi moi diem lui -d theo x (khong xoay). Buoc dau cung `now` voi ra2_bat_dau (nhu nut:
    _doc_ra2 roi step trong cung mot nhip). Tra (cau ra_xong, quang tien, cac lenh)."""
    b = L.NguoiLai(**kw)
    b.ra2_bat_dau(pts, 0.0, tien=True)
    d, t, lenh = 0.0, 0.0, []
    while t < giay:
        v, w, n = b.step([(x - d, y) for x, y in pts], t, False)
        lenh.append((v, w, n))
        if n == 'RA_XONG':
            break
        d += v * dt
        t += dt
    return b.ra_xong, d, lenh


_x11, _d11, _l11 = tien_tren_scan(_k1)
check('120 RA_TIEN tren scan K1: tien thang 0,10 m/s, dung TRUOC_15CM sau ~9,4 cm (mui con <= 0,15 m)',
      _x11.startswith('RA XONG TRUOC_15CM') and 0.09 <= _d11 <= 0.10 and all(v == 0.10 and w == 0.0 and n == 'RA_TIEN' for v, w, n in _l11[:-1]),
      '%s, %.3f m' % (_x11, _d11))
_x12, _d12, _ = tien_tren_scan(_k1, dt=0.1)
check('120 RA_TIEN tren scan K1, nhip 0,1 s nhu tren xe: dung trong 9,4-10 cm, bao 10 cm', _x12 == 'RA XONG TRUOC_15CM 10 cm' and 0.094 <= _d12 <= 0.101,
      '%s, %.3f m' % (_x12, _d12))
_vs = [(-0.15, -0.12, -0.09, 0.09)]                                     # vat sau duoi 1,5 cm, phia truoc hanh lang trong
_t13, _x13, _p13, _c13 = ra2_chay(hanh_lang(_vs), (0, 0, 0), tien=True)
check('120 RA_TIEN phia truoc trong: dung DA_TIEN_10CM, khong cham, khong xoay', _x13.startswith('RA XONG DA_TIEN_10CM')
      and 0.09 <= _p13[0] <= 0.115 and abs(_p13[1]) < 1e-9 and abs(_p13[2]) < 1e-9 and not _c13 and 'RA_TIEN' in _t13, '%s %s' % (_x13, _p13))
_t14, _x14, _, _ = ra2_chay(hanh_lang(_vs), (0, 0, 0), tien=True, v_esc=0.02)
check('120 RA_TIEN qua 3 s thi dung', _x14.startswith('RA XONG QUA_3S'), _x14)
_t15, _x15, _p15, _ = ra2_chay(hanh_lang(_vs), (0, 0, 0), tien=True, chi_nhin=True)
check('120 RA_TIEN chi nhin: xong ngay 0 cm', _x15 == 'RA XONG CHI_NHIN 0 cm' and _p15 == (0, 0, 0), _x15)
_b16 = L.NguoiLai()
_b16.ra2_bat_dau(_k1, 0.0, tien=True)
_r16 = [_b16.step(_k1, 0.1 * i, False) for i in range(1, 4)]
_b16.ra2_huy()
_h16 = _b16.step(_k1, 0.4, False)
check('120 RA_TIEN HUY: buoc ke tiep gui 0 va RA_XONG HUY', all(r[2] == 'RA_TIEN' and r[0] > 0 for r in _r16) and _h16 == (0.0, 0.0, 'RA_XONG')
      and _b16.ra_xong.startswith('RA XONG HUY') and _b16.state == 'BINH_THUONG', '%s %s' % (_h16, _b16.ra_xong))
_b17 = L.NguoiLai()
_b17.ra2_bat_dau([(0.20, 0.0), (0.21, 0.01)], 0.0, tien=True)               # mui con 5,5 cm: khong tien
check('120 RA_TIEN ma mui da <= 0,15 m: xong ngay TRUOC_15CM 0 cm', _b17.step([(0.20, 0.0), (0.21, 0.01)], 0.0, False)[0] is None
      and _b17.ra_xong == 'RA XONG TRUOC_15CM 0 cm', str(_b17.ra_xong))
check('120 doc_sep_ra2 RA_TIEN; doc_sep_thoat bo qua', L.doc_sep_ra2('100.00 RA_TIEN 55 901', 100.5) == dict(chu='RA_TIEN', ma_luot=55, ma=901)
      and L.doc_sep_thoat('100.00 RA_TIEN 55 901', 100.5) is None and L.doc_sep_ra2('100.00 RA_TIEN 55 x', 100.5) is None)
check('120 hang so va DEFAULT RA_TIEN', L.LOP2_X_SAU == -0.05 and L.DEFAULT['ra_tien_di_max'] == 0.10 and L.DEFAULT['ra_tien_truoc'] == 0.15
      and L.DEFAULT['ra_tien_w'] == 0.10 and L.DEFAULT['ra_tien_t_max'] == 3.0 and L.DEFAULT['v_esc'] == 0.10)
check('120 vung_sau: hanh lang 52 cm trong, xe giua => khong; vat sau 1,5 cm => co', not L.vung_sau(L.scan_to_xy(make_scan(hanh_lang(), (0, 0, 0))))
      and L.vung_sau(L.scan_to_xy(make_scan(hanh_lang(_vs), (0, 0, 0)))))

fails = [n for n, ok in results if not ok]
print('\nTONG: %d/%d dat' % (len(results) - len(fails), len(results)))
sys.exit(1 if fails else 0)
