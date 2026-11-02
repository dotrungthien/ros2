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
check('c2 khong co dich / dang tu thoat: khong bat dau', _b6.ra_bat_dau(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.0, False) == (False, 'RA BO QUA: khong co dich')
      and (setattr(_b6, 'state', 'THOAT') or _b6.ra_bat_dau(L.scan_to_xy(make_scan(_hep, (0, 0, 0))), 0.0, True)) == (False, 'RA BO QUA: dang THOAT'))
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

fails = [n for n, ok in results if not ok]
print('\nTONG: %d/%d dat' % (len(results) - len(fails), len(results)))
sys.exit(1 if fails else 0)
