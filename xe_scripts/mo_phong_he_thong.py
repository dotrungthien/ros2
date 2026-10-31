#!/usr/bin/env python3
"""Mo phong CA HE THONG (logic phoi hop), khong phai Nav2 that.

Gom: xuong (bo tri gan dung tu ban do, +-5-10 cm), xe vi sai ly tuong, LiDAR 410 tia, nguoi lai that (lai_xe_lidar.NguoiLai),
"Nav2 do choi" (quay tai cho, di thang ve diem, dung khi truoc mat khong thoang, bo cuoc sau 30 s) va logic doi tuyen cua chuyen.sh (TRY=1).
Nav2 do choi KHONG co lap duong ne vat, KHONG co RPP that, KHONG co trượt banh: ket qua chi noi ve CACH CAC LOP PHOI HOP,
khong noi xe that chay on dinh hay khong.
Chay: python3 -I mo_phong_he_thong.py
"""
import math
import random
import sys
from types import SimpleNamespace

import lai_xe_lidar as L

BEAMS, RANGE_MIN, RANGE_MAX, DT = 410, 0.10, 8.0, 0.1
WAIT_BT = 30.0         # giay: cay hanh vi cho roi bao hong (8 lan thu)
GAP_GIUA_DICH = 3.0    # giay: chuyen.sh kiem vi tri roi gui diem ke tiep (goal khong chay)

# ---- xuong (map, goc A = (0,0), x doc xuong, y sang trai) ----
TUONG_NGOAI = [(-0.55, -0.50, -0.43, 1.17), (1.885, 1.935, -0.43, 1.17), (-0.55, 1.935, -0.43, -0.38), (-0.55, 1.935, 1.12, 1.17)]
H0 = (0.44, 1.04, 0.37, 0.52)
K4 = (0.39, 0.94, -0.33, -0.18)
K2 = (0.88, 1.28, 0.97, 1.12)
XUONG = TUONG_NGOAI + [H0, K4, K2]

FOAM = (0.70, 0.75, -0.18, 0.37)                     # tam chan kin hanh lang duoi
NGUOI = (0.70, 0.88, 0.00, 0.18)                     # ong O18 giua hanh lang duoi

TUYEN = {
    1: [(1.45, 0.13, math.pi), (0.0, 0.0, 0.0)],
    2: [(1.45, 0.76, math.pi), (0.30, 0.72, math.pi), (0.0, 0.0, 0.0)],
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


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
    rs = []
    for i in range(BEAMS):
        a = th + (-math.pi + i * inc)
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for r in world:
            t = ray_rect(lx, ly, dx, dy, r)
            if t is not None and 0 < t < best:
                best = t
        rs.append(best if RANGE_MIN <= best <= RANGE_MAX else math.inf)
    return SimpleNamespace(ranges=rs, angle_min=-math.pi, angle_increment=inc, range_min=RANGE_MIN, range_max=RANGE_MAX)


def cham(world, pose):
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


def grid_ban_do(world, res=0.05, x0=-0.6, y0=-0.5, x1=2.0, y1=1.25):
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    data = []
    for j in range(h):
        for i in range(w):
            cx, cy = x0 + (i + 0.5) * res, y0 + (j + 0.5) * res
            data.append(100 if any(a < cx < b and c < cy < d for a, b, c, d in world) else 0)
    return dict(res=res, w=w, h=h, ox=x0, oy=y0, data=data)


GRID = grid_ban_do(XUONG)       # ban do chi co xuong co dinh (khong co tam chan, khong co nguoi)


def se_cham(pts, v, w, T=1.0, dt=0.1, le=0.01):
    """Giong 'collision ahead' cua RPP: chay thu lenh (v, w) trong T giay, xem than xe co dinh diem LiDAR nao khong (toa do than xe luc dau)."""
    if abs(v) < 1e-6:
        return False
    x = y = th = 0.0
    for _ in range(int(T / dt)):
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        c, s = math.cos(th), math.sin(th)
        for px, py in pts:
            dx, dy = px - x, py - y
            bx, by = dx * c + dy * s, -dx * s + dy * c
            if L.X_DUOI - le < bx < L.X_MUI + le and -L.Y_HONG - le < by < L.Y_HONG + le:
                return True
    return False


class Nav2DoChoi:
    """Quay tai cho khi lech huong > 45 do, di thang ve diem, dung khi truoc mat khong thoang < 0,12 m, bo cuoc sau WAIT_BT giay bi chan."""

    def __init__(self, goal, rng, sai_vi_tri=0.03, sai_goc_deg=2.0, pose0=None):
        self.goal = goal
        self.via = None
        if pose0 is not None:   # dang trong hanh lang duoi (K4-H0) ma dich o ngoai: ra cua phia B truoc (thay cho bo lap duong that)
            trong = 0.39 < pose0[0] < 1.06 and -0.18 < pose0[1] < 0.37
            ngoai = goal[0] > 1.10 or goal[1] > 0.40
            if trong and ngoai:
                self.via = (1.20, 0.09, 0.0)
        self.off = (rng.gauss(0, sai_vi_tri), rng.gauss(0, sai_vi_tri), math.radians(rng.gauss(0, sai_goc_deg)))
        self.chan = 0.0

    def cmd(self, pose, pts):
        gx, gy, gyaw = self.goal
        qua_via = self.via is not None
        if qua_via:
            if math.hypot(self.via[0] - pose[0], self.via[1] - pose[1]) < 0.15:
                self.via = None
                qua_via = False
            else:
                gx, gy, gyaw = self.via
        ex, ey, eth = pose[0] + self.off[0], pose[1] + self.off[1], pose[2] + self.off[2]
        dist = math.hypot(gx - ex, gy - ey)
        if dist > 0.15:
            err = wrap(math.atan2(gy - ey, gx - ex) - eth)
            if abs(err) > 0.785:
                self.chan = 0.0
                return 0.0, math.copysign(0.544, err), None
            vv, ww = 0.12, max(-0.6, min(0.6, 1.5 * err))
            if L.thoang(pts, 0.0) < 0.12 or se_cham(pts, vv, ww):
                self.chan += DT
                if self.chan >= WAIT_BT:
                    return 0.0, 0.0, 'ABORTED'
                return 0.0, 0.0, None
            self.chan = 0.0
            return vv, ww, None
        if qua_via:
            return 0.12, 0.0, None
        err = wrap(gyaw - eth)
        if abs(err) < 0.3:
            return 0.0, 0.0, 'SUCCEEDED'
        return 0.0, math.copysign(0.544, err), None


def chay(blockers, start=(1.60, 0.82, 0.0), tuyen_dau=1, t_max=240.0, seed=1, dung_L1=True, phat_lenh=True, bat_dau_chan=None, ban_do=GRID, tuyen=None):
    """blockers: ham (t, pose) -> danh sach hinh chu nhat vat can dang co mat."""
    rng = random.Random(seed)
    TUYEN_ = tuyen or TUYEN
    brain = L.NguoiLai()
    pose = start
    t = 0.0
    thu_tu = [tuyen_dau, 2 if tuyen_dau == 1 else 1]
    ds = 0                        # chi so tuyen trong thu_tu
    wi = 0                        # chi so diem trong tuyen
    goal_t0 = None
    nav = None
    gap_den = 0.0
    log = []
    min_gap = math.inf
    min_o = None
    can_thiep = []
    nhat_ky_l1 = []
    l1_thoat = 0
    dang_thoat = False
    ket_qua = 'HET_GIO'
    tuyen_hong = []
    while t < t_max:
        world = XUONG + list(blockers(t, pose))
        scan = make_scan(world, pose)
        pts = L.scan_to_xy(scan)
        if pts:
            mg = min(L.khoang_cach_mep(x, y) for x, y in pts)
            if mg < min_gap:
                min_gap, min_o = mg, (round(t, 1), round(pose[0], 2), round(pose[1], 2), round(math.degrees(pose[2])))
        # chuyen.sh: gui diem ke tiep khi khong co diem dang chay
        if nav is None and t >= gap_den:
            if ds >= len(thu_tu):
                ket_qua = 'HET_DUONG'
                break
            r = thu_tu[ds]
            if wi >= len(TUYEN_[r]):
                ket_qua = 'DEN_NOI_tuyen%d' % r
                break
            nav = Nav2DoChoi(TUYEN_[r][wi], rng, pose0=pose)
            goal_t0 = t
            log.append((round(t, 1), 'GUI tuyen %d diem %d' % (r, wi + 1)))
        goal_active = nav is not None
        v2 = w2 = 0.0
        st = None
        if nav is not None:
            v2, w2, st = nav.cmd(pose, pts)
        pl = (lambda d, p=pose: L.la_dia_hinh(d, p, ban_do)) if ban_do is not None else None
        nav_cmd = (v2, w2) if (nav is not None and st is None) else None
        v1, w1, ten = brain.step(pts, t, goal_active, pl, nav_cmd) if dung_L1 else (None, None, 'TAT')
        if not nhat_ky_l1 or nhat_ky_l1[-1][1] != ten:
            nhat_ky_l1.append((round(t, 1), ten))
        if brain.state == 'THOAT' and not dang_thoat:
            dang_thoat = True
            l1_thoat += 1
            log.append((round(t, 1), 'L1 BAT DAU THOAT (%s) tai x=%.2f y=%.2f huong=%d do; %s' % (brain.ly_do, pose[0], pose[1], round(math.degrees(pose[2])), L.mo_ta_vung(L.do_vung(pts)))))
            can_thiep.append((round(t, 1), brain.ly_do, tuple(round(v, 2) for v in pose[:2])))
        elif brain.state != 'THOAT' and dang_thoat:
            dang_thoat = False
        if ten.startswith('CHAN_') and (not nhat_ky_l1 or True) and (not log or log[-1][1] != 'L1 phu quyet ' + ten):
            log.append((round(t, 1), 'L1 phu quyet ' + ten))
        if nav is not None and st is not None:
            log.append((round(t, 1), 'Nav2 do choi: %s (tuyen %d diem %d, mat %.0f s) xe o x=%.2f y=%.2f huong=%d; %s' % (st, thu_tu[ds], wi + 1, t - goal_t0, pose[0], pose[1], round(math.degrees(pose[2])), L.mo_ta_vung(L.do_vung(pts)))))
            nav = None
            gap_den = t + GAP_GIUA_DICH
            if st == 'SUCCEEDED':
                wi += 1
            else:
                tuyen_hong.append(thu_tu[ds])
                ds += 1
                wi = 0
            v2 = w2 = 0.0
        if v1 is not None and phat_lenh:
            v, w = v1, w1                       # twist_mux: avoid (60) thang nav (20)
        else:
            v, w = v2, w2
        x, y, th = pose
        pose = (x + v * math.cos(th) * DT, y + v * math.sin(th) * DT, th + w * DT)
        if cham(XUONG + list(blockers(t, pose)), pose):
            ket_qua = 'CHAM_VAT'
            log.append((round(t, 1), 'CHAM VAT tai (%.2f, %.2f)' % (pose[0], pose[1])))
            break
        t += DT
    return dict(ket_qua=ket_qua, t=t, pose=pose, min_gap=min_gap, min_o=min_o, can_thiep=can_thiep, log=log, l1=nhat_ky_l1, l1_thoat=l1_thoat, tuyen_hong=tuyen_hong)


def in_ket_qua(ten, r, chi_tiet=True):
    print('%-48s -> %-18s t=%5.0f s  khe nho nhat=%.3f m (luc t,x,y,huong=%s)  L1 thoat=%d  tuyen hong=%s' % (ten, r['ket_qua'], r['t'], r['min_gap'], r['min_o'], r['l1_thoat'], r['tuyen_hong']))
    if chi_tiet:
        for t, m in r['log']:
            print('      %6.1f  %s' % (t, m))


def khong_chan(t, pose):
    return []


TUYEN_MOI = {
    1: [(1.45, 0.03, math.pi), (0.0, 0.0, 0.0)],
    2: [(1.30, 0.745, math.pi), (0.15, 0.72, math.pi), (0.0, 0.0, 0.0)],
}


def thong_ke(mc, ten, tuyen, tuyen_dau=1, blockers=None):
    ok = cham_n = thoat = 0
    gaps, ts = [], []
    ct = []
    for sd in range(mc):
        r = chay(blockers or khong_chan, seed=100 + sd, tuyen=tuyen, tuyen_dau=tuyen_dau)
        ok += r['ket_qua'].startswith('DEN_NOI')
        cham_n += r['ket_qua'] == 'CHAM_VAT'
        thoat += r['l1_thoat']
        gaps.append(r['min_gap'])
        ts.append(r['t'])
        ct += r['can_thiep']
    gaps.sort()
    print('%-34s den noi %d/%d, cham %d, nguoi lai thoat %d lan; khe nho nhat min=%.3f trung vi=%.3f m; t trung binh %.0f s' % (ten, ok, mc, cham_n, thoat, gaps[0], gaps[len(gaps) // 2], sum(ts) / len(ts)))
    for c in ct[:4]:
        print('      can thiep:', c)


def main():
    mc = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    print('=== A. Ca co dinh (B\' -> A, ve_A), TUYEN MOI (wp1=(1.45;0.03); tuyen 2: (1.30;.745), (.15;.72)) ===')
    chay_a = lambda *a, **k: chay(*a, tuyen=TUYEN_MOI, **k)
    r = chay_a(khong_chan)
    in_ket_qua('A1 binh thuong, khong vat can', r)
    r = chay_a(lambda t, p: [FOAM])
    in_ket_qua('A2 tam chan dat TRUOC (2a), Nav2 do choi khong lap lai duong', r)
    r = chay_a(lambda t, p: [FOAM] if p[0] < 1.00 else [])
    in_ket_qua('A3 tam chan xuat hien SAU khi xe vao hanh lang (2b)', r)
    r = chay_a(lambda t, p: [NGUOI] if t < 29 else [])
    in_ket_qua('A4 nguoi chan hanh lang ~15 s roi di (den t=29 s)', r)
    r = chay_a(lambda t, p: [FOAM] if p[0] < 1.00 else [], dung_L1=False)
    in_ket_qua('A5 nhu A3 nhung KHONG co nguoi lai LiDAR', r)
    r = chay_a(lambda t, p: [FOAM] if p[0] < 1.00 else [], phat_lenh=False)
    in_ket_qua('A6 nhu A3 nhung nguoi lai o che do CHI NHIN', r)
    r = chay_a(lambda t, p: [FOAM] if p[0] < 1.00 else [], tuyen_dau=2)
    in_ket_qua('A7 nhu A3 nhung di tuyen 2 truoc (hanh lang tren)', r)
    r = chay_a(khong_chan, start=(1.60, 0.82, 0.0), ban_do=None)
    in_ket_qua('A8 binh thuong, nguoi lai khong co ban do', r)
    # xe do sat tuong mui: mui cach tuong truoc 6 cm
    r = chay_a(khong_chan, start=(1.885 - 0.145 - 0.06, 0.82, 0.0))
    in_ket_qua('A9 mui cach tuong truoc (co trong ban do) 6 cm', r)
    r = chay_a(khong_chan, start=(0.90, -0.18 + 0.04 + 0.08, math.pi))
    in_ket_qua('A10 vao hanh lang duoi sat K4 (hong cach K4 ~4 cm)', r)

    r = chay(lambda t, p: [FOAM] if p[0] < 1.00 else [], tuyen=TUYEN)
    in_ket_qua('A3-cu nhu A3 nhung dung bang tuyen CU', r)
    print('\n=== B. Ngau nhien %d lan moi dong (sai vi tri 3 cm / 2 do), KHONG co vat can, B\' -> A ===' % mc)
    thong_ke(mc, 'tuyen 1 CU  wp1=(1.45;0.13)', TUYEN, 1)
    thong_ke(mc, 'tuyen 1 MOI wp1=(1.45;0.03)', TUYEN_MOI, 1)
    thong_ke(mc, 'tuyen 2 CU  wp2=(0.30;0.72)', TUYEN, 2)
    thong_ke(mc, 'tuyen 2 MOI wp1=(1.30;.745) wp2=(.15;.72)', TUYEN_MOI, 2)


if __name__ == '__main__':
    main()
