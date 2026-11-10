#!/usr/bin/env python3
"""ban_do_chung.py (10/11 Thien duyet 17:59 "SEP VA NAV2 DUNG CHUNG MOT BAN DO", CHUA CHAY TREN XE). Phan thuan, khong ROS.

1. Vong an toan quanh VAT CHAC (ong / ngoai ban do; gac_vat_la danh_gia 'chac'): ban kinh cum + 0,06 (vung do hong) + 0,01.
   gui_duong tinh tu moi lan quet, giu GIU_S s sau lan thay cuoi (GiuVong), gui /vong_vat_ds; lop_huong_dan ve luoi /vong_vat
   (CUNG hinh hoc /map_tinh, 0 | 100) cho StaticLayer rieng 'vong_vat' cua ca global va local costmap (100 => 254 LETHAL, inflation
   them nua than 0,08 => tam xe cach vat >= ban kinh cum + 0,15 = than cach vat >= 0,07). Tuong KHONG co vong.
2. Dem mem quanh tuong ban do (dem_tuong): gia tri < 99 (chi phi < 253) => KHONG BAO GIO chan, chi day duong Nav2 ra giua.
3. Khe (KHE, do_khe, chon_khe): do chieu rong thong cua khe duoi / tren tren /global_costmap/costmap (o < 99 = tam xe qua duoc).
"""
import math

import gac_vat_la as GV

LE_VONG = 0.06 + 0.01      # m: vung do hong (luat cam cua xe) + 0,01
GIU_S = 3.0                # s: giu vong sau lan thay cuoi
NHIP_S = 0.5               # s: tinh vong toi da 2 lan/s (global costmap 1 Hz; CPU)
CHAN = 99                  # OccupancyGrid cua costmap Nav2: 253 (inscribed) => 99, 254 => 100

# Khe tren ban do v3 (/map_tinh): tuyen = so /chon_tuyen (lop_huong_dan cam khe KIA, gia 99 => 251: ton, khong chan)
KHE = {'duoi': dict(r=1, x0=0.61, x1=1.16, y0=-0.32, y1=0.34),     # mep tren K4 (y -0,165) / tuong duoi (-0,315) - mep duoi H0 (0,335)
       'tren': dict(r=2, x0=0.81, x1=1.16, y0=0.53, y1=0.99)}      # mep tren H0 (0,535) - mep duoi K2 (0,985)


def vong_quet(pts_xe, pts_map, ban, t, theo=None, tq=None, r_max=2.5, le=LE_VONG):
    """MOI cum trong r_max (obstacle_max_range Nav2) - khong chi cum trong vung vang nhu danh_gia - phan loai ba dau hieu
    (theo: TheoDoiOng RIENG, cung luat on dinh). -> (vong [(x, y, R)] khung ban do cho cum ong / ngoai ban do, cum [dict(ks, nhan,
    loai)] cho hop den). Tam: khop tron (Kasa) neu r 0,02-0,20 va rms <= 0,01 (mat cung thay mot nua: trong tam lech ve LiDAR),
    khong thi trong tam diem; R = max(r khop, khoang xa nhat tam -> diem) + le."""
    if pts_map is None:
        return [], []
    vong, cum = [], []
    for a, b in GV.cum(pts_xe, t):
        ks = [(a + i) % len(pts_xe) for i in range((b - a) % len(pts_xe) + 1)]
        if len(ks) < t['so_diem'] or min(math.hypot(pts_xe[k][0], pts_xe[k][1]) for k in ks) > r_max:
            continue
        P = [pts_map[k] for k in ks if pts_map[k] is not None]
        Q = P if ban is None else [p for p in P if not ban.gan_tuong(p[0], p[1], t['ngoai_m'])]
        if ban is not None and P and len(Q) < t['ngoai_n']:
            cum.append(dict(ks=ks, nhan='tuong', loai='TUONG'))      # nam tren tuong ban do: khong the la ong / ngoai (CPU: bo khop)
            continue
        nhan = GV.ba_dau_hieu([pts_xe[k] for k in ks], [pts_map[k] for k in ks], ban, t, theo, tq)[0]
        cum.append(dict(ks=ks, nhan=nhan, loai='TUONG' if nhan == 'tuong' else 'VAT'))
        P = Q if len(Q) >= 3 else P                      # cum ong / vat dinh vao tuong: vong chi quanh phan NGOAI ban do (khong om tuong)
        if nhan not in ('ong', 'ngoai') or len(P) < 2:
            continue
        cx, cy, r0 = sum(p[0] for p in P) / len(P), sum(p[1] for p in P) / len(P), 0.0
        f = GV.khop_tron(P) if len(P) >= 3 else None
        if f is not None and 0.02 <= f[2] <= 0.20 and f[3] <= 0.01:
            cx, cy, r0 = f[0], f[1], f[2]
        r = max([r0] + [math.hypot(x - cx, y - cy) for x, y in P])
        vong.append((round(cx, 3), round(cy, 3), round(r + le, 3)))
    return vong, cum


class GiuVong:
    """Giu vong giu_s s sau lan thay cuoi. Vong moi thay cac vong cu co tam cach < 0,10 m hoac chong len nhau qua nua (cung vat)."""

    def __init__(self, giu_s=GIU_S):
        self.giu_s, self.ds = giu_s, []            # [(x, y, R, t)]

    def cap_nhat(self, vong, t):
        for x, y, r in vong:
            self.ds = [v for v in self.ds if math.hypot(v[0] - x, v[1] - y) >= max(0.10, 0.5 * min(r, v[2]))]
            self.ds.append((x, y, r, t))
        return self.hien(t)

    def hien(self, t):
        self.ds = [v for v in self.ds if t - v[3] <= self.giu_s + 1e-9]
        return [(x, y, r) for x, y, r, _ in self.ds]


def chuoi_vong(vong):
    return ';'.join('%.3f %.3f %.3f' % v for v in vong)


def doc_vong(s):
    """'x y r;x y r' -> [(x, y, r)] (phan hong bo qua)."""
    out = []
    for p in (s or '').split(';'):
        try:
            x, y, r = (float(v) for v in p.split())
        except ValueError:
            continue
        if 0 < r < 1.0:
            out.append((x, y, r))
    return out


def luoi_vong(hh, vong):
    """OccupancyGrid data (hang 0 o duoi): o co phan nao trong vong => 100, con lai 0 (KHONG -1: use_maximum gap NO_INFORMATION)."""
    w, h, res, ox, oy = hh['w'], hh['h'], hh['res'], hh['ox'], hh['oy']
    out = [0] * (w * h)
    for x, y, r in vong:
        for j in range(max(0, int((y - r - oy) / res) - 1), min(h, int((y + r - oy) / res) + 2)):
            for i in range(max(0, int((x - r - ox) / res) - 1), min(w, int((x + r - ox) / res) + 2)):
                nx = min(max(x, ox + i * res), ox + (i + 1) * res)        # diem cua o gan tam nhat
                ny = min(max(y, oy + j * res), oy + (j + 1) * res)
                if math.hypot(nx - x, ny - y) <= r:
                    out[j * w + i] = 100
    return out


def dem_tuong(w, h, res, tuong, r_dem=0.30, k=6.0, r_ins=0.08, tran=98):
    """Dem mem: o cach o tuong gan nhat d (tam o): d <= r_ins => tran; d <= r_dem => tran * exp(-k (d - r_ins)); xa hon 0.
    tuong: tap (i, j). tran 98 => chi phi 248 < 253 => KHONG chan."""
    out = [0] * (w * h)
    n = int(math.ceil(r_dem / res))
    for (ti, tj) in tuong:
        for j in range(max(0, tj - n), min(h, tj + n + 1)):
            for i in range(max(0, ti - n), min(w, ti + n + 1)):
                if (i, j) in tuong:
                    continue
                d = math.hypot(i - ti, j - tj) * res
                if d > r_dem + 1e-9:
                    continue
                v = tran if d <= r_ins + 1e-9 else int(tran * math.exp(-k * (d - r_ins)))
                if v > out[j * w + i]:
                    out[j * w + i] = v
    return out


def do_khe(o_cm, khe, buoc=0.05):
    """o_cm(x, y) -> gia tri costmap (0..100, -1, None). -> (rong_min m, (x giua, y tam doan thong o x giua) | None).
    Moi cot x trong khe: doan lien tuc dai nhat cac o 0 <= v < CHAN; rong_min = nho nhat qua cac cot (0 = khe dong voi Nav2)."""
    xs = [khe['x0'] + buoc * i for i in range(int(round((khe['x1'] - khe['x0']) / buoc)) + 1)]
    xg = xs[len(xs) // 2]
    rong, tam = None, None
    for x in xs:
        best, run, y0 = (0.0, None), 0, None
        y = khe['y0'] + buoc / 2
        while y < khe['y1']:
            v = o_cm(x, y)
            if v is not None and 0 <= v < CHAN:
                y0 = y if run == 0 else y0
                run += 1
                if run * buoc > best[0]:
                    best = (run * buoc, (y0 + y) / 2)
            else:
                run = 0
            y += buoc
        rong = best[0] if rong is None else min(rong, best[0])
        if abs(x - xg) < 1e-9 and best[1] is not None:
            tam = (round(xg, 3), round(best[1], 3))
    return (rong or 0.0), (tam if rong else None)


def chon_khe(o_cm, uu_tien=('duoi', 'tren')):
    """-> (ten, tam, rong) khe THONG dau tien theo uu_tien | None (ca hai dong voi Nav2: khong diem dan, Nav2 tu vach)."""
    for ten in uu_tien:
        rong, tam = do_khe(o_cm, KHE[ten])
        if rong > 0 and tam is not None:
            return ten, tam, rong
    return None


def can_dan(p, dich, khe):
    """Xe va dich o hai phia khe (theo x) => can di qua khe."""
    return (p[0] < khe['x0'] - 0.05 and dich[0] > khe['x1']) or (p[0] > khe['x1'] + 0.05 and dich[0] < khe['x0'])
