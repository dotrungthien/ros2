#!/usr/bin/env python3
"""the_gioi_gia.py - the gioi gia cho test (09/11 23:08 sep LiDAR; CHI DUNG TRONG TEST, KHONG chep len Pi).

- Tuong = o den cua du_lieu_0111/map_xuong_v3.png (anh cua map_xuong_v3_tinh.yaml = /map_tinh tren xe), canh bien gop thang hang.
- LiDAR gia (numpy): 420 tia 360 do tu (0,025; 0) khung xe, range_min 0,10 (nhu test_gui_duong.quet_gia) => diem khung xe theo tia.
- Duong Nav2 gia: Dijkstra 8 huong tren luoi 0,05 m, o di duoc = tam o cach tuong >= CACH (than + le); quanh diem dau / cuoi
  cho phep sat hon (xe dang dung sat tuong). Khong phai Nav2 that: chi de test luong quyet cua gui_duong.
"""
import heapq
import math
import os

import numpy as np
import yaml

DAY = os.path.dirname(os.path.abspath(__file__))
YAML_TINH = os.path.join(DAY, 'du_lieu_0111', 'map_xuong_v3_tinh.yaml')
CACH = 0.25          # m: tam o xe cach tam o tuong toi thieu khi vach duong gia (gia; giu mui xa goc tuong khi re)


def doc_ban_do(path=YAML_TINH):
    """-> dict(o (tap (i, j) tuong; j tu duoi len), w, h, res, ox, oy, data OccupancyGrid 0/100)."""
    from PIL import Image
    y = yaml.safe_load(open(path))
    im = Image.open(os.path.join(os.path.dirname(path), y['image'])).convert('L')
    w, h = im.size
    px = im.load()
    o = {(i, j) for i in range(w) for j in range(h) if px[i, h - 1 - j] == 0}
    data = [100 if (i, j) in o else 0 for j in range(h) for i in range(w)]
    return dict(o=o, w=w, h=h, res=y['resolution'], ox=y['origin'][0], oy=y['origin'][1], data=data)


def canh_tuong(bd):
    """Canh bien o tuong / o trong -> [((x1, y1), (x2, y2))], gop canh thang hang lien nhau."""
    o, r, ox, oy = bd['o'], bd['res'], bd['ox'], bd['oy']
    ngang, doc = {}, {}
    for i, j in o:
        if (i, j - 1) not in o:
            ngang.setdefault(j, []).append(i)
        if (i, j + 1) not in o:
            ngang.setdefault(j + 1, []).append(i)
        if (i - 1, j) not in o:
            doc.setdefault(i, []).append(j)
        if (i + 1, j) not in o:
            doc.setdefault(i + 1, []).append(j)
    out = []
    for j, xs in ngang.items():
        for a, b in _gop(sorted(set(xs))):
            out.append(((ox + a * r, oy + j * r), (ox + (b + 1) * r, oy + j * r)))
    for i, ys in doc.items():
        for a, b in _gop(sorted(set(ys))):
            out.append(((ox + i * r, oy + a * r), (ox + i * r, oy + (b + 1) * r)))
    return out


def _gop(ks):
    out, a = [], None
    for k in ks:
        if a is None:
            a = b = k
        elif k == b + 1:
            b = k
        else:
            out.append((a, b))
            a = b = k
    if a is not None:
        out.append((a, b))
    return out


def quet(doan, pose, n=420, r_min=0.10, r_max=8.0):
    """Doan khung ban do + tu the xe (x, y, yaw) -> [(x, y) | None] khung xe theo tia (tia k: goc -pi + 2 pi k / n)."""
    x, y, th = pose
    lx, ly = x + 0.025 * math.cos(th), y + 0.025 * math.sin(th)
    if not doan:
        return [None] * n
    s = np.array([(a[0], a[1], b[0], b[1]) for a, b in doan], dtype=float)
    ab = -math.pi + 2 * math.pi * np.arange(n) / n
    c, sn = np.cos(ab + th)[:, None], np.sin(ab + th)[:, None]
    ex, ey = (s[:, 2] - s[:, 0])[None, :], (s[:, 3] - s[:, 1])[None, :]
    ax, ay = (s[:, 0] - lx)[None, :], (s[:, 1] - ly)[None, :]
    den = c * ey - sn * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * ey - ay * ex) / den
        u = (sn * ax - c * ay) / den
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(ok, t, np.inf).min(axis=1)
    out = []
    for k in range(n):
        if not math.isfinite(t[k]) or t[k] < r_min or t[k] > r_max:
            out.append(None)
        else:
            out.append((0.025 + t[k] * math.cos(ab[k]), t[k] * math.sin(ab[k])))
    return out


def doan_hop(cx, cy, rong, dai=None, goc=0.0):
    """Vat hinh hop (nguoi / ong / thung) tam (cx, cy), rong x dai, quay goc -> 4 doan khung ban do."""
    dai = rong if dai is None else dai
    c, s = math.cos(goc), math.sin(goc)
    g = [(-dai / 2, -rong / 2), (dai / 2, -rong / 2), (dai / 2, rong / 2), (-dai / 2, rong / 2)]
    p = [(cx + a * c - b * s, cy + a * s + b * c) for a, b in g]
    return [(p[k], p[(k + 1) % 4]) for k in range(4)]


class LuoiDi:
    """Duong Nav2 gia tren luoi ban do: o di duoc = tam o cach o tuong (va vat them) >= CACH."""

    def __init__(self, bd):
        self.bd = bd
        self.r, self.ox, self.oy = bd['res'], bd['ox'], bd['oy']

    def o(self, x, y):
        return int(math.floor((x - self.ox) / self.r)), int(math.floor((y - self.oy) / self.r))

    def tam(self, k):
        return self.ox + (k[0] + 0.5) * self.r, self.oy + (k[1] + 0.5) * self.r

    def duong(self, a, b, vat=(), cam=lambda x, y: False, tha=0.30):
        """a, b (x, y) -> [(x, y), ...] (diem dau a, cuoi b) | None. vat: [(x, y)] diem vat them (LiDAR thay); cam(x, y): o khe bi to
        (/chon_tuyen: gia x20, khong chan - nhu lop_huong_dan that)."""
        tuong = [self.tam(k) for k in self.bd['o']] + list(vat)
        T = np.array(tuong) if tuong else np.zeros((0, 2))
        ka, kb = self.o(*a), self.o(*b)

        kc_t = lambda x, y: np.hypot(T[:, 0] - x, T[:, 1] - y).min() if len(T) else 9.0
        d_a, d_b = kc_t(*a), max(kc_t(*b), 0.14)                  # dich bi vat chiem (nguoi dung tren B): khong vach sat vat

        def di(k):
            x, y = self.tam(k)
            if not (0 <= k[0] < self.bd['w'] and 0 <= k[1] < self.bd['h']) or k in self.bd['o']:
                return False
            d = kc_t(x, y)
            gan_a, gan_b = math.hypot(x - a[0], y - a[1]) <= tha, math.hypot(x - b[0], y - b[1]) <= tha
            # quanh diem dau / cuoi: khong gan tuong hon chinh diem do (xe dang dung sat tuong thi di ra, khong di sat hon)
            ok = d >= CACH or (gan_a and d >= min(CACH, d_a) - 1e-9) or (gan_b and d >= min(CACH, d_b) - 1e-9)
            gia = (1.0 + 8.0 * max(0.0, 0.25 - d)) * (20.0 if cam(x, y) and not (gan_a or gan_b) else 1.0)   # gan tuong / khe bi to
            return gia if ok else False      # khe bi to (lop_huong_dan 99 => 251 < 253): rat ton nhung KHONG cam

        nho, kc, q = {}, {ka: 0.0}, [(0.0, ka)]
        while q:
            g, k = heapq.heappop(q)
            if k == kb:
                break
            if g > kc.get(k, math.inf):
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    m = (k[0] + dx, k[1] + dy)
                    if (dx or dy) and m not in nho:
                        nho[m] = di(m)
                    if (dx or dy) and nho[m]:
                        g2 = g + math.hypot(dx, dy) * nho[m]
                        if g2 < kc.get(m, math.inf):
                            kc[m] = g2
                            heapq.heappush(q, (g2, m))
                            nho.setdefault(('cha', m), None)
                            nho[('cha', m)] = k
        if kb not in kc:
            return None
        ds, k = [], kb
        while k != ka:
            ds.append(self.tam(k))
            k = nho[('cha', k)]
        return [a] + ds[::-1][:-1] + [b] if ds else [a, b]
