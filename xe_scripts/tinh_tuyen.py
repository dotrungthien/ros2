#!/usr/bin/env python3
"""tinh_tuyen (viec 98, soan 01/11/2026 o nha, CHUA THU TREN XE): tinh bang tuyen BANG=xs cho chuyen.sh tu costmap.

Doc du_lieu_0111/costmap_v3_020.pgm/.yaml. O 0 (do) cam; o 205 (vang) va 254 (xanh) di duoc (Thien chot 01/11).
Dijkstra 8 huong; gia moi buoc = do dai (o) x (1 + 60/d^3), d = khoang cach Euclid (o) tu tam o DI TOI toi tam o do
gan nhat. Buoc cheo khong duoc cat qua goc o do (hai o ke canh deu phai khong do).
Tach hanh lang: tuyen 1 (khe duoi) cam x 0,30-1,20 m, y >= 0,50 m; tuyen 2 (khe tren) cam x 0,30-1,20 m, y <= 0,40 m
(xet tam o). di_B: A -> B; ve_A: B -> A (tinh rieng, khong lat duong di_B).
Moi tuyen chia 6 diem cach deu theo chieu dai (diem 6 = dich); goc = tiep tuyen (huong tu s - 0,15 m toi s + 0,15 m
tren duong); rieng diem cuoi di_B giu 0 do nhu bang tt.
In 4 dong san dan vao case cua chuyen.sh, roi cac dong kiem (bat dau bang #); luu du_lieu_0111/tuyen_xs.png.
Chay: python3 tinh_tuyen.py
"""
import heapq
import math
import os
import re

DAY = os.path.dirname(os.path.abspath(__file__))
YAML = os.path.join(DAY, 'du_lieu_0111', 'costmap_v3_020.yaml')
PNG = os.path.join(DAY, 'du_lieu_0111', 'tuyen_xs.png')

A = (0.0, 0.0)
B = (1.60, 0.82)
K_GIA = 60.0           # gia = do dai x (1 + K_GIA / d^3)
SO_DIEM = 6
DELTA_TT = 0.15        # m, nua doan dung de tinh tiep tuyen
CAM = {                # tuyen -> (x0, x1, y0, y1) vung cam (m)
    1: (0.30, 1.20, 0.50, 1e9),
    2: (0.30, 1.20, -1e9, 0.40),
}
HUONG8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def doc_yaml(p):
    t = open(p).read()
    anh = re.search(r'^image:\s*(\S+)', t, re.M).group(1)
    res = float(re.search(r'^resolution:\s*([-0-9.eE]+)', t, re.M).group(1))
    o = [float(v) for v in re.search(r'^origin:\s*\[([^\]]*)\]', t, re.M).group(1).split(',')]
    return os.path.join(os.path.dirname(p), anh), res, o[0], o[1]


def doc_pgm(p):
    """P5 -> (w, h, bytes theo hang anh, hang tren truoc)."""
    b = open(p, 'rb').read()
    tok, i = [], 0
    while len(tok) < 4:
        while b[i:i + 1].isspace():
            i += 1
        if b[i:i + 1] == b'#':
            while b[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not b[j:j + 1].isspace():
            j += 1
        tok.append(b[i:j])
        i = j
    if tok[0] != b'P5' or int(tok[3]) != 255:
        raise SystemExit('chi doc duoc PGM P5, max 255')
    w, h = int(tok[1]), int(tok[2])
    return w, h, b[i + 1:i + 1 + w * h]


class Ban:
    """Costmap: o (i, j), i = cot theo x, j = hang tu duoi len theo y."""

    def __init__(self, yaml_path):
        pgm, self.res, self.ox, self.oy = doc_yaml(yaml_path)
        self.w, self.h, px = doc_pgm(pgm)
        self.v = {(i, j): px[(self.h - 1 - j) * self.w + i] for i in range(self.w) for j in range(self.h)}
        do = [k for k, v in self.v.items() if v == 0]
        self.d = {k: (0.0 if v == 0 else min(math.hypot(k[0] - a, k[1] - b) for a, b in do)) for k, v in self.v.items()}

    def o(self, x, y):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def tam(self, k):
        return self.ox + (k[0] + 0.5) * self.res, self.oy + (k[1] + 0.5) * self.res

    def do(self, k):
        return self.v.get(k, 0) == 0          # ngoai ban do cung coi la do


def dijkstra(ban, dau, cuoi, cam):
    """Duong (danh sach o) tu o dau toi o cuoi; cam(o) = True thi khong vao."""
    gia = {dau: 0.0}
    truoc = {}
    pq = [(0.0, dau)]
    while pq:
        g, u = heapq.heappop(pq)
        if u == cuoi:
            break
        if g > gia[u]:
            continue
        for di, dj in HUONG8:
            v = (u[0] + di, u[1] + dj)
            if ban.do(v) or cam(v):
                continue
            if di and dj and (ban.do((u[0] + di, u[1])) or ban.do((u[0], u[1] + dj))):
                continue
            g2 = g + math.hypot(di, dj) * (1.0 + K_GIA / ban.d[v] ** 3)
            if g2 < gia.get(v, math.inf):
                gia[v] = g2
                truoc[v] = u
                heapq.heappush(pq, (g2, v))
    if cuoi not in gia:
        raise SystemExit('khong tim duoc duong %s -> %s' % (dau, cuoi))
    duong = [cuoi]
    while duong[-1] != dau:
        duong.append(truoc[duong[-1]])
    return duong[::-1]


def diem_tai(pl, cum, s):
    """Diem tren duong gap khuc pl (do dai tich luy cum) o chieu dai s."""
    s = max(0.0, min(cum[-1], s))
    for k in range(1, len(pl)):
        if cum[k] >= s:
            t = (s - cum[k - 1]) / (cum[k] - cum[k - 1]) if cum[k] > cum[k - 1] else 0.0
            return (pl[k - 1][0] + t * (pl[k][0] - pl[k - 1][0]), pl[k - 1][1] + t * (pl[k][1] - pl[k - 1][1]))
    return pl[-1]


def chia_diem(pl, n=SO_DIEM, delta=DELTA_TT):
    """n diem cach deu theo chieu dai (diem n = cuoi), moi diem kem goc tiep tuyen (rad)."""
    cum = [0.0]
    for k in range(1, len(pl)):
        cum.append(cum[-1] + math.hypot(pl[k][0] - pl[k - 1][0], pl[k][1] - pl[k - 1][1]))
    L = cum[-1]
    ra = []
    for k in range(1, n + 1):
        s = L * k / n
        x, y = diem_tai(pl, cum, s)
        a, b = diem_tai(pl, cum, s - delta), diem_tai(pl, cum, s + delta)
        ra.append((x, y, math.atan2(b[1] - a[1], b[0] - a[0])))
    return ra, L


def so(v, n):
    t = '%.*f' % (n, v)
    return t[1:] if t.startswith('-') and float(t) == 0 else t


def dong_case(huong, tuyen, diem):
    wp = ' '.join('"%s %s %s %s"' % (so(x, 2), so(y, 2), so(math.sin(a / 2), 4), so(math.cos(a / 2), 4)) for x, y, a in diem)
    return '    xs:%s:%d) WP=(%s) ;;' % (huong, tuyen, wp)


def ve(ban, kq, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError:
        return False
    mau = {0: 0, 205: 1, 254: 2}
    anh = [[mau[ban.v[(i, j)]] for i in range(ban.w)] for j in range(ban.h - 1, -1, -1)]
    ext = [ban.ox, ban.ox + ban.w * ban.res, ban.oy, ban.oy + ban.h * ban.res]
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.imshow(anh, cmap=ListedColormap(['#d9534f', '#f0c419', '#9fd89a']), vmin=0, vmax=2, extent=ext, interpolation='nearest')
    for t, (x0, x1, y0, y1) in CAM.items():
        y0, y1 = max(y0, ext[2]), min(y1, ext[3])
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls='--', lw=1, ec='k'))
        ax.text(x0 + 0.02, y1 - 0.06 if t == 1 else y0 + 0.02, 'cam tuyen %d' % t, fontsize=8)
    kieu = {('di_B', 1): ('#1f4e9c', '-'), ('di_B', 2): ('#1f4e9c', '--'), ('ve_A', 1): ('#7b2d8e', '-'), ('ve_A', 2): ('#7b2d8e', '--')}
    for (huong, tuyen), (pl, diem, _) in kq.items():
        c, ls = kieu[(huong, tuyen)]
        ax.plot([p[0] for p in pl], [p[1] for p in pl], ls=ls, color=c, lw=1.6, label='%s %d' % (huong, tuyen))
        for k, (x, y, a) in enumerate(diem, 1):
            ax.plot(x, y, 'o', color=c, ms=5)
            ax.annotate('', xy=(x + 0.08 * math.cos(a), y + 0.08 * math.sin(a)), xytext=(x, y),
                        arrowprops=dict(arrowstyle='->', color=c, lw=1.2))
            ax.text(x + 0.02, y + (0.03 if huong == 'di_B' else -0.06), str(k), color=c, fontsize=8)
    for ten, (x, y) in (('A', A), ('B', B)):
        ax.plot(x, y, 'k*', ms=12)
        ax.text(x + 0.03, y + 0.03, ten, fontsize=11, weight='bold')
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('BANG=xs tu costmap_v3_020 (do = cam; vang, xanh = di duoc)')
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


def main():
    ban = Ban(YAML)
    oA, oB = ban.o(*A), ban.o(*B)
    kq = {}
    for huong, dau, cuoi, p_dau, p_cuoi in (('di_B', oA, oB, A, B), ('ve_A', oB, oA, B, A)):
        for tuyen, (x0, x1, y0, y1) in CAM.items():
            cam = lambda k, x0=x0, x1=x1, y0=y0, y1=y1: x0 <= ban.tam(k)[0] <= x1 and y0 <= ban.tam(k)[1] <= y1
            duong = dijkstra(ban, dau, cuoi, cam)
            pl = [p_dau] + [ban.tam(k) for k in duong[1:-1]] + [p_cuoi]
            diem, L = chia_diem(pl)
            if huong == 'di_B':
                diem[-1] = (diem[-1][0], diem[-1][1], 0.0)
            kq[(huong, tuyen)] = (pl, diem, (duong, L))
    for huong in ('di_B', 've_A'):
        for tuyen in (1, 2):
            print(dong_case(huong, tuyen, kq[(huong, tuyen)][1]))
    print()
    print('# tuyen      dai(m)  khe_min_6diem(m)  A/B->diem1(m)  d_min(cm)  o_vang/tong  goc 6 diem (do)')
    for (huong, tuyen), (pl, diem, (duong, L)) in sorted(kq.items()):
        khe = min(math.hypot(diem[k][0] - diem[k - 1][0], diem[k][1] - diem[k - 1][1]) for k in range(1, len(diem)))
        dau = math.hypot(diem[0][0] - pl[0][0], diem[0][1] - pl[0][1])
        dmin = min(ban.d[k] for k in duong) * ban.res * 100
        vang = sum(1 for k in duong if ban.v[k] == 205)
        goc = ' '.join('%.0f' % math.degrees(a) for _, _, a in diem)
        print('# %s %d   %6.2f  %16.2f  %13.2f  %9.1f  %5d/%-5d  %s' % (huong, tuyen, L, khe, dau, dmin, vang, len(duong), goc))
    print('# anh: %s' % (PNG if ve(ban, kq, PNG) else 'KHONG VE (thieu matplotlib)'))


if __name__ == '__main__':
    main()
