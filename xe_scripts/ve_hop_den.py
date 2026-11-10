#!/usr/bin/env python3
"""ve_hop_den.py (10/11 muc 0, chay tren PC): ve hop den gui_duong.
  python3 ve_hop_den.py <thu muc chuyen | tep.npz> [thu muc ra]
Moi tep .npz -> PNG cung ten (global costmap xam, o local costmap >= 99 do nhat, duong Nav2 /plan xanh, xe 25 x 16 + mui, diem quet
theo nhan: tuong xam dam / ong cam / vat do / khac xanh nhat, vong an toan xanh la net dut). Thu muc -> them _chuyen.png: vet xe that
(tu the moi tep) + su kien. Khong in noi dung tep cau hinh nao (khong co ACCESS trong hop den).
"""
import glob
import math
import os
import sys

import numpy as np

MAU = {0: '#9ecae1', 1: '#444444', 2: '#fd8d3c', 3: '#d62728'}
THAN = [(0.145, 0.08), (0.145, -0.08), (-0.105, -0.08), (-0.105, 0.08)]


def _anh(ax, g, info, **k):
    if g.size == 0:
        return
    w, h, res, ox, oy = info
    ax.imshow(np.where(g < 0, 50, g), origin='lower', cmap='Greys', vmin=0, vmax=100,
              extent=(ox, ox + w * res, oy, oy + h * res), **k)


def _xe(ax, pose):
    if pose is None or not np.all(np.isfinite(pose)):
        return
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    p = [(x + a * c - b * s, y + a * s + b * c) for a, b in THAN + THAN[:1]]
    ax.plot([q[0] for q in p], [q[1] for q in p], '-', color='#1f77b4', lw=1.5)
    ax.plot([x, x + 0.145 * c], [y, y + 0.145 * s], '-', color='#1f77b4', lw=1.5)


def ve_tep(p, ra=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    z = np.load(p, allow_pickle=False)
    fig, ax = plt.subplots(figsize=(8, 5.5))
    _anh(ax, z['cm_g'], z['cm_g_info'])
    l, li, mo = z['cm_l'], z['cm_l_info'], z['map_odom']
    if l.size and np.all(np.isfinite(mo)):                      # local costmap (khung odom) -> khung ban do
        w, h, res, ox, oy = li
        jj, ii = np.nonzero(l >= 99)
        xo, yo = ox + (ii + 0.5) * res, oy + (jj + 0.5) * res
        c, s = math.cos(mo[2]), math.sin(mo[2])
        ax.plot(mo[0] + xo * c - yo * s, mo[1] + xo * s + yo * c, 's', ms=3, color='#ff9896', alpha=0.6, label='local >= 99')
    if len(z['plan']):
        ax.plot(z['plan'][:, 0], z['plan'][:, 1], '-', color='#2ca02c', lw=1.2, label='/plan')
    pts, nh = z['pts'], z['nhan']
    for k, ten in enumerate(('khac', 'tuong', 'ong', 'vat')):
        m = (nh == k) & np.isfinite(pts[:, 0])
        if m.any():
            ax.plot(pts[m, 0], pts[m, 1], '.', ms=3, color=MAU[k], label=ten)
    for x, y, r in z['vong']:
        ax.add_patch(plt.Circle((x, y), r, fill=False, ls='--', color='#2ca02c'))
    _xe(ax, z['pose'])
    ax.set_aspect('equal')
    ax.set_title('%s  %s\n%s' % (str(z['su_kien']), os.path.basename(p)[:10], str(z['dong'])[:110]), fontsize=8)
    ax.legend(fontsize=6, loc='upper left')
    out = os.path.join(ra or os.path.dirname(p), os.path.basename(p)[:-4] + '.png')
    fig.savefig(out, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return out


def ve_chuyen(tm, ra=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ts = sorted(glob.glob(os.path.join(tm, '*.npz')))
    if not ts:
        return None
    zs = [np.load(t, allow_pickle=False) for t in ts]
    fig, ax = plt.subplots(figsize=(8, 5.5))
    z = next((z for z in reversed(zs) if z['cm_g'].size), zs[-1])
    _anh(ax, z['cm_g'], z['cm_g_info'])
    xy = np.array([z['pose'][:2] for z in zs if np.all(np.isfinite(z['pose']))])
    if len(xy):
        ax.plot(xy[:, 0], xy[:, 1], '-o', ms=2, color='#1f77b4', lw=1, label='vet xe')
    for z in zs:
        if str(z['su_kien']) != 'DI' and np.all(np.isfinite(z['pose'])):
            ax.annotate(str(z['su_kien']), z['pose'][:2], fontsize=6, color='#d62728')
    ax.set_aspect('equal')
    ax.set_title('%s: %d tep' % (os.path.basename(os.path.normpath(tm)), len(zs)), fontsize=9)
    ax.legend(fontsize=6)
    out = os.path.join(ra or tm, '_chuyen.png')
    fig.savefig(out, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return out


def main():
    if len(sys.argv) < 2:
        raise SystemExit('Dung: python3 ve_hop_den.py <thu muc chuyen | tep.npz> [thu muc ra]')
    src, ra = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else None)
    if ra:
        os.makedirs(ra, exist_ok=True)
    if src.endswith('.npz'):
        print(ve_tep(src, ra))
        return
    for t in sorted(glob.glob(os.path.join(src, '*.npz'))):
        print(ve_tep(t, ra))
    print(ve_chuyen(src, ra))


if __name__ == '__main__':
    main()
