#!/usr/bin/env python3
"""test_gac_vat_la.py (09/11 20:33 vung quanh xe: LiDAR lai, ban do nhac): python3 -m pytest -q test_gac_vat_la.py"""
import math
import os
import sys

DAY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DAY)
import gac_vat_la as GV  # noqa: E402

T = GV.tham_so()


def quet(*doan, n=420):
    """Quet gia: LiDAR (0,025; 0) khung xe, n tia 360 do, range_min 0,10; doan khung xe -> [(x, y) | None] theo tia."""
    out = []
    for k in range(n):
        a = -math.pi + 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        best = None
        for (x1, y1), (x2, y2) in doan:
            ex, ey = x2 - x1, y2 - y1
            den = c * ey - s * ex
            if abs(den) < 1e-12:
                continue
            ax, ay = x1 - GV.X_LIDAR, y1
            t, u = (ax * ey - ay * ex) / den, (s * ax - c * ay) / den
            if t > 0 and 0 <= u <= 1 and (best is None or t < best):
                best = t
        out.append(None if best is None or best < 0.10 else (GV.X_LIDAR + best * c, best * s))
    return out


def test_01_so_tu_hanh_vi_do_va_vang():
    assert (T['truoc_do'], T['hong_do'], T['duoi_do']) == (0.10, 0.06, 0.06)
    assert (T['vang_truoc'], T['vang_hong'], T['vang_sau']) == (0.30, 0.12, 0.10)
    assert (T['vat_dai'], T['tuong_dai'], T['gan_tuong'], T['so_diem'], T['sach_s']) == (0.25, 0.40, 0.18, 3, 2.0)


def test_02_vung_diem_theo_tam_xe():
    v = lambda x, y: GV.vung_diem((x, y), T)
    r = lambda t4: (t4[0], round(t4[1], 3), t4[2], t4[3])
    assert r(v(0.245, 0.0)) == ('truoc', 0.10, True, True) and r(v(0.255, 0.0)) == ('truoc', 0.11, True, False)
    assert r(v(0.445, 0.0)) == ('truoc', 0.30, True, False) and v(0.46, 0.0)[2] is False
    assert r(v(0.20, 0.12))[0:3] == ('truoc', 0.068, True) and v(0.20, 0.12)[3] is False     # cheo truoc: khong phai do
    assert r(v(0.0, 0.14)) == ('hong', 0.06, True, True) and r(v(0.0, -0.20)) == ('hong', 0.12, True, False)
    assert r(v(-0.165, 0.0)) == ('sau', 0.06, True, True) and r(v(-0.205, 0.0)) == ('sau', 0.10, True, False)


def test_03_cum_cat_o_tia_khong_ve_va_noi_qua_tia_0():
    pts = [None] * 10
    pts[2:5] = [(1.0, 0.0), (1.0, 0.01), (1.0, 0.02)]
    pts[8], pts[9], pts[0] = (2.0, 0.0), (2.0, 0.01), (2.0, 0.02)                   # qua tia 0
    assert sorted(GV.cum(pts, T)) == [(2, 4), (8, 0)]
    assert GV.cum([(1.0, 0.0)] * 4, T) == [(0, 3)]


def test_04_lidar_vat_tuong_khong_chac():
    for doan, mong in (([((0.30, -0.05), (0.30, 0.05))], 'VAT'),                         # mat hop 0,10, ho hai ben
                       ([((0.40, -0.07), (0.33, 0.0)), ((0.33, 0.0), (0.40, 0.07))], 'VAT'),   # goc hop
                       ([((-1.5, 0.13), (1.5, 0.13))], 'TUONG'),                          # tuong doc hong
                       ([((0.30, -0.16), (0.30, 0.16))], 'KHONG_CHAC')):                  # 0,32: giua hai nguong
        ds = GV.danh_gia(quet(*doan), None, None, T)
        assert ds and all(d['lidar'] == mong and d['loai'] == ('TUONG' if mong == 'TUONG' else 'VAT') for d in ds), (mong, ds)
    # vat ngan nhung bi vat gan hon che mot ben => khong chac => VAT
    ds = GV.danh_gia(quet(((0.40, -0.05), (0.40, 0.05)), ((0.30, 0.03), (0.30, 0.15))), None, None, T)
    d = [d for d in ds if d['kc'] > 0.2][0]
    assert d['lidar'] == 'KHONG_CHAC' and d['loai'] == 'VAT', ds


def test_05_nhieu_it_diem_bo():
    assert GV.danh_gia(quet(((0.30, -0.002), (0.30, 0.002))), None, None, T) == []


def test_06_ban_do_chi_nhac_LECH():
    w = h = 80
    data = [0] * (w * h)
    for j in range(h):
        data[j * w + int((0.40 + 2.0) / 0.05)] = 99                                   # o tuong ban do x 0,40..0,45
    ban = GV.BanDo(w, h, 0.05, -2.0, -2.0, data)
    assert ban.gan_tuong(0.30, 0.0, 0.18) and not ban.gan_tuong(0.10, 0.0, 0.18)
    q = quet(((0.305, -0.05), (0.305, 0.05)))
    d = GV.danh_gia(q, q, ban, T)[0]                                                    # khung ban do = khung xe
    assert (d['loai'], d['lidar'], d['bd'], d['lech']) == ('VAT', 'VAT', 'TUONG', True)
    assert GV.dong_mem(d) == 'MEM: truoc %d diem cach 0.16 m (vat) LECH ban do=tuong' % d['n']
    q = quet(((-1.5, -0.20), (-1.5, 0.20)), ((-1.5, 0.20), (1.5, 0.20)))               # tuong xa ban do => nhac VAT, LiDAR TUONG
    d = [d for d in GV.danh_gia(q, q, ban, T) if d['vung'] == 'hong'][0]
    assert (d['loai'], d['bd'], d['lech']) == ('TUONG', 'VAT', True)


def test_07_do_vat_vang_vat_khoang_sau():
    ds = GV.danh_gia(quet(((0.235, -0.05), (0.235, 0.05)), ((-1.5, -0.13), (1.5, -0.13))), None, None, T)
    assert GV.do_vat(ds)['vung'] == 'truoc' and round(GV.do_vat(ds)['kc'], 2) == 0.09
    ds = GV.danh_gia(quet(((-1.5, -0.12), (1.5, -0.12))), None, None, T)                # tuong hong 0,04: khong bao gio do
    assert GV.do_vat(ds) is None and GV.vang_vat(ds) is None
    ds = GV.danh_gia(quet(((-0.185, -0.05), (-0.185, 0.05))), None, None, T)            # vat sau 0,08: vang sau, khong giu cong
    assert ds and ds[0]['vung'] == 'sau' and GV.vang_vat(ds) is None and GV.do_vat(ds) is None
    assert round(GV.khoang_sau(quet(((-0.185, -0.05), (-0.185, 0.05))), T), 3) == 0.08
    assert GV.khoang_sau(quet(((0.30, -0.05), (0.30, 0.05))), T) is None
    assert round(GV.khoang_sau(quet(((-0.40, -1.0), (-0.40, 1.0))), T), 3) == 0.295        # tuong sau (moi vat)


def test_08_mem_chi_in_khi_doi():
    m = GV.Mem()
    d = lambda kc: GV.danh_gia(quet(((0.145 + kc, -0.05), (0.145 + kc, 0.05))), None, None, T)
    assert len(m.cap_nhat(d(0.20))) == 1 and m.cap_nhat(d(0.20)) == [] and m.cap_nhat(d(0.18)) == []
    assert m.cap_nhat(d(0.17)) == ['MEM: truoc %d diem cach 0.17 m (vat)' % d(0.17)[0]['n']]
    assert m.cap_nhat([]) == ['MEM: sach'] and m.cap_nhat([]) == []


def test_09_diem_quet_va_bien_doi():
    p = GV.diem_quet([0.5, float('inf'), 0.05, 20.0], math.pi / 2, 0.1, 0.1, 12.0)
    assert p[1:] == [None, None, None] and abs(p[0][0]) < 1e-12 and abs(p[0][1] - 0.5) < 1e-12
    q = GV.bien_doi([(1.0, 0.0), None], (1.0, 2.0, math.pi / 2))
    assert abs(q[0][0] - 1.0) < 1e-12 and abs(q[0][1] - 3.0) < 1e-12 and q[1] is None


def test_10_ban_do_that_xuong_v3_xe_trong_khe_tuong_khong_lam_dung():
    """Ban do goc v3 (du_lieu_0111): xe giua khe 1 (1,00; 0,085) nhin 360 do bang tia gia lap toi tuong => khong VAT trong do."""
    import trang_v10 as V
    w, h, hang = V.doc_png(os.path.join(DAY, 'du_lieu_0111', 'map_xuong_v3.png'))
    data = []
    for j in range(h):                                                # OccupancyGrid: hang 0 o DUOI
        for v in hang[h - 1 - j]:
            p = (255 - v) / 255.0
            data.append(100 if p > 0.65 else 0 if p < 0.25 else -1)
    ban = GV.BanDo(w, h, 0.05, -0.566, -0.415, data)
    xe = (1.00, 0.085, 0.0)
    pts_map = []
    for k in range(420):                                              # tia toi o tuong dau tien (buoc 1 cm)
        a = -math.pi + 2 * math.pi * k / 420
        hit = None
        for s in range(10, 300):
            x, y = xe[0] + GV.X_LIDAR + s / 100.0 * math.cos(a), xe[1] + s / 100.0 * math.sin(a)
            g = ban.gia(int((x + 0.566) / 0.05), int((y + 0.415) / 0.05))
            if g is None or g >= 65:
                hit = (x, y)
                break
        pts_map.append(hit)
    pts_xe = GV.bien_doi(pts_map, (-xe[0], -xe[1], 0.0))
    ds = GV.danh_gia(pts_xe, pts_map, ban, T)
    assert sum(p is not None for p in pts_map) > 300 and GV.do_vat(ds) is None, [GV.dong_mem(d) for d in ds]
