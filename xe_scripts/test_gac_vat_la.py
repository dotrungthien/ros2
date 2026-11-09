#!/usr/bin/env python3
"""test_gac_vat_la.py (dung im ban 2, A1 + phan A5 cua gac): python3 -m pytest -q test_gac_vat_la.py"""
import math
import os
import sys

DAY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DAY)
import gac_vat_la as GV  # noqa: E402

T = GV.tham_so()
RES = 0.05


def ban_trong(tuong=()):
    """Ban do 4 x 4 m o 0,05 (goc (-2; -2)), tat ca o trong; tuong = [(x, y)] toa do o tuong."""
    w = h = 80
    data = [0] * (w * h)
    for x, y in tuong:
        data[int((y + 2) / RES) * w + int((x + 2) / RES)] = 100
    return GV.BanDo(w, h, RES, -2.0, -2.0, data)


def cum(x, y, n=3, d=0.01):
    """n diem LiDAR quanh (x, y) (khung xe = khung ban do: xe o goc, huong 0)."""
    return [(x, y + k * d) for k in range(n)]


def gac1(pts, ban, t=0.0, g=None):
    g = g or GV.Gac(T)
    return g, g.cap_nhat(t, pts, pts, ban)


def test_01_so_gac_tu_hanh_vi():
    assert (T['truoc'], T['hong'], T['ra_truoc'], T['ra_hong'], T['le_tuong'], T['so_diem'], T['sach_s'], T['le_rong']) == \
        (0.30, 0.10, 0.40, 0.15, 0.10, 3, 2.0, 0.06) and T['bat'] is True and T['scan_cu_s'] == 0.5


def test_02_vung_gac_hinh_hoc():
    v = lambda x, y, ra=False: GV.vung_cua(x, y, T, ra)
    assert v(0.146, 0.0) == 'truoc' and v(0.444, 0.0) == 'truoc' and v(0.446, 0.0) is None          # mui 0,145 + 0,30
    assert v(0.30, 0.14) == 'truoc' and v(0.30, 0.141) is None                                     # rong 0,28
    assert v(0.0, 0.081) == 'trai' and v(0.0, 0.18) == 'trai' and v(0.0, 0.181) is None            # hong 0,10
    assert v(-0.105, -0.12) == 'phai' and v(-0.11, -0.12) is None                                  # sau: khong gac
    assert v(0.0, 0.0) is None                                                                     # trong than
    assert v(0.54, 0.0, True) == 'truoc' and v(0.0, 0.229, True) == 'trai' and v(0.30, 0.19, True) == 'truoc'   # vung RA rong hon


def test_03_vat_la_truoc_trai_phai_dung():
    ban = ban_trong()
    for (x, y), mong in (((0.35, -0.01), 'truoc'), ((0.0, 0.13), 'trai'), ((0.05, -0.15), 'phai')):
        _, r = gac1(cum(x, y), ban)
        assert r['chan'] and r['moi'] and r['vung'] == mong and r['n'] == 3, (mong, r)


def test_04_hai_diem_khong_dung_ba_diem_dung():
    ban = ban_trong()
    assert not gac1(cum(0.35, 0.0, n=2), ban)[1]['chan']
    assert gac1(cum(0.35, 0.0, n=3), ban)[1]['chan']
    assert not gac1(cum(0.35, 0.0, n=2) + cum(0.0, 0.13, n=2), ban)[1]['chan']     # 2 + 2 o hai vung: moi vung chua du


def test_05_tuong_sat_hong_trong_le_khong_dung():
    # tuong (o 0,05) doc canh trai, mep o cach than 0,02 m; diem LiDAR tren mat tuong / lech dinh vi 7 cm van trong le 0,10
    tuong = [(x / 100.0, 0.10) for x in range(-15, 20, 5)]
    ban = ban_trong(tuong)
    for y in (0.10, 0.11, 0.17):                                    # tren tuong; lech 1 cm; lech 7 cm (vao o trong canh tuong)
        assert not gac1(cum(0.0, y, n=5, d=0.0) + [(0.05, y), (-0.05, y)], ban)[1]['chan'], y
    assert not ban.la_vat_la(0.0, 0.24, 0.10) and ban.la_vat_la(0.0, 0.26, 0.10)   # mep tren o tuong 0,15: le 0,10 => 0,25


def test_06_o_chua_biet_va_ngoai_ban_do_khong_phai_vat_la():
    ban = ban_trong()
    ban.data[int((0 + 2) / RES) * ban.w + int((0.35 + 2) / RES)] = -1
    assert not ban.la_vat_la(0.36, 0.01, 0.10) and not ban.la_vat_la(5.0, 0.0, 0.10)
    assert not gac1(cum(0.36, 0.01), ban)[1]['chan']


def test_07_mep_vung_khong_nhap_nhay_ra_khi_sach_2s():
    """K2: vat dao 0,28 <-> 0,32 m truoc mui (quanh mep vao 0,30): vao 1 lan, KHONG ra; ra chi khi > 0,40 sach 2 s lien tuc."""
    ban = ban_trong()
    g, n_vao, n_ra = GV.Gac(T), 0, 0
    for k in range(100):                                              # 10 s, 10 Hz
        d = 0.28 if k % 2 else 0.32
        r = g.cap_nhat(k * 0.1, cum(0.145 + d, 0.0), cum(0.145 + d, 0.0), ban)
        n_vao += r['moi']
        n_ra += r['het']
    assert n_vao == 1 and n_ra == 0 and r['chan']
    t0 = 10.0                                                         # vat lui ra 0,42 m (ngoai vung RA 0,40): dem 2 s
    for k in range(19):
        r = g.cap_nhat(t0 + k * 0.1, cum(0.145 + 0.42, 0.0), cum(0.145 + 0.42, 0.0), ban)
        assert r['chan'] and not r['het'], k
    r = g.cap_nhat(t0 + 1.95, cum(0.145 + 0.38, 0.0), cum(0.145 + 0.38, 0.0), ban)    # quay lai 0,38 (trong vung RA): dat lai
    assert r['chan'] and r['sach_s'] == 0.0
    t1 = t0 + 2.0
    for k in range(20):
        r = g.cap_nhat(t1 + k * 0.1, [], [], ban)
        assert not r['het']
    r = g.cap_nhat(t1 + 2.0, [], [], ban)
    assert r['het'] and not r['chan']
    r = g.cap_nhat(t1 + 2.1, cum(0.145 + 0.25, 0.0), cum(0.145 + 0.25, 0.0), ban)
    assert r['moi'] and r['chan']                                     # vao lai duoc


def test_08_tu_the_luc_quet_khong_bien_tuong_thanh_vat_la():
    """K7: xe chay 0,11 m/s; tu the cu 1 s lech 0,11 m => diem tuong truoc mat thanh vat la; tu the luc quet thi khong."""
    tuong = [(0.70, y / 100.0) for y in range(-30, 35, 5)]
    ban = ban_trong(tuong)
    xe_luc_quet = (0.30, 0.0, 0.0)                                    # xe that luc quet: mui 0,445, tuong 0,70 (cach mui 0,255)
    pts_xe = [(0.40, -0.02), (0.40, 0.0), (0.40, 0.02)]               # mat tuong 0,70 trong khung xe (x = 0,70 - 0,30)
    dung = GV.danh_gia(pts_xe, GV.bien_doi(pts_xe, xe_luc_quet), ban, T)
    cu = GV.danh_gia(pts_xe, GV.bien_doi(pts_xe, (0.30 - 0.11 * 1.0 - 0.0, 0.0, 0.0)), ban, T)
    assert dung is None and cu is not None and cu[0] == 'truoc'


def test_09_diem_quet_va_bien_doi():
    p = GV.diem_quet([0.05, 0.5, float('inf'), float('nan'), 13.0], 0.0, math.pi / 2, 0.10, 12.0)
    assert len(p) == 1 and abs(p[0][0]) < 1e-12 and abs(p[0][1] - 0.5) < 1e-12
    q = GV.bien_doi([(1.0, 0.0)], (1.0, 2.0, math.pi / 2))
    assert abs(q[0][0] - 1.0) < 1e-12 and abs(q[0][1] - 3.0) < 1e-12


def test_10_ban_do_that_xuong_v3_xe_trong_khe_khong_bao_vat_la():
    """Ban do goc v3 (du_lieu_0111): xe giua khe 1 (1,00; 0,085) nhin 360 do bang tia gia lap toi tuong => 0 vat la."""
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
    for k in range(360):                                              # tia toi o tuong dau tien (buoc 1 cm)
        a = math.radians(k)
        for s in range(10, 300):
            x, y = xe[0] + s / 100.0 * math.cos(a), xe[1] + s / 100.0 * math.sin(a)
            g = ban.gia(int((x + 0.566) / 0.05), int((y + 0.415) / 0.05))
            if g is None or g >= 65:
                pts_map.append((x, y))
                break
    c, s_ = math.cos(-xe[2]), math.sin(-xe[2])
    pts_xe = [((x - xe[0]) * c - (y - xe[1]) * s_, (x - xe[0]) * s_ + (y - xe[1]) * c) for x, y in pts_map]
    assert len(pts_map) > 300 and GV.danh_gia(pts_xe, pts_map, ban, T) is None
