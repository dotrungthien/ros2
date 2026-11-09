#!/usr/bin/env python3
"""test_lop_huong_dan.py (bo 1e "mat lai", 08/11/2026): phan thuan cua lop_huong_dan.py, KHONG can ROS.
Chay: python3 xe_scripts/test_lop_huong_dan.py   hoac   python3 -m pytest xe_scripts/test_lop_huong_dan.py -q
KHONG chung minh chay tren xe: chi kiem hinh hoc luoi, o cam, vung ton diem, doc kinh nghiem."""
import math
import os
import tempfile

import lop_huong_dan as LH
import tinh_tuyen as TT
import trang_v10 as V

DAY = os.path.dirname(os.path.abspath(__file__))
YAML_V3 = os.path.join(DAY, 'du_lieu_0111', 'map_xuong_v3.yaml')
HH = LH.hinh_hoc_ban_do(YAML_V3)


def tam(hh, k):
    i, j = k % hh['w'], k // hh['w']
    return hh['ox'] + (i + 0.5) * hh['res'], hh['oy'] + (j + 0.5) * hh['res']


def test_01_hinh_hoc_trung_ban_do_goc():
    """3b #6: kich thuoc + goc + do phan giai cua luoi = cua ban do goc map_xuong_v3.yaml (doc doc lap bang trang_v10)."""
    assert (HH['w'], HH['h'], HH['res'], HH['ox'], HH['oy']) == (52, 32, 0.05, -0.566, -0.415)
    w, h, res, ox, oy, occ, anh = V.doc_ban_do(YAML_V3)              # doc ca anh PNG (giai nen), khac duong voi header IHDR
    assert (w, h, res, ox, oy) == (HH['w'], HH['h'], HH['res'], HH['ox'], HH['oy'])
    assert os.path.samefile(anh, HH['anh'])
    assert len(LH.luoi(HH, None, [])) == w * h


def test_02_phat_ngay_toan_0():
    assert set(LH.luoi(HH, None, [])) == {0}
    assert LH.doc_chon_tuyen('0') is None


def test_03_chon_tuyen_cam_khe_kia_tru_dia_quanh_xe():
    for r in (1, 2):
        xe = (0.35, 0.60) if r == 1 else (0.35, 0.30)                 # xe dung TRONG vung cam cua tuyen r
        d = LH.luoi(HH, LH.doc_chon_tuyen('%d %.2f %.2f' % ((r,) + xe)), [])
        x0, x1, y0, y1 = TT.CAM[r]
        cam = trong = 0
        for k, v in enumerate(d):
            cx, cy = tam(HH, k)
            trong_cam = x0 <= cx <= x1 and y0 <= cy <= y1
            gan_xe = math.hypot(cx - xe[0], cy - xe[1]) <= LH.R_THA
            assert v == (LH.GIA_CAM if trong_cam and not gan_xe else 0), (r, cx, cy, v)
            cam += v == LH.GIA_CAM
            trong += trong_cam and gan_xe
        assert cam > 50 and trong > 0, (r, cam, trong)


def test_04_chon_tuyen_hong():
    for s in ('', '3 0 0', '1 0', '1 a b', '2 0.1 0.2 9', 'x'):
        try:
            LH.doc_chon_tuyen(s)
        except ValueError:
            continue
        raise AssertionError('khong bao loi: %r' % s)
    assert LH.doc_chon_tuyen(' 2 0.10 -0.20 ') == (2, 0.10, -0.20)


def test_05_vung_ton_diem_cum_3_dong():
    ba0 = [(0.70, 0.80), (0.72, 0.81), (0.71, 0.79)]
    assert LH.GIA_TON == 0 and set(LH.luoi(HH, None, ba0)) == {0}                   # 09/11 gia X: GIA_TON 0 => khong to vung ton
    goc = LH.GIA_TON                                                                    # ma van giu: thu lai voi gia cu 50
    LH.GIA_TON = 50
    try:
        _vung_ton_50()
    finally:
        LH.GIA_TON = goc


def _vung_ton_50():
    hai = [(0.70, 0.80), (0.72, 0.81)]
    assert LH.tam_vung_ton(hai) == [] and set(LH.luoi(HH, None, hai)) == {0}            # 2 dong: chua thanh cum
    ba = hai + [(0.71, 0.79), (1.50, 0.10)]                                            # 3 dong trong 0,20 m + 1 dong le
    t = LH.tam_vung_ton(ba)
    assert len(t) == 1 and abs(t[0][0] - 0.71) < 1e-6 and abs(t[0][1] - 0.80) < 1e-6, t
    d = LH.luoi(HH, None, ba)
    assert set(d) == {0, LH.GIA_TON} and max(d) < TT.NGUONG_CAM                         # khong bao gio thanh cam (< 72)
    for k, v in enumerate(d):
        cx, cy = tam(HH, k)
        assert (v == LH.GIA_TON) == (math.hypot(cx - 0.71, cy - 0.80) <= LH.R_VUNG)


def test_06_cam_khong_bi_ha_boi_vung_ton():
    ba = [(0.70, 0.80), (0.72, 0.81), (0.71, 0.79)]
    d = LH.luoi(HH, (1, 0.0, 0.0), ba)                                                   # tuyen 1 cam phia tren (gom 0,71; 0,80)
    gia = {v for k, v in enumerate(d) if math.hypot(tam(HH, k)[0] - 0.71, tam(HH, k)[1] - 0.80) <= LH.R_VUNG}
    assert gia == {LH.GIA_CAM}, gia


def test_07_doc_kinh_nghiem_bo_dong_hong():
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'kinh_nghiem.csv')
    assert LH.doc_kinh_nghiem(p) == []                                                   # khong co tep
    with open(p, 'w') as f:
        f.write('gio,x,y,huong,tuyen,ly_do\n2026-11-08 20:00:00,0.70,0.80,di_B,1,KET_CO_DUONG\nhong\n,,,\n'
                'g,nan,0.1,di_B,1,KET_CO_DUONG\n2026-11-08 20:01:00,0.71,0.80,ve_A,2,KET_CO_DUONG\n'
                '2026-11-08 20:02:00,0.30,0.10,di_B,1,VAT_CHAN\n2026-11-08 20:03:00,0.31,0.10,di_B,1,CHO_VAT_30s\n')
    assert LH.doc_kinh_nghiem(p) == [(0.70, 0.80), (0.71, 0.80)]                         # 09/11 #5: chi KET_CO_DUONG
    assert LH.doc_kinh_nghiem(p, ly_do=('VAT_CHAN',)) == [(0.30, 0.10)]


def test_08_kich_thuoc_anh_png_pgm():
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'a.pgm')
    with open(p, 'wb') as f:
        f.write(b'P5\n# chu thich\n7 5\n255\n' + bytes(35))
    assert LH.kich_thuoc_anh(p) == (7, 5)
    assert LH.kich_thuoc_anh(HH['anh']) == (52, 32)
    q = os.path.join(d, 'b.txt')
    with open(q, 'w') as f:
        f.write('khong phai anh')
    try:
        LH.kich_thuoc_anh(q)
    except ValueError:
        return
    raise AssertionError('khong bao loi anh hong')


def test_09_khop_hinh_hoc_cho_kiem_xe():
    from types import SimpleNamespace as N
    info = N(width=52, height=32, resolution=0.05, origin=N(position=N(x=-0.566, y=-0.415)))
    assert LH.khop_hinh_hoc(info, HH)[0]
    info.width = 60                                                                        # SLAM /map kich thuoc khac
    assert not LH.khop_hinh_hoc(info, HH)[0]


def test_10_tom_tat_mot_dong():
    d = LH.luoi(HH, (2, 0.0, 0.0), [])
    t = LH.tom_tat(d, (2, 0.0, 0.0), 0, 0)
    assert t.startswith('PHAT /lop_huong_dan: chon=tuyen 2 tu (0.00; 0.00) o_cam=') and '\n' not in t


if __name__ == '__main__':
    import sys
    import traceback
    ca = sorted((k, f) for k, f in globals().items() if k.startswith('test_') and callable(f))
    dat = 0
    for k, f in ca:
        try:
            f()
            dat += 1
            print('PASS ', k)
        except Exception:
            print('FAIL ', k)
            traceback.print_exc()
    print('TONG: %d/%d dat' % (dat, len(ca)))
    sys.exit(0 if dat == len(ca) else 1)
