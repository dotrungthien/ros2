#!/usr/bin/env python3
"""test_sep_lidar.py (09/11 23:08 sep LiDAR mot nguoi lai): python3 -m pytest -q test_sep_lidar.py. Phan thuan, khong ROS.
LiDAR gia = the_gioi_gia.quet (doan khung ban do, xe dat o tu the). KHONG chung minh chay tren xe."""
import json
import math
import os
import sys

DAY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DAY)
import gac_vat_la as GV  # noqa: E402
import sep_lidar as SL  # noqa: E402
import the_gioi_gia as TG  # noqa: E402

T = SL.tham_so()
P0 = (0.0, 0.0, 0.0)


def _pl(doan, p=P0):
    return SL.diem_loc(TG.quet(doan, p), T['gac'])


def _ds(doan, p=P0):
    return GV.danh_gia(TG.quet(doan, p), None, None, T['gac'])


TUONG_PHAI = lambda g: [((-1.0, -0.08 - g), (1.0, -0.08 - g))]          # tuong doc hong phai cach than g
TUONG_SAU = lambda g: [((-0.105 - g, -0.5), (-0.105 - g, 0.5))]          # tuong ngang sau duoi cach g
TUONG_TRUOC = lambda g: [((0.145 + g, -0.5), (0.145 + g, 0.5))]
HOP = lambda x, y, r=0.06: TG.doan_hop(x, y, r)                         # vat that nho (ho hai ben)


def test_01_so_lay_tu_hanh_vi():
    assert (T['hong_do'], T['truoc_do'], T['mui_do'], T['cho_chat'], T['nhac_doi']) == (0.06, 0.10, 0.07, 0.12, 0.14)
    assert abs(T['r_xoay'] - 0.186) < 1e-9 and abs(T['sector'] - math.radians(10)) < 1e-12
    assert (T['trong'], T['r_min'], T['r_max']) == (0.40, 0.30, 0.40)


def test_02_khe_theo_da_giac_than_va_vung():
    assert SL.kc_than((0.20, 0.0))[0] == 'truoc' and abs(SL.kc_than((0.20, 0.0))[1] - 0.055) < 1e-9 and SL.kc_than((0.0, -0.11))[0] == 'hong'
    assert abs(SL.kc_than((0.0, -0.11))[1] - 0.03) < 1e-9 and SL.kc_than((0.0, 0.0)) == ('hong', 0.0)
    assert abs(SL.kc_than((0.185, 0.12))[1] - math.hypot(0.04, 0.04)) < 1e-9          # cheo: toi goc than
    k = SL.khe(_pl(TUONG_PHAI(0.05) + TUONG_TRUOC(0.20)), T)
    assert abs(k['hong'] - 0.05) < 0.003 and abs(k['hl'] - 0.20) < 0.003 and abs(k['sau'] - 0.05) < 0.003, k     # tuong dai: cheo sau duoi
    assert abs(k['hs'] - min(k['hong'], k['cheo'])) < 1e-12 and abs(k['nho'] - 0.05) < 0.003


def test_03_cham_diem_doi_vung_khe_chung_hong_duoi():
    """Ca 21:50: tuong hong cheo 25 do (gan duoi): tien => diem hong thanh diem duoi, khe tang => kha thi; lui => khe giam => bo."""
    tg = math.tan(math.radians(25))
    cheo = [((-0.4, -0.11 + 0.295 * tg), (0.6, -0.11 - 0.705 * tg))]
    pl = _pl(cheo)
    k0 = SL.khe(pl, T)
    assert abs(k0['hong'] - 0.03) < 0.003, k0
    c = SL.cham(pl, ('tien', 0.15), T)
    assert c['kha_thi'] and c['cuoi']['hs'] > k0['hs'], c
    c = SL.cham(pl, ('lui', 0.05), T)
    assert not c['kha_thi'] and c['ly'].startswith('khe hong+duoi'), c


def test_04_cham_truoc_va_xoay():
    pl = _pl(TUONG_TRUOC(0.12))
    assert not SL.cham(pl, ('tien', 0.05), T)['kha_thi']                                # truoc vao <= 0,10 / khe cheo giam => bo
    assert SL.cham(_pl(TUONG_TRUOC(0.08)), ('lui', 0.05), T)['kha_thi']                # dang <= 0,10 thi lui (ra xa) duoc
    c = SL.cham(_pl(TUONG_PHAI(0.08)), ('xoay', math.radians(30)), T)
    assert not c['kha_thi'] and c['ly'].startswith('xoay: diem cach tam'), c          # 0,16 < 0,186: khong xoay
    assert SL.cham(_pl(TUONG_PHAI(0.20)), ('xoay', math.radians(30)), T)['kha_thi']


def test_05_chon_dong_tac_giam_thieu_vung_do_truoc_roi_khe():
    """Goc kin ba phia (truoc 0,05 VAT, hong 0,07): lui toi khi truoc > 0,10 (thieu do = 0); kep moi huong => None."""
    tui = [((-0.3, 0.15), (0.195, 0.15)), ((-0.3, -0.15), (0.195, -0.15)), ((0.195, -0.15), (0.195, 0.15))]
    dt, c, _ = SL.chon_dong_tac(_pl(tui), T, None)
    assert dt == ('lui', 0.10) and SL.thieu(c['cuoi'], T) == 0.0, (dt, c)
    kep = HOP(0.20, 0.0, 0.10) + HOP(-0.16, 0.0, 0.10) + TUONG_PHAI(0.05) + [((-1.0, 0.13), (1.0, 0.13))]
    dt, c, bo = SL.chon_dong_tac(_pl(kep), T, None)
    assert dt is None and len(bo) == len(SL.DONG_TAC), bo


def test_06_kinh_nghiem_thu_truoc_neu_kha_thi():
    pl = _pl(TUONG_SAU(0.03) + TUONG_PHAI(0.20))
    assert SL.chon_dong_tac(pl, T, 0.0, ('tien', 0.05))[0] == ('tien', 0.05)            # kha thi + tot len: thu truoc
    dt, _, bo = SL.chon_dong_tac(pl, T, 0.0, ('lui', 0.10))
    assert dt[0] == 'tien' and bo[0][0] == 'lui=0.10' and bo[0][1].startswith('khe hong+duoi'), bo
    assert SL.ten_dong_tac(('xoay', math.radians(-30))) == 'xoay=-30' and SL.doc_ten('lui=0.35') == ('lui', 0.35)
    assert SL.doc_ten('cho=30') is None and SL.doc_ten(None) is None


def test_07_sector_gan_huong_dich_va_costmap():
    """Cho thoang: sector 10 do gan huong dich nhat co trong >= 0,40; o costmap >= 253 => thu gan hon / sector ke."""
    pl = _pl(HOP(0.45, 0.0, 0.10))                                       # vat ngay huong dich (0 do)
    phi, r, (x, y) = SL.chon_sector(pl, T, 0.0)
    assert abs(phi) >= math.radians(10) - 1e-9 and abs(phi) <= math.radians(40) and 0.30 <= r <= 0.40, (phi, r)
    phi2, _, _ = SL.chon_sector(pl, T, 0.0, kiem_o=lambda a, b: not (b * phi > 0))      # phia do bi costmap cam => phia kia
    assert phi2 * phi < 0
    assert SL.chon_sector(_pl(TG.doan_hop(0.0, 0.0, 0.70)), T, 0.0) is None              # hop kin 0,70: khong sector


def test_08_huy_dong_tac_moi_dieu_kien_Q3():
    """Thien Q3: huy khi (a) truoc <= 0,07 moi vat (ke ca tuong); (b) truoc <= 0,10 vat; (c) hong <= 0,06 vat; (d) khe hong / duoi nho
    hon luc bat dau. Da vi pham ngay luc bat dau => chi huy khi nho hon luc bat dau."""
    k0 = SL.tom_tat(_ds([]))
    a = SL.ly_do_huy(SL.tom_tat(_ds(TUONG_TRUOC(0.06))), k0, T)
    assert a == 'truoc 0.06 <= 0.07 (moi vat)', a
    assert SL.ly_do_huy(SL.tom_tat(_ds(TUONG_TRUOC(0.09))), k0, T) is None               # tuong 0,09: khong phai vat
    b = SL.ly_do_huy(SL.tom_tat(_ds(HOP(0.145 + 0.09 + 0.05, 0.0, 0.10))), k0, T)
    assert b == 'truoc vat 0.09 <= 0.10', b
    c = SL.ly_do_huy(SL.tom_tat(_ds(HOP(0.0, -0.08 - 0.05 - 0.05, 0.10))), k0, T)
    assert c == 'hong vat 0.05 <= 0.06', c
    k0 = SL.tom_tat(_ds(TUONG_PHAI(0.10)))
    d = SL.ly_do_huy(SL.tom_tat(_ds(TUONG_PHAI(0.09))), k0, T)
    assert d == 'khe hong+duoi 0.09 < luc bat dau 0.100', d
    assert SL.ly_do_huy(SL.tom_tat(_ds(TUONG_PHAI(0.09))), k0, T, xet_khe=False) is None
    k0 = SL.tom_tat(_ds(HOP(0.145 + 0.08 + 0.05, 0.0, 0.10)))                            # luc dau vat truoc 0,08
    assert SL.ly_do_huy(SL.tom_tat(_ds(HOP(0.145 + 0.09 + 0.05, 0.0, 0.10))), k0, T) is None     # ra xa: khong huy
    assert SL.ly_do_huy(SL.tom_tat(_ds(HOP(0.145 + 0.075 + 0.05, 0.0, 0.10))), k0, T).startswith('truoc vat 0.07')


def test_09_nhac_doi_chi_vat_chac_khong_LECH():
    """Q4: vat chac = LiDAR VAT (cum ho hai ben) va khong LECH, trong hanh lang truoc mui <= 0,14. Khong chac / LECH / ngoai 0,14 /
    ngoai hanh lang: khong nhac doi. Khong co ban do: khong LECH => van nhac doi."""
    ds = _ds(HOP(0.145 + 0.13 + 0.05, 0.0, 0.10))
    assert SL.vat_chac(ds, T) is not None and abs(ds[0]['hl'] - 0.13) < 0.003
    assert SL.vat_chac(_ds(HOP(0.145 + 0.16 + 0.05, 0.0, 0.10)), T) is None
    assert SL.vat_chac(_ds(HOP(0.145 + 0.10 + 0.05, 0.20, 0.10)), T) is None                # cheo, ngoai hanh lang than
    d = [dict(x, lech=True) for x in ds]
    assert SL.vat_chac(d, T) is None                                                       # LECH => coi la tuong
    d = [dict(x, lidar='KHONG_CHAC') for x in ds]
    assert SL.vat_chac(d, T) is None                                                       # khong chac: chi vung do
    assert SL.vung_do(_ds(HOP(0.145 + 0.09 + 0.05, 0.0, 0.10)), T) is not None


def test_10_tin_sep_chi_la_tin():
    j = lambda **k: json.dumps(dict(dict(ra_lenh=False, di='DI', di_ly='', muc_tin='TIN', nav2_song='SONG'), **k))
    assert SL.doc_tin_sep(j(), 0.5, 3.0) == dict(tin=True, muc_tin='TIN', di='DI', ly_do=None, nav2_song='SONG')
    for mt, tin in (('KHONG_TIN', False), ('BAO', False), ('NGHI', True), ('TIN', True)):
        assert SL.doc_tin_sep(j(muc_tin=mt), 0.5, 3.0)['tin'] is tin
    assert SL.doc_tin_sep(j(di='DUNG', di_ly='KHONG_TIN_QUA_15s'), 0.5, 3.0)['ly_do'] == 'KHONG_TIN_QUA_15s'
    assert SL.doc_tin_sep(j(), 3.5, 3.0) is None and SL.doc_tin_sep('{hong', 0.1, 3.0) is None
    assert SL.doc_tin_sep(None, None, 3.0) is None and SL.doc_tin_sep('[1]', 0.1, 3.0) is None


def test_11_khong_phat_toc_do():
    """Rang buoc cung: sep_lidar / gui_duong khong phat cmd_vel (moi dong tac la action Nav2)."""
    for ten in ('sep_lidar.py', 'gui_duong.py'):
        s = open(os.path.join(DAY, ten)).read()
        assert 'Twist' not in s and "'/cmd_vel" not in s and "'cmd_vel" not in s and 'create_publisher(Twist' not in s, ten
