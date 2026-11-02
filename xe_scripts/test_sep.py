#!/usr/bin/env python3
"""Thu sep.py (buoc 1, chi nhin) KHONG can ROS: python3 -m pytest -q test_sep.py
Xuong, LiDAR va ban do lay tu mo_phong_he_thong.py (gan dung +-5-10 cm). Ca 1-7 = 7 dau hieu cua muc 5 THIET_KE_SEP.md.
"""
import math
import os
import re

import lai_xe_lidar as L
import mo_phong_he_thong as M
import sep as S

A = (0.0, 0.0, 0.0)
CRIT = 'CRITICAL FAILURE: SERVER controller_server IS DOWN after not receiving a heartbeat for 4000 ms. Shutting down related nodes.'
DAY = os.path.dirname(os.path.abspath(__file__))


def quet(pose, them=()):
    return L.scan_to_xy(M.make_scan(M.XUONG + list(them), pose))


def vao(pose=A, them=(), **kw):
    """Thong tin binh thuong cho Sep.vong: scan moi, tf moi, ban do xuong, map->odom dung yen."""
    v = dict(pts=quet(pose, them), scan_tuoi=0.05, pose=pose, tf_tuoi=0.1, mo=(0.0, 0.0, 0.0), grid=M.GRID, ket=False, gio='t')
    v.update(kw)
    return v


class Chay:
    """Goi Sep.vong lien tiep, moi lan cach 0,1 s."""

    def __init__(self, v):
        self.s, self.v, self.t = S.Sep(), v, 0.0

    def __call__(self, **kw):
        self.t += 0.1
        return self.s.vong(self.t, dict(self.v, **kw))


def co(log, chu):
    return any(chu in d for d in log)


def test_01_plan_lech_va_0_poses():
    d, vuot = S.lech_dau_plan((0.07, 0.14), (1.17, 0.64, 0.0))       # loi 30/10: ~1,1 m
    assert vuot and d > 1.0
    assert S.lech_dau_plan((1.0, 0.0), (1.1, 0.0, 0.0))[1] is False
    c = Chay(vao())
    c()
    c(goals=[[(b'g0', 2)]])                                           # 02/11 17:33: PLAN_LECH chi xet khi co dich dang chay
    tt, log = c(plans=[[(1.20, 0.60), (0.0, 0.0)]])
    assert co(log, 'PLAN_LECH') and tt['tu_the'] == 'KHONG_DI'
    tt, log = c(plans=[[(0.02, 0.0), (1.0, 0.0)]])
    assert co(log, 'PLAN_SAI') and tt['tu_the'] != 'KHONG_DI'
    tt, log = c(plans=[[]])                                           # /plan rong (0 poses)
    assert co(log, 'PLAN_RONG') and tt['tu_the'] == 'KHONG_DI'
    c(goals=[[(b'g1', 2)]])
    for _ in range(4):
        c()
    _, log = c(goals=[[(b'g1', 6)]])                                  # ABORTED sau 0,5 s
    assert co(log, 'ABORT_NHANH')


def test_02_tf_cu():
    assert S.tf_cu(6.0) and S.tf_cu(None) and not S.tf_cu(4.0)
    assert S.chon_tu_the(dict(tf_cu=True, goal=True)) == ('CHO', 'TF_CU')
    c = Chay(vao(tf_tuoi=6.0))
    tt, log = c()
    assert co(log, 'TF_CU') and tt['tu_the'] == 'CHO'


def test_03_ket_that_va_vi_tri_sai():
    assert S.kiem_ket(True, False, 0.005, 0.0) == 'KET_THAT'           # lai_xe_lidar bao ket, LiDAR khong doi
    assert S.kiem_ket(False, True, 0.10, 0.005) == 'VI_TRI_SAI'        # /ket bat, LiDAR doi, vi tri khong doi
    assert S.kiem_ket(False, False, 0.0, 0.0) is None
    assert S.kiem_ket(False, True, 0.10, 0.10) is None                 # xe dang di that
    c = Chay(vao())
    for _ in range(25):
        tt, _ = c(ket=True)
    assert tt['ket'] == 'KET_THAT' and tt['tu_the'] == 'THOAT'


def test_04_succeeded_gia():
    assert S.xet_succeeded((1.0, 0.0), (0.82, 0.0), 5.0, 1) == ('GUI_LAI', 18)
    assert S.xet_succeeded((1.0, 0.0), (0.82, 0.0), 5.0, 2) == ('COI_LA_TOI', 18)
    assert S.xet_succeeded((1.0, 0.0), (0.75, 0.0), 5.0, 1)[0] == 'GUI_LAI'
    assert S.xet_succeeded((1.0, 0.0), (0.75, 0.0), 5.0, 2)[0] == 'NAV2_GIA'
    assert S.xet_succeeded((1.0, 0.0), (1.0, 0.0), 0.03, 1)[0] == 'NAV2_GIA'
    assert S.xet_succeeded((1.0, 0.0), (0.9, 0.0), 5.0, 1) == ('DAT', 10)
    c = Chay(vao())
    _, log = c(goals=[[(b'cu', 4), (b'g2', 2)]])                      # dich 'cu' xong truoc khi sep chay: bo qua
    assert not co(log, 'SUCCEEDED')
    c(plans=[[(0.0, 0.0), (0.5, 0.0)]])
    _, log = c(goals=[[(b'g2', 4)]])                                  # dich = diem cuoi /plan, xe o A: lech 50 cm
    assert co(log, 'SUCCEEDED_GUI_LAI')


def test_05_nhay_vi_tri():
    assert S.nhay((0, 0, 0), (0.12, 0, 0), 0.5)[2]
    assert not S.nhay((0, 0, 0), (0.12, 0, 0), 3.0)[2]                # troi cham, khong phai nhay
    assert S.nhay((0, 0, 0), (0, 0, math.radians(6)), 0.5)[2]
    d_m, d_deg, la = S.nhay((0, 0, 0), (0, 0, math.radians(3)), 0.5, p_odom=(2.0, 0.0))
    assert la and d_m > 0.10 and d_deg < 5                             # xoay 3 do nhung xe cach goc odom 2 m
    c = Chay(vao())
    c()
    tt, log = c(mo=(0.12, 0.0, 0.0))
    assert co(log, 'NHAY_VI_TRI') and tt['tu_the'] == 'CHO'


def test_06_nav2_mat_nhip_va_dem_theo_chuyen():
    assert S.doc_rosout('lifecycle_manager_navigation', CRIT)[0] == 'NAV2_CRITICAL'
    assert S.doc_rosout('lifecycle_manager_navigation', 'SERVER controller_server IS DOWN')[0] == 'NAV2_CRITICAL'
    assert S.nav2_song(True, True)[0] == 'MAT_NHIP'
    assert S.nav2_song(False, True)[0] == 'SONG'
    assert S.nav2_song(False, False)[0] == 'KHONG_BIET'
    c = Chay(vao())
    c(csv=['2026-11-03 10:00:00,ve_A_tt,1,1,0,KIEM_DAT,0,0.02,-,-'])
    assert c.s.chuyen == 've_A_tt'
    for lan in (1, 2, 3):
        tt, _ = c(rosout=[('lifecycle_manager_navigation', CRIT)])
        assert c.s.nav2_lan == lan
        if lan < 3:
            assert tt['tu_the'] == 'CHO'
            c(rosout=[('lifecycle_manager_navigation', 'Managed nodes are active')])
    assert tt['tu_the'] == 'DUNG_VA_BAO'
    _, log = c(csv=['2026-11-03 10:05:00,ve_A_tt,1,tong,0,DEN_NOI,300,0.02,-,-'])
    assert c.s.chuyen is None and co(log, 'KET_THUC')
    c(csv=['2026-11-03 10:06:00,di_B_tt,1,1,0,KIEM_DAT,0,0.02,-,-'])
    assert c.s.nav2_lan == 0


def test_07_vat_la_hay_dia_hinh():
    xe = (0.40, 0.09, 0.0)                                            # hanh lang duoi, mui o x = 0,545
    ong = (0.62, 0.80, 0.0, 0.18)                                     # khong co trong ban do, cach mui 7,5 cm
    assert S.vat_truoc(quet(xe, [ong]), xe, M.GRID) == 'VAT_LA'
    xe2 = (1.885 - 0.145 - 0.06, 0.82, 0.0)                           # mui cach tuong ngoai 6 cm
    assert S.vat_truoc(quet(xe2), xe2, M.GRID) == 'DIA_HINH'
    assert S.vat_truoc(quet(xe, [ong]), xe, None) is None             # khong co ban do: khong ket luan
    assert S.vat_truoc(quet(xe), xe, M.GRID) is None                  # khong co vat
    plan = [(0.40 + 0.05 * i, 0.09) for i in range(13)]
    assert S.nhin_xa(plan, xe, quet(xe, [ong])) < 0.05
    assert S.nhin_xa(plan, xe, quet(xe)) > 0.20
    assert S.chon_tu_the(dict(vat_truoc='VAT_LA', t_cho=10, goal=True))[0] == 'CHO'
    assert S.chon_tu_the(dict(vat_truoc='VAT_LA', t_cho=31, goal=True))[0] == 'DOI_HANH_LANG'


def test_08_do_tin_cay_tren_xuong():
    for xe in [A, (1.45, 0.45, math.pi), (0.70, 0.09, math.pi), (1.60, 0.82, 0.0)]:
        pts = quet(xe)
        tc, n = S.do_tin_cay(pts, xe, M.GRID)
        assert tc >= 0.9 and n > 300
        for dx, dy in [(0.15, 0.0), (0.0, 0.15), (0.106, 0.106)]:      # vi tri lech 15 cm
            assert S.do_tin_cay(pts, (xe[0] + dx, xe[1] + dy, xe[2]), M.GRID)[0] < 0.70
    assert S.do_tin_cay(quet(A), A, None) == (None, 0)


def test_09_muc_tin_theo_thoi_gian():
    assert S.muc_tin(0.80, None, 0.0) == 'TIN'
    assert S.muc_tin(0.60, None, 0.0) == 'NGHI'
    assert S.muc_tin(0.40, 0.0, 4.9) == 'NGHI'
    assert S.muc_tin(0.40, 0.0, 5.0) == 'KHONG_TIN'
    assert S.muc_tin(None, 0.0, 15.0) == 'BAO'
    assert S.chon_tu_the(dict(muc_tin='BAO', goal=True))[0] == 'DUNG_VA_BAO'
    c = Chay(vao(pose=(0.15, 0.15, 0.0), pts=quet(A)))                # xe that o A, sep tuong lech 21 cm
    for _ in range(int(S.T_BAO / 0.1) + 2):
        tt, _ = c()
    assert tt['muc_tin'] == 'BAO' and tt['tu_the'] == 'DUNG_VA_BAO'


def test_10_nguoi_chan_va_tin_cay_060_chi_cho():
    bt = dict(vat_truoc='VAT_LA', t_cho=35, muc_tin='NGHI', tin_cay=0.60, goal=True)
    assert S.chon_tu_the(bt)[0] == 'CHO'
    assert S.chon_tu_the(dict(bt, muc_tin='TIN'))[0] == 'DOI_HANH_LANG'


def test_11_khong_tin_va_ket():
    tt, ly = S.chon_tu_the(dict(muc_tin='KHONG_TIN', ket='KET_THAT', goal=True))
    assert tt == 'CHO' and '#19' in ly                                 # LiDAR van duoc lui/cung, sep khong thoat theo ban do
    assert S.chon_tu_the(dict(muc_tin='TIN', ket='KET_THAT', goal=True))[0] == 'THOAT'
    assert S.chon_tu_the(dict(muc_tin='TIN', ket='KET_THAT', luot_thoat=3, goal=True))[0] == 'DOI_HANH_LANG'


def test_12_khe_rong_dan_va_vat_tai_B():
    rong = [(t * 0.1, 0.08 + 0.002 * t) for t in range(31)]            # 3 s, rong them 6 cm
    dung = [(t * 0.1, 0.08) for t in range(31)]
    assert S.khe_rong_dan(rong) and not S.khe_rong_dan(dung)
    bt = dict(vat_truoc='VAT_LA', muc_tin='TIN', goal=True)
    assert S.chon_tu_the(dict(bt, t_cho=35, khe_rong=True))[0] == 'CHO'
    assert S.chon_tu_the(dict(bt, t_cho=35, khe_rong=False))[0] == 'DOI_HANH_LANG'
    assert S.chon_tu_the(dict(bt, t_cho=41, khe_rong=True))[0] == 'DOI_HANH_LANG'
    assert S.chon_tu_the(dict(bt, t_cho=45, tai_B=True))[0] == 'CHO'
    assert S.chon_tu_the(dict(bt, t_cho=61, tai_B=True))[0] == 'DUNG_VA_BAO'


def test_13_sep_treo_bat_dau_lai():
    s, v = S.Sep(), vao()
    _, log = s.vong(0.0, v)
    assert co(log, 'TU_THE')
    _, log = s.vong(0.1, v)
    assert not log
    _, log = s.vong(1.7, v)
    assert co(log, 'sep treo 1.6 s') and co(log, 'TU_THE')            # bo quyet dinh cu: ghi lai tu the tu dau


def test_14_sep_khong_co_ma_ra_lenh():
    src = open(os.path.join(DAY, 'sep.py')).read()
    assert not re.search(r'Twist|subprocess|os\.system|ACCESS', src)   # muc B: duoc ghi ~/sep_lenh (khi ra_lenh)
    pubs = re.findall(r'create_publisher\(([^,]+),', src)
    assert pubs == ['String']
    assert 'ActionClient' not in src and 'create_client' not in src


def test_15_cau_log_lai_xe_lidar_van_con():
    src = open(os.path.join(DAY, 'lai_xe_lidar.py')).read()
    doi = '%s -> %s (%s)%s | %s'                                      # hai cau log sep do, chep dung tu lai_xe_lidar.py
    dinh_ky = 'vung gan nhat: %s | dich=%s trang_thai=%s'
    assert "'%s'" % doi in src and "'%s'" % dinh_ky in src and "super().__init__('lai_xe_lidar')" in src
    m = doi % ('XAC_NHAN', 'THOAT_CUNG', 'KET_HINH_HOC', '', 'TRAI=0.05')
    assert S.doc_rosout('lai_xe_lidar', m) == ('L1', 'THOAT_CUNG', 'KET_HINH_HOC')
    m = dinh_ky % ('TRAI=0.05', True, 'CHO_3')
    assert S.doc_rosout('lai_xe_lidar', m) == ('L1', 'CHO_3', '')
    c = Chay(vao())
    tt, log = c(rosout=[('lai_xe_lidar', doi % ('XAC_NHAN', 'THOAT_CUNG', 'KET_HINH_HOC', '', 'x'))])
    assert c.s.luot_thoat == 1 and c.s.l1_ket and tt['tu_the'] == 'CHO' and co(log, 'BAT_DAU_THOAT')
    c(rosout=[('lai_xe_lidar', doi % ('THOAT_CUNG', 'XONG', '', '', 'x'))])
    assert not c.s.l1_ket


def dung_n_giay(c, giay, **kw):
    """Goi c() trong `giay` giay (0,1 s moi lan); tra (trang thai cuoi, moi dong nhat ky)."""
    moi = []
    for _ in range(int(round(giay / 0.1))):
        tt, log = c(**kw)
        moi += log
    return tt, moi


def test_16_ket_sat_K4_phia_truoc():
    xe = (0.60, 0.02, -math.pi / 2)                                    # mui huong -y, cach canh tren K4 5,5 cm (01/11)
    d, phia = S.sat_than_xe(quet(xe))
    assert phia == 'TRUOC' and abs(d - 0.055) < 0.01
    c = Chay(vao(pose=xe))
    _, log = c(goals=[[(b'g', 2)]])
    tt, moi = dung_n_giay(c, 15)
    log += moi
    dong = [d for d in log if ',KET_SAT_VAT,' in d and ',BAT,' in d]
    assert len(dong) == 1 and 'TRUOC' in dong[0] and 'THOAT luot 1/3' in dong[0]
    assert float(dong[0].split(',')[0]) >= 10.0                       # chua du 10 s thi chua bao
    assert dong[0].split(',')[-3:] == ['0.60', '0.02', '-90']          # cot x, y, goc_do
    assert tt['tu_the'] == 'THOAT' and tt['kep'] == 'KET_SAT_VAT' and not co(log, 'DUNG_YEN_KHONG_RO')


def test_17_ong_la_truoc_8cm_chi_cho():
    xe = (0.40, 0.09, 0.0)                                            # mui o x = 0,545
    c = Chay(vao(pose=xe, them=[(0.625, 0.80, 0.0, 0.18)]))           # ong khong co trong ban do, cach mui 8 cm
    c(goals=[[(b'g', 2)]])
    tt, log = dung_n_giay(c, 15)
    assert tt['vat_truoc'] == 'VAT_LA' and tt['tu_the'] == 'CHO'
    assert not co(log, 'KET_SAT_VAT') and not co(log, 'DUNG_YEN_KHONG_RO')


def test_18_dung_yen_khong_vat_gan():
    c = Chay(vao())                                                   # o A, vat gan nhat xa hon 10 cm
    c(goals=[[(b'g', 2)]])
    tt, log = dung_n_giay(c, 15)
    assert co(log, 'DUNG_YEN_KHONG_RO') and tt['tu_the'] == 'CHO' and not co(log, 'KET_SAT_VAT')
    _, log = dung_n_giay(Chay(vao()), 15)                             # khong co dich: dung yen la binh thuong
    assert not co(log, 'DUNG_YEN_KHONG_RO')


def test_19_dang_di_trong_khe_hong_17cm_khong_bao():
    c = Chay(vao())
    c(goals=[[(b'g', 2)]])
    log = []
    for k in range(120):                                              # 12 s, tien 0,033 m/s doc khe duoi
        xe = (0.45 + 0.0033 * k, 0.07, 0.0)
        pts = quet(xe)
        if k == 0:
            d, phia = S.sat_than_xe(pts)
            assert phia == 'PHAI' and 0.16 < d < 0.18                 # hong phai cach K4 17 cm
        _, l = c(pose=xe, pts=pts)
        log += l
    for chu in ('KET_SAT_VAT', 'DUNG_YEN_KHONG_RO', 'VAT_LA_TRUOC'):
        assert not co(log, chu)


def test_20_nhat_ky_cu_khac_tieu_de_thi_doi_ten():
    import shutil
    import tempfile
    d = tempfile.mkdtemp()
    try:
        p = os.path.join(d, 'sep_nhat_ky.csv')
        assert S.chuan_bi_nhat_ky(p) is None                          # chua co tep
        cu = 'gio_mono,gio,loai,dau_hieu,nguon_nghi,phep_kiem,ket_qua,se_lam\n1.00,t,TU_THE,CHO,,,,\n'
        open(p, 'w').write(cu)
        assert S.chuan_bi_nhat_ky(p) == os.path.join(d, 'sep_nhat_ky_cu1.csv') and not os.path.exists(p)
        open(p, 'w').write(cu)
        assert S.chuan_bi_nhat_ky(p) == os.path.join(d, 'sep_nhat_ky_cu2.csv')   # cu1 da co thi thanh cu2
        assert open(os.path.join(d, 'sep_nhat_ky_cu1.csv')).read() == cu
        open(p, 'w').write(S.COT_NHAT_KY + '\n')
        assert S.chuan_bi_nhat_ky(p) is None and os.path.exists(p)    # dung tieu de moi: giu nguyen
    finally:
        shutil.rmtree(d)


def test_21_di_hay_khong():
    assert S.di_hay_khong(dict(muc_tin='TIN')) == ('DI', '')
    assert S.di_hay_khong(dict(muc_tin='NGHI', tin_cay=0.65))[0] == 'DI'          # muc NGHI khong chan
    assert S.di_hay_khong(dict(muc_tin='BAO'))[0] == 'DUNG'
    assert S.di_hay_khong(dict(muc_tin='TIN', nav2='MAT_NHIP'))[0] == 'DUNG'
    for k in ('mat_scan', 'tf_cu', 'nhay'):
        assert S.di_hay_khong({'muc_tin': 'TIN', k: True})[0] == 'CHO'
    assert S.di_hay_khong(dict(muc_tin='KHONG_TIN'))[0] == 'CHO'
    assert S.di_hay_khong(dict(muc_tin='TIN', plan_xau='ABORT_NHANH', plan_xau_s=4.0))[0] == 'CHO'
    assert S.di_hay_khong(dict(muc_tin='TIN', plan_xau='ABORT_NHANH', plan_xau_s=11.0))[0] == 'DI'


def test_22_ca_1506_abort_nhanh_thi_cho_10s():
    c = Chay(vao())
    c(goals=[[(b'g1', 2)]])
    tt, log = c(goals=[[(b'g1', 6)]])                                  # ABORTED sau 0,1 s (nhu 01/11 15:06)
    assert co(log, 'ABORT_NHANH') and tt['di'] == 'CHO' and 'ABORT_NHANH' in tt['di_ly']
    assert co(log, ',LENH,CHO,')
    tt, _ = dung_n_giay(c, 9.5)
    assert tt['di'] == 'CHO'
    tt, log = dung_n_giay(c, 1.0)
    assert tt['di'] == 'DI' and co(log, ',LENH,DI,')


def test_23_tin_cay_065_van_di():
    c = Chay(vao(pose=(0.15, 0.0, 0.0), pts=quet(A)))                 # sep tuong lech 15 cm: tin cay ~0,63
    tt, _ = dung_n_giay(c, 20)
    assert tt['muc_tin'] == 'NGHI' and 0.50 <= tt['tin_cay'] < 0.70 and tt['di'] == 'DI'


def test_24_khong_tin_16s_thi_dung():
    c = Chay(vao(pose=(0.15, 0.15, 0.0), pts=quet(A)))                # sep tuong lech 21 cm: tin cay < 0,50
    tt, _ = dung_n_giay(c, 4)
    assert tt['di'] == 'DI'                                            # duoi 5 s: van la NGHI
    tt, _ = dung_n_giay(c, 6)
    assert tt['di'] == 'CHO' and tt['di_ly'] == 'KHONG_TIN'
    tt, _ = dung_n_giay(c, 6)
    assert tt['di'] == 'DUNG' and tt['di_ly'].startswith('KHONG_TIN_QUA')


def test_25_ghi_lenh_doi_ten_nguyen_tep():
    import shutil
    import tempfile
    d = tempfile.mkdtemp()
    try:
        p = os.path.join(d, 'sep_lenh')
        S.ghi_lenh(p, 1234.5, 'CHO', 'ABORT_NHANH 3s truoc')
        assert open(p).read() == '1234.50 CHO ABORT_NHANH 3s truoc\n'
        assert os.listdir(d) == ['sep_lenh']                           # khong con tep tam
    finally:
        shutil.rmtree(d)


def _chay_kep(day, vong, plan=None):
    """Xe dung yen o K4 (co dich), khoang cach LiDAR gan than xe bi thay bang chuoi `day` (m, lap vong, moi vong mot
    gia tri). Chay 11 s cho du t_dung roi `vong` vong. Tra (c, toan bo nhat ky)."""
    that, i = S.sat_than_xe, [0]

    def gia(pts):
        i[0] += 1
        return day[i[0] % len(day)], 'TRUOC'
    S.sat_than_xe = gia
    try:
        c = Chay(vao(pose=(0.60, 0.02, -math.pi / 2)))
        c.thoats = []                                                 # viec 109c-c1: tt['thoat'] moi vong
        tt, log = c(goals=[[(b'g', 2)]], plans=[plan] if plan else [])
        for _ in range(110 + vong):
            tt, l = c()
            log += l
            c.thoats.append(tt['thoat'])
        c.tt = tt
    finally:
        S.sat_than_xe = that
    return c, log


def test_26_dao_dong_9_5_10_5_cm_khong_nhay():
    c, log = _chay_kep([0.105, 0.095], 40)
    bat = [d for d in log if ',KET_SAT_VAT,' in d and ',BAT,' in d]
    het = [d for d in log if ',KET_SAT_VAT,' in d and ',HET' in d]
    assert len(bat) == 1 and not het and c.s.luot_thoat == 1          # bat mot lan o 9,5 cm; 10,5 cm chua tat (< 12 cm)
    c, log = _chay_kep([0.095, 0.115, 0.125], 40)                     # 11,5 van giu; 12,5 > 12 moi tat => co the bat lai
    het = [d for d in log if ',KET_SAT_VAT,' in d and ',HET' in d]
    assert het                                                        # > 12 cm thi tat that


def test_27_20_lan_ket_toi_da_3_luot_roi_mot_dung_va_bao():
    c, log = _chay_kep([0.05, 0.20], 40)                              # 0,05 = ket; 0,20 > 12 cm = het: moi 2 vong mot lan ket
    luot = [int(m) for d in log for m in re.findall(r'luot (\d+)/3', d)]
    assert sorted(set(luot)) == [1, 2, 3], luot
    assert c.s.luot_thoat == 3 and c.s.het_luot
    assert sum(',TU_THE,DUNG_VA_BAO,' in d for d in log) == 1         # dung mot dong DUNG_VA_BAO
    assert not any(re.search(r'luot ([4-9]|\d\d)/3', d) for d in log)
    c.s._csv(dict(ket_qua='KIEM_DAT', huong='di_B', tuyen='1', diem='1'), lambda *a, **k: None)   # KIEM_DAT dat lai
    assert c.s.luot_thoat == 0 and not c.s.het_luot


def test_28_l1_thoat_luc_ket_sat_vat_bat_khong_cong():
    doi = '%s -> %s (%s)%s | %s'
    xe = (0.60, 0.02, -math.pi / 2)
    c = Chay(vao(pose=xe))
    c(goals=[[(b'g', 2)]])
    dung_n_giay(c, 11)
    assert c.s.co.get('KET_SAT_VAT') and c.s.luot_thoat == 1
    tt, log = c(rosout=[('lai_xe_lidar', doi % ('XAC_NHAN', 'THOAT_CUNG', 'KET_HINH_HOC', '', 'x'))])
    assert c.s.luot_thoat == 1 and co(log, 'BAT_DAU_THOAT') and co(log, 'khong cong')
    # khong co KET_SAT_VAT: L1 tu cong, va co tran 3
    c2 = Chay(vao())
    for k in range(5):
        c2(rosout=[('lai_xe_lidar', doi % ('THOAT_CUNG', 'XONG', '', '', 'x'))])
        c2(rosout=[('lai_xe_lidar', doi % ('XAC_NHAN', 'THOAT_CUNG', 'KET_HINH_HOC', '', 'x'))])
    assert c2.s.luot_thoat == 3



PLAN_K4 = [(0.60, 0.02), (1.05, 0.06)]        # /plan: tu xe toi diem xs ke tiep (1,05; 0,06)


def test_29_sep_ghi_thoat_luot_1_co_P():
    c, log = _chay_kep([0.05], 0, PLAN_K4)
    th = [t for t in c.thoats if t]
    assert th and all(t['luot'] == 1 and t['p'] == (1.05, 0.06) for t in th) and len({t['ma'] for t in th}) == 1
    dong = [d for d in log if ',KET_SAT_VAT,' in d and ',BAT,' in d]
    assert len(dong) == 1 and 'THOAT luot 1/3' in dong[0] and 'phia ket TRUOC' in dong[0] and 'P=(1.05;0.06)' in dong[0]   # 1.7
    assert S.ly_thoat(th[0]) == '1.05 0.06 1 %d' % th[0]['ma']


def test_30_thoat_tat_khi_dau_hieu_tat_va_P_trong_khi_khong_tin():
    c, log = _chay_kep([0.05, 0.20], 0, PLAN_K4)                      # het dot ket => thoat None ngay vong sau
    assert c.thoats[-1] is None and any(c.thoats)                     # co luc co, cuoi cung het
    c, log = _chay_kep([0.05], 0)                                     # khong co /plan => chua co dich
    th = [t for t in c.thoats if t]
    assert th and th[0]['p'] is None and S.ly_thoat(th[0]).startswith('- - 1 ')
    s = S.Sep()
    s.dich = (1.05, 0.06)
    assert s._p_thoat('TIN') == (1.05, 0.06) and s._p_thoat('NGHI') is None and s._p_thoat('KHONG_TIN') is None


def test_31_luot_4_khong_ghi_thoat_va_ra_DUNG():
    c, log = _chay_kep([0.05, 0.20], 40, PLAN_K4)
    assert c.s.het_luot and c.thoats[-1] is None and c.tt['di'] == 'DUNG' and 'HET_3_LUOT_THOAT' in c.tt['di_ly']
    assert S.di_hay_khong(dict(het_luot=True)) == ('DUNG', 'HET_3_LUOT_THOAT')
    assert S.di_hay_khong(dict()) == ('DI', '')
    ma = {t['ma'] for t in c.thoats if t}
    assert len(ma) == 3                                               # ba luot, ba ma khac nhau, luot 4 khong co ma
    c.s._csv(dict(ket_qua='KIEM_DAT', huong='di_B', tuyen='1', diem='1'), lambda *a, **k: None)
    assert S.di_hay_khong(dict(het_luot=c.s.het_luot)) == ('DI', '')   # KIEM_DAT moi dat lai


def test_32_luat_G_sep_thoat_giua_sep_va_lai_xe_lidar():
    c, _ = _chay_kep([0.05], 0, PLAN_K4)
    th = next(t for t in c.thoats if t)
    dong = '%.2f THOAT %s' % (100.0, S.ly_thoat(th))                  # dung khuon ghi_lenh(path, t, 'THOAT', ly_thoat)
    assert L.doc_sep_thoat(dong, 101.0) == dict(luot=1, ma=th['ma'], p=(1.05, 0.06))
    assert L.doc_sep_thoat(dong, 103.5) is None                       # tuoi > 3 s
    assert L.doc_sep_thoat('100.00 THOAT - - 2 77', 101.0) == dict(luot=2, ma=77, p=None)
    assert L.doc_sep_thoat('100.00 CHO x', 101.0) is None and L.doc_sep_thoat('', 101.0) is None and L.doc_sep_thoat('a THOAT 1 2 3 4', 1.0) is None
    src = open(os.path.join(DAY, 'sep.py')).read()
    assert "declare_parameter('ra_thoat', False)" in src and "'THOAT', ly_thoat(" in src      # ten tham so, mac dinh tat, chu THOAT
    sh = open(os.path.join(DAY, 'chuyen.sh')).read()
    assert re.search(r'\n\s*DUNG\)', sh) and 'read -r T C L < "$SEP_LENH"' in sh                # chuyen.sh doc chu DUNG tu ~/sep_lenh
    cau = L.cau_se_lam(dict(luot=1, ma=5, p=(1.05, 0.06)), quet((0.60, 0.02, -math.pi / 2)), 0.3)
    assert cau.startswith('SE LAM') and S.doc_rosout('lai_xe_lidar', cau) is None   # sep khong nham la cau cua may trang thai



def test_33_luat_G_cau_log_RA_cua_lai_xe_lidar():
    doi = '%s -> %s (%s)%s | %s'                                      # khuon cau doi trang thai that cua lai_xe_lidar
    assert S.doc_rosout('lai_xe_lidar', doi % ('-', 'RA_LUI', '', '', 'TRUOC=0.02')) == ('L1', 'RA_LUI', '')
    for cau in ('RA BAT DAU LUI (thoang truoc 0.02 m, sau 1.00 m)', 'RA XONG DU_CHO_XOAY 4 cm', 'RA KHONG DI: hai dau deu chan (thoang truoc 0.02 m, sau 0.02 m)'):
        assert S.doc_rosout('lai_xe_lidar', cau) is None              # cau RA XONG / KHONG DI: sep khong bat (chi ghi log cua lai_xe_lidar)
    c = Chay(vao())
    for tt_moi in ('RA_LUI', 'RA_XONG'):
        c(rosout=[('lai_xe_lidar', doi % ('-', tt_moi, '', '', 'x'))])
    assert c.s.luot_thoat == 0 and c.s.l1 == 'RA_XONG'                # ten RA_* khong bat dau bang THOAT: khong cong luot


def test_34_luat_G_cau_log_XOAY_cua_lai_xe_lidar():
    doi = '%s -> %s (%s)%s | %s'
    assert S.doc_rosout('lai_xe_lidar', doi % ('-', 'XOAY_P', '', '', 'x')) == ('L1', 'XOAY_P', '')
    for cau in ('XOAY XONG DEN_HUONG 61 do', 'XOAY KHONG: khong co huong toi P (P la "-" hoac tf cu > 1 s)'):
        assert S.doc_rosout('lai_xe_lidar', cau) is None              # sep khong bat hai kieu cau nay
    c = Chay(vao())
    for tt_moi in ('RA_XONG', 'XOAY_P', 'XOAY_XONG'):
        c(rosout=[('lai_xe_lidar', doi % ('-', tt_moi, '', '', 'x'))])
    assert c.s.luot_thoat == 0 and c.s.l1 == 'XOAY_XONG'              # XOAY_* khong bat dau bang THOAT: khong cong luot


def test_35_hai_kenh_dich_gop_theo_tung_kenh():
    c = Chay(vao())
    c(goals=[[(b'n0', 4)]])                                           # tin dau kenh nav: dich cu da xong => bo qua
    _, log = c(goals_fp=[[(b'f0', 4)]])                               # tin dau kenh fp: cung bo qua (tung kenh rieng)
    assert not co(log, ',SUCCEEDED')
    c(goals=[[(b'n1', 2)]])
    assert c.s.goal_active
    c(goals_fp=[[(b'f1', 2)]])
    c(goals=[[(b'n1', 4)]])                                           # nav xong nhung fp con chay: van co dich
    assert c.s.goal_active and c.s.goal_kenh == {'nav': False, 'fp': True}
    _, log = c(goals_fp=[[(b'f1', 6)]])
    assert not c.s.goal_active and co(log, ',ABORTED,') and not co(log, 'ABORT_NHANH')   # fp: ABORT som khong phai "0 poses"
    _, log = c(goals=[[(b'n2', 2)]])
    _, log = c(goals=[[(b'n2', 6)]])
    assert co(log, 'ABORT_NHANH')                                     # nav: van nhu cu


def test_36_duong_day_dich_va_P_phia_truoc():
    assert S.diem_truoc([(0, 0), (0.1, 0), (0.2, 0), (0.5, 0)]) == (0.4, 0.0)
    assert S.diem_truoc([(0, 0), (0.1, 0)]) == (0.1, 0)
    xe = (0.60, 0.02, -math.pi / 2)
    day = [(0.60, 0.02), (0.60, 0.10), (0.70, 0.15), (1.05, 0.10), (1.60, 0.82)]
    c = Chay(vao(pose=xe))
    _, log = c(goals_fp=[[(b'f', 2)]], duong_day=[day])
    assert c.s.dich == (1.60, 0.82) and not c.s.plan_sai and not co(log, 'PLAN_LECH')   # dau duong canh xe: khong bao lech
    th = []
    for _ in range(110):
        tt, l = c(duong_day=[day])
        th.append(tt['thoat'])
    t = next(t for t in th if t)
    assert t['p'] == (0.60, 0.10) == S.dinh_ke_tiep(day)[0] and t['p'] != (1.60, 0.82)   # 18:09: P = dinh ke tiep tren duong day
    s = S.Sep()
    s.dich, s.day_pts, s.day_t = (1.60, 0.82), day, 0.0
    assert s._p_thoat('TIN', 1.0) == t['p'] and s._p_thoat('TIN', 5.0) == (1.6, 0.82)   # /duong_day cu > 3 s: ve diem cuoi nhu c1
    assert s._p_thoat('KHONG_TIN', 1.0) is None


def test_37_luat_G_kenh_109b():
    sp = open(os.path.join(DAY, 'sep.py')).read()
    lx = open(os.path.join(DAY, 'lai_xe_lidar.py')).read()
    gd = open(os.path.join(DAY, 'gui_duong.py')).read()
    bag = open(os.path.join(DAY, 'ghi_bag.sh')).read()
    st = "'/follow_path/_action/status'"
    assert st in sp and st in lx and "'/navigate_to_pose/_action/status'" in sp and "'/navigate_to_pose/_action/status'" in lx
    assert "'/duong_day'" in sp and "create_publisher(Path, '/duong_day'" in gd and "'/plan'" in sp
    assert "declare_parameter('nghe_follow_path', True)" in sp and "declare_parameter('nghe_follow_path', True)" in lx
    for t in ('/duong_day', '/sep/trang_thai', '/cmd_vel_nav', '/cmd_vel_avoid', '/rosout', '/follow_path/_action/status', '/navigate_to_pose/_action/status'):
        assert ' %s ' % t in bag.replace('\\\n', ' ') or ' %s >' % t in bag, t
    assert "create_publisher(String, '/sep/trang_thai'" in sp


def test_38_plan_lech_chi_xet_khi_co_dich_dang_chay():
    xa = [(1.20, 0.60), (0.0, 0.0)]                                   # dau duong cach xe ~1,3 m
    c = Chay(vao())
    c()
    tt, log = c(plans=[xa])                                           # /plan cua buoc KIEM (chua co dich): khong xet
    assert not co(log, 'PLAN_LECH') and tt['di'] == 'DI' and tt['tu_the'] != 'KHONG_DI'
    tt, log = c(goals=[[(b'g1', 2)]], plans=[xa])                     # /plan cung vong voi dich moi: la duong cua dich moi
    assert co(log, 'PLAN_LECH') and tt['di'] == 'CHO' and tt['tu_the'] == 'KHONG_DI'
    c(goals=[[(b'g1', 4)]])
    tt, log = c(goals=[[(b'g2', 2)]])                                 # dich moi: PLAN_LECH cu het gia tri ngay, chua can /plan moi
    assert co(log, ',PLAN_SAI,') and tt['di'] == 'DI' and tt['tu_the'] != 'KHONG_DI'
    tt, log = c(plans=[[(0.02, 0.0), (1.0, 0.0)]])
    assert not co(log, 'PLAN_LECH') and tt['di'] == 'DI'
    c(goals=[[(b'g2', 4)]])
    tt, log = c(duong_day=[xa])                                       # /duong_day khi khong co dich (gui_duong phat truoc khi gui): khong xet
    assert not co(log, 'PLAN_LECH') and tt['di'] == 'DI'


def _ra_thoat_mo_dot(c, doi):
    """Co dich fp + /duong_day + 1,5 s; roi phu lai vao CHO_SEP. Tra trang thai."""
    day = [(0.0, 0.0), (0.05, 0.0), (0.10, 0.0), (0.15, 0.05), (0.20, 0.10)]
    c(goals_fp=[[(b'f', 2)]], duong_day=[day], ra_thoat=True)
    for _ in range(14):
        c(duong_day=[day], ra_thoat=True)
    return c(rosout=[('lai_xe_lidar', doi % ('CHAN_XOAY', 'CHO_SEP', 'XOAY_BI_CHAN_QUA_LAU', '', 'x'))], duong_day=[day], ra_thoat=True), day


def test_39_ra_thoat_luot_chi_dem_khi_ra_xoay_chay():
    doi = '%s -> %s (%s)%s | %s'
    c = Chay(vao())
    (tt, log), day = _ra_thoat_mo_dot(c, doi)
    assert tt['thoat']['luot'] == 1 and c.s.luot_thoat == 0 and tt['thoat']['p'] == (0.1, 0.0) and abs(tt['thoat']['yaw']) < 1e-9
    assert ' 0.0000' in S.ly_thoat(tt['thoat'])                       # cot 7 = huong doan
    ma1 = tt['thoat']['ma']
    for n in ('RA_LUI', 'RA_XONG', 'XOAY_P', 'XOAY_XONG'):
        tt, _ = c(rosout=[('lai_xe_lidar', doi % ('-', n, '', '', 'x'))], duong_day=[day], ra_thoat=True)
    assert c.s.luot_thoat == 1 and tt['thoat'] and tt['thoat']['ma'] == ma1   # XOAY xong: dot con mo T_LAI_THOAT s
    tt, _ = c(rosout=[('lai_xe_lidar', doi % ('XOAY_XONG', 'CHO_SEP', 'XOAY_BI_CHAN_QUA_LAU', '', 'x'))], duong_day=[day], ra_thoat=True)
    for _ in range(31):                                               # van ket (phu lai lai CHO_SEP) sau 3 s: giao luot moi, ma moi
        tt, log = c(duong_day=[day], ra_thoat=True)
        if tt['thoat'] and tt['thoat']['ma'] != ma1:
            break
    assert tt['thoat'] and tt['thoat']['ma'] != ma1 and tt['thoat']['luot'] == 2 and c.s.luot_thoat == 1


def test_40_ra_thoat_het_luot_va_phu_lai_khong_lam_gi():
    doi = '%s -> %s (%s)%s | %s'
    c = Chay(vao())
    (tt, _), day = _ra_thoat_mo_dot(c, doi)
    c.s.luot_thoat = 3                                                # da RA/XOAY du 3 luot trong chuyen
    c.s.thoat_ma = None
    c(rosout=[('lai_xe_lidar', doi % ('CHO_SEP', 'NGHI', '', '', 'x'))], ra_thoat=True)
    tt, log = c(rosout=[('lai_xe_lidar', doi % ('NGHI', 'CHO_SEP', 'KET_HINH_HOC', '', 'x'))], ra_thoat=True)
    assert c.s.het_luot and tt['thoat'] is None and tt['di'] == 'DUNG' and any('HET 3 LUOT' in d for d in log)
    c = Chay(vao())
    (tt, _), day = _ra_thoat_mo_dot(c, doi)
    for vong in range(4):                                             # 3 dot lien tiep phu lai XOAY KHONG (khong RA/XOAY gi): het
        c(rosout=[('lai_xe_lidar', doi % ('-', 'XOAY_XONG', '', '', 'x'))], duong_day=[day], ra_thoat=True)
        c(rosout=[('lai_xe_lidar', doi % ('XOAY_XONG', 'CHO_SEP', 'XOAY_BI_CHAN_QUA_LAU', '', 'x'))], duong_day=[day], ra_thoat=True)
        for _ in range(31):
            tt, _ = c(duong_day=[day], ra_thoat=True)
    assert c.s.luot_thoat == 0 and c.s.het_luot and tt['di'] == 'DUNG'
    c = Chay(vao())                                                   # ra_thoat = false: nhu cu, CHO_SEP khong mo dot
    c(goals_fp=[[(b'f', 2)]])
    tt, _ = c(rosout=[('lai_xe_lidar', doi % ('CHAN_XOAY', 'CHO_SEP', 'XOAY_BI_CHAN_QUA_LAU', '', 'x'))])
    assert tt['thoat'] is None and c.s.luot_thoat == 0


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
