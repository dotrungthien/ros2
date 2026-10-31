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
    assert S.nav2_song({}, True)[0] == 'MAT_NHIP'
    assert S.nav2_song({'controller_server#ab': 5.0, 'bt_navigator#cd': 0.3}, False) == ('MAT_NHIP', 'controller_server#ab')
    assert S.nav2_song({'bt_navigator#cd': 0.3}, False)[0] == 'SONG'
    assert S.nav2_song({}, False)[0] == 'KHONG_BIET'
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
    assert not re.search(r'Twist|sep_lenh|subprocess|os\.system|ACCESS', src)
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
