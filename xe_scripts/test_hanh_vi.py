#!/usr/bin/env python3
"""test_hanh_vi.py (bo 1e "nao xe", 09/11): hanh_vi.py + hanh_vi_xe.yaml (+ kiem_hanh_vi.py, tong_ket_kinh_nghiem.py, ~/kinh_nghiem.csv).
Chay: python3 -m pytest -q test_hanh_vi.py"""
import contextlib
import io as _io
import math
import os
import sys
import tempfile

DAY = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DAY)
import hanh_vi as HV  # noqa: E402

# So CU, chep tay tu ma o 2607fee (mat-lai) ngay 09/11 (doi chieu bang script voi git show 2607fee: 77/77 khop).
CU_2607FEE = {
    've_lai_duong_hz': 1.0,
    'progress_s': 10.0,
    'vung_phanh_m': 0.07,
    'nhin_truoc_phanh_s': 1.5,
    'lui_thoat_goc_m': 0.25,
    'toc_do_lui_mps': 0.1,
    'lui_ra_khe_m': 0.35,
    'cho_khe_s': 30.0,
    'tien_toi_thieu_m': 0.1,
    'gan_dich_m': 0.6,
    'gan_dich_s': 10.0,
    'cho_dich_s': 60.0,
    'cho_sep_s': 10.0,
    'sep_lenh_tuoi_max_s': 3.0,
    'toi_sai_so_m': 0.2,
    'tf_tuoi_max_s': 1.0,
    'cho_chon_tuyen_s': 2.0,
    'nhip_in_cho_s': 5.0,
    'cho_san_sang_s': 15.0,
    'nhip_vi_tri_s': 1.0,
    'cho_may_chu_s': 5.0,
    'cho_huy_s': 2.0,
    'cho_nghe_chon_tuyen_s': 3.0,
    'luot_lan_gui': 3,
    'cho_hong_s': 30.0,
    'qua_gio_lan_gui_s': 130.0,
    'lop2_cho_s': 12.0,
    'lop2_tra_s': 2.0,
    'nhip_duong_day_s': 0.5,
    'lo_quanh_xe_m': 0.35,
    'kinh_nghiem_so_lan': 3,
    'kinh_nghiem_ban_kinh_m': 0.2,
    'vung_ton_diem_m': 0.15,
    'vung_ton_diem_gia_tri': 0,      # 09/11 gia X (Thien chot 17:50-17:57): 50 -> 0, thoi to vung ton diem (2607fee: 50)
    'o_cam_gia_tri': 100,
    't_cho_s': 30.0,
    't_cho_them_s': 10.0,
    't_cho_b_s': 60.0,
    'lech_max_m': 0.3,
    'tf_cu_s': 5.0,
    'scan_cu_s': 0.5,
    'r_tin_cay_m': 3.0,
    'tc_tin': 0.7,
    'tc_nghi': 0.5,
    't_khong_tin_s': 5.0,
    't_bao_s': 15.0,
    'cua_so_ket_s': 2.0,
    'lidar_doi_m': 0.02,
    'pose_doi_m': 0.02,
    'suc_lech_m': 0.15,
    'suc_lech_max_m': 0.2,
    'suc_nhanh_s': 0.05,
    'nhay_m': 0.1,
    'nhay_deg': 5.0,
    'nhay_dt_s': 1.0,
    'nav2_lan_max': 2,
    'luot_thoat_max': 3,
    'day_tuoi_s': 3.0,
    'p_day_xa_m': 0.4,
    'goc_dinh_deg': 5.0,
    't_lai_thoat_s': 3.0,
    'khe_tang_m': 0.02,
    'khe_cua_so_s': 3.0,
    'nhin_xa_m': 0.6,
    'xa_truoc_m': 0.15,
    't_plan_xau_s': 10.0,
    'r_b_m': 0.2,
    't_ra2_tra_s': 3.0,
    't_ra2_cho_s': 12.0,
    'r_vat_dich_m': 0.3,
    'treo_s': 1.0,
    'abort_nhanh_s': 2.0,
    'cua_so_vong_s': 10.0,
    't_dung_s': 10.0,
    'dich_dung_m': 0.03,
    'sat_vat_m': 0.1,
    'sat_vat_tat_m': 0.12,
}
CU_THAN_THE = {'v_min_mps': 0.092, 'lidar_range_min_m': 0.10, 'ban_kinh_xoay_m': 0.186, 'o_luoi_m': 0.05, 'rong_xe_m': 0.16, 'dai_xe_m': 0.25}
MOI_NAO_XE = {'kinh_nghiem_quan_sat_s': 15.0,           # so MOI cua nao xe (khong co o 2607fee)
              'gac_vat_la': True, 'gac_truoc_m': 0.30, 'gac_hong_m': 0.10, 'gac_ra_truoc_m': 0.40, 'gac_ra_hong_m': 0.15,
              'gac_le_tuong_m': 0.10, 'gac_so_diem': 3, 'gac_sach_s': 2.0, 'gac_scan_cu_s': 0.5,   # dung im ban 2 (A1)
              'footprint_padding_m': 0.03,                                                        # A3
              'gia_x_bat': True, 'gia_x_giay_mac_dinh': 10.0, 'gia_x_r_m': 0.2, 'gia_x_v_ms': 0.11}           # gia X 09/11
CU_AN_TOAN = {'cach_vat_toi_thieu_m': 0.06, 'sep_phat_toc_do': False}


def _tam(noi_dung):
    p = os.path.join(tempfile.mkdtemp(), 'hanh_vi_xe.yaml')
    with open(p, 'w') as f:
        f.write(noi_dung)
    return p


def test_01_mac_dinh_tung_so_bang_2607fee():
    sai = [(k, HV.MAC_DINH.get(k), v) for k, v in CU_2607FEE.items() if HV.MAC_DINH.get(k) != v or type(HV.MAC_DINH.get(k)) is not type(v)]
    assert not sai and set(HV.MAC_DINH) - set(CU_2607FEE) == set(MOI_NAO_XE) and all(HV.MAC_DINH[k] == v and type(HV.MAC_DINH[k]) is type(v) for k, v in MOI_NAO_XE.items()), sai
    assert HV.MAC_DINH_THAN_THE == CU_THAN_THE and HV.MAC_DINH_AN_TOAN == CU_AN_TOAN


def test_02_tep_yaml_cua_kho_bang_mac_dinh():
    tq, at, tt, loi = HV.doc(os.path.join(DAY, 'hanh_vi_xe.yaml'))
    assert loi == [] and tq == HV.MAC_DINH and at == HV.MAC_DINH_AN_TOAN and tt == HV.MAC_DINH_THAN_THE, loi
    import yaml
    y = yaml.safe_load(open(os.path.join(DAY, 'hanh_vi_xe.yaml')))
    assert all(e.get('dung_o') for e in y['thoi_quen'].values())                      # moi so ghi tep doc no
    assert all('nguon' in e and 'ngay_do' in e for e in y['ban_nang']['than_the'].values())


def test_03_cac_tep_lay_so_tu_hanh_vi_bang_so_cu():
    import gui_duong as G
    import lop_huong_dan as LH
    import sep as S
    c = G.cau_hinh({})
    cap = dict(cho_khe='cho_khe_s', tien_min='tien_toi_thieu_m', gan_dich_m='gan_dich_m', gan_dich_s='gan_dich_s', cho_dich='cho_dich_s',
               cho_sep='cho_sep_s', sep_tuoi='sep_lenh_tuoi_max_s', toi_sai_so='toi_sai_so_m', tf_tuoi='tf_tuoi_max_s',
               cho_chon='cho_chon_tuyen_s', nhip_cho='nhip_in_cho_s', lui_m='lui_ra_khe_m', lui_v='toc_do_lui_mps', luot='luot_lan_gui',
               cho_hong='cho_hong_s', to='qua_gio_lan_gui_s', lop2_cho='lop2_cho_s', lop2_tra='lop2_tra_s')
    assert c.pop('gac') is True                                                        # dung im ban 2: so MOI
    assert set(c) == set(cap) and all(c[k] == CU_2607FEE[v] and type(c[k]) is type(CU_2607FEE[v]) for k, v in cap.items()), c
    assert G.LUI_BT_M == 0.25 and G.cho_tf_costmap.__defaults__[0] == 15.0 and G.doc_sep_lenh.__defaults__[0] == 3.0
    assert (LH.R_THA, LH.R_CUM, LH.N_CUM, LH.R_VUNG, LH.GIA_CAM, LH.GIA_TON) == (0.35, 0.20, 3, 0.15, 100, 0)
    assert (S.T_CHO, S.T_CHO_THEM, S.T_CHO_B, S.LECH_MAX, S.TF_CU, S.SCAN_CU, S.R_TIN_CAY, S.TC_TIN, S.TC_NGHI) == (30.0, 10.0, 60.0, 0.30, 5.0, 0.5, 3.0, 0.70, 0.50)
    assert (S.T_KHONG_TIN, S.T_BAO, S.CUA_SO_KET, S.LIDAR_DOI, S.POSE_DOI, S.SUC_LECH, S.SUC_LECH_MAX, S.SUC_NHANH) == (5.0, 15.0, 2.0, 0.02, 0.02, 0.15, 0.20, 0.05)
    assert (S.NHAY_M, S.NHAY_DEG, S.NHAY_DT, S.NAV2_LAN_MAX, S.LUOT_THOAT_MAX, S.DAY_TUOI, S.P_DAY_XA) == (0.10, 5.0, 1.0, 2, 3, 3.0, 0.40)
    assert S.GOC_DINH == math.radians(5.0) and (S.T_LAI_THOAT, S.KHE_TANG, S.KHE_CUA_SO, S.NHIN_XA, S.XA_TRUOC) == (3.0, 0.02, 3.0, 0.6, 0.15)
    assert (S.T_PLAN_XAU, S.R_B, S.T_RA2_TRA, S.T_RA2_CHO, S.R_VAT_DICH, S.TREO, S.ABORT_NHANH, S.CUA_SO_VONG) == (10.0, 0.20, 3.0, 12.0, 0.30, 1.0, 2.0, 10.0)
    assert S.B_XY == (1.60, 0.82) and S.P_MAC_DINH == dict(t_dung=10.0, dich_dung=0.03, sat_vat=0.10, sat_vat_tat=0.12, lop2_sau=True, dem_luot_thoat=True)


def test_04_thieu_yaml_dung_mac_dinh_in_mot_dong():
    tq, at, tt, loi = HV.doc('/khong/co/hanh_vi_xe.yaml')
    assert tq == HV.MAC_DINH and at == HV.MAC_DINH_AN_TOAN and tt == HV.MAC_DINH_THAN_THE and loi == ['khong co /khong/co/hanh_vi_xe.yaml']
    cu = os.environ.get('HANH_VI_YAML')
    os.environ['HANH_VI_YAML'] = '/khong/co/hanh_vi_xe.yaml'
    e = _io.StringIO()
    try:
        with contextlib.redirect_stderr(e):
            HV.nap_lai()
            assert HV.so('cho_khe_s') == 30.0 and HV.so('gan_dich_m') == 0.60
    finally:
        os.environ.pop('HANH_VI_YAML')
        if cu is not None:
            os.environ['HANH_VI_YAML'] = cu
    dong = e.getvalue().splitlines()
    assert dong == ['CANH BAO hanh_vi: dung mac dinh (khong co /khong/co/hanh_vi_xe.yaml)'], dong
    with contextlib.redirect_stderr(_io.StringIO()):
        HV.nap_lai()


def test_05_yaml_sai_kieu_tung_khoa_ve_mac_dinh():
    p = _tam('ban_nang:\n  an_toan:\n    sep_phat_toc_do: {gia_tri: 1}\n  than_the: {}\n'
             'thoi_quen:\n  cho_khe_s: {gia_tri: "abc"}\n  luot_lan_gui: {gia_tri: 3.5}\n  tien_toi_thieu_m: {gia_tri: 0.2}\n'
             '  gan_dich_m: {gia_tri: true}\n  kinh_nghiem_so_lan: 4\n  ten_la: {gia_tri: 1}\n')
    tq, at, tt, loi = HV.doc(p)
    assert tq['cho_khe_s'] == 30.0 and tq['luot_lan_gui'] == 3 and tq['gan_dich_m'] == 0.60      # sai kieu => mac dinh
    assert tq['tien_toi_thieu_m'] == 0.2 and tq['kinh_nghiem_so_lan'] == 4                        # dung kieu => lay (ke ca so tran)
    assert at['sep_phat_toc_do'] is False and tt == HV.MAC_DINH_THAN_THE
    for s in ("cho_khe_s sai kieu ('abc')", 'luot_lan_gui sai kieu (3.5)', 'gan_dich_m sai kieu (True)', 'sep_phat_toc_do sai kieu (1)',
              'ten_la la (bo qua)', 'thieu v_min_mps', 'thieu tc_tin'):
        assert s in loi, (s, loi)
    p2 = _tam('thoi_quen: [1, 2\n')                                              # yaml hong
    tq, at, tt, loi = HV.doc(p2)
    assert tq == HV.MAC_DINH and len(loi) == 1 and loi[0].endswith('hong (ParserError)'), loi
    tq, _, _, loi = HV.doc(_tam('- 1\n- 2\n'))
    assert tq == HV.MAC_DINH and loi[0].endswith('khong phai bang')


def _thu_muc_chep():
    import shutil
    d = tempfile.mkdtemp()
    for f in ('nav2_params_mat.yaml', 'nav2_params_mat_rpp.yaml', 'bt_mat.xml', 'sep.py'):
        shutil.copy(os.path.join(DAY, f), d)
    return d


def _sua(d, tep, cu, moi):
    p = os.path.join(d, tep)
    s = open(p).read()
    assert s.count(cu) == 1, (tep, cu)
    open(p, 'w').write(s.replace(cu, moi))


def test_06_kiem_hanh_vi_dat_tren_tep_kho_va_bat_3_ca_lech():
    import kiem_hanh_vi as K
    tq, at, tt = dict(HV.MAC_DINH), dict(HV.MAC_DINH_AN_TOAN), dict(HV.MAC_DINH_THAN_THE)
    kq = K.kiem(DAY, tq, at, tt)
    assert all(d for _, _, d in kq) and len(kq) == 23 and {m for m, _, _ in kq} == {'a', 'b', 'c'}, kq
    # (a) so trong tep chay that lech yaml: progress 10 -> 12 (tep rpp), backup_dist 0.25 -> 0.30
    d = _thu_muc_chep()
    _sua(d, 'nav2_params_mat_rpp.yaml', 'movement_time_allowance: 10.0', 'movement_time_allowance: 12.0')
    _sua(d, 'bt_mat.xml', 'backup_dist="0.25"', 'backup_dist="0.30"')
    lech = [nd for m, nd, ok in K.kiem(d, tq, at, tt) if not ok]
    assert lech == ['nav2_params_mat_rpp.yaml progress_s = 12, yaml 10', 'bt_mat.xml lui_thoat_goc_m = 0.3, yaml 0.25'], lech
    # (b) thoi quen vuot than the: toc do lui 0.08 < v_min 0.092 (tep bt sua theo cho khop (a))
    d = _thu_muc_chep()
    _sua(d, 'bt_mat.xml', 'backup_speed="0.10"', 'backup_speed="0.08"')
    lech = [(m, nd) for m, nd, ok in K.kiem(d, dict(tq, toc_do_lui_mps=0.08), at, tt) if not ok]
    assert lech == [('b', 'toc_do_lui_mps 0.08 >= v_min_mps 0.092')], lech
    # (c) trai an toan: vung phanh 0.05 < 6 cm; sep.py co publisher Twist
    d = _thu_muc_chep()
    _sua(d, 'nav2_params_mat.yaml', 'points: [0.215, 0.08, 0.215, -0.08,', 'points: [0.195, 0.08, 0.195, -0.08,')
    _sua(d, 'nav2_params_mat_rpp.yaml', 'points: [0.215, 0.08, 0.215, -0.08,', 'points: [0.195, 0.08, 0.195, -0.08,')
    with open(os.path.join(d, 'sep.py'), 'a') as f:
        f.write('\n# gia: self.create_publisher(Twist, "/cmd_vel_avoid", 10)\n')
    lech = [(m, nd) for m, nd, ok in K.kiem(d, dict(tq, vung_phanh_m=0.05), at, tt) if not ok]
    assert lech == [('c', 'vung_phanh_m 0.05 >= cach_vat_toi_thieu_m 0.06'), ('c', 'sep_phat_toc_do false: sep.py CO publisher Twist')], lech
    # 09/11 A3: footprint_padding 2 costmap lech nhau (global con 0.03, local 0.02) => KHONG DAT
    d2 = _thu_muc_chep()
    p2 = os.path.join(d2, 'nav2_params_mat.yaml')
    s2 = open(p2).read()
    open(p2, 'w').write(s2.replace('footprint_padding: 0.03', 'footprint_padding: 0.02', 1))
    lech = [nd for m, nd, ok in K.kiem(d2, tq, at, tt) if not ok]
    assert lech == ['nav2_params_mat.yaml footprint_padding_m = nan, yaml 0.03'], lech
    # 09/11 A4: vung gac nho hon luat => KHONG DAT
    lech = [(m, nd) for m, nd, ok in K.kiem(DAY, dict(tq, gac_hong_m=0.05, gac_truoc_m=0.05), at, tt) if not ok]
    assert lech == [('b', 'gac_hong_m 0.05 >= cach_vat_toi_thieu_m 0.06'), ('b', 'gac_truoc_m 0.05 >= vung_phanh_m 0.07')], lech
    # thieu tep: KHONG DAT, khong vo
    os.remove(os.path.join(d, 'bt_mat.xml'))
    assert any(not ok and nd.startswith('bt_mat.xml: khong doc duoc (FileNotFoundError') for _, nd, ok in K.kiem(d, tq, at, tt))
    e = _io.StringIO()
    with contextlib.redirect_stdout(e):
        assert K.main(['--thu-muc', DAY, '--yaml', os.path.join(DAY, 'hanh_vi_xe.yaml')]) == 0
        assert K.main(['--thu-muc', d, '--yaml', os.path.join(DAY, 'hanh_vi_xe.yaml')]) == 1
    assert e.getvalue().splitlines()[23] == 'KIEM HANH VI: DAT (23 dong, 0 lech)'
    kx = open(os.path.join(DAY, 'kiem_xe.sh')).read()
    assert 'python3 ~/kiem_hanh_vi.py || KQ=1' in kx and kx.index('# 8e.') > kx.index('# 8d.')
