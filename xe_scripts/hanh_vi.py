#!/usr/bin/env python3
"""hanh_vi.py (bo 1e "nao xe", 09/11, CHUA THU TREN XE): doc ~/hanh_vi_xe.yaml (Pi: /home/project1/hanh_vi_xe.yaml).

So hanh vi cua gui_duong.py, lop_huong_dan.py, sep.py lay tu day: so(ten) = thoi_quen, than_the(ten), an_toan(ten).
Khong co tep / tep hong / thieu khoa / sai kieu => dung mac dinh trong ma (MAC_DINH = DUNG so o 2607fee) va in MOT dong
"CANH BAO hanh_vi: ..." ra stderr (moi tien trinh 1 lan). Bien HANH_VI_YAML chi duong dan khac (test, gia lap).
Kieu: mac dinh int => yaml phai int; float => int hoac float; bool => bool (bool KHONG tinh la so).
Module nay CHI DOC; khong bao gio ghi yaml (xe khong tu sua tep hanh vi).
"""
import os
import sys

TEP = os.path.expanduser('~/hanh_vi_xe.yaml')

MAC_DINH_AN_TOAN = {
    'cach_vat_toi_thieu_m': 0.06,
    'sep_phat_toc_do': False,
}
MAC_DINH_THAN_THE = {
    'v_min_mps': 0.092,
    'lidar_range_min_m': 0.1,
    'ban_kinh_xoay_m': 0.186,
    'o_luoi_m': 0.05,
    'rong_xe_m': 0.16,
    'dai_xe_m': 0.25,
}
# thoi_quen: ten -> (gia tri 2607fee, tep doc)
MAC_DINH = {
    've_lai_duong_hz': 1.0,     # bt_mat.xml RateController hz
    'progress_s': 10.0,    # progress_checker movement_time_allowance
    'vung_phanh_m': 0.07,    # collision_monitor ThanTruoc mui 0,215 - footprint 0,145
    'nhin_truoc_phanh_s': 1.5,     # collision_monitor ThanTruoc time_before_collision
    'lui_thoat_goc_m': 0.25,    # bt_mat BackUp backup_dist; gui_duong LUI_BT_M
    'toc_do_lui_mps': 0.1,     # bt_mat BackUp backup_speed; gui_duong cau_hinh lui_v
    'lui_ra_khe_m': 0.35,    # cau_hinh lui_m
    'cho_khe_s': 30.0,    # cau_hinh cho_khe (bien CHO_KHE)
    'tien_toi_thieu_m': 0.1,     # cau_hinh tien_min (bien TIEN_MIN)
    'gan_dich_m': 0.6,     # cau_hinh gan_dich_m (bien GAN_DICH_M)
    'gan_dich_s': 10.0,    # cau_hinh gan_dich_s (bien GAN_DICH_S)
    'cho_dich_s': 60.0,    # cau_hinh cho_dich (bien CHO_DICH)
    'cho_sep_s': 10.0,    # cau_hinh cho_sep (bien SEP_CHO_S)
    'sep_lenh_tuoi_max_s': 3.0,     # cau_hinh sep_tuoi (bien SEP_TUOI_MAX); doc_sep_lenh tuoi_max
    'toi_sai_so_m': 0.2,     # cau_hinh toi_sai_so (bien TOI_SAI_SO)
    'tf_tuoi_max_s': 1.0,     # cau_hinh tf_tuoi
    'cho_chon_tuyen_s': 2.0,     # cau_hinh cho_chon
    'nhip_in_cho_s': 5.0,     # cau_hinh nhip_cho
    'cho_san_sang_s': 15.0,    # cho_tf_costmap / cho_san_sang giay
    'nhip_vi_tri_s': 1.0,     # ~/vi_tri_xe moi 1 s
    'cho_may_chu_s': 5.0,     # wait_for_server + cho chap nhan dich
    'cho_huy_s': 2.0,     # cho Nav2 tra loi huy dich
    'cho_nghe_chon_tuyen_s': 3.0,     # /chon_tuyen cho co nguoi nghe
    'luot_lan_gui': 3,       # kieu cu: cau_hinh luot (bien LUOT)
    'cho_hong_s': 30.0,    # kieu cu: cau_hinh cho_hong (bien CHO_HONG)
    'qua_gio_lan_gui_s': 130.0,   # kieu cu: cau_hinh to (bien TO)
    'lop2_cho_s': 12.0,    # kieu cu: cau_hinh lop2_cho
    'lop2_tra_s': 2.0,     # kieu cu: cau_hinh lop2_tra
    'nhip_duong_day_s': 0.5,     # kieu cu: phat /duong_day 2 lan/giay
    'lo_quanh_xe_m': 0.35,    # R_THA
    'kinh_nghiem_so_lan': 3,       # N_CUM
    'kinh_nghiem_ban_kinh_m': 0.2,     # R_CUM
    'vung_ton_diem_m': 0.15,    # R_VUNG
    'vung_ton_diem_gia_tri': 50,      # GIA_TON
    'o_cam_gia_tri': 100,     # GIA_CAM
    't_cho_s': 30.0,    # T_CHO
    't_cho_them_s': 10.0,    # T_CHO_THEM
    't_cho_b_s': 60.0,    # T_CHO_B
    'lech_max_m': 0.3,     # LECH_MAX
    'tf_cu_s': 5.0,     # TF_CU
    'scan_cu_s': 0.5,     # SCAN_CU
    'r_tin_cay_m': 3.0,     # R_TIN_CAY
    'tc_tin': 0.7,     # TC_TIN
    'tc_nghi': 0.5,     # TC_NGHI
    't_khong_tin_s': 5.0,     # T_KHONG_TIN
    't_bao_s': 15.0,    # T_BAO
    'cua_so_ket_s': 2.0,     # CUA_SO_KET
    'lidar_doi_m': 0.02,    # LIDAR_DOI
    'pose_doi_m': 0.02,    # POSE_DOI
    'suc_lech_m': 0.15,    # SUC_LECH
    'suc_lech_max_m': 0.2,     # SUC_LECH_MAX
    'suc_nhanh_s': 0.05,    # SUC_NHANH
    'nhay_m': 0.1,     # NHAY_M
    'nhay_deg': 5.0,     # NHAY_DEG
    'nhay_dt_s': 1.0,     # NHAY_DT
    'nav2_lan_max': 2,       # NAV2_LAN_MAX
    'luot_thoat_max': 3,       # LUOT_THOAT_MAX
    'day_tuoi_s': 3.0,     # DAY_TUOI
    'p_day_xa_m': 0.4,     # P_DAY_XA
    'goc_dinh_deg': 5.0,     # GOC_DINH = radians(5.0)
    't_lai_thoat_s': 3.0,     # T_LAI_THOAT
    'khe_tang_m': 0.02,    # KHE_TANG
    'khe_cua_so_s': 3.0,     # KHE_CUA_SO
    'nhin_xa_m': 0.6,     # NHIN_XA
    'xa_truoc_m': 0.15,    # XA_TRUOC
    't_plan_xau_s': 10.0,    # T_PLAN_XAU
    'r_b_m': 0.2,     # R_B
    't_ra2_tra_s': 3.0,     # T_RA2_TRA
    't_ra2_cho_s': 12.0,    # T_RA2_CHO
    'r_vat_dich_m': 0.3,     # R_VAT_DICH
    'treo_s': 1.0,     # TREO
    'abort_nhanh_s': 2.0,     # ABORT_NHANH
    'cua_so_vong_s': 10.0,    # CUA_SO_VONG
    't_dung_s': 10.0,    # P_MAC_DINH['t_dung'] (tham so ROS)
    'dich_dung_m': 0.03,    # P_MAC_DINH['dich_dung'] (tham so ROS)
    'sat_vat_m': 0.1,     # P_MAC_DINH['sat_vat'] (tham so ROS)
    'sat_vat_tat_m': 0.12,    # P_MAC_DINH['sat_vat_tat'] (tham so ROS)
}

_DA = {}


def _dung_kieu(mac_dinh, x):
    if isinstance(mac_dinh, bool):
        return isinstance(x, bool)
    if isinstance(x, bool):
        return False
    if isinstance(mac_dinh, int):
        return isinstance(x, int)
    return isinstance(x, (int, float))


def _lay(nhom, mac_dinh, ten_nhom, loi):
    out = dict(mac_dinh)
    if nhom is None:
        loi.append('thieu muc %s' % ten_nhom)
        return out
    if not isinstance(nhom, dict):
        loi.append('muc %s khong phai bang' % ten_nhom)
        return out
    for k, md in mac_dinh.items():
        if k not in nhom:
            loi.append('thieu %s' % k)
            continue
        e = nhom[k]
        x = e.get('gia_tri') if isinstance(e, dict) else e
        if not _dung_kieu(md, x):
            loi.append('%s sai kieu (%r)' % (k, x))
            continue
        out[k] = float(x) if isinstance(md, float) else x
    for k in nhom:
        if k not in mac_dinh:
            loi.append('%s la (bo qua)' % k)
    return out


def doc(path=None):
    """-> (thoi_quen, an_toan, than_the, canh_bao): ba dict day du khoa (thieu / sai => mac dinh) va danh sach loi (rong = tot)."""
    p = path or os.environ.get('HANH_VI_YAML') or TEP
    try:
        import yaml
        with open(p) as f:
            y = yaml.safe_load(f)
    except ImportError:
        return dict(MAC_DINH), dict(MAC_DINH_AN_TOAN), dict(MAC_DINH_THAN_THE), ['khong co PyYAML']
    except OSError:
        return dict(MAC_DINH), dict(MAC_DINH_AN_TOAN), dict(MAC_DINH_THAN_THE), ['khong co %s' % p]
    except Exception as e:                       # noqa: BLE001 (yaml hong: moi loi phan tich)
        return dict(MAC_DINH), dict(MAC_DINH_AN_TOAN), dict(MAC_DINH_THAN_THE), ['%s hong (%s)' % (p, e.__class__.__name__)]
    if not isinstance(y, dict):
        return dict(MAC_DINH), dict(MAC_DINH_AN_TOAN), dict(MAC_DINH_THAN_THE), ['%s khong phai bang' % p]
    loi = []
    bn = y.get('ban_nang') if isinstance(y.get('ban_nang'), dict) else {}
    tq = _lay(y.get('thoi_quen'), MAC_DINH, 'thoi_quen', loi)
    at = _lay(bn.get('an_toan'), MAC_DINH_AN_TOAN, 'ban_nang.an_toan', loi)
    tt = _lay(bn.get('than_the'), MAC_DINH_THAN_THE, 'ban_nang.than_the', loi)
    return tq, at, tt, loi


def _nap():
    if not _DA:
        tq, at, tt, loi = doc()
        _DA.update(tq=tq, at=at, tt=tt, loi=loi)
        if loi:
            print('CANH BAO hanh_vi: dung mac dinh (%s)' % '; '.join(loi)[:300], file=sys.stderr, flush=True)
    return _DA


def so(ten):
    """So thoi_quen `ten` (KeyError neu khong co trong MAC_DINH: go sai ten la loi lap trinh)."""
    return _nap()['tq'][ten]


def than_the(ten):
    return _nap()['tt'][ten]


def an_toan(ten):
    return _nap()['at'][ten]


def nap_lai():
    """Doc lai tep (test). Tien trinh dang chay KHONG doc lai: so co dinh tu luc khoi dong."""
    _DA.clear()
    return _nap()
