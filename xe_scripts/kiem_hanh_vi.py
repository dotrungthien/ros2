#!/usr/bin/env python3
"""kiem_hanh_vi.py (bo 1e "nao xe" viec 3, 09/11, CHUA THU TREN XE): so ~/hanh_vi_xe.yaml voi tep chay that.

Chay: python3 ~/kiem_hanh_vi.py [--thu-muc DIR] [--yaml TEP]   (DIR mac dinh = thu muc chua tep nay; Pi: ~)
  (a) so trong nav2_params_mat.yaml, nav2_params_mat_rpp.yaml, bt_mat.xml khop thoi_quen: vung_phanh_m (mui ThanTruoc -
      mui footprint), nhin_truoc_phanh_s (time_before_collision), progress_s (movement_time_allowance), ve_lai_duong_hz
      (RateController hz); 23:08 sep LiDAR: xoay_thoat_rad_s (min_rotational_vel), behavior_plugins co drive_on_heading + backup +
      spin, bt_mat.xml KHONG co BackUp (muc 5: mot chu thoat); footprint_padding_m
      (local + global costmap, phai bang nhau; 09/11 dung im A3); 09/11 20:33 vung quanh xe: inflation_radius_m, cost_scaling_factor
      (local + global bang nhau; tep RPP: inflation_cost_scaling_factor cung so), backup_nhin_truoc_s (simulate_ahead_time);
  (b) thoi_quen khong vuot than_the: toc_do_thoat_mps >= v_min_mps; tien_toi_thieu_m >= o_luoi_m;
      09/11 20:33: vang khong nho hon do (hong, truoc, duoi, phanh); inflation_radius_m >= rong_xe_m/2 + vang_hong_m;
      vat_dai_max_m < tuong_dai_min_m; so_luot_chan, so_luot_gan_dich, thoat_hong_max, thoat_buoc_max >= 1; nhac_doi_m > truoc_vat_do_m;
      cho_chat_m >= cach_vat_toi_thieu_m; thoat_diem_min_m <= thoat_diem_max_m, thoat_trong_m - truoc_vat_do_m >= thoat_diem_min_m;
  (c) thoi_quen khong trai an_toan: vung_phanh_m >= cach_vat_toi_thieu_m; sep_phat_toc_do false => sep.py khong tao
      publisher Twist; 09/11 20:33: BackUp (23:08: toc_do_thoat_mps, backup_nhin_truoc_s, chu ky 10 Hz) nhin truoc >= duoi_do_m.
In bang ngan (DAT / KHONG DAT moi dong) roi "KIEM HANH VI: DAT | KHONG DAT"; ma thoat 0 / 1. CHI DOC, khong sua tep nao.
"""
import ast
import os
import re
import sys

import hanh_vi as HV

DAY = os.path.dirname(os.path.abspath(__file__))
TEP_NAV2 = ('nav2_params_mat.yaml', 'nav2_params_mat_rpp.yaml')
SAI_SO = 1e-6
CHU_KY_BACKUP_HZ = 10.0     # behavior_server cycle_frequency (nav2_params_mat*.yaml)


def _bang(a, b):
    return float(a) if a == b else float('nan')      # 2 costmap / 2 cho lech nhau => nan => KHONG DAT


def nhin_lui_m(v, t, hz=CHU_KY_BACKUP_HZ):
    """Humble drive_on_heading.hpp isCollisionFree: mo phong int(hz * t) chu ky, vi tri k * v / hz (k = 0..n-1) => xa nhat."""
    return v * (int(hz * t + 1e-9) - 1) / hz


def _so_nav2(path):
    """-> {ten thoi_quen: so} doc tu mot tep nav2_params_mat*.yaml."""
    import yaml
    with open(path) as f:
        y = yaml.safe_load(f)
    cm = y['collision_monitor']['ros__parameters']['ThanTruoc']
    fp = ast.literal_eval(y['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    pad = [y[k][k]['ros__parameters'].get('footprint_padding', 0.01) for k in ('local_costmap', 'global_costmap')]   # mac dinh Nav2 0.01
    inf = [y[k][k]['ros__parameters']['inflation_layer'] for k in ('local_costmap', 'global_costmap')]
    fpth = y['controller_server']['ros__parameters']['FollowPath']
    cs = [i['cost_scaling_factor'] for i in inf]
    if 'regulated_pure_pursuit' in fpth['plugin'].lower():
        cs.append(fpth.get('inflation_cost_scaling_factor', 3.0))         # mac dinh RPP Humble 3.0
    return {'footprint_padding_m': float(pad[0]) if pad[0] == pad[1] else float('nan'),     # 2 costmap lech nhau => nan => KHONG DAT
            'inflation_radius_m': _bang(inf[0]['inflation_radius'], inf[1]['inflation_radius']),
            'cost_scaling_factor': float(cs[0]) if len(set(cs)) == 1 else float('nan'),
            'backup_nhin_truoc_s': float(y['behavior_server']['ros__parameters']['simulate_ahead_time']),
            'vung_phanh_m': round(max(cm['points'][0::2]) - max(p[0] for p in fp), 6),
            'nhin_truoc_phanh_s': float(cm['time_before_collision']),
            'progress_s': float(y['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']),
            'xoay_thoat_rad_s': float(y['behavior_server']['ros__parameters']['min_rotational_vel'])}


def _plugin_nav2(path):
    """23:08 sep LiDAR: behavior_server phai co spin / backup / drive_on_heading (dong tac thoat cua gui_duong)."""
    import yaml
    with open(path) as f:
        b = yaml.safe_load(f)['behavior_server']['ros__parameters']
    can = {'spin': 'nav2_behaviors/Spin', 'backup': 'nav2_behaviors/BackUp', 'drive_on_heading': 'nav2_behaviors/DriveOnHeading'}
    return all(k in b['behavior_plugins'] and b.get(k, {}).get('plugin') == v for k, v in can.items())


def _so_bt(path):
    with open(path) as f:
        x = f.read()
    lay = lambda mau: float(re.search(mau, x).group(1))
    return {'ve_lai_duong_hz': lay(r'<RateController hz="([0-9.]+)"')}


def kiem(thu_muc, tq, at, tt):
    """-> [(muc, noi dung, dat)]: muc 'a' | 'b' | 'c'."""
    out = []
    for ten_tep, doc in [(t, _so_nav2) for t in TEP_NAV2] + [('bt_mat.xml', _so_bt)]:
        p = os.path.join(thu_muc, ten_tep)
        try:
            so = doc(p)
        except Exception as e:                   # noqa: BLE001 (thieu tep / khoa / hong: mot dong KHONG DAT)
            out.append(('a', '%s: khong doc duoc (%s: %s)' % (ten_tep, e.__class__.__name__, str(e)[:60]), False))
            continue
        for ten, x in so.items():
            out.append(('a', '%s %s = %g, yaml %g' % (ten_tep, ten, x, tq[ten]), abs(x - tq[ten]) <= SAI_SO))   # nan => False
        if ten_tep in TEP_NAV2:
            co = _plugin_nav2(p)
            out.append(('a', '%s behavior_plugins spin + backup + drive_on_heading: %s' % (ten_tep, 'co' if co else 'THIEU'), co))
        else:
            with open(p) as f:
                bt = f.read()
            co = '<BackUp ' in bt
            out.append(('a', 'bt_mat.xml %s BackUp tu dong (23:08 muc 5: mot chu thoat)' % ('CON' if co else 'khong co'), not co))
    out.append(('b', 'toc_do_thoat_mps %g >= v_min_mps %g' % (tq['toc_do_thoat_mps'], tt['v_min_mps']), tq['toc_do_thoat_mps'] >= tt['v_min_mps']))
    out.append(('b', 'tien_toi_thieu_m %g >= o_luoi_m %g' % (tq['tien_toi_thieu_m'], tt['o_luoi_m']), tq['tien_toi_thieu_m'] >= tt['o_luoi_m'] - SAI_SO))
    # 09/11 20:33 vung quanh xe: vang (huong dan) khong nho hon do (luat cung)
    for v, d, nhom in (('vang_hong_m', 'cach_vat_toi_thieu_m', at), ('vang_truoc_m', 'truoc_vat_do_m', at), ('vang_sau_m', 'duoi_do_m', at),
                       ('vang_truoc_m', 'vung_phanh_m', tq)):
        out.append(('b', '%s %g >= %s %g' % (v, tq[v], d, nhom[d]), tq[v] >= nhom[d] - SAI_SO))
    r = tt['rong_xe_m'] / 2 + tq['vang_hong_m']
    out.append(('b', 'inflation_radius_m %g >= rong_xe_m/2 + vang_hong_m %g' % (tq['inflation_radius_m'], r), tq['inflation_radius_m'] >= r - SAI_SO))
    out.append(('b', 'vat_dai_max_m %g < tuong_dai_min_m %g' % (tq['vat_dai_max_m'], tq['tuong_dai_min_m']), tq['vat_dai_max_m'] < tq['tuong_dai_min_m']))
    # 09/11 gia X: dai cac so gia X; vung ton diem khong bao gio thanh o cam (tinh_tuyen.NGUONG_CAM)
    out.append(('b', 'gia_x_r_m %g >= o_luoi_m %g' % (tq['gia_x_r_m'], tt['o_luoi_m']), tq['gia_x_r_m'] >= tt['o_luoi_m'] - SAI_SO))
    out.append(('b', 'gia_x_v_ms %g >= v_min_mps %g' % (tq['gia_x_v_ms'], tt['v_min_mps']), tq['gia_x_v_ms'] >= tt['v_min_mps']))
    out.append(('b', 'gia_x_giay_mac_dinh %g >= 0' % tq['gia_x_giay_mac_dinh'], tq['gia_x_giay_mac_dinh'] >= 0))
    for ten in ('so_luot_chan', 'so_luot_gan_dich', 'thoat_hong_max', 'thoat_buoc_max'):     # 20:33 / 23:08 sep LiDAR
        out.append(('b', '%s %d >= 1' % (ten, tq[ten]), tq[ten] >= 1))
    out.append(('b', 'nhac_doi_m %g > truoc_vat_do_m %g' % (tq['nhac_doi_m'], at['truoc_vat_do_m']), tq['nhac_doi_m'] > at['truoc_vat_do_m']))
    out.append(('b', 'cho_chat_m %g >= cach_vat_toi_thieu_m %g' % (tq['cho_chat_m'], at['cach_vat_toi_thieu_m']),
                tq['cho_chat_m'] >= at['cach_vat_toi_thieu_m']))
    r = tq['thoat_trong_m'] - at['truoc_vat_do_m']        # sep_lidar.chon_sector: r = min(max, trong - 0,10) >= min
    out.append(('b', 'thoat_diem_min_m %g <= thoat_diem_max_m %g; thoat_trong_m - truoc_vat_do_m %g >= thoat_diem_min_m' % (
        tq['thoat_diem_min_m'], tq['thoat_diem_max_m'], r), tq['thoat_diem_min_m'] <= tq['thoat_diem_max_m'] and r >= tq['thoat_diem_min_m'] - SAI_SO))
    out.append(('b', 'cho_lai_s %g > 0 (het luot cho nguoi, khong thoat)' % tq['cho_lai_s'], tq['cho_lai_s'] > 0))   # 09/11 18:47 (i)
    # 10/11 A+: phan loai ba dau hieu
    out.append(('b', 'phan_loai_cum %s thuoc {cu, ba_dau_hieu}' % tq['phan_loai_cum'], tq['phan_loai_cum'] in ('cu', 'ba_dau_hieu')))
    out.append(('b', 'cung_r_min_m %g < cung_r_max_m %g, cung_so_diem %d >= 3' % (tq['cung_r_min_m'], tq['cung_r_max_m'], tq['cung_so_diem']),
                tq['cung_r_min_m'] < tq['cung_r_max_m'] and tq['cung_so_diem'] >= 3))
    out.append(('b', 'ong: 0 < ong_ti_le %g <= 1, ong_so_quet_min %d >= 2, ong_cach_cum_m %g > 0' % (tq['ong_ti_le'], tq['ong_so_quet_min'], tq['ong_cach_cum_m']),
                0 < tq['ong_ti_le'] <= 1 and tq['ong_so_quet_min'] >= 2 and tq['ong_cach_cum_m'] > 0))     # 10/11 A++
    out.append(('c', 'khong_cham_m %g > 0 va <= cach_vat_toi_thieu_m %g (tuong hong chi nhe hon vat)' % (tq['khong_cham_m'], at['cach_vat_toi_thieu_m']),
                0 < tq['khong_cham_m'] <= at['cach_vat_toi_thieu_m'] + SAI_SO))
    import tinh_tuyen as TT
    out.append(('c', 'vung_ton_diem_gia_tri %g < NGUONG_CAM %g' % (tq['vung_ton_diem_gia_tri'], TT.NGUONG_CAM),
                tq['vung_ton_diem_gia_tri'] < TT.NGUONG_CAM))
    out.append(('c', 'vung_phanh_m %g >= cach_vat_toi_thieu_m %g' % (tq['vung_phanh_m'], at['cach_vat_toi_thieu_m']),
                tq['vung_phanh_m'] >= at['cach_vat_toi_thieu_m'] - SAI_SO))
    L = nhin_lui_m(tq['toc_do_thoat_mps'], tq['backup_nhin_truoc_s'])
    out.append(('c', 'BackUp / DriveOnHeading nhin %.2f m >= duoi_do_m %g (dung khi vat cach %.2f-%.2f m)' % (
        L, at['duoi_do_m'], L - tq['toc_do_thoat_mps'] / CHU_KY_BACKUP_HZ, L + tt['o_luoi_m']), L >= at['duoi_do_m'] - SAI_SO))
    if not at['sep_phat_toc_do']:
        try:
            with open(os.path.join(thu_muc, 'sep.py')) as f:
                co = re.search(r'create_publisher\(\s*Twist', f.read()) is not None
            out.append(('c', 'sep_phat_toc_do false: sep.py %s publisher Twist' % ('CO' if co else 'khong co'), not co))
        except OSError:
            out.append(('c', 'sep_phat_toc_do false: khong doc duoc sep.py', False))
    return out


def main(argv):
    thu_muc, tep = DAY, None
    if '--thu-muc' in argv:
        thu_muc = argv[argv.index('--thu-muc') + 1]
    if '--yaml' in argv:
        tep = argv[argv.index('--yaml') + 1]
    tq, at, tt, loi = HV.doc(tep)
    if loi:
        print('CANH BAO hanh_vi: dung mac dinh (%s)' % '; '.join(loi)[:300])
    kq = kiem(thu_muc, tq, at, tt)
    for muc, nd, dat in kq:
        print('%-10s (%s) %s' % ('DAT' if dat else 'KHONG DAT', muc, nd))
    ok = all(d for _, _, d in kq)
    print('KIEM HANH VI: %s (%d dong, %d lech)' % ('DAT' if ok else 'KHONG DAT', len(kq), sum(not d for _, _, d in kq)))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
