#!/usr/bin/env python3
"""kiem_hanh_vi.py (bo 1e "nao xe" viec 3, 09/11, CHUA THU TREN XE): so ~/hanh_vi_xe.yaml voi tep chay that.

Chay: python3 ~/kiem_hanh_vi.py [--thu-muc DIR] [--yaml TEP]   (DIR mac dinh = thu muc chua tep nay; Pi: ~)
  (a) so trong nav2_params_mat.yaml, nav2_params_mat_rpp.yaml, bt_mat.xml khop thoi_quen: vung_phanh_m (mui ThanTruoc -
      mui footprint), nhin_truoc_phanh_s (time_before_collision), progress_s (movement_time_allowance), lui_thoat_goc_m
      (BackUp backup_dist), toc_do_lui_mps (BackUp backup_speed), ve_lai_duong_hz (RateController hz), footprint_padding_m
      (local + global costmap, phai bang nhau; 09/11 dung im A3);
  (b) thoi_quen khong vuot than_the: toc_do_lui_mps >= v_min_mps; lui_thoat_goc_m, lui_ra_khe_m, tien_toi_thieu_m >= o_luoi_m;
      09/11 dung im A4: gac_hong_m >= cach_vat_toi_thieu_m; gac_truoc_m >= vung_phanh_m;
  (c) thoi_quen khong trai an_toan: vung_phanh_m >= cach_vat_toi_thieu_m; sep_phat_toc_do false => sep.py khong tao
      publisher Twist.
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


def _so_nav2(path):
    """-> {ten thoi_quen: so} doc tu mot tep nav2_params_mat*.yaml."""
    import yaml
    with open(path) as f:
        y = yaml.safe_load(f)
    cm = y['collision_monitor']['ros__parameters']['ThanTruoc']
    fp = ast.literal_eval(y['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    pad = [y[k][k]['ros__parameters'].get('footprint_padding', 0.01) for k in ('local_costmap', 'global_costmap')]   # mac dinh Nav2 0.01
    return {'footprint_padding_m': float(pad[0]) if pad[0] == pad[1] else float('nan'),     # 2 costmap lech nhau => nan => KHONG DAT
            'vung_phanh_m': round(max(cm['points'][0::2]) - max(p[0] for p in fp), 6),
            'nhin_truoc_phanh_s': float(cm['time_before_collision']),
            'progress_s': float(y['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance'])}


def _so_bt(path):
    with open(path) as f:
        x = f.read()
    lay = lambda mau: float(re.search(mau, x).group(1))
    return {'lui_thoat_goc_m': lay(r'<BackUp[^>]*backup_dist="([0-9.]+)"'), 'toc_do_lui_mps': lay(r'<BackUp[^>]*backup_speed="([0-9.]+)"'),
            've_lai_duong_hz': lay(r'<RateController hz="([0-9.]+)"')}


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
    out.append(('b', 'toc_do_lui_mps %g >= v_min_mps %g' % (tq['toc_do_lui_mps'], tt['v_min_mps']), tq['toc_do_lui_mps'] >= tt['v_min_mps']))
    for ten in ('lui_thoat_goc_m', 'lui_ra_khe_m', 'tien_toi_thieu_m'):
        out.append(('b', '%s %g >= o_luoi_m %g' % (ten, tq[ten], tt['o_luoi_m']), tq[ten] >= tt['o_luoi_m'] - SAI_SO))
    # 09/11 dung im ban 2 A4: vung gac khong nho hon luat cach vat / vung phanh
    out.append(('b', 'gac_hong_m %g >= cach_vat_toi_thieu_m %g' % (tq['gac_hong_m'], at['cach_vat_toi_thieu_m']),
                tq['gac_hong_m'] >= at['cach_vat_toi_thieu_m'] - SAI_SO))
    out.append(('b', 'gac_truoc_m %g >= vung_phanh_m %g' % (tq['gac_truoc_m'], tq['vung_phanh_m']), tq['gac_truoc_m'] >= tq['vung_phanh_m'] - SAI_SO))
    # 09/11 gia X: dai cac so gia X; vung ton diem khong bao gio thanh o cam (tinh_tuyen.NGUONG_CAM)
    out.append(('b', 'gia_x_r_m %g >= o_luoi_m %g' % (tq['gia_x_r_m'], tt['o_luoi_m']), tq['gia_x_r_m'] >= tt['o_luoi_m'] - SAI_SO))
    out.append(('b', 'gia_x_v_ms %g >= v_min_mps %g' % (tq['gia_x_v_ms'], tt['v_min_mps']), tq['gia_x_v_ms'] >= tt['v_min_mps']))
    out.append(('b', 'gia_x_giay_mac_dinh %g >= 0' % tq['gia_x_giay_mac_dinh'], tq['gia_x_giay_mac_dinh'] >= 0))
    out.append(('b', 'so_vong_khe %d >= 1' % tq['so_vong_khe'], tq['so_vong_khe'] >= 1))     # 09/11 18:44 ②a
    out.append(('b', 'cho_lai_s %g > 0 (het vong cho nguoi, khong thoat)' % tq['cho_lai_s'], tq['cho_lai_s'] > 0))   # 09/11 18:47 (i)
    import tinh_tuyen as TT
    out.append(('c', 'vung_ton_diem_gia_tri %g < NGUONG_CAM %g' % (tq['vung_ton_diem_gia_tri'], TT.NGUONG_CAM),
                tq['vung_ton_diem_gia_tri'] < TT.NGUONG_CAM))
    out.append(('c', 'vung_phanh_m %g >= cach_vat_toi_thieu_m %g' % (tq['vung_phanh_m'], at['cach_vat_toi_thieu_m']),
                tq['vung_phanh_m'] >= at['cach_vat_toi_thieu_m'] - SAI_SO))
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
