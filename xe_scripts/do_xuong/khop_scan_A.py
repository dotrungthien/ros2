#!/usr/bin/env python3
"""do_xuong/khop_scan_A.py (viec 118 B10, soan 06/11/2026, CHUA CHAY TREN XE).

Khop MOT scan voi map_xuong_v3 nhu RA_SOAT_118 B2/B2b, ra xe that lech bao nhieu so voi tu the tin (khung than).
  Cham diem: ty le tia roi vao o vat v3, cho sai 1 o (o do hoac 8 o ke); "dung o" (sai 0 o) la tieu chi phu khi hoa.
  Do: dx, dy +-12 cm buoc 0,5 cm; dtheta +-6 do buoc 0,5 do; xoay quanh tam xe. Scan -> than: lai_xe_lidar.scan_to_xy.
  Doi sang khung than theo goc TIN: doc = dx cos + dy sin (+ = ve mui), ngang = -dx sin + dy cos (+ = trai).

Hai cach dung:
  1) Tren Pi (xe dang chay SLAM, dung yen):  python3 ~/do_xuong/khop_scan_A.py chup [tep]
     Lay 1 /scan va tu the tin map -> base_footprint dung luc scan, ghi ra tep (mac dinh ~/scan_<gio>.txt).
     Chi can rclpy + tf2_ros; KHONG phat lenh.
  2) Khop (may co numpy + PIL, trong kho):  python3 do_xuong/khop_scan_A.py <tep_scan> [x y goc_do] [--ban-do tep.yaml]
     Khong ghi x y goc_do thi lay dong '# tu_the_tin:' trong tep (do 'chup' ghi).
"""
import argparse
import math
import os
import sys
import time

DAY = os.path.dirname(os.path.abspath(__file__))
BAN_DO = os.path.join(DAY, '..', 'du_lieu_0511_toi', 'map_xuong_v3.yaml')


# ---------- 1) chup ----------
def chup(tep):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('khop_scan_A_chup')
    buf = Buffer()
    TransformListener(buf, node)
    sc = []                                          # 20 scan gan nhat; tf dung luc scan moi nhat thuong toi tre vai chuc ms
    node.create_subscription(LaserScan, '/scan', lambda m: (sc.append(m), sc.pop(0) if len(sc) > 20 else None), qos_profile_sensor_data)
    han = time.monotonic() + 10.0
    tf, m = None, None
    while time.monotonic() < han and tf is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        for m in reversed(sc):                       # scan moi nhat ma da co tf dung luc do
            try:
                tf = buf.lookup_transform('map', 'base_footprint', Time.from_msg(m.header.stamp), Duration(seconds=0.0))
                break
            except Exception:                       # chua co (hoac khong con) tf dung luc scan nay
                pass
    if tf is None:
        print('LOI: khong lay duoc /scan va tf map -> base_footprint dung luc scan trong 10 s.')
        return 1
    q = tf.transform.rotation
    g = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    x, y = tf.transform.translation.x, tf.transform.translation.y
    tep = tep or os.path.expanduser(time.strftime('~/scan_%H%M%S.txt'))
    with open(tep, 'w') as f:
        f.write('# tu_the_tin: %.4f %.4f %.3f\n' % (x, y, g))
        f.write('header:\n  stamp:\n    sec: %d\n    nanosec: %d\n  frame_id: %s\n' % (m.header.stamp.sec, m.header.stamp.nanosec, m.header.frame_id))
        for k in ('angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max'):
            f.write('%s: %r\n' % (k, float(getattr(m, k))))
        f.write('ranges:\n' + ''.join('- %r\n' % float(r) for r in m.ranges))
        f.write('intensities:\n' + ''.join('- %r\n' % float(r) for r in m.intensities))
    print('DA CHUP %s: %d tia, tu the tin (%.3f; %.3f; %.1f do), stamp %d.%09d' % (
        tep, len(m.ranges), x, y, g, m.header.stamp.sec, m.header.stamp.nanosec))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


# ---------- 2) khop ----------
def doc_scan(tep):
    sys.path.insert(0, os.path.join(DAY, '..'))         # lai_xe_lidar.py: trong kho la xe_scripts/, tren Pi la ~/
    import lai_xe_lidar as L
    d = open(tep).read().split('\n')
    g = lambda k: float([l for l in d if l.startswith(k + ':')][0].split()[1])

    class Sc:
        pass
    sc = Sc()
    sc.ranges = [float(l[2:]) for l in d[d.index('ranges:') + 1:d.index('intensities:')] if l.startswith('- ')]
    sc.angle_min, sc.angle_increment, sc.range_min, sc.range_max = g('angle_min'), g('angle_increment'), g('range_min'), g('range_max')
    tu_the = [l.split(':', 1)[1].split() for l in d if l.startswith('# tu_the_tin:')]
    return L.scan_to_xy(sc), len(sc.ranges), (tuple(float(v) for v in tu_the[0]) if tu_the else None)


def doc_ban_do(yaml_tep):
    import numpy as np
    from PIL import Image
    kv = {}
    for l in open(yaml_tep):
        if ':' in l and not l.lstrip().startswith('#'):
            k, v = l.split(':', 1)
            kv[k.strip()] = v.strip()
    o = [float(v) for v in kv['origin'].strip('[]').split(',')]
    res, nguong = float(kv['resolution']), float(kv['occupied_thresh'])
    am = int(kv.get('negate', '0')) == 1
    im = np.asarray(Image.open(os.path.join(os.path.dirname(yaml_tep), kv['image'])).convert('L'), dtype=float)
    p = im / 255.0 if am else (255.0 - im) / 255.0
    vat = (p > nguong)[::-1, :].T                       # vat[i, j]: i theo x, j theo y (hang duoi anh = y nho nhat)
    return vat, o[0], o[1], res


def khop(P, X, Y, G, vat, OX, OY, RES):
    import numpy as np
    MW, MH = vat.shape
    pad = np.pad(vat, 1)
    vat1 = np.zeros_like(vat)
    for a in range(3):
        for b in range(3):
            vat1 |= pad[a:a + MW, b:b + MH]
    DX = np.array([k * 0.005 for k in range(-24, 25)])
    DT = np.array([math.radians(k * 0.5) for k in range(-12, 13)])
    P = np.asarray(P)
    S = np.zeros((49, 49, 25))
    S0 = np.zeros_like(S)
    gx, gy = np.meshgrid(DX, DX, indexing='ij')         # [a, b] = (dx, dy)
    for c, dt in enumerate(DT):
        t = G + dt
        qx = X + P[:, 0] * math.cos(t) - P[:, 1] * math.sin(t)
        qy = Y + P[:, 0] * math.sin(t) + P[:, 1] * math.cos(t)
        i = np.floor((qx[None, None, :] + gx[..., None] - OX) / RES).astype(int)
        j = np.floor((qy[None, None, :] + gy[..., None] - OY) / RES).astype(int)
        ok = (i >= 0) & (i < MW) & (j >= 0) & (j < MH)
        ic, jc = np.where(ok, i, 0), np.where(ok, j, 0)
        S[:, :, c] = (vat1[ic, jc] & ok).mean(axis=2)
        S0[:, :, c] = (vat[ic, jc] & ok).mean(axis=2)
    best = np.unravel_index(np.argmax(S * 1e6 + S0 * 1e3), S.shape)
    dinh = []
    for a in range(49):
        for b in range(49):
            for c in range(25):
                if S[a, b, c] >= S[max(a - 1, 0):a + 2, max(b - 1, 0):b + 2, max(c - 1, 0):c + 2].max():
                    dinh.append((S[a, b, c], S0[a, b, c], a, b, c))
    xa = lambda a, b, c: math.hypot(DX[a] - DX[best[0]], DX[b] - DX[best[1]]) > 0.02 or abs(DT[c] - DT[best[2]]) > math.radians(1.0)
    hai = max((p for p in dinh if xa(*p[2:])), default=None)
    hoa = [tuple(p) for p in np.argwhere(S >= S[best] - 1e-12)]
    return DX, DT, S, S0, best, hai, hoa


def than(dx, dy, G):
    return dx * math.cos(G) + dy * math.sin(G), -dx * math.sin(G) + dy * math.cos(G)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'chup':
        return chup(sys.argv[2] if len(sys.argv) > 2 else None)
    ap = argparse.ArgumentParser(description='Khop 1 scan voi map_xuong_v3 (nhu RA_SOAT_118 B2/B2b).')
    ap.add_argument('tep_scan')
    ap.add_argument('tu_the', nargs='*', type=float, help='x y goc_do tin (bo trong = lay dong # tu_the_tin trong tep)')
    ap.add_argument('--ban-do', default=BAN_DO, help='tep .yaml cua ban do (mac dinh du_lieu_0511_toi/map_xuong_v3.yaml)')
    a = ap.parse_args()
    P, n, tt = doc_scan(a.tep_scan)
    if a.tu_the:
        if len(a.tu_the) != 3:
            ap.error('can du 3 so: x y goc_do')
        tt = tuple(a.tu_the)
    if tt is None:
        ap.error('tep khong co dong "# tu_the_tin:": ghi them x y goc_do')
    X, Y, G = tt[0], tt[1], math.radians(tt[2])
    vat, OX, OY, RES = doc_ban_do(a.ban_do)
    DX, DT, S, S0, best, hai, hoa = khop(P, X, Y, G, vat, OX, OY, RES)
    f = lambda a_, b_, c_: 'dx %+.1f dy %+.1f cm dth %+.1f do | than: doc %+.1f ngang %+.1f cm' % (
        DX[a_] * 100, DX[b_] * 100, math.degrees(DT[c_]), *[v * 100 for v in than(DX[a_], DX[b_], G)])
    print('scan %s: tu the tin (%.3f; %.3f; %.1f do), tia hop le %d/%d, ban do %s' % (
        os.path.basename(a.tep_scan), X, Y, tt[2], len(P), n, os.path.basename(a.ban_do)))
    print('  (0,0,0): diem %.3f (dung o %.3f)' % (S[24, 24, 12], S0[24, 24, 12]))
    print('  CAO NHAT: %s | diem %.3f (dung o %.3f), |lech| %.1f cm' % (f(*best), S[best], S0[best], math.hypot(DX[best[0]], DX[best[1]]) * 100))
    if hai:
        print('  dinh thu hai: %s | diem %.3f (dung o %.3f)' % (f(*hai[2:]), hai[0], hai[1]))
    tv = [than(DX[p[0]], DX[p[1]], G) for p in hoa]
    print('  tap hoa diem: %d nghiem; than doc %+.1f..%+.1f, ngang %+.1f..%+.1f cm; dth %+.1f..%+.1f do' % (
        len(hoa), min(v[0] for v in tv) * 100, max(v[0] for v in tv) * 100, min(v[1] for v in tv) * 100,
        max(v[1] for v in tv) * 100, math.degrees(min(DT[p[2]] for p in hoa)), math.degrees(max(DT[p[2]] for p in hoa))))
    print('  tu the chinh (theo dinh): (%.3f; %.3f; %.1f do)' % (X + DX[best[0]], Y + DX[best[1]], math.degrees(G + DT[best[2]])))
    if S[best] < 0.9:
        print('  CANH BAO: diem cao nhat < 0,90: scan khop kem (vat la? sai ban do? lech > 12 cm / 6 do?). Dung tin so tren.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
