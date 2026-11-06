#!/usr/bin/env bash
# do_xuong/do_toc_do.sh (viec 118 B10, soan 06/11/2026, CHUA CHAY TREN XE).
# Phat /cmd_vel_teleop thang 0,05 m/s trong 10 s (twist_mux uu tien 100: de len Nav2 20 va phu lai 60),
# ghi odom dau / cuoi de so voi thuoc do cua Thien. Dong ket qua noi them vao do_toc_do.csv canh tep nay.
# HUY: Ctrl+C (hoac mat ssh / kill) -> phat Twist 0 trong 1 s roi thoat. Neu tien trinh bi giet cung (kill -9),
# twist_mux tu bo lenh teleop sau 0,3 s (timeout trong twist_mux.yaml).
# Truoc khi chay: Nav2 KHONG co dich, khong chay gui_duong / chuyen.sh (het 10 s, lenh tra ve uu tien thap hon).
# Chay:  source ~/ros2_ws/install/setup.bash; bash ~/do_xuong/do_toc_do.sh
python3 -c 'import rclpy' 2>/dev/null || { echo "LOI: chua source ROS (source ~/ros2_ws/install/setup.bash)"; exit 1; }
CSV="$(cd "$(dirname "$0")" && pwd)/do_toc_do.csv" exec python3 - <<'PY'
import math, os, signal, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.signals import SignalHandlerOptions

V, T_CHAY, T_DUNG, HZ = 0.05, 10.0, 1.0, 20.0


def huy(*_):
    raise KeyboardInterrupt


for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(s, huy)

# rclpy KHONG cai bat tin hieu rieng: neu cai, Ctrl+C se tat context truoc khi kip phat Twist 0
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('do_toc_do')
pub = node.create_publisher(Twist, '/cmd_vel_teleop', 10)
od = {}
node.create_subscription(Odometry, '/odom', lambda m: od.update(p=m.pose.pose, t=time.monotonic()), 10)


def tu_the():
    p = od['p']
    return p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w)


def phat(v, giay):
    het = time.monotonic() + giay
    while time.monotonic() < het:
        tw = Twist(); tw.linear.x = v
        pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=1.0 / HZ)


def ghi(dau, cuoi, t, trang_thai):
    q = math.hypot(cuoi[0] - dau[0], cuoi[1] - dau[1])
    doc = (cuoi[0] - dau[0]) * math.cos(dau[2]) + (cuoi[1] - dau[1]) * math.sin(dau[2])
    dg = math.degrees(math.atan2(math.sin(cuoi[2] - dau[2]), math.cos(cuoi[2] - dau[2])))
    print('odom dau  (%.3f; %.3f; %.1f do)' % (dau[0], dau[1], math.degrees(dau[2])))
    print('odom cuoi (%.3f; %.3f; %.1f do)' % (cuoi[0], cuoi[1], math.degrees(cuoi[2])))
    print('%s: lenh %.2f m/s trong %.1f s | odom: quang thang %.3f m, doc huong dau %.3f m, goc doi %+.1f do' % (trang_thai, V, t, q, doc, dg))
    print('-> Do THUOC quang thang tu vach dau toi vach cuoi (cung mot diem tren xe) roi ghi canh so odom.')
    moi = not os.path.exists(os.environ['CSV'])
    with open(os.environ['CSV'], 'a') as f:
        if moi:
            f.write('gio,trang_thai,v_lenh,t_chay_s,x0,y0,goc0_do,x1,y1,goc1_do,odom_quang_m,odom_doc_m,goc_doi_do\n')
        f.write('%s,%s,%.3f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f\n' % (
            time.strftime('%Y-%m-%d %H:%M:%S'), trang_thai, V, t, dau[0], dau[1], math.degrees(dau[2]),
            cuoi[0], cuoi[1], math.degrees(cuoi[2]), q, doc, dg))


dau, t0, trang_thai = None, None, 'XONG'
try:
    han = time.monotonic() + 5.0
    while 'p' not in od and time.monotonic() < han:
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'p' not in od:
        print('LOI: khong nhan /odom sau 5 s (odom_tu_lenh chua chay?). KHONG phat lenh.')
        sys.exit(1)
    dau = tu_the()
    print('CHAY: %.2f m/s thang trong %.0f s. Ctrl+C = HUY (xe dung ngay); tu tab khac: kill -INT %d' % (V, T_CHAY, os.getpid()), flush=True)
    t0 = time.monotonic()
    phat(V, T_CHAY)
except KeyboardInterrupt:
    trang_thai = 'HUY'
finally:
    t_chay = time.monotonic() - t0 if t0 else 0.0
    try:
        phat(0.0, T_DUNG)                        # dung chac chan: Twist 0 lien tuc 1 s
    except KeyboardInterrupt:
        phat(0.0, T_DUNG)
    print('DA DUNG (Twist 0 trong %.0f s).' % T_DUNG)
if dau is not None:
    han = time.monotonic() + 0.5                 # lay odom moi nhat sau khi dung
    while time.monotonic() < han:
        rclpy.spin_once(node, timeout_sec=0.05)
    ghi(dau, tu_the(), t_chay, trang_thai)
node.destroy_node()
rclpy.shutdown()
sys.exit(130 if trang_thai == 'HUY' else 0)
PY
