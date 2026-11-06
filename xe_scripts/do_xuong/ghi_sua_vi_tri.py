#!/usr/bin/env python3
"""do_xuong/ghi_sua_vi_tri.py (viec 118 B10, soan 06/11/2026, CHUA CHAY TREN XE).

CHI NGHE /tf (khong phat gi). Ghi MOI lan map -> odom doi (SLAM sua vi tri) ra CSV, mot dong moi lan:
  gio, t_epoch, t_stamp, mo_x, mo_y, mo_goc_do (map -> odom sau khi doi),
  buoc_x_cm, buoc_y_cm, buoc_goc_do, buoc_cm (do lon lan doi, khung map),
  x_tin, y_tin, goc_tin_do (vi tri xe tin ngay sau khi doi = map -> odom -> base_footprint moi nhat).
"Doi" = khac lan truoc > 1e-6 m hoac > 1e-6 rad (slam_toolbox phat lai cung mot gia tri 20 lan/giay khi khong sua).
Chay:  source ~/ros2_ws/install/setup.bash; python3 ~/do_xuong/ghi_sua_vi_tri.py [tep.csv]
       (mac dinh ~/sua_vi_tri_<ngay_gio>.csv). Dung: Ctrl+C.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


def goc(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    tep = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(time.strftime('~/sua_vi_tri_%Y%m%d_%H%M%S.csv'))
    f = open(tep, 'a')
    if f.tell() == 0:
        f.write('gio,t_epoch,t_stamp,mo_x,mo_y,mo_goc_do,buoc_x_cm,buoc_y_cm,buoc_goc_do,buoc_cm,x_tin,y_tin,goc_tin_do\n')
    rclpy.init()
    node = rclpy.create_node('ghi_sua_vi_tri')
    tt = {'mo': None, 'ob': None, 'n': 0}

    def nhan(msg):
        for t in msg.transforms:
            p, r = t.transform.translation, t.transform.rotation
            cur = (p.x, p.y, goc(r))
            if t.header.frame_id.lstrip('/') == 'odom' and t.child_frame_id.lstrip('/') == 'base_footprint':
                tt['ob'] = cur
            elif t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                cu = tt['mo']
                tt['mo'] = cur
                if cu is None:
                    print('map -> odom dau tien: (%.3f; %.3f; %.2f do). Ghi vao %s' % (cur[0], cur[1], math.degrees(cur[2]), tep), flush=True)
                    continue
                dg = math.atan2(math.sin(cur[2] - cu[2]), math.cos(cur[2] - cu[2]))
                if abs(cur[0] - cu[0]) <= 1e-6 and abs(cur[1] - cu[1]) <= 1e-6 and abs(dg) <= 1e-6:
                    continue
                bx, by = (cur[0] - cu[0]) * 100, (cur[1] - cu[1]) * 100
                xt = yt = gt = float('nan')
                if tt['ob'] is not None:
                    ox, oy, og = tt['ob']
                    c, s = math.cos(cur[2]), math.sin(cur[2])
                    xt, yt, gt = cur[0] + c * ox - s * oy, cur[1] + s * ox + c * oy, cur[2] + og
                    gt = math.atan2(math.sin(gt), math.cos(gt))
                now = time.time()
                f.write('%s,%.3f,%.3f,%.4f,%.4f,%.3f,%.2f,%.2f,%.3f,%.2f,%.4f,%.4f,%.2f\n' % (
                    time.strftime('%H:%M:%S', time.localtime(now)), now,
                    t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, cur[0], cur[1], math.degrees(cur[2]),
                    bx, by, math.degrees(dg), math.hypot(bx, by), xt, yt, math.degrees(gt)))
                f.flush()
                tt['n'] += 1
                if math.hypot(bx, by) >= 5.0:
                    print('%s SUA %.1f cm (%+.1f; %+.1f) %+.1f do' % (time.strftime('%H:%M:%S'), math.hypot(bx, by), bx, by, math.degrees(dg)), flush=True)

    node.create_subscription(TFMessage, '/tf', nhan, QoSProfile(depth=100))
    print('ghi_sua_vi_tri: nghe /tf, ghi moi lan map -> odom doi. Ctrl+C de dung.', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        f.close()
        print('DUNG: %d lan doi da ghi vao %s' % (tt['n'], tep), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
