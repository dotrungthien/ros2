#!/usr/bin/env python3
# Xe gia cho gia lap vong kin tren PC: thay dieu_khien + dong co + YDLidar.
# - Nghe /cmd_vel (dau ra twist_mux), tinh banh trai/phai nhu dieu_khien.py: duty = |v|/0.437,
#   duty < duty_ngung 0.05 -> 0, duty < duty_min 0.21 -> 0.21 (0.092 m/s), cat o 1.0. Mat lenh 0.5 s -> dung.
# - Vi tri that bat dau o A = (0; 0; 0) trong khung map v3, tich phan 50 Hz.
# - Phat /scan 10 Hz, 500 tia, ban tren map_xuong_v3.png (o chiem: p > 0.65), frame laser_frame.
# - Phat static TF base_footprint -> laser_frame (x 0.025). Khong phat TF odom (odom_tu_lenh lam).
# - canh_pid: PID cua launch mapping; launch do chet thi xe_gia tu thoat (de "bat_xe.sh tat" don sach).
import math
import os
import sys

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PNG = os.path.join(DIR, '..', 'du_lieu_0111', 'map_xuong_v3.png')
RES, OX, OY = 0.05, -0.566, -0.415           # map_xuong_v3.yaml
MAX_WHEEL, WHEEL_SEP = 0.437, 0.155          # mapping_my_bot.launch.py, dieu_khien.py
DUTY_MIN, DUTY_NGUNG, WATCHDOG = 0.21, 0.05, 0.5
LIDAR_X = 0.025
N_TIA, R_MIN, R_MAX, BUOC = 500, 0.10, 8.0, 0.01


def banh(v):
    """Toc do that cua mot banh [m/s] tu toc do lenh, theo dieu_khien.set_motor."""
    duty = min(abs(v) / MAX_WHEEL, 1.0)
    if duty < DUTY_NGUNG:
        return 0.0
    duty = max(duty, DUTY_MIN)
    return math.copysign(duty * MAX_WHEEL, v)


class XeGia(Node):
    def __init__(self):
        super().__init__('xe_gia')
        self.declare_parameter('canh_pid', 0)
        self.canh_pid = self.get_parameter('canh_pid').value
        img = np.array(Image.open(MAP_PNG).convert('L'))
        self.chiem = (255 - img.astype(float)) / 255.0 > 0.65   # hang 0 = y lon nhat
        self.h, self.w = self.chiem.shape
        self.x = self.y = self.th = 0.0
        self.v = self.w_lenh = 0.0
        self.t_lenh = None
        self.goc = -math.pi + np.arange(N_TIA) * (2 * math.pi / N_TIA)
        self.r = np.arange(R_MIN, R_MAX, BUOC)
        self.create_subscription(Twist, '/cmd_vel', self.lenh, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_footprint', 'laser_frame'
        t.transform.translation.x = LIDAR_X
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        self.t_cu = self.get_clock().now()
        self.create_timer(0.02, self.tich_phan)
        self.create_timer(0.1, self.quet)
        self.create_timer(0.2, self.canh)
        self.get_logger().info('xe_gia: ban do %dx%d, %d o chiem, vi tri dau (0; 0; 0)'
                               % (self.w, self.h, int(self.chiem.sum())))

    def lenh(self, msg):
        self.v, self.w_lenh = msg.linear.x, msg.angular.z
        self.t_lenh = self.get_clock().now()

    def tich_phan(self):
        now = self.get_clock().now()
        dt = (now - self.t_cu).nanoseconds * 1e-9
        self.t_cu = now
        if self.t_lenh is None or (now - self.t_lenh).nanoseconds * 1e-9 > WATCHDOG:
            return
        vl = banh(self.v - self.w_lenh * WHEEL_SEP / 2.0)
        vr = banh(self.v + self.w_lenh * WHEEL_SEP / 2.0)
        v, w = (vl + vr) / 2.0, (vr - vl) / WHEEL_SEP
        self.x += v * math.cos(self.th + w * dt / 2.0) * dt
        self.y += v * math.sin(self.th + w * dt / 2.0) * dt
        self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))

    def quet(self):
        lx = self.x + LIDAR_X * math.cos(self.th)
        ly = self.y + LIDAR_X * math.sin(self.th)
        a = self.th + self.goc[:, None]
        cot = np.floor((lx + self.r * np.cos(a) - OX) / RES).astype(int)
        hang = self.h - 1 - np.floor((ly + self.r * np.sin(a) - OY) / RES).astype(int)
        trong = (cot >= 0) & (cot < self.w) & (hang >= 0) & (hang < self.h)
        trung = np.zeros_like(trong)
        trung[trong] = self.chiem[hang[trong], cot[trong]]
        co = trung.any(axis=1)
        dau = trung.argmax(axis=1)
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_frame'
        m.angle_min = float(self.goc[0])
        m.angle_increment = 2 * math.pi / N_TIA
        m.angle_max = float(self.goc[-1])
        m.scan_time = 0.1
        m.range_min, m.range_max = R_MIN, R_MAX
        m.ranges = np.where(co, self.r[dau], np.inf).astype(np.float32).tolist()
        self.pub.publish(m)

    def canh(self):
        if self.canh_pid <= 0:
            return
        try:
            os.kill(self.canh_pid, 0)
        except ProcessLookupError:
            self.get_logger().info('launch mapping (PID %d) da tat -> xe_gia thoat' % self.canh_pid)
            raise SystemExit


def main():
    rclpy.init()
    n = XeGia()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
