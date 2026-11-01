#!/usr/bin/env python3
# Xe gia cho gia lap vong kin tren PC: thay dieu_khien + dong co + YDLidar.
# - Nghe /cmd_vel (dau ra twist_mux), chuyen dong that theo mo hinh san cua odom_tu_lenh.py (mo_hinh_san=True,
#   so mac dinh cua odom_tu_lenh, launch khong ghi de): banh duty < 0.05 -> 0, duty < 0.21 -> banh bi nang chay
#   0.21 x 0.437 va w x he_so_xoay 0.46; xoay tai cho tre 0.5 s moi quay. Mat lenh 0.5 s -> dung.
# - Phat /gyro_yaw (std_msgs/Float64, rad, [-pi, pi]) nhu gyro_mpu6050, gia tri = goc that, 50 Hz.
# - Vi tri that bat dau o (x0; y0; goc0_do), mac dinh A = (0; 0; 0), khung map v3, tich phan 50 Hz.
# - Phat /scan 10 Hz, 500 tia, ban tren map_xuong_v3.png (o chiem: p > 0.65), frame laser_frame.
# - Phat static TF base_footprint -> laser_frame (x 0.025). Khong phat TF odom (odom_tu_lenh lam).
# - canh_pid: PID cua launch mapping; launch do chet thi xe_gia tu thoat (de "bat_xe.sh tat" don sach).
# - Moi 1 s va khi tat: ghi /tmp/gia_lap/vi_tri_that.csv = vi tri THAT, vi tri xe tin (tf map -> base_footprint)
#   va khoang cach nho nhat tu than xe that toi o chiem cua map v3 trong 1 s vua qua (do 10 Hz).
import math
import os
import sys
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PNG = os.path.join(DIR, '..', 'du_lieu_0111', 'map_xuong_v3.png')
RES, OX, OY = 0.05, -0.566, -0.415           # map_xuong_v3.yaml
MAX_WHEEL, WHEEL_SEP = 0.437, 0.155          # odom_tu_lenh.py (mac dinh)
DUTY_MIN, DUTY_NGUNG, WATCHDOG = 0.21, 0.05, 0.5
HE_SO_TRAI, HE_SO_XOAY, TRE_XOAY = 0.92, 0.46, 0.5   # odom_tu_lenh.py (mac dinh)
# ---- tham so hieu chuan ----
BANH_TRAI_THAT = 1.0 / HE_SO_TRAI   # do manh banh trai that / banh phai: dieu_khien bu vua du -> lenh thang thi di thang
LIDAR_X = 0.025
N_TIA, R_MIN, R_MAX, BUOC = 500, 0.10, 8.0, 0.01
X_MUI, X_DUOI, Y_HONG = 0.145, -0.105, 0.08   # lai_xe_lidar.py
TEP_VI_TRI = '/tmp/gia_lap/vi_tri_that.csv'


def van_toc_san(v, w, banh_trai_that):
    """(v, w) that tren san tu lenh (chua tinh tre xoay). Duty moi banh nhu dieu_khien (trai x he_so_trai);
    toc do that = duty x 0.437 x do manh banh (trai: banh_trai_that, phai: 1). Banh bi nang -> w x he_so_xoay."""
    eff, nang = [], False
    for s, k, manh in ((v + w * WHEEL_SEP / 2.0, 1.0, 1.0),
                       (v - w * WHEEL_SEP / 2.0, HE_SO_TRAI, banh_trai_that)):   # phai, trai
        duty = min(1.0, abs(s) * k / MAX_WHEEL)
        if duty < DUTY_NGUNG:
            duty = 0.0
        elif duty < DUTY_MIN:     # banh bi nang: chay duty toi thieu
            nang = True
            duty = DUTY_MIN
        eff.append(math.copysign(duty * MAX_WHEEL * manh, s))
    v, w = (eff[0] + eff[1]) / 2.0, (eff[0] - eff[1]) / WHEEL_SEP
    return v, (w * HE_SO_XOAY if nang else w)


class XeGia(Node):
    def __init__(self):
        super().__init__('xe_gia')
        self.declare_parameter('canh_pid', 0)
        self.canh_pid = self.get_parameter('canh_pid').value
        self.declare_parameter('banh_trai_that', BANH_TRAI_THAT)
        self.banh_trai_that = self.get_parameter('banh_trai_that').value
        for k in ('x0', 'y0', 'goc0_do'):
            self.declare_parameter(k, 0.0)
        img = np.array(Image.open(MAP_PNG).convert('L'))
        self.chiem = (255 - img.astype(float)) / 255.0 > 0.65   # hang 0 = y lon nhat
        self.h, self.w = self.chiem.shape
        hang, cot = np.nonzero(self.chiem)                       # moi o chiem lay 6 x 6 diem (ca canh)
        kx, ky = np.meshgrid(np.linspace(0.0, RES, 6), np.linspace(0.0, RES, 6))
        self.o_x = ((OX + cot * RES)[:, None] + kx.ravel()).ravel()
        self.o_y = ((OY + (self.h - 1 - hang) * RES)[:, None] + ky.ravel()).ravel()
        self.d_min = math.inf
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        with open(TEP_VI_TRI, 'w') as f:
            f.write('gio,t_epoch,nguon,x,y,goc_do,x_tin,y_tin,goc_tin_do,d_min_cm\n')
        self.x, self.y = self.get_parameter('x0').value, self.get_parameter('y0').value
        self.th = math.radians(self.get_parameter('goc0_do').value)
        self.v = self.w_lenh = 0.0
        self.t_lenh = None
        self.bat_dau_xoay = None
        self.goc = -math.pi + np.arange(N_TIA) * (2 * math.pi / N_TIA)
        self.r = np.arange(R_MIN, R_MAX, BUOC)
        self.create_subscription(Twist, '/cmd_vel', self.lenh, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_gyro = self.create_publisher(Float64, '/gyro_yaw', 10)
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
        self.create_timer(1.0, lambda: self.ghi('dinh_ky'))
        self.get_logger().info('xe_gia: ban do %dx%d, %d o chiem, vi tri dau (%.2f; %.2f; %.1f do), banh_trai_that %.4f'
                               % (self.w, self.h, int(self.chiem.sum()), self.x, self.y, math.degrees(self.th),
                                  self.banh_trai_that))

    def lenh(self, msg):
        self.v, self.w_lenh = msg.linear.x, msg.angular.z
        self.t_lenh = self.get_clock().now()

    def tich_phan(self):
        now = self.get_clock().now()
        dt = (now - self.t_cu).nanoseconds * 1e-9
        self.t_cu = now
        self.pub_gyro.publish(Float64(data=self.th))
        if self.t_lenh is None or (now - self.t_lenh).nanoseconds * 1e-9 > WATCHDOG:
            self.bat_dau_xoay = None
            return
        v, w = van_toc_san(self.v, self.w_lenh, self.banh_trai_that)
        if v == 0.0 and w != 0.0:                 # xoay tai cho: TRE_XOAY s dau chua quay
            if self.bat_dau_xoay is None:
                self.bat_dau_xoay = now
            if (now - self.bat_dau_xoay).nanoseconds * 1e-9 < TRE_XOAY:
                w = 0.0
        else:
            self.bat_dau_xoay = None
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
        self.d_min = min(self.d_min, self.cach_o_chiem())

    def cach_o_chiem(self):
        """Khoang cach [m] tu hinh chu nhat than xe that toi diem gan nhat cua o chiem (0 neu cham)."""
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = self.o_x - self.x, self.o_y - self.y
        px, py = c * dx + s * dy, -s * dx + c * dy
        ex = np.maximum(np.maximum(X_DUOI - px, px - X_MUI), 0.0)
        ey = np.maximum(np.abs(py) - Y_HONG, 0.0)
        return float(np.hypot(ex, ey).min())

    def ghi(self, nguon):
        tin = ',,'
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = t.transform.rotation
            g = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            tin = '%.4f,%.4f,%.2f' % (t.transform.translation.x, t.transform.translation.y, g)
        except Exception:
            pass
        d = self.d_min if self.d_min < math.inf else self.cach_o_chiem()
        bay_gio = time.time()
        with open(TEP_VI_TRI, 'a') as f:
            f.write('%s,%.2f,%s,%.4f,%.4f,%.2f,%s,%.1f\n' % (
                time.strftime('%F %T', time.localtime(bay_gio)), bay_gio, nguon,
                self.x, self.y, math.degrees(self.th), tin, d * 100))
        self.d_min = math.inf

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
        n.ghi('tat')
        n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
