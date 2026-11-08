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
# - G3 phan B (03/11): ong gia. Moi lan quet (10 Hz) doc ~/ong_gia ("x y", m, khung map = khung vi tri that); co tep thi LiDAR
#   gia thay them hinh tron ban kinh R_ONG = 0,09 m (O 18 cm) o do, xoa tep la ong bien mat. Co ong thi ghi 10 Hz vao
#   /tmp/gia_lap/ong_gia.csv: vi tri that, khoang cach than xe (hinh chu nhat) va mui xe (0,145 m truoc tam) toi MAT ong
#   (am = da cham). Ong KHONG tinh vao d_min_cm cua vi_tri_that.csv (cot do giu nghia cu: o chiem map v3).
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
TEP_ONG, R_ONG = os.path.expanduser('~/ong_gia'), 0.09
TEP_ONG_CSV = '/tmp/gia_lap/ong_gia.csv'
# 06/11 viec 118 B7b: tat "hut quang di" (mac dinh = nhu cu) + ghi map -> odom va vet that / tin.
TEP_MO = '/tmp/gia_lap/map_odom.csv'      # moi lan map -> odom doi: gio, buoc doi, lech that - tin truoc / sau (khung than tin)
TEP_VET = '/tmp/gia_lap/vet_that.csv'     # 10 Hz: that va tin


def doc_ong(path):
    """[(x, y), ...] cac ong trong ~/ong_gia ("x y", 08/11 bo 1e: them "x1 y1 x2 y2 ..." = nhieu ong); None neu khong co tep.
    Tep hong (so chan so, khong rong) => ValueError."""
    try:
        with open(path) as f:
            o = f.read().split()
    except FileNotFoundError:
        return None
    if not o or len(o) % 2:
        raise ValueError('~/ong_gia can "x y" (hoac nhieu cap), co %r' % o)
    v = [float(t) for t in o]
    return list(zip(v[0::2], v[1::2]))


def tia_cat_tron(lx, ly, goc, cx, cy, r):
    """Khoang cach tu (lx, ly) theo tung huong `goc` (mang) toi mat hinh tron (cx, cy, r); inf neu khong cat."""
    dx, dy = np.cos(goc), np.sin(goc)
    ux, uy = cx - lx, cy - ly
    b = dx * ux + dy * uy
    disc = b * b - (ux * ux + uy * uy - r * r)
    t = b - np.sqrt(np.maximum(disc, 0.0))
    return np.where((disc >= 0.0) & (t >= R_MIN) & (t <= R_MAX), t, np.inf)


def cach_ong(x, y, th, cx, cy, r):
    """(than, mui): khoang cach [m] tu hinh chu nhat than xe va tu diem mui (X_MUI truoc tam) toi mat ong; am = da cham."""
    c, s = math.cos(th), math.sin(th)
    px, py = c * (cx - x) + s * (cy - y), -s * (cx - x) + c * (cy - y)
    than = math.hypot(max(X_DUOI - px, px - X_MUI, 0.0), max(abs(py) - Y_HONG, 0.0)) - r
    return than, math.hypot(px - X_MUI, py) - r


def van_toc_san(v, w, banh_trai_that, v_nang_that=None):
    """(v, w) that tren san tu lenh (chua tinh tre xoay). Duty moi banh nhu dieu_khien (trai x he_so_trai);
    toc do that = duty x 0.437 x do manh banh (trai: banh_trai_that, phai: 1). Banh bi nang -> w x he_so_xoay.
    B7b (M2): v_nang_that (m/s) = toc do that cua banh bi nang (x do manh banh); None = duty_min x 0.437 nhu cu."""
    eff, nang = [], False
    for s, k, manh in ((v + w * WHEEL_SEP / 2.0, 1.0, 1.0),
                       (v - w * WHEEL_SEP / 2.0, HE_SO_TRAI, banh_trai_that)):   # phai, trai
        duty = min(1.0, abs(s) * k / MAX_WHEEL)
        if duty < DUTY_NGUNG:
            duty = 0.0
        elif duty < DUTY_MIN:     # banh bi nang: chay duty toi thieu
            nang = True
            duty = DUTY_MIN
            if v_nang_that is not None:
                eff.append(math.copysign(v_nang_that * manh, s))
                continue
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
        # B7b: tat (mac dinh = nhu cu)
        self.declare_parameter('v_nang_that', -1.0)        # M2: < 0 = nhu cu
        self.declare_parameter('he_so_quang', 1.0)         # M1: toc do thang that = mo hinh x he_so_quang
        self.declare_parameter('nhieu_lidar', 0.0)         # S-a: sigma (m) nhieu khoang cach moi tia
        self.declare_parameter('tia_tam_o', False)         # S-a: tia dung o tam o (thay vi mep gan)
        self.declare_parameter('chu_ky_lidar', 0.1)        # S-a: s (0,1 = 10 Hz; 0,0833 = 12 Hz)
        vn = self.get_parameter('v_nang_that').value
        self.v_nang_that = vn if vn >= 0 else None
        self.he_so_quang = self.get_parameter('he_so_quang').value
        self.nhieu_lidar = self.get_parameter('nhieu_lidar').value
        self.tia_tam_o = self.get_parameter('tia_tam_o').value
        self.rng = np.random.default_rng(118)
        self.mo_cu = None                                  # map -> odom lan truoc (x, y, goc)
        self.tin = None                                    # tu the tin moi nhat (x, y, goc)
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
        with open(TEP_ONG_CSV, 'w') as f:
            f.write('t_epoch,ong_x,ong_y,x,y,goc_do,d_than_cm,d_mui_cm\n')
        with open(TEP_MO, 'w') as f:
            f.write('t_epoch,mo_x,mo_y,mo_goc_do,buoc_x_cm,buoc_y_cm,buoc_goc_do,buoc_cm,x,y,goc_do,'
                    'truoc_doc_cm,truoc_ngang_cm,sau_doc_cm,sau_ngang_cm\n')
        with open(TEP_VET, 'w') as f:
            f.write('t_epoch,x,y,goc_do,x_tin,y_tin,goc_tin_do,v,w\n')
        self.ong_loi = ''
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
        self.create_timer(self.get_parameter('chu_ky_lidar').value, self.quet)
        self.create_timer(0.05, self.theo_doi)
        self.create_timer(0.1, self.ghi_vet)
        self.create_timer(0.2, self.canh)
        self.create_timer(1.0, lambda: self.ghi('dinh_ky'))
        self.get_logger().info('xe_gia: ban do %dx%d, %d o chiem, vi tri dau (%.2f; %.2f; %.1f do), banh_trai_that %.4f'
                               % (self.w, self.h, int(self.chiem.sum()), self.x, self.y, math.degrees(self.th),
                                  self.banh_trai_that))
        self.get_logger().info('xe_gia B7b: v_nang_that %s, he_so_quang %.3f, nhieu_lidar %.3f, tia_tam_o %s, chu_ky_lidar %.4f'
                               % (self.v_nang_that, self.he_so_quang, self.nhieu_lidar, self.tia_tam_o,
                                  self.get_parameter('chu_ky_lidar').value))

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
        v, w = van_toc_san(self.v, self.w_lenh, self.banh_trai_that, self.v_nang_that)
        v *= self.he_so_quang
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
        if self.tia_tam_o:                        # B7b S-a: dung o tam o chiem (them nua o theo huong tia)
            dau = np.minimum(dau + int(round(RES / 2 / BUOC)), len(self.r) - 1)
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_frame'
        m.angle_min = float(self.goc[0])
        m.angle_increment = 2 * math.pi / N_TIA
        m.angle_max = float(self.goc[-1])
        m.scan_time = 0.1
        m.range_min, m.range_max = R_MIN, R_MAX
        rr = np.where(co, self.r[dau], np.inf)
        if self.nhieu_lidar > 0:                  # B7b S-a: nhieu khoang cach
            rr = np.where(co, np.maximum(rr + self.rng.normal(0.0, self.nhieu_lidar, rr.shape), R_MIN), rr)
        try:
            ong = doc_ong(TEP_ONG)
        except ValueError as e:                    # tep hong: bao 1 lan, coi nhu khong co ong
            if str(e) != self.ong_loi:
                self.get_logger().error(str(e))
                self.ong_loi = str(e)
            ong = None
        for cx, cy in ong or ():
            rr = np.minimum(rr, tia_cat_tron(lx, ly, a[:, 0], cx, cy, R_ONG))
        m.ranges = rr.astype(np.float32).tolist()
        self.pub.publish(m)
        self.d_min = min(self.d_min, self.cach_o_chiem())
        if ong:                                    # nhieu ong: ghi ong gan THAN xe nhat (sap theo than, roi mui)
            than, mui, cx, cy = min(cach_ong(self.x, self.y, self.th, cx, cy, R_ONG) + (cx, cy) for cx, cy in ong)
            with open(TEP_ONG_CSV, 'a') as f:
                f.write('%.3f,%.3f,%.3f,%.4f,%.4f,%.2f,%.1f,%.1f\n' % (time.time(), cx, cy, self.x, self.y,
                                                                     math.degrees(self.th), than * 100, mui * 100))

    def theo_doi(self):
        """B7b: 20 Hz. map -> odom va tu the tin; ghi moi lan map -> odom doi (> 1 mm hoac > 0,05 do)."""
        try:
            a = self.tf_buf.lookup_transform('map', 'odom', rclpy.time.Time())
            b = self.tf_buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        def xyg(t):
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        mo, ob = xyg(a), xyg(b)
        def ghep(m):
            c, s = math.cos(m[2]), math.sin(m[2])
            return (m[0] + c * ob[0] - s * ob[1], m[1] + s * ob[0] + c * ob[1], m[2] + ob[2])
        self.tin = ghep(mo)
        cu = self.mo_cu
        if cu is None:
            self.mo_cu = mo
            return
        dg = math.atan2(math.sin(mo[2] - cu[2]), math.cos(mo[2] - cu[2]))
        if math.hypot(mo[0] - cu[0], mo[1] - cu[1]) <= 0.001 and abs(math.degrees(dg)) <= 0.05:
            return
        def lech(tin):                                  # that - tin, khung than tin (doc + = mui, ngang + = trai)
            dx, dy = self.x - tin[0], self.y - tin[1]
            c, s = math.cos(tin[2]), math.sin(tin[2])
            return dx * c + dy * s, -dx * s + dy * c
        tr, sa = lech(ghep(cu)), lech(self.tin)
        with open(TEP_MO, 'a') as f:
            f.write('%.3f,%.4f,%.4f,%.3f,%.2f,%.2f,%.3f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\n' % (
                time.time(), mo[0], mo[1], math.degrees(mo[2]), (mo[0] - cu[0]) * 100, (mo[1] - cu[1]) * 100,
                math.degrees(dg), math.hypot(mo[0] - cu[0], mo[1] - cu[1]) * 100, self.x, self.y, math.degrees(self.th),
                tr[0] * 100, tr[1] * 100, sa[0] * 100, sa[1] * 100))
        self.mo_cu = mo

    def ghi_vet(self):
        t = self.tin
        with open(TEP_VET, 'a') as f:
            f.write('%.3f,%.4f,%.4f,%.2f,%s,%.3f,%.3f\n' % (time.time(), self.x, self.y, math.degrees(self.th),
                    ('%.4f,%.4f,%.2f' % (t[0], t[1], math.degrees(t[2]))) if t else ',,', self.v, self.w_lenh))

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
