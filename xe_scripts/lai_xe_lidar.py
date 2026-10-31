#!/usr/bin/env python3
"""lai_xe_lidar (ban 1, soan 31/10/2026 o nha, CHUA THU TREN XE): "nguoi lai" dung LiDAR.

Chinh sach 5 buoc (Thien chot 31/10 01:10):
  1. Di theo duong thoang nhat: viec cua Nav2, nut nay khong dung vao.
  2. Vang/do: chi can thiep khi DO (<= 8 cm tu mep than xe): chan truoc / chan sau / sat hong. Co them QUYEN PHU QUYET:
     nghe /cmd_vel_nav (lenh Nav2); lenh xoay tai cho khi co vat trong ban kinh quet (17,5 cm), lenh tien khi truoc do, lenh lui khi sau do
     bi thay bang lenh 0 NGAY (Nav2/RPP khong kiem va cham luc xoay).
  3. Bi chan phia truoc (chi vung TRUOC do): DUNG va CHO toi da t_cho giay (30).
  4. Het t_cho giay, hoac ket hong/goc: THOAT bang LiDAR (xoay ve huong thoang nhat neu xoay an toan;
     khong thi lui/tien thang; neu sat tuong ben hong thi di cung ra xa); xong thi tra lai cho Nav2.
  5. Het duong (viec cua chuyen.sh + giao dien, KHONG nam trong nut nay): nut chi "bo cuoc" va nha quyen.

Mac dinh CHI NHIN: phat_lenh = false -> nut chi in "se lam gi", khong gui lenh nao.
Muon gui lenh: ros2 param set /lai_xe_lidar phat_lenh true   (tat lai: ... false).
Gui vao /cmd_vel_avoid (twist_mux uu tien 60: cao hon Nav2 = 20, thap hon lai tay = 100).

Toa do than xe (tam = base_footprint): x truoc, y trai. LiDAR cach tam +0,025 m ve phia mui, goc 0 = mui xe.
Khoang cach tinh tu MEP THAN XE den vat, khong tu LiDAR.
Ma thuan (khong can ROS) o nua tren de thu bang test_lai_xe_lidar.py.
"""
import math
import time

INF = 1e9

X_MUI, X_DUOI, Y_HONG = 0.145, -0.105, 0.08     # m, so voi tam xe
LIDAR_X = 0.025                                   # m, LiDAR cach tam ve phia mui
CORNERS = ((X_MUI, Y_HONG), (X_MUI, -Y_HONG), (X_DUOI, Y_HONG), (X_DUOI, -Y_HONG))
R_VETO = math.hypot(X_MUI, Y_HONG) + 0.01       # vat trong ban kinh nay se bi quet trung khi xe xoay tai cho
R_XOAY = math.hypot(X_MUI, Y_HONG) + 0.02        # xoay tai cho an toan khi moi diem LiDAR cach tam xa hon so nay

VUNG_TRUOC = ((1, 0), (1, 1), (1, -1))
VUNG_SAU = ((-1, 0), (-1, 1), (-1, -1))
VUNG_HONG = ((0, 1), (0, -1))
TEN_VUNG = {(1, 0): 'TRUOC', (-1, 0): 'SAU', (0, 1): 'TRAI', (0, -1): 'PHAI',
            (1, 1): 'TR-TRAI', (1, -1): 'TR-PHAI', (-1, 1): 'SAU-TRAI', (-1, -1): 'SAU-PHAI', (0, 0): 'TRONG'}

DEFAULT = dict(
    do=0.08,                 # m, vung do: tu mep than xe tro vao (de xuat, cho do)
    vang=0.15,               # m, vung vang; mui sat vat <= vang va xe dung yen thi tinh la bi chan phia truoc
    thoat=0.12,              # m, thoat xong khi nhom dang xu ly cach vat it nhat bay nhieu
    t_cho=40.0,              # s, cho khi bi chan phia truoc. 30 s la cua cay hanh vi Nav2; nut cho them 10 s roi moi xen vao (tranh hai dong ho cong don)
    t_thoat_max=10.0,        # s, thoat qua lau thi bo cuoc
    t_nghi_sau_xong=3.0,     # s, nghi sau khi thoat xong (cho Nav2 lap lai duong)
    t_nghi_sau_bo_cuoc=20.0, # s, nghi sau khi bo cuoc
    v_esc=0.10,              # m/s, >= 0,092 (san duty toi thieu)
    w_esc=1.0,               # rad/s, xoay thoat
    w_cung=0.3,              # rad/s, di cung ra xa tuong ben hong
    xoay_den_deg=20.0,       # do, xoay cho toi khi huong tot nhat lech duoi goc nay
    dem_diem_do=2,           # so diem LiDAR toi thieu trong vung do moi tinh la do (loc nhieu)
    so_quet_xac_nhan=2,      # so lan quet lien tiep co do (chi truoc mat) moi giu lenh 0 / thoat vi dia hinh
    so_quet_ket=30,          # so lan quet lien tiep vua SAT (sau / hong / nhieu phia) vua XE DUNG YEN moi coi la KET (~2,5 s o 12 Hz; dieu kien chinh la xe dung yen; Thien dong y 31/10 02:27)
    dung_neu_mep=0.02,       # m, mep nao sat hon so nay thi dung han
    thoang_toi_thieu=0.05,   # m, huong di phai thoang it nhat bay nhieu
)


# ---------- ham thuan ----------

def scan_to_xy(scan, angle_offset=0.0):
    """LaserScan -> danh sach (x, y) theo toa do than xe (tam xe). Bo diem khong hop le va diem trong vung mu cua LiDAR."""
    out = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        th = scan.angle_min + i * scan.angle_increment + angle_offset
        out.append((LIDAR_X + r * math.cos(th), r * math.sin(th)))
    return out


def khoang_cach_mep(x, y):
    """Khoang cach tu diem (x, y) toi hinh chu nhat than xe (0 neu o trong)."""
    dx = max(X_DUOI - x, 0.0, x - X_MUI)
    dy = max(-Y_HONG - y, 0.0, y - Y_HONG)
    return math.hypot(dx, dy)


def vung(x, y):
    sx = 1 if x > X_MUI else (-1 if x < X_DUOI else 0)
    sy = 1 if y > Y_HONG else (-1 if y < -Y_HONG else 0)
    return (sx, sy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def thoang_dem(pts, phi, nguong, le=0.02, cap=1.0):
    """Nhu thoang(), tra them so diem cach mep dai nhin truoc <= nguong (de loc nhieu): (quang_duong, so_diem)."""
    ux, uy = math.cos(phi), math.sin(phi)
    nx, ny = -uy, ux
    h = max(cx * ux + cy * uy for cx, cy in CORNERS)
    w = max(abs(cx * nx + cy * ny) for cx, cy in CORNERS) + le
    best, dem = cap, 0
    for x, y in pts:
        s = x * ux + y * uy
        if s < h:
            continue
        if abs(x * nx + y * ny) <= w:
            best = min(best, s - h)
            if s - h <= nguong:
                dem += 1
    return best, dem


def thoang(pts, phi, le=0.02, cap=1.0):
    """Quang duong tu mep than xe di theo huong phi (than xe) den vat dau tien, trong dai rong bang than xe + le."""
    return thoang_dem(pts, phi, -1.0, le, cap)[0]


def chon_huong(pts, cho_xoay=True, cap=0.5, toi_thieu=0.05):
    """Huong (rad, trong than xe) tot nhat de thoat: thoang, it phai quay, huong ra xa vat gan nhat. None neu khong co loi."""
    if not pts:
        return 0.0
    gan = min(pts, key=lambda q: khoang_cach_mep(q[0], q[1]))
    phi_xa = math.atan2(-gan[1], -gan[0])
    cands = [math.radians(a) for a in range(-150, 151, 30)] + [math.pi] if cho_xoay else [0.0, math.pi]
    best, best_s = None, -INF
    for phi in cands:
        f = thoang(pts, phi, cap=cap)
        if f < toi_thieu:
            continue
        s = min(f, cap) / cap - 0.20 * abs(wrap(phi)) / math.pi + 0.15 * math.cos(wrap(phi - phi_xa))
        if s > best_s:
            best, best_s = phi, s
    return best


def o_chiem(grid, wx, wy, tol=1):
    """True neu o ban do chua (wx, wy) hoac o lan can (+-tol o) la o chiem (>= 65). grid: dict res, w, h, ox, oy, data."""
    res = grid['res']
    i = int(math.floor((wx - grid['ox']) / res))
    j = int(math.floor((wy - grid['oy']) / res))
    w, h, data = grid['w'], grid['h'], grid['data']
    for dj in range(-tol, tol + 1):
        for di in range(-tol, tol + 1):
            a, b = i + di, j + dj
            if 0 <= a < w and 0 <= b < h and data[b * w + a] >= 65:
                return True
    return False


def la_dia_hinh(diem_chan, pose, grid, tol=1, ty_le=0.7):
    """Cac diem LiDAR dang chan xe (toa do than xe) co nam tren tuong/hop co trong ban do khong?
    pose = (x, y, yaw) cua tam xe tren ban do. True = dia hinh (tuong, hop co dinh); False = vat la (nguoi, tam chan);
    None = khong ket luan duoc (thieu ban do, thieu vi tri, khong co diem). Coi la dia hinh khi >= ty_le so diem trung o chiem."""
    if grid is None or pose is None or not diem_chan:
        return None
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    trung = sum(1 for x, y in diem_chan if o_chiem(grid, px + x * c - y * s, py + x * s + y * c, tol))
    return trung >= ty_le * len(diem_chan)


def do_vung(pts):
    """Khoang cach mep nho nhat trong tung vung trong 8 vung quanh than xe (de ghi nhat ky)."""
    z = {}
    for x, y in pts:
        k = vung(x, y)
        d = khoang_cach_mep(x, y)
        if d < z.get(k, INF):
            z[k] = d
    return z


def mo_ta_vung(z):
    return ' '.join('%s=%.2f' % (TEN_VUNG[k], v) for k, v in sorted(z.items(), key=lambda kv: kv[1])[:4])


class NguoiLai:
    """May trang thai: BINH_THUONG -> (CHO) -> THOAT -> BINH_THUONG. step() tra (v, w, ten); v, w = None nghia la KHONG phat gi."""

    def __init__(self, **kw):
        self.p = dict(DEFAULT)
        self.p.update(kw)
        self.state = 'BINH_THUONG'
        self.t_truoc = None
        self.t0 = None
        self.trigger = set()
        self.nghi_den = -INF
        self.n_do = 0
        self.n_veto = 0
        self.n_f = 0
        self.n_ket = 0
        self.ly_do = ''

    def phan_tich(self, pts):
        """Tra (tt, red). tt: ff, fb = quang duong thoang di thang tien/lui (dai rong bang than xe), dl, dr = khoang cach
        mep hong trai/phai. red: tap {'F','B','H'} = bi chan truoc / sau / sat hong (can >= dem_diem_do diem de loc nhieu)."""
        do, n = self.p['do'], self.p['dem_diem_do']
        ff, nf = thoang_dem(pts, 0.0, do)
        _f15, nf15 = thoang_dem(pts, 0.0, self.p['vang'])
        fb, nb = thoang_dem(pts, math.pi, do)
        dl = dr = INF
        nl = nr = 0
        for x, y in pts:
            if X_DUOI <= x <= X_MUI:
                d = khoang_cach_mep(x, y)
                if y > Y_HONG:
                    dl = min(dl, d)
                    nl += d <= do
                elif y < -Y_HONG:
                    dr = min(dr, d)
                    nr += d <= do
        red = set()
        if nf >= n:
            red.add('F')
        if nb >= n:
            red.add('B')
        if nl >= n or nr >= n:
            red.add('H')
        return dict(ff=ff, fb=fb, dl=dl, dr=dr, nf15=nf15), red

    def diem_chan_truoc(self, pts, xa=0.12):
        """Cac diem trong dai di thang phia truoc cach mep mui <= xa (nhung diem dang chan xe)."""
        h = X_MUI
        w = Y_HONG + 0.02
        return [(x, y) for x, y in pts if x >= h and abs(y) <= w and x - h <= xa]

    def _veto(self, pts, red, nav_cmd):
        """Phu quyet lenh cua Nav2 neu no se dam vao vat: xoay tai cho khi co vat trong ban kinh quet, tien khi truoc do, lui khi sau do."""
        if nav_cmd is None:
            return None
        v, w = nav_cmd
        if abs(v) < 0.02 and abs(w) > 0.1:
            if pts and min(math.hypot(x, y) for x, y in pts) < R_VETO:
                return 'CHAN_XOAY'
        elif v > 0.01 and 'F' in red:
            return 'CHAN_TIEN'
        elif v < -0.01 and 'B' in red:
            return 'CHAN_LUI'
        return None

    def step(self, pts, now, goal_active, phan_loai=None, nav_cmd=None):
        """phan_loai: ham tuy chon nhan danh sach diem chan, tra True (dia hinh) / False (vat la) / None (khong biet).
        nav_cmd: (v, w) lenh Nav2 dang muon gui (tuy chon); co thi nguoi lai phu quyet lenh se dam vao vat va biet xe co dang dung yen."""
        p = self.p
        tt, red = self.phan_tich(pts)
        self.n_do = self.n_do + 1 if red else 0
        if self.state == 'THOAT':
            return self._thoat(pts, tt, now)
        dung_yen = nav_cmd is None or (abs(nav_cmd[0]) < 0.02 and abs(nav_cmd[1]) < 0.05)
        f_chan = tt['nf15'] >= p['dem_diem_do'] and tt['ff'] <= p['vang']       # mui sat vat (<= 15 cm), Nav2 dung xe o 8-13 cm
        solo_f = (red == {'F'} or (not red and f_chan)) and dung_yen     # Nav2 dang xoay/di thi KHONG giu lenh 0: de Nav2 tu thoat
        if not goal_active:
            if not red and not f_chan:
                self.t_truoc = None          # gioi han 1 dong ho cho: giua hai dich lien tiep (~3 s) KHONG dat lai dong ho neu van bi chan
            self.n_veto = 0
            return None, None, 'KHONG_CO_DICH'
        ten_veto = self._veto(pts, red, nav_cmd)
        if ten_veto is not None:
            self.n_veto += 1
            if ten_veto == 'CHAN_XOAY' and self.n_veto >= p['so_quet_ket']:       # xoay bi chan qua lau: coi la sat tuong, thoat
                gan = min(pts, key=lambda q: math.hypot(q[0], q[1]))
                k = vung(gan[0], gan[1])
                trig = {'F'} if k == (1, 0) else ({'B'} if k == (-1, 0) else {'H'})
                self.n_veto = 0
                return self._bat_dau(trig, now, 'XOAY_BI_CHAN_QUA_LAU', pts, tt)
            return 0.0, 0.0, ten_veto
        self.n_veto = 0
        if now < self.nghi_den:
            return None, None, 'NGHI'
        if red == {'F'} and not dung_yen:
            self.t_truoc = None
            self.n_f = 0
            return None, None, 'NAV_DANG_DI'
        if not red and not solo_f:
            self.t_truoc = None
            self.n_f = 0
            return None, None, 'XANH_VANG'
        if red and red != {'F'}:                         # sat hong / sau / nhieu phia
            # KET = sat VA xe khong tien duoc (Nav2 dung yen) keo dai; xe dang di/xoay luot qua goc thi KHONG phai ket
            self.n_ket = self.n_ket + 1 if dung_yen else 0
            if self.n_ket < p['so_quet_ket']:
                return None, None, 'XAC_NHAN' if dung_yen else 'NAV_DANG_DI'
            self.t_truoc = None
            self.n_ket = 0
            return self._bat_dau(red, now, 'KET_HINH_HOC', pts, tt)
        self.n_ket = 0
        self.n_f += 1                                    # chi bi chan phia truoc
        if self.n_f < p['so_quet_xac_nhan']:
            return None, None, 'XAC_NHAN'
        if phan_loai is not None and phan_loai(self.diem_chan_truoc(pts)) is True:
            self.t_truoc = None                          # vat chan nam tren ban do (tuong, hop co dinh): khong phai nguoi, khong cho
            return self._bat_dau({'F'}, now, 'CHAN_TRUOC_LA_DIA_HINH', pts, tt)
        if self.t_truoc is None:
            self.t_truoc = now
        cho = now - self.t_truoc
        if cho < p['t_cho']:
            return 0.0, 0.0, 'CHO_%d' % int(cho)
        return self._bat_dau({'F'}, now, 'CHAN_TRUOC_QUA_%ds' % int(p['t_cho']), pts, tt)

    def _bat_dau(self, red, now, ly_do, pts, tt):
        self.state = 'THOAT'
        self.t0 = now
        self.trigger = set(red)
        self.ly_do = ly_do
        return self._thoat(pts, tt, now)

    def _ket_thuc(self, now, ten, nghi):
        self.state = 'BINH_THUONG'
        self.t_truoc = None
        self.nghi_den = now + nghi
        return None, None, ten

    def _thoat(self, pts, tt, now):
        p = self.p
        if now - self.t0 > p['t_thoat_max']:
            return self._ket_thuc(now, 'BO_CUOC_HET_GIO', p['t_nghi_sau_bo_cuoc'])
        ff, fb, dl, dr = tt['ff'], tt['fb'], tt['dl'], tt['dr']
        dH = min(dl, dr)
        xoay_duoc = all(math.hypot(x, y) > R_XOAY for x, y in pts)
        T = self.trigger
        act_f = 'F' in T and ff < (0.25 if xoay_duoc else p['thoat'])   # xoay duoc thi can mui huong ve cho thoang >= 0,25 m
        act_b = 'B' in T and fb < p['thoat']
        trai, phai = dl < p['thoat'], dr < p['thoat']
        # sat hong chi sua duoc khi MOT ben sat va ben kia con cho; hanh lang hep deu hai ben thi khong lam gi
        act_h = 'H' in T and trai != phai and max(dl, dr) >= p['thoat'] + 0.04
        bo = lambda ten: self._ket_thuc(now, ten, p['t_nghi_sau_bo_cuoc'])
        if act_f and act_b:
            return bo('BO_CUOC_KE_HAI_DAU')
        if act_f:
            if xoay_duoc:
                phi = chon_huong(pts, True, toi_thieu=p['thoang_toi_thieu'])
                if phi is None:
                    return bo('BO_CUOC_KHONG_LOI')
                if abs(phi) > math.radians(p['xoay_den_deg']):
                    return 0.0, math.copysign(p['w_esc'], phi), 'THOAT_XOAY'
                if ff < p['thoang_toi_thieu']:
                    return bo('BO_CUOC_KHONG_LOI')
                return p['v_esc'], 0.0, 'THOAT_TIEN'
            if fb < p['thoang_toi_thieu']:
                return bo('BO_CUOC_KHONG_LUI_DUOC')
            return -p['v_esc'], 0.0, 'THOAT_LUI'
        if act_b:
            if ff < p['thoang_toi_thieu']:
                return bo('BO_CUOC_KHONG_TIEN_DUOC')
            return p['v_esc'], 0.0, 'THOAT_TIEN'
        if act_h:
            if dH <= p['dung_neu_mep']:
                return bo('BO_CUOC_QUA_SAT')
            if ff >= fb and ff >= 0.10:
                v = p['v_esc']
            elif fb >= 0.10:
                v = -p['v_esc']
            else:
                return bo('BO_CUOC_KHONG_CUNG')
            w = (-1.0 if dl < dr else 1.0) * math.copysign(1.0, v) * p['w_cung']
            return v, w, 'THOAT_CUNG'
        return self._ket_thuc(now, 'XONG', p['t_nghi_sau_xong'])


# ---------- nut ROS 2 (chi dung khi co ROS) ----------

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import OccupancyGrid
    from rclpy.time import Time
    import tf2_ros
except ImportError:      # may khong co ROS: van import duoc phan ham thuan de thu
    rclpy = None

if rclpy is not None:
    class LaiXeLidar(Node):
        def __init__(self):
            super().__init__('lai_xe_lidar')
            self.declare_parameter('enabled', True)
            self.declare_parameter('phat_lenh', False)       # false = chi nhin, khong gui lenh
            self.declare_parameter('log_hz', 1.0)
            self.declare_parameter('cmd_topic', '/cmd_vel_avoid')
            self.declare_parameter('lidar_angle_offset_deg', 0.0)
            self.declare_parameter('scan_timeout', 0.5)
            self.declare_parameter('dung_ban_do', True)       # phan biet dia hinh (co trong ban do) voi vat la
            self.declare_parameter('tf_tuoi_toi_da', 1.0)     # s, vi tri xe cu hon so nay thi khong tin (coi la vat la)
            for k, v in DEFAULT.items():
                self.declare_parameter(k, v)
            self.brain = NguoiLai()
            self.scan, self.scan_time = None, 0.0
            self.goal_active = False
            self.last_name, self.last_log = '', 0.0
            self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
            self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
            st_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._on_status, st_qos)
            self.grid = None
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
            self.nav_cmd, self.nav_time = None, 0.0
            self.create_subscription(Twist, '/cmd_vel_nav', self._on_nav, 10)
            self.create_timer(0.1, self._tick)
            self.get_logger().info('lai_xe_lidar san sang. phat_lenh=%s (false = chi nhin)' % self.get_parameter('phat_lenh').value)

        def _on_scan(self, msg):
            self.scan, self.scan_time = msg, time.monotonic()

        def _on_map(self, msg):
            i = msg.info
            self.grid = dict(res=i.resolution, w=i.width, h=i.height, ox=i.origin.position.x, oy=i.origin.position.y, data=list(msg.data))
            self.get_logger().info('nhan ban do %dx%d, %.2f m/o' % (i.width, i.height, i.resolution))

        def _pose_ban_do(self):
            """(x, y, yaw) cua xe tren ban do tu tf, hoac None neu khong co hoac cu hon tf_tuoi_toi_da."""
            try:
                tf = self.tf_buf.lookup_transform('map', 'base_footprint', Time())
            except Exception:
                return None
            st = tf.header.stamp
            tuoi = self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
            if tuoi > self.get_parameter('tf_tuoi_toi_da').value:
                self.get_logger().warning('tf map->base_footprint cu %.1f s: coi la vat la' % tuoi, throttle_duration_sec=5.0)
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (tf.transform.translation.x, tf.transform.translation.y, yaw)

        def _on_nav(self, msg):
            self.nav_cmd, self.nav_time = (msg.linear.x, msg.angular.z), time.monotonic()

        def _on_status(self, msg):
            self.goal_active = any(s.status in (1, 2) for s in msg.status_list)   # ACCEPTED, EXECUTING

        def _tick(self):
            if not self.get_parameter('enabled').value:
                return
            now = time.monotonic()
            phat = self.get_parameter('phat_lenh').value
            for k in DEFAULT:
                self.brain.p[k] = self.get_parameter(k).value
            if self.scan is None or now - self.scan_time > self.get_parameter('scan_timeout').value:
                if self.goal_active:
                    self.get_logger().warning('mat /scan: %s' % ('gui lenh dung' if phat else 'chi nhin'), throttle_duration_sec=2.0)
                    if phat:
                        self.pub.publish(Twist())
                return
            pts = scan_to_xy(self.scan, math.radians(self.get_parameter('lidar_angle_offset_deg').value))
            phan_loai = None
            if self.get_parameter('dung_ban_do').value:
                phan_loai = lambda diem: la_dia_hinh(diem, self._pose_ban_do(), self.grid)   # chi tinh khi can
            nav = self.nav_cmd if now - self.nav_time < 0.5 else None
            v, w, ten = self.brain.step(pts, now, self.goal_active, phan_loai, nav)
            if v is not None and phat:
                t = Twist()
                t.linear.x, t.angular.z = float(v), float(w)
                self.pub.publish(t)
            z = do_vung(pts)
            if ten != self.last_name:
                self.get_logger().info('%s -> %s (%s)%s | %s' % (
                    self.last_name or '-', ten, self.brain.ly_do if self.brain.state == 'THOAT' else '',
                    '' if phat or v is None else '  [chi nhin: SE gui v=%.2f w=%.2f]' % (v, w), mo_ta_vung(z)))
                self.last_name = ten
            hz = self.get_parameter('log_hz').value
            if hz > 0 and now - self.last_log >= 1.0 / hz:
                self.last_log = now
                self.get_logger().info('vung gan nhat: %s | dich=%s trang_thai=%s' % (mo_ta_vung(z), self.goal_active, ten))


    def main():
        rclpy.init()
        node = LaiXeLidar()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
else:
    def main():
        raise SystemExit('can ROS 2 de chay nut; chi import duoc phan ham thuan')


if __name__ == '__main__':
    main()
