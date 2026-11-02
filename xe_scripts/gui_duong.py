#!/usr/bin/env python3
"""gui_duong (viec 109a, soan 02/11/2026 o truong, CHUA CHAY TREN XE): gui DUONG DAY cua tinh_tuyen.py qua /follow_path.

Tu 02/11 18:09 duong gui di la DUONG DOAN THANG da chia day (tinh_tuyen.doan_thang): dinh re dat cho du cho xoay,
RPP xoay tai cho khi lech > 20 do (rotate_to_heading_min_angle 0.35) thay vi cua rong.
Ly do (xe 02/11 17:03): bang xs gui 6 diem qua /navigate_to_pose; moi diem Nav2 nhan "da toi" khi con cach 14-15 cm
(xy_goal_tolerance 0,15) nen sai so don lai, xe troi 14 cm ve K4 roi ket. Duong day la MOT dich: chi diem cuoi co dung sai.

Mot tien trinh rclpy, KHONG goi ros2 CLI. Chay (chuyen.sh BANG=day goi):  python3 ~/gui_duong.py di_B|ve_A [tuyen_dau 1|2]
  1. KIEM: tf map->base_footprint moi (<= 1 s) va /global_costmap/costmap da co, xe khong dung trong o do. Sai => SAI_VI_TRI (ma 3).
  2. Moi tuyen (thu tu nhu chuyen.sh), toi da LUOT lan (18:09: phu lai XOAY xong => huy duong cu, tinh lai tu cho xe, ghi
     TINH_LAI, KHONG tinh la mot lan hong; toi da LUOT lan tinh lai moi tuyen): hoi sep (~/sep_lenh: DUNG => dung; CHO => cho SEP_CHO_S s, toi da 2 lan),
     cho khi phu lai dang RA / XOAY / THOAT (/rosout cua lai_xe_lidar) hoac sep dang giao thoat (~/sep_thoat moi);
     tinh duong tu vi tri HIEN TAI tren costmap HIEN TAI (tinh_tuyen.tinh_duong), phat /duong_day, gui FollowPath.
     Trong luc chay: ~/sep_lenh DUNG => huy dich, DUNG_SEP (ma 4); /duong_day cap nhat (phan con lai) 2 lan/giay.
     SUCCEEDED ma xe cach dich <= TOI_SAI_SO m => DEN_NOI (ma 0). Hong => cho (>= CHO_HONG s, toi da gap doi, ton trong
     CHO/DUNG va phu lai) roi tinh lai tu vi tri hien tai. Het LUOT lan => tuyen kia. Ca hai hong => HET_DUONG (ma 1).
  CSV: ~/ket_qua_chuyen.csv cung cot va cung chu voi chuyen.sh (KIEM_DAT, SUCCEEDED, KHONG_DAT, DEN_NOI, HET_DUONG,
  SAI_VI_TRI, DUNG_SEP) va NGAT (bi ngat, dong ket thuc chuyen); cot 2 = "<huong>_day".
  Ma thoat: 0 den noi, 1 het duong, 2 sai doi so, 3 sai vi tri, 4 sep bao dung, 130 bi ngat (NGAT).
Bien moi truong: LUOT (3), CHO_HONG (30 s), TO (130 s/lan gui), TOI_SAI_SO (0,20 m); cung ten voi chuyen.sh: SEP_CHO_S (10 s),
  SEP_TUOI_MAX (3 s). Phu lai lai mai thi tong thoi gian cho giua hai lan gui toi da 3 x CHO_HONG.
Dung khan: TAT CONG TAC MOTOR. Ctrl+C / SIGTERM: huy dich dang chay (in HUY_DICH_KHI_NGAT ...), ghi dong NGAT, thoat 130.
Phan thuan (khong ROS) o nua tren, thu bang test_gui_duong.py.
"""
import math
import os
import signal
import sys
import time

import tinh_tuyen as TT

DICH = {'di_B': (TT.B, 0.0), 've_A': (TT.A, None)}     # (diem, goc cuoi): di_B giu 0 do nhu bang tt/xs; ve_A theo tiep tuyen
COT_CSV = 'gio,huong,tuyen,diem,lan,ket_qua,giay,lech_kiem_m,lech_costmap_m,tuoi_costmap_s'
PHU_LAI_BAN = ('RA_TIEN', 'RA_LUI', 'XOAY_P')           # ten trang thai lai_xe_lidar dang tu lai banh (cong moi ten THOAT_*)


def cau_hinh(env=os.environ):
    f = lambda k, d: float(env.get(k, d))
    return dict(luot=int(env.get('LUOT', 3)), cho_hong=f('CHO_HONG', 30.0), to=f('TO', 130.0), cho_sep=f('SEP_CHO_S', 10.0),
                toi_sai_so=f('TOI_SAI_SO', 0.20), sep_tuoi=f('SEP_TUOI_MAX', 3.0), tf_tuoi=1.0)


def doc_sep_lenh(dong, t_now, tuoi_max=3.0):
    """'<giay BOOTTIME> DI|CHO|DUNG <ly do>' (sep.py ghi_lenh) -> (chu, ly do) neu con moi; None neu khong co / cu / hong.
    Cung luat tuoi voi doc_sep cua chuyen.sh (-1 s <= tuoi <= tuoi_max)."""
    o = (dong or '').split(None, 2)
    if len(o) < 2 or o[1] not in ('DI', 'CHO', 'DUNG'):
        return None
    try:
        t = float(o[0])
    except ValueError:
        return None
    if t <= 0 or not -1.0 <= t_now - t <= tuoi_max:
        return None
    return o[1], (o[2].strip() if len(o) > 2 else '')


def cat_con_lai(duong, pose):
    """Phan duong (x, y, yaw) tu diem gan xe nhat tro di (cho /duong_day: diem dau = canh xe)."""
    if not duong or pose is None:
        return duong
    k = min(range(len(duong)), key=lambda i: math.hypot(duong[i][0] - pose[0], duong[i][1] - pose[1]))
    return duong[k:]


def cho_tf_costmap(co_cm, lay_ban, lay_pose, now, ngu, giay=15.0):
    """02/11 18:2x (xe 18:18, 18:20: lan 1 luon 'khong co tf hoac costmap moi', cho 30 s moi thu lan 2): doi toi `giay` s.
    Costmap TRUOC (doi luoi ra Ban mat vai giay tren Pi va khong spin, tf cu di), roi spin toi khi co tf moi. Tra (pose, ban)."""
    t_het = now() + giay
    while not co_cm() and now() < t_het:
        ngu(0.1)
    ban = lay_ban() if co_cm() else None
    pose = lay_pose()
    while pose is None and now() < t_het:
        ngu(0.05)
        pose = lay_pose()
    return pose, ban


def thieu_gi(pose, ban):
    return ' va '.join(t for t, ok in (('tf moi', pose is not None), ('costmap', ban is not None)) if not ok)


def phu_lai_ban(l1):
    l1 = l1 or ''
    return l1 in PHU_LAI_BAN or l1.startswith('THOAT')


def chay(io, huong, tuyen_dau, cfg):
    """Mot chuyen. io: nut ROS (GuiDuongNode) hoac gia trong ca thu. Tra ma thoat."""
    nhan = '%s_day' % huong
    thu_tu = (1, 2) if tuyen_dau == 1 else (2, 1)
    (dx, dy), yaw_cuoi = DICH[huong]
    t_chuyen = io.now()

    pose, ban = io.cho_san_sang()
    if pose is None or ban is None or ban.do(ban.o(pose[0], pose[1])):
        ly = ('khong co %s sau 15 s' % thieu_gi(pose, ban)) if pose is None or ban is None else 'xe nam trong o do cua costmap'
        io.bao('DUNG: SAI_VI_TRI (%s). KHONG gui lenh nao.' % ly)
        io.ghi(nhan, '-', 1, 0, 'SAI_VI_TRI', 0)
        return 3
    io.ghi(nhan, thu_tu[0], 1, 0, 'KIEM_DAT', 0)
    for r in thu_tu:
        lan = tinh_lai = 0
        while lan < cfg['luot']:
            lan += 1
            if cho_truoc_khi_gui(io, cfg) == 'DUNG':
                return dung_sep(io, nhan, r, lan)
            pose, ban = io.cho_san_sang()                 # 18:2x: doi toi 15 s (khong tinh la hong), chi bao khi qua han
            if pose is None or ban is None:
                io.bao('   lan %d: khong co %s sau 15 s' % (lan, thieu_gi(pose, ban)))
                io.ghi(nhan, r, 1, lan, 'KHONG_DAT', 0)
                if cho_sau_hong(io, cfg) == 'DUNG':
                    return dung_sep(io, nhan, r, lan)
                continue
            canh_bao = []
            try:
                duong = TT.tinh_duong(ban, (pose[0], pose[1]), (dx, dy), r, yaw_cuoi=yaw_cuoi, canh_bao=canh_bao)
            except TT.KhongCoDuong as e:
                io.bao('   tuyen %d: %s => doi tuyen' % (r, e))
                io.ghi(nhan, r, 1, lan, 'KHONG_DAT', 0)
                break
            for x, y, c in canh_bao:
                io.bao('   CANH BAO: dinh re (%.2f; %.2f) chi cach o do %.2f m (< %.2f), khong doi duoc' % (x, y, c, TT.R_XOAY_CAN))
            t0 = io.now()
            io.bao('-- tuyen %d, lan %d/%d: %d diem tu (%.2f; %.2f) toi (%.2f; %.2f)' % (r, lan, cfg['luot'], len(duong), pose[0], pose[1], dx, dy))
            kq = theo_doi(io, duong, cfg)
            giay = int(round(io.now() - t0))
            if kq == 'DUNG':
                return dung_sep(io, nhan, r, lan)
            if kq == 'TINH_LAI' and tinh_lai < cfg['luot']:   # 18:09: phu lai vua RA/XOAY xong: tinh lai tu cho xe, khong tinh la hong
                tinh_lai += 1
                io.bao('   phu lai vua XOAY xong: tinh lai duong tu cho xe (lan %d/%d).' % (tinh_lai, cfg['luot']))
                io.ghi(nhan, r, 1, lan, 'TINH_LAI', giay)
                lan -= 1
                continue
            pose = io.pose()
            lech = None if pose is None else math.hypot(pose[0] - dx, pose[1] - dy)
            if kq == 'SUCCEEDED' and lech is not None and lech <= cfg['toi_sai_so']:
                io.ghi(nhan, r, 1, lan, 'SUCCEEDED', giay, '%.2f' % lech)
                io.ghi(nhan, r, 'tong', 0, 'DEN_NOI', int(round(io.now() - t_chuyen)))
                io.bao('DEN_NOI: %s xong theo tuyen %d (cach dich %.2f m).' % (huong, r, lech))
                return 0
            io.bao('   khong den duoc (%s%s).' % (kq, '' if lech is None else ', cach dich %.2f m' % lech))
            io.ghi(nhan, r, 1, lan, 'KHONG_DAT', giay, '-' if lech is None else '%.2f' % lech)
            if lan < cfg['luot'] and cho_sau_hong(io, cfg) == 'DUNG':
                return dung_sep(io, nhan, r, lan)
        io.bao('== Tuyen %d hong. ==' % r)
    io.bao('HET_DUONG: ca hai tuyen deu khong di duoc. Xe dung yen.')
    io.ghi(nhan, '-', 'het', 0, 'HET_DUONG', 0)
    return 1


def dung_sep(io, nhan, r, lan):
    io.huy()
    io.bao('DUNG_SEP: sep bao dung. Xe dung yen, khong gui them lenh.')
    io.ghi(nhan, r, 1, lan, 'DUNG_SEP', 0)
    return 4


def chay_co_ngat(io, huong, tuyen_dau, cfg, bo_qua_ngat=lambda: None):
    """chay(); bi ngat (SIGINT/SIGTERM => KeyboardInterrupt) thi huy dich dang chay, ghi dong NGAT (dong ket thuc chuyen,
    sep.py dong chuyen), tra 130. bo_qua_ngat: tat bat tin hieu lan 2 trong luc huy (main dat SIG_IGN)."""
    try:
        return chay(io, huong, tuyen_dau, cfg)
    except KeyboardInterrupt:
        bo_qua_ngat()
        io.bao('HUY_DICH_KHI_NGAT %s' % io.huy_khi_ngat())
        io.ghi('%s_day' % huong, '-', 'ngat', 0, 'NGAT', 0)
        return 130


def cho_truoc_khi_gui(io, cfg):
    """Truoc moi lan gui: DUNG => 'DUNG'; CHO => cho cho_sep s (toi da 2 lan); phu lai dang lai / sep dang giao thoat => cho
    toi khi xong (toi da cho_hong s). Tra 'DUNG' hoac None."""
    lan_cho = 0
    t_het = io.now() + cfg['cho_hong']
    while True:
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        if lenh and lenh[0] == 'CHO' and lan_cho < 2:
            lan_cho += 1
            io.bao('   sep bao CHO (%s): dung yen %d s (lan %d/2).' % (lenh[1], cfg['cho_sep'], lan_cho))
            if cho(io, cfg['cho_sep']) == 'DUNG':
                return 'DUNG'
            continue
        if (io.phu_lai_ban() or io.sep_dang_giao_thoat()) and io.now() < t_het:
            if cho(io, 0.5) == 'DUNG':
                return 'DUNG'
            continue
        return None


def cho_sau_hong(io, cfg):
    """Sau mot lan hong: cho it nhat cho_hong s; het thi cho them toi khi phu lai xong (toi da gap doi). DUNG => 'DUNG'."""
    t0 = io.now()
    while io.now() - t0 < 2 * cfg['cho_hong']:
        if io.now() - t0 >= cfg['cho_hong'] and not io.phu_lai_ban() and not io.sep_dang_giao_thoat():
            return None
        if cho(io, 0.5) == 'DUNG':
            return 'DUNG'
    return None


def cho(io, giay):
    """Cho `giay` s; giua chung ~/sep_lenh bao DUNG thi tra 'DUNG' ngay."""
    t0 = io.now()
    while io.now() - t0 < giay:
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            return 'DUNG'
        io.ngu(min(0.1, giay))
    return None


def theo_doi(io, duong, cfg):
    """Gui duong, theo doi toi khi xong. Tra 'SUCCEEDED' | 'ABORTED' | 'CANCELED' | 'TU_CHOI' | 'QUA_GIO' | 'DUNG'."""
    io.phat_duong(duong)
    n_xong = io.so_lan_phu_lai_xong()
    if not io.gui(duong):
        return 'TU_CHOI'
    t0 = t_phat = io.now()
    while True:
        if io.so_lan_phu_lai_xong() != n_xong:        # 18:09: phu lai XOAY xong (xe da doi cho, doi huong): bo duong cu
            io.huy()
            io.da_xu_ly_thoat()
            return 'TINH_LAI'
        kq = io.ket_qua()
        if kq is not None:
            return kq
        lenh = io.lenh_sep()
        if lenh and lenh[0] == 'DUNG':
            io.huy()
            return 'DUNG'
        if io.now() - t0 > cfg['to']:
            io.huy()
            return 'QUA_GIO'
        if io.now() - t_phat >= 0.5:
            io.phat_duong(cat_con_lai(duong, io.pose()))
            t_phat = io.now()
        io.ngu(0.1)


# ---------- nut ROS 2 ----------

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from nav2_msgs.action import FollowPath
    from rcl_interfaces.msg import Log
    import tf2_ros
except ImportError:
    rclpy = None

if rclpy is not None:
    import sep as S          # chi dung doc_rosout (mot khuon cau log lai_xe_lidar cho ca sep va gui_duong)
    import lai_xe_lidar as L  # chi dung doc_sep_thoat

    class GuiDuongNode(Node):
        def __init__(self, cfg):
            super().__init__('gui_duong')
            self.cfg = cfg
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            # costmap cua Nav2: KeepLast(1), TRANSIENT_LOCAL, RELIABLE => nghe cung QoS de nhan ngay ban dang giu
            giu = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_costmap, giu)
            self.create_subscription(Log, '/rosout', self._on_rosout, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
            self.pub = self.create_publisher(Path, '/duong_day', giu)
            self.ac = ActionClient(self, FollowPath, 'follow_path')
            self.cm, self.cm_ban, self.l1 = None, None, ''
            self.n_xong, self.ma_xu_ly = 0, None       # 18:09: so lan phu lai XOAY xong; ma ~/sep_thoat da xu ly
            self.goal, self.kq = None, None
            self.tep_lenh = os.path.expanduser('~/sep_lenh')
            self.tep_thoat = os.path.expanduser('~/sep_thoat')
            self.csv = os.path.expanduser('~/ket_qua_chuyen.csv')

        # --- nghe ---
        def _on_costmap(self, msg):
            self.cm, self.cm_ban = msg, None

        def _on_rosout(self, msg):
            if msg.name == 'lai_xe_lidar':
                e = S.doc_rosout(msg.name, msg.msg)
                if e and e[0] == 'L1':
                    if e[1] == 'XOAY_XONG' and self.l1 != 'XOAY_XONG':
                        self.n_xong += 1
                    self.l1 = e[1]

        # --- io cho chay() ---
        def now(self):
            return time.monotonic()

        def ngu(self, giay):
            t_het = time.monotonic() + giay
            while True:
                con = t_het - time.monotonic()
                if con <= 0:
                    return
                rclpy.spin_once(self, timeout_sec=min(con, 0.05))

        def bao(self, cau):
            print(cau, flush=True)

        def ghi(self, nhan, tuyen, diem, lan, ket_qua, giay, lech='-', lech_fp='-', tuoi='-'):
            moi = not os.path.exists(self.csv)
            with open(self.csv, 'a') as f:
                if moi:
                    f.write(COT_CSV + '\n')
                f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), nhan, tuyen, diem, lan, ket_qua, giay, lech, lech_fp, tuoi))

        def pose(self):
            """(x, y, yaw) tu tf map->base_footprint, None neu khong co hoac cu hon tf_tuoi."""
            try:
                tf = self.tf_buf.lookup_transform('map', 'base_footprint', Time())
            except Exception:
                return None
            st = tf.header.stamp
            if self.get_clock().now().nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9) > self.cfg['tf_tuoi']:
                return None
            q = tf.transform.rotation
            return (tf.transform.translation.x, tf.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

        def ban(self):
            if self.cm is None:
                return None
            if self.cm_ban is None:
                i = self.cm.info
                self.cm_ban = TT.Ban.tu_luoi(i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, list(self.cm.data))
            return self.cm_ban

        def cho_san_sang(self, giay=15.0):
            return cho_tf_costmap(lambda: self.cm is not None, self.ban, self.pose, time.monotonic, self.ngu, giay)

        def lenh_sep(self):
            try:
                with open(self.tep_lenh) as f:
                    return doc_sep_lenh(f.readline(), time.clock_gettime(time.CLOCK_BOOTTIME), self.cfg['sep_tuoi'])
            except OSError:
                return None

        def _thoat_moi(self):
            try:
                with open(self.tep_thoat) as f:
                    return L.doc_sep_thoat(f.readline(), time.clock_gettime(time.CLOCK_BOOTTIME))
            except OSError:
                return None

        def sep_dang_giao_thoat(self):
            """Sep dang giao mot dot thoat CHUA xu ly (ma moi): cho phu lai lam truoc roi moi gui."""
            t = self._thoat_moi()
            return t is not None and t['ma'] != self.ma_xu_ly

        def da_xu_ly_thoat(self):
            t = self._thoat_moi()
            if t is not None:
                self.ma_xu_ly = t['ma']

        def so_lan_phu_lai_xong(self):
            return self.n_xong

        def phu_lai_ban(self):
            return phu_lai_ban(self.l1)

        def phat_duong(self, duong):
            self.pub.publish(self._path(duong))

        def _path(self, duong):
            p = Path()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            for x, y, yaw in duong:
                q = PoseStamped()
                q.header = p.header
                q.pose.position.x, q.pose.position.y = float(x), float(y)
                q.pose.orientation.z, q.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                p.poses.append(q)
            return p

        def gui(self, duong):
            if not self.ac.wait_for_server(timeout_sec=5.0):
                self.bao('   khong thay may chu /follow_path (controller_server)')
                return False
            g = FollowPath.Goal()
            g.path = self._path(duong)
            g.controller_id = 'FollowPath'          # nhu bt_cho.xml; goal_checker_id rong = plugin duy nhat (general_goal_checker)
            self.goal, self.kq = None, None
            fut = self.ac.send_goal_async(g)
            t_het = time.monotonic() + 5.0
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            if not fut.done() or not fut.result().accepted:
                self.bao('   /follow_path tu choi dich')
                return False
            self.goal = fut.result()
            self.goal.get_result_async().add_done_callback(self._on_result)
            return True

        def _on_result(self, fut):
            st = fut.result().status
            self.kq = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(st, 'ABORTED')

        def ket_qua(self):
            return self.kq

        def huy(self):
            if self.goal is not None and self.kq is None:
                fut = self.goal.cancel_goal_async()
                t_het = time.monotonic() + 2.0
                while not fut.done() and time.monotonic() < t_het:
                    self.ngu(0.05)

        def huy_khi_ngat(self):
            """Bi ngat (SIGINT/SIGTERM): huy dich /follow_path dang chay (handle hien co) khi context con song, cho <= 2 s."""
            if self.goal is None or self.kq is not None:
                return 'khong_co_dich'
            fut = self.goal.cancel_goal_async()
            t_het = time.monotonic() + 2.0
            while not fut.done() and time.monotonic() < t_het:
                self.ngu(0.05)
            return 'ok' if fut.done() else 'khong_tra_loi'

    def main():
        a = sys.argv[1:]
        if not a or a[0] not in DICH or (len(a) > 1 and a[1] not in ('1', '2')):
            print('SAI DOI SO. Dung: python3 ~/gui_duong.py di_B | ve_A [tuyen_dau 1|2]')
            sys.exit(2)
        # 03/11 (viec 5): rclpy KHONG cai bo bat tin hieu (no tat context truoc, huy dich khong di duoc);
        # SIGINT/SIGTERM => KeyboardInterrupt, huy dich khi context con song, roi thoat 130
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

        def ngat(signum, frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, ngat)
        signal.signal(signal.SIGTERM, ngat)
        node = GuiDuongNode(cau_hinh())

        def bo_qua_ngat():                                   # khong ngat lan 2 trong luc huy
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)

        ma = 1
        try:
            ma = chay_co_ngat(node, a[0], int(a[1]) if len(a) > 1 else 1, node.cfg, bo_qua_ngat)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        sys.exit(ma)
else:
    def main():
        raise SystemExit('can ROS 2 (rclpy, nav2_msgs) de chay nut; chi import duoc phan ham thuan')


if __name__ == '__main__':
    main()
