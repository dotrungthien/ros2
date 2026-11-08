#!/usr/bin/env python3
"""lop_huong_dan.py (bo 1e "mat lai", soan 08/11/2026, CHUA CHAY TREN XE): luoi huong dan /lop_huong_dan cho global_costmap.

Chi chay khi MAT_LAI=true (navigation_my_bot.launch.py mat_lai:=true: python3 /home/project1/lop_huong_dan.py).
- Luoi CUNG kich thuoc, do phan giai, goc voi ban do goc /map_tinh: doc cung tep map_xuong_v3.yaml (+ kich thuoc anh tu
  header PNG / PGM). Khac => Nav2 doi co ca costmap moi lan nhan (Humble 1.1.20 static_layer.cpp:193-210).
- Bat len: phat ngay luoi toan 0 (planner Nav2 cho moi lop "current" KHONG han gio, planner_server.cpp:274-281).
- /chon_tuyen (std_msgs/String) "r x y" (r = 1 | 2, x y = vi tri xe, m): o co tam trong tinh_tuyen.CAM[r] = 100 (cam: khe kia),
  tru dia R_THA quanh (x, y) (xe co the dang dung trong vung cam). "0": bo het o cam.
- LUON cong vung ton diem: cum >= N_CUM dong ~/kinh_nghiem.csv (09/11: chi ly_do KET_CO_DUONG) trong R_CUM => dia R_VUNG gia tri GIA_TON (50 < 72: di duoc
  nhung ton, KHONG BAO GIO thanh cam). Doc lai csv MOI lan nhan /chon_tuyen; dong hong bo qua.
- Moi lan phat in 1 dong (PHAT ...).
- QoS phat: RELIABLE + TRANSIENT_LOCAL depth 1 (global_costmap lop_huong_dan.map_subscribe_transient_local: true).
Kiem (kiem_xe.sh buoc 8d): python3 ~/lop_huong_dan.py --kiem-costmap  => /global_costmap/costmap co tin trong 10 s va cung
  kich thuoc / do phan giai / goc voi ban do goc. Ma thoat 0 = DAT, 1 = KHONG DAT.
Phan thuan (khong ROS) o nua tren, thu bang test_lop_huong_dan.py.
"""
import csv
import math
import os
import re
import struct
import sys

import tinh_tuyen as TT

YAML_MAC_DINH = '/home/project1/map_xuong_v3.yaml'
TEP_KINH_NGHIEM = os.path.expanduser('~/kinh_nghiem.csv')
R_THA = 0.35          # m: dia quanh xe khong cam (thiet ke 1e muc 2 #4)
R_CUM = 0.20          # m: cac dong kinh nghiem trong ban kinh nay tinh la mot cum
N_CUM = 3             # so dong toi thieu cua mot cum
R_VUNG = 0.15         # m: ban kinh vung ton diem
GIA_CAM, GIA_TON = 100, 50


def doc_khoa(text, khoa):
    m = re.search(r'^[ \t]*%s:[ \t]*(.+?)[ \t]*(?:#.*)?$' % re.escape(khoa), text, re.M)
    return m.group(1).strip().strip('"\'') if m else None


def kich_thuoc_anh(path):
    """(rong, cao) tu header PNG (IHDR) hoac PGM (P2/P5), khong giai nen anh."""
    with open(path, 'rb') as f:
        dau = f.read(64)
    if dau[:8] == b'\x89PNG\r\n\x1a\n':
        if dau[12:16] != b'IHDR':
            raise ValueError('PNG thieu IHDR: %s' % path)
        return struct.unpack('>II', dau[16:24])
    if dau[:2] in (b'P2', b'P5'):
        so = []
        for dong in dau.split(b'\n')[1:]:
            dong = dong.split(b'#')[0]
            so += [int(x) for x in dong.split()]
            if len(so) >= 2:
                return so[0], so[1]
    raise ValueError('anh ban do khong phai PNG / PGM: %s' % path)


def hinh_hoc_ban_do(yaml_path):
    """map_server yaml -> {'w', 'h', 'res', 'ox', 'oy', 'anh'} (nhu map_server: origin x, y; do phan giai; kich thuoc anh)."""
    with open(yaml_path) as f:
        txt = f.read()
    anh, res, org = doc_khoa(txt, 'image'), doc_khoa(txt, 'resolution'), doc_khoa(txt, 'origin')
    if None in (anh, res, org):
        raise ValueError('%s thieu image / resolution / origin' % yaml_path)
    o = [float(v) for v in org.strip('[]').split(',')]
    p = anh if os.path.isabs(anh) else os.path.join(os.path.dirname(os.path.abspath(yaml_path)), anh)
    w, h = kich_thuoc_anh(p)
    return {'w': w, 'h': h, 'res': float(res), 'ox': o[0], 'oy': o[1], 'anh': p}


def doc_chon_tuyen(s):
    """'0' -> None (bo cam); 'r x y' (r 1 | 2) -> (r, x, y). Sai => ValueError."""
    o = (s or '').split()
    if o == ['0']:
        return None
    if len(o) != 3 or o[0] not in ('1', '2'):
        raise ValueError('chon_tuyen phai la "0" hoac "r x y" (r = 1 | 2): %r' % s)
    return int(o[0]), float(o[1]), float(o[2])


def doc_kinh_nghiem(path=TEP_KINH_NGHIEM, ly_do=('KET_CO_DUONG',)):
    """Diem (x, y) cua ~/kinh_nghiem.csv (cot gio,x,y,huong,tuyen,ly_do; gui_duong.py ghi). Khong co tep => []; dong hong bo qua.
    09/11 buoc 3 #5: chi dem dong co ly_do trong `ly_do` (KET_CO_DUONG = co duong ma ket; VAT_CHAN = vat chan, khong dem)."""
    out = []
    try:
        with open(path, newline='') as f:
            for r in csv.DictReader(f):
                if (r.get('ly_do') or '').strip() not in ly_do:
                    continue
                try:
                    x, y = float(r['x']), float(r['y'])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(x) and math.isfinite(y):
                    out.append((x, y))
    except OSError:
        return []
    return out


def tam_vung_ton(diem):
    """Tam cac vung ton diem: moi diem co >= N_CUM diem (tinh ca no) trong R_CUM => tam = trung binh cac diem do (bo trung)."""
    tam = []
    for x, y in diem:
        gan = [(a, b) for a, b in diem if math.hypot(a - x, b - y) <= R_CUM]
        if len(gan) >= N_CUM:
            c = (round(sum(a for a, _ in gan) / len(gan), 6), round(sum(b for _, b in gan) / len(gan), 6))
            if c not in tam:
                tam.append(c)
    return tam


def luoi(hh, chon, diem_kn):
    """Du lieu OccupancyGrid (hang 0 o duoi, i theo x): 0 | GIA_TON | GIA_CAM. chon = doc_chon_tuyen(...)."""
    w, h, res, ox, oy = hh['w'], hh['h'], hh['res'], hh['ox'], hh['oy']
    tam = tam_vung_ton(diem_kn)
    out = [0] * (w * h)
    cam = None
    if chon is not None:
        r, xe_x, xe_y = chon
        cam = TT.CAM[r]
    for j in range(h):
        cy = oy + (j + 0.5) * res
        for i in range(w):
            cx = ox + (i + 0.5) * res
            v = 0
            if cam is not None:
                x0, x1, y0, y1 = cam
                if x0 <= cx <= x1 and y0 <= cy <= y1 and math.hypot(cx - xe_x, cy - xe_y) > R_THA:
                    v = GIA_CAM
            if v < GIA_TON and any(math.hypot(cx - a, cy - b) <= R_VUNG for a, b in tam):
                v = GIA_TON
            out[j * w + i] = v
    return out


def tom_tat(data, chon, n_kn, n_vung):
    return 'PHAT /lop_huong_dan: chon=%s o_cam=%d o_ton=%d (kinh nghiem %d dong, %d vung)' % (
        'khong' if chon is None else 'tuyen %d tu (%.2f; %.2f)' % chon, sum(1 for v in data if v == GIA_CAM),
        sum(1 for v in data if v == GIA_TON), n_kn, n_vung)


def khop_hinh_hoc(info, hh, tol=1e-6):
    """So OccupancyGrid.info voi hinh hoc ban do goc: (dat?, mo ta)."""
    o = info.origin.position
    ok = (info.width == hh['w'] and info.height == hh['h'] and abs(info.resolution - hh['res']) < tol
          and abs(o.x - hh['ox']) < tol and abs(o.y - hh['oy']) < tol)
    return ok, '%dx%d o %.3f m, goc (%.3f; %.3f); ban do goc %dx%d o %.3f m, goc (%.3f; %.3f)' % (
        info.width, info.height, info.resolution, o.x, o.y, hh['w'], hh['h'], hh['res'], hh['ox'], hh['oy'])


# ---------- nut ROS 2 ----------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
except ImportError:
    rclpy = None

if rclpy is not None:
    def qos_giu():
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class LopHuongDan(Node):
        def __init__(self):
            super().__init__('lop_huong_dan')
            self.declare_parameter('yaml', os.environ.get('LOP_HUONG_DAN_YAML', YAML_MAC_DINH))
            self.declare_parameter('kinh_nghiem', TEP_KINH_NGHIEM)
            self.hh = hinh_hoc_ban_do(self.get_parameter('yaml').value)
            self.pub = self.create_publisher(OccupancyGrid, '/lop_huong_dan', qos_giu())
            self.create_subscription(String, '/chon_tuyen', self._on_chon, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.chon = None
            self.phat()

        def phat(self):
            kn = doc_kinh_nghiem(self.get_parameter('kinh_nghiem').value)
            data = luoi(self.hh, self.chon, kn)
            m = OccupancyGrid()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.info.map_load_time = m.header.stamp
            m.info.resolution = self.hh['res']
            m.info.width, m.info.height = self.hh['w'], self.hh['h']
            m.info.origin.position.x, m.info.origin.position.y = self.hh['ox'], self.hh['oy']
            m.info.origin.orientation.w = 1.0
            m.data = data
            self.pub.publish(m)
            print(tom_tat(data, self.chon, len(kn), len(tam_vung_ton(kn))), flush=True)

        def _on_chon(self, msg):
            try:
                self.chon = doc_chon_tuyen(msg.data)
            except ValueError as e:
                print('BO_QUA /chon_tuyen hong: %s' % e, flush=True)
                return
            self.phat()

    def kiem_costmap(giay=10.0):
        """kiem_xe.sh 8d: /global_costmap/costmap co tin trong `giay` s va cung hinh hoc ban do goc. Tra ma thoat."""
        y = os.environ.get('LOP_HUONG_DAN_YAML', YAML_MAC_DINH)
        try:
            hh = hinh_hoc_ban_do(y)
        except (OSError, ValueError) as e:
            print('KHONG DAT  global_costmap: khong doc duoc ban do goc %s (%s)' % (y, e))
            return 1
        rclpy.init()
        n = rclpy.create_node('kiem_lop_huong_dan')
        nhan = []
        n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: nhan.append(m), qos_giu())
        import time
        t_het = time.monotonic() + giay
        while not nhan and time.monotonic() < t_het:
            rclpy.spin_once(n, timeout_sec=0.1)
        if not nhan:
            print('KHONG DAT  global_costmap: chua co tin /global_costmap/costmap sau %.0f s (planner se treo)' % giay)
            ma = 1
        else:
            ok, mo_ta = khop_hinh_hoc(nhan[0].info, hh)
            print('%s global_costmap cung kich thuoc ban do goc: %s' % ('DAT       ' if ok else 'KHONG DAT ', mo_ta))
            ma = 0 if ok else 1
        n.destroy_node()
        rclpy.shutdown()
        return ma

    def main():
        if len(sys.argv) > 1 and sys.argv[1] == '--kiem-costmap':
            sys.exit(kiem_costmap(float(sys.argv[2]) if len(sys.argv) > 2 else 10.0))
        rclpy.init()
        node = LopHuongDan()
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
        raise SystemExit('can ROS 2 (rclpy, nav_msgs, std_msgs) de chay nut; chi import duoc phan ham thuan')


if __name__ == '__main__':
    main()
