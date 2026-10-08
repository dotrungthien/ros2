#!/usr/bin/env python3
"""막힘(끼임) 감지 노드 (ket_detector).

로봇을 제어하지 않고 "앞으로 가라는 명령이 계속 나가는데 실제로는 못 움직이는 상태" 만 알린다.
- 입력 : /cmd_vel (geometry_msgs/Twist), /odom_rf2o (nav_msgs/Odometry), /scan (sensor_msgs/LaserScan)
- 출력 : /ket (std_msgs/Bool) - 막힘이면 True, 그 외에는 False. 평가 주기마다 발행한다.

판정 (cua_so 초 길이의 슬라이딩 윈도우):
  1) 윈도우 안의 모든 샘플에서 |v 명령| >= v_toi_thieu 이고 |w 명령| <= w_toi_da 이면 "전진 명령 중".
  2) 전진 명령 중인데 rf2o 위치 이동이 dich_chuyen_toi_thieu 미만이면 'nghi_ket' (의심).
  3) 의심이면서 /scan 전방 원뿔의 최소 거리가 거의 변하지 않으면(scan_doi_toi_thieu 미만) 'ket' (막힘).

주의: 모든 숫자 파라미터의 기본값은 측정하지 않은 가정값이다. 실제 로봇에서 측정해 조정할 것.
시간은 전부 time.monotonic() 으로 잰다 (header.stamp 는 쓰지 않는다).
"""
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# 샘플은 윈도우보다 이만큼(초) 더 오래 보관한다 (윈도우 시작 경계 바로 앞의 샘플을 찾기 위해)
GIU_THEM = 0.5
# 윈도우가 "충분하다" 고 보는 최소 비율 (가장 오래된 샘플과 가장 새 샘플의 간격 / cua_so)
TI_LE_DU = 0.95


def normalize_angle(theta):
    """각도를 [-pi, pi] 로 정규화."""
    return math.atan2(math.sin(theta), math.cos(theta))


class KetDetectorNode(Node):
    def __init__(self):
        super().__init__('ket_detector')

        # ---- 파라미터 (시작할 때 한 번만 읽는다; 기본값은 모두 미측정 가정값) ----
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_rf2o')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ket_topic', '/ket')
        self.declare_parameter('cua_so', 2.0)                 # s, 판정 윈도우 길이 (가정값)
        # v_toi_thieu: Nav2 는 접근 구간에서 0.06 m/s 까지 느린 명령을 낼 수 있다 (nav2_params.yaml:120-121).
        #   dieu_khien 은 duty_ngung(0.05) 이상의 모든 명령을 duty_min(0.21) 이상으로 올린다
        #   (0.21 x max_wheel_speed 0.437 = 약 0.092 m/s, duty_ngung 0.05 는 약 0.022 m/s; 이 값들은 파라미터에서
        #   계산한 것이며 차량에서 측정하지 않았다). 따라서 0.06 보다 낮아야 느린 접근 중 막힘도 놓치지 않는다.
        self.declare_parameter('v_toi_thieu', 0.05)           # m/s, 이 이상의 전진 명령이 윈도우 내내 있어야 함 (가정값, 미측정)
        self.declare_parameter('w_toi_da', 0.2)               # rad/s, 회전 명령이 이보다 작아야 "직진" 으로 봄 (가정값)
        self.declare_parameter('dich_chuyen_toi_thieu', 0.02) # m, 윈도우 동안 이보다 덜 움직이면 의심 (가정값)
        self.declare_parameter('goc_truoc_rad', 0.0)          # rad, 전방 방향 (실측: /scan 의 0 rad 가 로봇 앞)
        self.declare_parameter('nua_goc_non_rad', 0.35)       # rad, 전방 원뿔의 반각 (가정값)
        self.declare_parameter('scan_khoang_min', 0.15)       # m, 전방 최소 거리가 이 범위 안일 때만 유효 (가정값)
        self.declare_parameter('scan_khoang_max', 1.5)        # m (가정값)
        self.declare_parameter('scan_doi_toi_thieu', 0.02)    # m, 윈도우 동안 전방 거리 변화가 이보다 작으면 "안 변함" (가정값)
        self.declare_parameter('tan_so_danh_gia', 10.0)       # Hz, 평가 주기 (가정값)
        self.declare_parameter('tuoi_toi_da', 0.5)            # s, cmd/odom/scan 메시지가 이보다 오래되면 없는 것으로 봄 (가정값)

        def param(name):
            return self.get_parameter(name).value

        self.cmd_topic = param('cmd_topic')
        self.odom_topic = param('odom_topic')
        self.scan_topic = param('scan_topic')
        self.ket_topic = param('ket_topic')
        self.cua_so = float(param('cua_so'))
        self.v_toi_thieu = float(param('v_toi_thieu'))
        self.w_toi_da = float(param('w_toi_da'))
        self.dich_chuyen_toi_thieu = float(param('dich_chuyen_toi_thieu'))
        self.goc_truoc = float(param('goc_truoc_rad'))
        self.nua_goc_non = float(param('nua_goc_non_rad'))
        self.scan_khoang_min = float(param('scan_khoang_min'))
        self.scan_khoang_max = float(param('scan_khoang_max'))
        self.scan_doi_toi_thieu = float(param('scan_doi_toi_thieu'))
        self.tan_so_danh_gia = float(param('tan_so_danh_gia'))
        self.tuoi_toi_da = float(param('tuoi_toi_da'))

        # goc_truoc_rad 를 제외한 모든 숫자는 0 보다 커야 한다 (잘못된 값은 조용히 쓰지 않고 종료)
        for ten in ('cua_so', 'v_toi_thieu', 'w_toi_da', 'dich_chuyen_toi_thieu', 'nua_goc_non_rad',
                    'scan_khoang_min', 'scan_khoang_max', 'scan_doi_toi_thieu',
                    'tan_so_danh_gia', 'tuoi_toi_da'):
            if float(param(ten)) <= 0.0:
                self.get_logger().error(f"{ten} 는 0 보다 커야 합니다 (현재 {param(ten)}). 노드를 종료합니다.")
                raise SystemExit(1)
        if self.scan_khoang_min >= self.scan_khoang_max:
            self.get_logger().error(
                f'scan_khoang_min({self.scan_khoang_min}) 은 scan_khoang_max({self.scan_khoang_max}) 보다 '
                f'작아야 합니다. 노드를 종료합니다.')
            raise SystemExit(1)

        # ---- 상태: 콜백은 최신 값과 수신 시각(time.monotonic())만 저장한다 ----
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.cmd_time = None        # 아직 명령이 없으면 None
        self.odom_xy = None
        self.odom_time = None
        self.front_min = None       # 전방 원뿔 최소 거리 (유효 범위 밖이면 None)
        self.scan_time = None
        self.scan_key = None        # (angle_min, angle_increment, 점 개수), 바뀌면 전방 인덱스를 다시 계산
        self.front_idx = []

        self.mau = deque()          # 샘플: (t, |v|, |w|, (x, y) 또는 None, front_min 또는 None)
        self.trang_thai = 'khong_ro'

        self.ket_pub = self.create_publisher(Bool, self.ket_topic, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / self.tan_so_danh_gia, self.tick)

        self.get_logger().info(
            f'ket_detector 시작: {self.cmd_topic}, {self.odom_topic}, {self.scan_topic} -> {self.ket_topic} '
            f'(cua_so={self.cua_so} s, v_toi_thieu={self.v_toi_thieu} m/s, '
            f'dich_chuyen_toi_thieu={self.dich_chuyen_toi_thieu} m)')

    # ------------------------------------------------------------------
    # 콜백: 값과 수신 시각만 저장한다
    # ------------------------------------------------------------------
    def cmd_callback(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.cmd_time = time.monotonic()

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        self.odom_time = time.monotonic()

    def scan_callback(self, msg):
        n = len(msg.ranges)
        key = (msg.angle_min, msg.angle_increment, n)
        if key != self.scan_key:
            # 각도 배치가 바뀔 때만 전방 원뿔에 속하는 인덱스를 다시 계산한다
            self.scan_key = key
            self.front_idx = [
                i for i in range(n)
                if abs(normalize_angle(msg.angle_min + i * msg.angle_increment - self.goc_truoc))
                <= self.nua_goc_non]
        best = None
        for i in self.front_idx:
            r = msg.ranges[i]
            if math.isfinite(r) and r >= msg.range_min and (best is None or r < best):
                best = r
        # 유효 범위 [scan_khoang_min, scan_khoang_max] 밖이거나 유효한 점이 없으면 None
        if best is not None and not (self.scan_khoang_min <= best <= self.scan_khoang_max):
            best = None
        self.front_min = best
        self.scan_time = time.monotonic()

    # ------------------------------------------------------------------
    # 판정
    # ------------------------------------------------------------------
    def con_tuoi(self, t_tin, now):
        """메시지를 받은 적이 있고 tuoi_toi_da 보다 오래되지 않았으면 True."""
        return t_tin is not None and now - t_tin <= self.tuoi_toi_da

    def lay_cua_so(self, t_moi):
        """최근 cua_so 초를 덮는 샘플 목록. 시작 경계 이전의 가장 가까운 샘플부터 포함한다.

        타이머가 흔들리면 경계 바로 뒤의 가장 오래된 샘플이 경계에서 0.1 s 보다 더 떨어질 수 있고,
        그러면 첫 샘플과 마지막 샘플의 간격이 TI_LE_DU * cua_so (1.9 s) 보다 짧아져 상태가
        'khong_ro' 로 떨어진다. 이를 피하려고 경계 바로 앞의 샘플을 하나 더 포함시킨다.
        다만 K7 시험 (흔들림 +-5 ms, 80 tick, seed 7) 에서는 이 샘플을 넣은 경우와 넣지 않은 경우
        모두 상태가 번갈아 바뀌지 않았다. 즉 이 효과는 더 큰 흔들림에서만 나타날 수 있고,
        그 경우의 이득은 아직 측정하지 못했다.
        """
        moc = t_moi - self.cua_so
        bat_dau = 0
        for i, m in enumerate(self.mau):
            if m[0] <= moc:
                bat_dau = i
            else:
                break
        return list(self.mau)[bat_dau:]

    def phan_loai(self, mau):
        """샘플 목록으로 (상태, 수치 dict) 를 판정한다. 부수 효과 없음.

        상태: 'khong_ro'(판단 불가), 'binh_thuong'(전진 명령 없음 또는 움직이는 중),
              'nghi_ket'(의심, 아직 미확정), 'ket'(막힘).
        """
        if len(mau) < 2 or mau[-1][0] - mau[0][0] < TI_LE_DU * self.cua_so:
            return 'khong_ro', {}
        if not all(m[1] >= self.v_toi_thieu and m[2] <= self.w_toi_da for m in mau):
            return 'binh_thuong', {}
        dau, cuoi = mau[0], mau[-1]
        if dau[3] is None or cuoi[3] is None:
            return 'khong_ro', {}      # 전진 명령 중이지만 rf2o 위치가 없어 판단할 수 없음
        dich = math.hypot(cuoi[3][0] - dau[3][0], cuoi[3][1] - dau[3][1])
        if dich >= self.dich_chuyen_toi_thieu:
            return 'binh_thuong', {}
        so = {'v_min': min(m[1] for m in mau), 'dich': dich, 'doi_front': None}
        if dau[4] is None or cuoi[4] is None:
            return 'nghi_ket', so
        so['doi_front'] = abs(cuoi[4] - dau[4])
        if so['doi_front'] < self.scan_doi_toi_thieu:
            return 'ket', so
        return 'nghi_ket', so

    def tick(self):
        now = time.monotonic()
        v = abs(self.cmd_v) if self.con_tuoi(self.cmd_time, now) else 0.0
        w = abs(self.cmd_w) if self.con_tuoi(self.cmd_time, now) else 0.0
        xy = self.odom_xy if self.con_tuoi(self.odom_time, now) else None
        front = self.front_min if self.con_tuoi(self.scan_time, now) else None
        self.mau.append((now, v, w, xy, front))
        while self.mau and self.mau[0][0] < now - (self.cua_so + GIU_THEM):
            self.mau.popleft()

        moi, so = self.phan_loai(self.lay_cua_so(now))
        self.ket_pub.publish(Bool(data=(moi == 'ket')))

        # 로그는 상태가 바뀔 때만 한 줄씩 남긴다 (매 tick 마다 남기지 않는다)
        cu = self.trang_thai
        if moi != cu:
            if moi == 'ket':
                doi = so['doi_front']
                self.get_logger().warning(
                    f"막힘 감지: 최근 {self.cua_so} s 동안 전진 명령(|v| 최소 {so['v_min']:.3f} m/s) "
                    f"이 있었지만 rf2o 이동 {so['dich']:.3f} m, 전방 거리 변화 {doi:.3f} m")
            else:
                if cu == 'ket':
                    self.get_logger().info(f'막힘 해제 ({cu} -> {moi})')
                if moi == 'nghi_ket':
                    self.get_logger().info(
                        f"막힘 의심: rf2o 이동 {so['dich']:.3f} m < {self.dich_chuyen_toi_thieu} m "
                        f'(전방 거리로 아직 확인하지 못함)')
            self.trang_thai = moi


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = KetDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        exit_code = e.code if isinstance(e.code, int) else 1   # 설정 오류는 0 이 아닌 코드로 종료
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
