#!/usr/bin/env python3
"""장애물 회피 / 결정 로직 노드 (logic_quyet_dinh).

- /scan (LaserScan) 과 /detections (Detection2DArray) 를 합쳐 안전 판단을 한다.
- 위험이 없으면 아무것도 발행하지 않는다 (twist_mux 가 timeout 후 Nav2 로 넘김).
- 위험이 있으면 /cmd_vel_avoid (twist_mux 우선순위 60) 로 정지 또는 제자리 회전 명령을 발행한다.

좌표계: 로봇 기준 x = 앞, y = 왼쪽, 각도 = 왼쪽이 양수 (ROS REP-103).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray


# ---------- 순수 함수 (ROS 없이 테스트 가능) ----------

def scan_to_points(scan, min_valid, angle_offset):
    """LaserScan 을 (거리, 각도, x, y) 점 목록으로 변환. 무효 값(inf, nan, 범위 밖)은 제외."""
    pts = []
    lo = max(scan.range_min, min_valid)
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < lo or r > scan.range_max:
            continue
        th = scan.angle_min + i * scan.angle_increment + angle_offset
        th = math.atan2(math.sin(th), math.cos(th))  # -pi ~ pi 로 정규화
        pts.append((r, th, r * math.cos(th), r * math.sin(th)))
    return pts


def front_distance(pts, half_width, max_look):
    """전방 복도(|y| < half_width, 0 < x < max_look) 안에서 가장 가까운 x. 없으면 max_look."""
    d = max_look
    for _r, _th, x, y in pts:
        if 0.0 < x < d and abs(y) < half_width:
            d = x
    return d


def corridor_has_ray(pts, half_width, max_look):
    """전방 복도 안에 유효한 점이 하나라도 있는지. front_distance 와 같은 영역 조건을 쓴다.

    front_distance 는 점이 없으면 max_look 을 돌려주므로 "트임"과 "측정 불가"를 구분할 수 없다.
    장애물이 min_valid_range 안쪽으로 들어오면 점이 모두 걸러져 front 가 갑자기 커지는데,
    이 함수로 그 상황을 구분한다.
    """
    for _r, _th, x, y in pts:
        if 0.0 < x < max_look and abs(y) < half_width:
            return True
    return False


def side_clearance(pts, look):
    """전방 좌/우 사분면에서 가장 가까운 거리. 값이 클수록 그쪽이 트여 있다."""
    left = right = look
    for r, _th, x, y in pts:
        if x > 0.0 and r < look:
            if y > 0.0:
                left = min(left, r)
            else:
                right = min(right, r)
    return left, right


def pixel_to_bearing(px, width, hfov_deg):
    """이미지 x 픽셀을 로봇 기준 방위각(rad)으로 변환. 이미지 오른쪽 = 로봇 오른쪽 = 음수."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return -math.atan((px - width / 2.0) / fx)


def range_in_window(pts, th_min, th_max):
    """각도 창 [th_min, th_max] 안에서 가장 가까운 거리. 없으면 None."""
    best = None
    for r, th, _x, _y in pts:
        if th_min <= th <= th_max and (best is None or r < best):
            best = r
    return best


# ---------- 노드 ----------

class LogicNode(Node):
    def __init__(self):
        super().__init__('logic_quyet_dinh')
        d = self.declare_parameter
        d('enabled', True)
        d('cmd_topic', '/cmd_vel_avoid')
        # 라이다 기준 거리 (라이다 장착 위치가 정해지면 다시 조정)
        d('stop_distance', 0.30)          # m, 이 안에 장애물이 들어오면 회피 시작
        d('clear_margin', 0.10)           # m, stop_distance + 이 값을 넘어야 회피 해제 (히스테리시스)
        d('corridor_half_width', 0.12)    # m, 차체 반폭 0.08 + 여유 0.04
        d('max_look', 2.0)                # m
        d('side_look', 1.0)               # m, 좌/우 여유 비교 범위
        d('min_valid_range', 0.12)        # m, X4 Pro 최소 측정 거리 근처 노이즈 제외
        d('lidar_angle_offset_deg', 0.0)  # 라이다 0도 방향이 로봇 앞과 다를 때 보정
        d('escape_angular', 1.0)          # rad/s, 회피 회전 속도 (듀티 0.2 미만이면 안 움직임)
        d('escape_hold_time', 0.5)        # s, 복도에 유효한 점이 없을 때 회피 상태/방향을 유지하는 시간
        d('scan_timeout', 0.5)            # s, 이 시간 동안 /scan 이 없으면 안전 정지
        # 사람 검출 결합
        d('person_class_ids', ['person', '1'])
        d('person_min_score', 0.5)
        d('person_stop_distance', 0.60)   # m
        d('detection_hold', 1.5)          # s, 검출 주기 ~1.1 Hz 보다 길게
        d('image_width', 640)
        d('camera_hfov_deg', 53.5)        # OV5647 수평 화각 (센서 모드에 따라 다를 수 있음)
        d('bearing_margin_deg', 5.0)

        self.scan = None
        self.scan_time = 0.0
        self.persons = []
        self.persons_time = 0.0
        self.escape_dir = 0      # +1 = 왼쪽 회전, -1 = 오른쪽 회전, 0 = 회피 안 함
        self.no_ray_since = None  # 복도에 유효한 점이 없어진 시각 (time.monotonic 기준), 있으면 None
        self.was_active = False
        self.last_state = ''

        self.pub = self.create_publisher(Twist, self.p('cmd_topic'), 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections', self.det_callback, 10)
        self.create_timer(0.1, self.tick)  # 10 Hz (twist_mux 의 avoid timeout 보다 충분히 빠르게)

    def p(self, name):
        # 매번 읽어서 ros2 param set 으로 실행 중 튜닝 가능
        return self.get_parameter(name).value

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def det_callback(self, msg):
        ids = [str(v) for v in self.p('person_class_ids')]
        persons = []
        for det in msg.detections:
            if not det.results:
                continue
            res = det.results[0]
            # vision_msgs 버전 차이 대응 (4.x: hypothesis.class_id, 3.x: id)
            hyp = getattr(res, 'hypothesis', res)
            class_id = str(getattr(hyp, 'class_id', getattr(hyp, 'id', '')))
            score = float(getattr(hyp, 'score', 1.0))
            if class_id not in ids or score < self.p('person_min_score'):
                continue
            c = det.bbox.center
            cx = c.position.x if hasattr(c, 'position') else c.x
            persons.append((float(cx), float(det.bbox.size_x)))
        self.persons = persons
        self.persons_time = time.monotonic()

    def nearest_person(self, pts, now):
        """최근 검출된 사람의 방위각 창 안에서 라이다로 잰 가장 가까운 거리. 없으면 None."""
        if not self.persons or now - self.persons_time > self.p('detection_hold'):
            return None
        w = self.p('image_width')
        fov = self.p('camera_hfov_deg')
        margin = math.radians(self.p('bearing_margin_deg'))
        best = None
        for cx, sx in self.persons:
            b1 = pixel_to_bearing(cx - sx / 2.0, w, fov)
            b2 = pixel_to_bearing(cx + sx / 2.0, w, fov)
            dist = range_in_window(pts, min(b1, b2) - margin, max(b1, b2) + margin)
            if dist is not None and (best is None or dist < best):
                best = dist
        return best

    def tick(self):
        if not self.p('enabled'):
            return
        now = time.monotonic()
        twist = None
        state = 'CLEAR'
        front = None
        person_d = None

        if self.scan is None or now - self.scan_time > self.p('scan_timeout'):
            # 안전 우선: 센서가 없으면 자율주행 명령을 막는다 (teleop 은 우선순위가 더 높아 영향 없음)
            twist = Twist()
            state = 'NO_SCAN'
            self.no_ray_since = None  # scan 이 돌아오면 유지 시간을 처음부터 다시 잰다
            self.get_logger().warning('scan 수신 없음: 안전 정지 명령 발행', throttle_duration_sec=2.0)
        else:
            pts = scan_to_points(
                self.scan, self.p('min_valid_range'),
                math.radians(self.p('lidar_angle_offset_deg')))
            front = front_distance(pts, self.p('corridor_half_width'), self.p('max_look'))
            person_d = self.nearest_person(pts, now)
            stop_d = self.p('stop_distance')

            if person_d is not None and person_d < self.p('person_stop_distance'):
                twist = Twist()          # 사람은 움직이므로 회전하지 않고 정지
                state = 'PERSON_STOP'
                self.escape_dir = 0
                self.no_ray_since = None
            else:
                if self.escape_dir != 0:
                    if corridor_has_ray(pts, self.p('corridor_half_width'), self.p('max_look')):
                        # 유효한 점이 있으면 일반 히스테리시스로 해제 판단
                        self.no_ray_since = None
                        if front > stop_d + self.p('clear_margin'):
                            self.escape_dir = 0
                    else:
                        # 장애물이 min_valid_range 안쪽으로 들어와 점이 사라진 경우:
                        # front 가 max_look 으로 튀므로 해제하지 않고 escape_hold_time 동안
                        # 상태와 회전 방향을 그대로 유지한다 (명령은 계속 발행).
                        if self.no_ray_since is None:
                            self.no_ray_since = now
                        if now - self.no_ray_since > self.p('escape_hold_time'):
                            self.escape_dir = 0   # 유지 시간 초과: CLEAR 로 복귀
                            self.no_ray_since = None
                elif front < stop_d:
                    left, right = side_clearance(pts, self.p('side_look'))
                    self.escape_dir = 1 if left >= right else -1
                if self.escape_dir != 0:
                    twist = Twist()
                    twist.angular.z = self.escape_dir * self.p('escape_angular')
                    state = 'ESCAPE_LEFT' if self.escape_dir > 0 else 'ESCAPE_RIGHT'

        # 회피가 끝난 직후 정지 명령을 한 번 보내 회전 관성을 막는다
        if twist is None and self.was_active:
            twist = Twist()
        self.was_active = (state != 'CLEAR')

        if state != self.last_state:
            self.get_logger().info(
                f'상태 {self.last_state or "-"} -> {state} (front={front}, person={person_d})')
            self.last_state = state
        if twist is not None:
            self.pub.publish(twist)


def main():
    rclpy.init()
    node = LogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
