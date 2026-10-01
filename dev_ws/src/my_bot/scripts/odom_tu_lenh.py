#!/usr/bin/env python3
"""명령 기반 오도메트리 추정 노드 (odom_tu_lenh).

엔코더가 없으므로 /cmd_vel (twist_mux 의 최종 출력) 을 적분해 /odom 과 TF 를 만든다.
- 입력 : /cmd_vel (geometry_msgs/Twist)
- 출력 : /odom (nav_msgs/Odometry), TF odom -> base_footprint
- /cmd_vel 이 없어도 publish_rate 로 계속 발행한다 (TF 가 항상 최신이어야 하므로).
- URDF 에 base_footprint -> base_link 가 이미 있으므로 odom -> base_link 는 발행하지 않는다.

주의: 이것은 오픈 루프 추정이다. 바닥 미끄러짐, 모터 데드존, 가감속은 반영되지 않으므로
scale_linear / scale_angular / min_*_effective 를 학교 바닥에서 실측해 보정할 것.
나중에 다른 소스(예: rf2o 라이다 오도메트리)로 바꿀 때는 odom_source 와 publish_tf 로 전환한다.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Transform, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# 공분산 (6x6 행 우선, 대각선만 사용). x, y, z, roll, pitch, yaw 순서.
# 측정할 수 없는 축(z, roll, pitch)은 1e6 으로 "정보 없음" 을 표시한다.
# 오픈 루프이므로 x, y, yaw 는 값을 크게 잡았다 (실측 후 조정).
POSE_COV_DIAG = [0.1, 0.1, 1e6, 1e6, 1e6, 0.2]
# 속도: vx, vy, vz, wx, wy, wz. 차동 구동은 vy = 0 이 보장되므로 작게 둔다.
TWIST_COV_DIAG = [0.05, 1e-3, 1e6, 1e6, 1e6, 0.1]


def diag_covariance(diag):
    """대각 원소 6개로 36 칸 공분산 배열을 만든다."""
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 6 + i] = float(v)
    return cov


def normalize_angle(theta):
    """각도를 [-pi, pi] 로 정규화."""
    return math.atan2(math.sin(theta), math.cos(theta))


class OdomTuLenhNode(Node):
    def __init__(self):
        super().__init__('odom_tu_lenh')

        # ---- 파라미터 (시작할 때 한 번만 읽는다) ----
        self.declare_parameter('odom_source', 'cmd')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 20.0)       # Hz
        self.declare_parameter('cmd_timeout', 0.5)         # s, dieu_khien 의 watchdog 과 동일하게
        self.declare_parameter('publish_tf', True)         # 다른 노드가 TF 를 발행하면 False
        self.declare_parameter('scale_linear', 1.0)        # 선속도 보정 계수 (실측 필요)
        self.declare_parameter('scale_angular', 1.0)       # 각속도 보정 계수 (실측 필요)
        self.declare_parameter('min_linear_effective', 0.0)   # m/s, 이보다 작으면 정지로 간주
        self.declare_parameter('min_angular_effective', 0.0)  # rad/s, 이보다 작으면 정지로 간주

        def param(name):
            return self.get_parameter(name).value

        # 지원하지 않는 소스는 조용히 무시하지 않고 오류를 남기고 종료한다
        odom_source = param('odom_source')
        if odom_source != 'cmd':
            self.get_logger().error(
                f"odom_source='{odom_source}' 는 아직 구현되지 않았습니다. "
                f"현재는 'cmd' 만 지원합니다. 노드를 종료합니다.")
            raise SystemExit(1)

        self.cmd_topic = param('cmd_topic')
        self.odom_frame = param('odom_frame')
        self.base_frame = param('base_frame')
        self.publish_rate = float(param('publish_rate'))
        self.cmd_timeout = float(param('cmd_timeout'))
        self.publish_tf = bool(param('publish_tf'))
        self.scale_linear = float(param('scale_linear'))
        self.scale_angular = float(param('scale_angular'))
        self.min_linear = float(param('min_linear_effective'))
        self.min_angular = float(param('min_angular_effective'))

        if self.publish_rate <= 0.0:
            self.get_logger().error('publish_rate 는 0 보다 커야 합니다. 노드를 종료합니다.')
            raise SystemExit(1)

        # ---- 상태 ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = None   # time.monotonic() 기준, 아직 명령이 없으면 None
        self.last_tick_time = None  # 이전 적분 시각 (time.monotonic())

        self.pose_cov = diag_covariance(POSE_COV_DIAG)
        self.twist_cov = diag_covariance(TWIST_COV_DIAG)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            f'odom_tu_lenh 시작: {self.cmd_topic} -> /odom, TF {self.odom_frame}->{self.base_frame} '
            f'(publish_tf={self.publish_tf}, rate={self.publish_rate} Hz, timeout={self.cmd_timeout} s)')

    def cmd_callback(self, msg):
        """최신 명령과 수신 시각을 저장한다. 적분은 tick 에서 한다."""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def effective_velocity(self, now):
        """watchdog, 데드존, 보정 계수를 적용한 (v, w) 를 돌려준다."""
        # dieu_khien 의 watchdog 을 흉내: 명령이 없거나 오래되면 정지로 본다
        if self.last_cmd_time is None or now - self.last_cmd_time > self.cmd_timeout:
            return 0.0, 0.0
        v = self.cmd_linear
        w = self.cmd_angular
        # 데드존: 명령이 너무 작으면 모터가 실제로 움직이지 않는다고 간주
        if abs(v) < self.min_linear:
            v = 0.0
        if abs(w) < self.min_angular:
            w = 0.0
        return v * self.scale_linear, w * self.scale_angular

    def tick(self):
        now = time.monotonic()   # dt 는 monotonic 으로만 계산 (시스템 시계 점프 방지)
        v, w = self.effective_velocity(now)

        if self.last_tick_time is not None:
            dt = now - self.last_tick_time
            # 원호 적분: 구간 중간 방향(theta + w*dt/2)을 사용
            mid = self.theta + w * dt / 2.0
            self.x += v * math.cos(mid) * dt
            self.y += v * math.sin(mid) * dt
            self.theta = normalize_angle(self.theta + w * dt)
        self.last_tick_time = now

        stamp = self.get_clock().now().to_msg()   # 메시지 헤더는 ROS 시계 사용
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self.pose_cov
        odom.twist.twist.linear.x = v      # 보정 후 유효 속도
        odom.twist.twist.angular.z = w
        odom.twist.covariance = self.twist_cov
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform = Transform()
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = OdomTuLenhNode()
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
