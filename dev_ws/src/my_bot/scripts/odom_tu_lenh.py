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
        # yaw_nguon: theta(방향) 의 출처. 'lenh' = 명령 적분(기본), 'rf2o' = rf2o 라이다 오도메트리의 yaw.
        # 'rf2o' 에서도 x, y 는 계속 명령 속도로 적분한다. odom_source 와는 별개의 파라미터이다.
        self.declare_parameter('yaw_nguon', 'lenh')
        self.declare_parameter('rf2o_topic', '/odom_rf2o')
        self.declare_parameter('rf2o_timeout', 0.5)           # s, 이 시간 넘게 rf2o 가 없으면 명령 적분으로 복귀
        self.declare_parameter('rf2o_cho_tin_dau', 3.0)       # s, 시작 후 이 시간 넘게 rf2o 첫 메시지가 없으면 경고

        def param(name):
            return self.get_parameter(name).value

        # 지원하지 않는 소스는 조용히 무시하지 않고 오류를 남기고 종료한다
        odom_source = param('odom_source')
        if odom_source != 'cmd':
            self.get_logger().error(
                f"odom_source='{odom_source}' 는 아직 구현되지 않았습니다. "
                f"현재는 'cmd' 만 지원합니다. 노드를 종료합니다.")
            raise SystemExit(1)

        yaw_nguon = param('yaw_nguon')
        if yaw_nguon not in ('lenh', 'rf2o'):
            self.get_logger().error(
                f"yaw_nguon='{yaw_nguon}' 는 지원하지 않습니다. "
                f"'lenh' 또는 'rf2o' 만 사용할 수 있습니다. 노드를 종료합니다.")
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
        self.yaw_nguon = yaw_nguon
        self.rf2o_topic = param('rf2o_topic')
        self.rf2o_timeout = float(param('rf2o_timeout'))
        self.rf2o_cho_tin_dau = float(param('rf2o_cho_tin_dau'))

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
        # rf2o yaw 상태 (yaw_nguon == 'rf2o' 일 때만 사용)
        self.rf2o_yaw = 0.0         # 마지막으로 받은 rf2o yaw [rad]
        self.rf2o_time = None       # 마지막 수신 시각 (time.monotonic()), 아직 없으면 None
        self.rf2o_offset = 0.0      # theta = yaw_rf2o + offset (시작 시점 기준으로 맞추기 위한 보정)
        self.rf2o_active = False    # True 이면 현재 theta 를 rf2o 에서 가져오는 중
        self.rf2o_khoi_dong = time.monotonic()  # 노드 시작 시각 (첫 메시지 대기 시간 계산용)
        self.rf2o_da_canh_bao = False           # 첫 메시지 미수신 경고를 이미 냈는지

        self.pose_cov = diag_covariance(POSE_COV_DIAG)
        self.twist_cov = diag_covariance(TWIST_COV_DIAG)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        if self.yaw_nguon == 'rf2o':
            self.create_subscription(Odometry, self.rf2o_topic, self.rf2o_callback, 10)
        self.create_timer(1.0 / self.publish_rate, self.tick)

        self.get_logger().info(
            f'odom_tu_lenh 시작: {self.cmd_topic} -> /odom, TF {self.odom_frame}->{self.base_frame} '
            f'(publish_tf={self.publish_tf}, rate={self.publish_rate} Hz, timeout={self.cmd_timeout} s)')

    def cmd_callback(self, msg):
        """최신 명령과 수신 시각을 저장한다. 적분은 tick 에서 한다."""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def rf2o_callback(self, msg):
        """rf2o 오도메트리에서 yaw 와 수신 시각(monotonic)만 저장한다. header.stamp 는 쓰지 않는다."""
        q = msg.pose.pose.orientation
        self.rf2o_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rf2o_time = time.monotonic()

    def rf2o_theta(self, now):
        """rf2o 가 쓸 수 있으면 보정된 새 theta 를, 아니면 None 을 돌려준다 (None 이면 명령 적분 사용).

        상태가 바뀔 때(수신 시작/복구, 끊김)만 로그를 한 줄 남기고, 매 tick 마다 로그를 남기지 않는다.
        """
        if self.yaw_nguon != 'rf2o':
            return None
        fresh = self.rf2o_time is not None and now - self.rf2o_time < self.rf2o_timeout
        # rf2o 노드가 아예 안 떠 있으면 rf2o_time 이 계속 None 이라 조용히 명령 적분으로만 돌게 된다.
        # 시작 후 rf2o_cho_tin_dau 초가 지나도 첫 메시지가 없으면 경고를 한 번만 남긴다.
        if (self.yaw_nguon == 'rf2o' and self.rf2o_time is None and not self.rf2o_da_canh_bao
                and now - self.rf2o_khoi_dong > self.rf2o_cho_tin_dau):
            self.get_logger().warning(
                f'yaw_nguon=rf2o nhưng chưa nhận tin nào trên {self.rf2o_topic} '
                f'sau {self.rf2o_cho_tin_dau} s: đang dùng theta từ lệnh. '
                f'Kiểm tra node rf2o và tên topic.')
            self.rf2o_da_canh_bao = True
        if fresh and not self.rf2o_active:
            # 첫 수신 또는 끊긴 뒤 복구: theta 가 갑자기 튀지 않도록 offset 을 다시 맞춘다
            self.rf2o_offset = normalize_angle(self.theta - self.rf2o_yaw)
            self.rf2o_active = True
            self.get_logger().info(
                f'{self.rf2o_topic} 수신: theta 를 rf2o yaw 로 전환 (offset={self.rf2o_offset:.3f} rad)')
        elif not fresh and self.rf2o_active:
            self.rf2o_active = False
            self.get_logger().warning(
                f'{self.rf2o_topic} 가 {self.rf2o_timeout} s 이상 없음: 명령 적분 theta 로 임시 복귀')
        if not fresh:
            return None
        return normalize_angle(self.rf2o_yaw + self.rf2o_offset)

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

        theta_moi = self.rf2o_theta(now)   # rf2o 를 쓸 수 없으면 None
        if self.last_tick_time is not None:
            dt = now - self.last_tick_time
            if theta_moi is not None:
                # rf2o yaw 사용: 구간 중간 방향 = 이전 theta 와 새 theta 의 중간 (각도 차이는 [-pi, pi])
                mid = self.theta + normalize_angle(theta_moi - self.theta) / 2.0
                self.x += v * math.cos(mid) * dt
                self.y += v * math.sin(mid) * dt
                self.theta = theta_moi
            else:
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
