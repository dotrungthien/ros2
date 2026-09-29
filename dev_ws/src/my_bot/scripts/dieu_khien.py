#!/usr/bin/env python3
"""
dieu_khien.py — 차동 구동(differential drive) 2륜 모터 제어 ROS2 노드.

역할:
    /cmd_vel (geometry_msgs/Twist) 를 구독하여 좌/우 바퀴 속도를 계산하고,
    gpiozero.Motor 로 TB6612 계열 드라이버에 PWM 신호를 출력한다.
    엔코더가 없으므로 오픈 루프(open-loop) 제어이다.

핀 배치 (BCM 번호):
    왼쪽 바퀴 : PWM=GPIO18, AIN1=GPIO22, AIN2=GPIO27
    오른쪽 바퀴: PWM=GPIO23, BIN1=GPIO25, BIN2=GPIO24

안전 기능:
    watchdog_timeout (기본 0.5초) 동안 새로운 /cmd_vel 이 없으면 두 모터를 자동 정지한다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor


def clamp(value, low=0.0, high=1.0):
    """값을 [low, high] 범위로 제한한다."""
    return max(low, min(high, value))


class DieuKhienNode(Node):
    def __init__(self):
        super().__init__('dieu_khien')

        # ---- 파라미터 (launch 파일 또는 CLI 로 변경 가능) ----
        # wheel_separation: 좌/우 바퀴 중심 사이 거리 [m].
        # 0.3 은 임시값이다. 실제 로봇(RB080 섀시)에서 반드시 측정한 뒤 값을 갱신할 것!
        self.declare_parameter('wheel_separation', 0.3)
        # max_wheel_speed: PWM 듀티 1.0 일 때의 바퀴 선속도 [m/s].
        # 엔코더가 없으므로 실제 주행으로 대략 측정해서 보정해야 한다.
        self.declare_parameter('max_wheel_speed', 0.23)
        # watchdog_timeout: 이 시간 [s] 동안 /cmd_vel 이 없으면 모터 정지.
        self.declare_parameter('watchdog_timeout', 0.5)
        # dao_chieu: 배선이 반대로 되어 있을 때, GPIO 에 쓰기 전에 좌/우 바퀴 속도의 부호를 뒤집는다.
        # True(기본값) 이면 linear.x 가 양수일 때 실제로 차량이 전진하도록 맞춰준다.
        self.declare_parameter('dao_chieu', True)
        # he_so_trai: 왼쪽 바퀴 보정 계수. 좌/우 모터 특성 차이를 보정하기 위해 왼쪽 속도에만 곱한다.
        self.declare_parameter('he_so_trai', 0.92)

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').get_parameter_value().double_value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.dao_chieu = self.get_parameter('dao_chieu').get_parameter_value().bool_value
        self.he_so_trai = self.get_parameter('he_so_trai').get_parameter_value().double_value

        # ---- 모터 초기화 ----
        # gpiozero.Motor: forward/backward 핀에 PWM 을 출력하고, enable 핀은 HIGH 로 유지한다.
        self.left_motor = Motor(forward=22, backward=27, enable=18, pwm=True)
        self.right_motor = Motor(forward=25, backward=24, enable=23, pwm=True)

        # ---- /cmd_vel 구독 ----
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ---- 워치독 타이머 ----
        # 마지막 /cmd_vel 수신 시각을 기록하고, 주기적으로 타임아웃 여부를 검사한다.
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_active = False  # True 이면 이미 타임아웃으로 정지된 상태
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.stop_motors()
        self.get_logger().info(
            f'dieu_khien 시작: wheel_separation={self.wheel_separation} m, '
            f'max_wheel_speed={self.max_wheel_speed} m/s, '
            f'watchdog_timeout={self.watchdog_timeout} s, '
            f'he_so_trai={self.he_so_trai}'
        )

    def cmd_vel_callback(self, msg):
        """/cmd_vel 수신 시 차동 구동 공식으로 좌/우 바퀴 속도를 계산하여 모터에 적용한다."""
        self.last_cmd_time = self.get_clock().now()
        if self.watchdog_active:
            self.get_logger().info('/cmd_vel 수신 재개 — 워치독 해제')
            self.watchdog_active = False

        linear = msg.linear.x
        angular = msg.angular.z

        # 차동 구동 표준 공식:
        #   v_left  = v - ω * L / 2
        #   v_right = v + ω * L / 2
        left_speed = linear - angular * self.wheel_separation / 2.0
        right_speed = linear + angular * self.wheel_separation / 2.0

        self.set_motor(self.left_motor, left_speed, is_left=True)
        self.set_motor(self.right_motor, right_speed, is_left=False)

    def set_motor(self, motor, wheel_speed, is_left=False):
        """바퀴 선속도 [m/s] 를 PWM 듀티로 변환하고 [0, 1] 로 제한한 뒤 방향에 맞게 출력한다."""
        # he_so_trai: 왼쪽 바퀴에만 보정 계수를 곱한다 (오른쪽은 그대로).
        if is_left:
            wheel_speed = wheel_speed * self.he_so_trai
        # dao_chieu 가 True 이면 GPIO 에 쓰기 직전에 부호를 뒤집는다 (watchdog/clamp 로직에는 영향 없음).
        if self.dao_chieu:
            wheel_speed = -wheel_speed
        duty = clamp(abs(wheel_speed) / self.max_wheel_speed)
        if duty == 0.0:
            motor.stop()
        elif wheel_speed > 0.0:
            motor.forward(duty)
        else:
            motor.backward(duty)

    def stop_motors(self):
        """두 모터를 즉시 정지한다."""
        self.left_motor.stop()
        self.right_motor.stop()

    def watchdog_callback(self):
        """마지막 /cmd_vel 이후 watchdog_timeout 이 지나면 두 모터를 정지한다."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout and not self.watchdog_active:
            self.stop_motors()
            self.watchdog_active = True
            self.get_logger().warn(
                f'{self.watchdog_timeout} 초 동안 /cmd_vel 없음 — 모터 정지'
            )

    def destroy_node(self):
        """노드 종료 시 모터를 정지하고 GPIO 자원을 해제한다."""
        self.stop_motors()
        self.left_motor.close()
        self.right_motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DieuKhienNode()
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
