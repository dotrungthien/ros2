#!/usr/bin/env python3
"""MPU6050 자이로 Z 축 적분 노드 (gyro_mpu6050).

I2C 로 MPU6050 의 GYRO_ZOUT(0x47) 을 읽어 yaw 각도를 적분해 발행한다. 로봇을 제어하지 않는다.
- 출력 : /gyro_yaw (std_msgs/Float64) - yaw [rad], [-pi, pi] 로 정규화. 보정이 끝난 뒤부터 매 tick 발행한다.

동작:
  1) 시작 후 calib_s 초 동안은 raw 값의 평균을 bias 로 구하고 적분하지 않는다 (이때 로봇은 정지해 있어야 한다).
     raw 표준편차가 30 을 넘으면 로봇이 움직이고 있을 수 있다고 경고한다.
  2) 그 뒤에는 yaw += dau_yaw * (raw - bias) / gyro_scale * dt * pi / 180 로 적분한다 (raw 단위: LSB, 131 LSB = 1 deg/s).
  3) I2C 오류(OSError) 가 나면 그 tick 은 건너뛰고, 끊긴 구간은 적분하지 않는다 (시간 기준점을 다시 잡는다).

주의: 모든 숫자 파라미터의 기본값은 측정하지 않은 가정값이다. 자이로 bias 는 온도에 따라 변하므로
오래 쓰면 yaw 가 서서히 밀린다 (드리프트). 시간은 전부 time.monotonic() 으로 잰다.
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# MPU6050 레지스터
REG_PWR_MGMT_1 = 0x6B     # 0x00 을 쓰면 sleep 해제
REG_GYRO_ZOUT_H = 0x47    # GYRO_ZOUT 상위 바이트 (하위 바이트가 바로 뒤에 온다)

CANH_BAO_GIAN_CACH = 5.0  # 초, I2C 오류 경고는 이 간격에 최대 한 번만 남긴다
STD_TOI_DA = 30.0         # 보정 구간 raw 표준편차가 이보다 크면 로봇이 움직였을 수 있다고 경고 (가정값)


def normalize_angle(theta):
    """각도를 [-pi, pi] 로 정규화."""
    return math.atan2(math.sin(theta), math.cos(theta))


class GyroMpu6050Node(Node):
    def __init__(self, bus=None):
        super().__init__('gyro_mpu6050')

        # ---- 파라미터 (시작할 때 한 번만 읽는다; 기본값은 모두 미측정 가정값) ----
        self.declare_parameter('i2c_bus', 1)                  # I2C 버스 번호 (Raspberry Pi: /dev/i2c-1)
        self.declare_parameter('i2c_addr', 0x68)              # MPU6050 주소 (AD0 핀이 HIGH 이면 0x69)
        self.declare_parameter('rate_hz', 100.0)              # Hz, 읽기/적분 주기
        self.declare_parameter('calib_s', 2.0)                # s, 시작 후 bias 를 구하는 시간 (정지 상태 필요)
        self.declare_parameter('gyro_scale', 131.0)           # LSB/(deg/s), 기본 범위 +-250 deg/s
        self.declare_parameter('dau_yaw', 1.0)                # yaw 부호 (+1 / -1), 장착 방향에 따라 바꾼다
        self.declare_parameter('topic_yaw', '/gyro_yaw')

        def param(name):
            return self.get_parameter(name).value

        self.i2c_bus = int(param('i2c_bus'))
        self.i2c_addr = int(param('i2c_addr'))
        self.rate_hz = float(param('rate_hz'))
        self.calib_s = float(param('calib_s'))
        self.gyro_scale = float(param('gyro_scale'))
        self.dau_yaw = float(param('dau_yaw'))
        self.topic_yaw = param('topic_yaw')

        for ten in ('rate_hz', 'calib_s', 'gyro_scale'):
            if float(param(ten)) <= 0.0:
                self.get_logger().error(f"{ten} 는 0 보다 커야 합니다 (현재 {param(ten)}). 노드를 종료합니다.")
                raise SystemExit(1)

        # ---- I2C 버스 (bus 를 넘기면 그대로 쓴다: 시험용 가짜 버스) ----
        if bus is None:
            try:
                import smbus2   # 필요할 때만 import (가짜 버스를 쓸 때는 필요 없다)
            except ImportError as e:
                self.get_logger().error(
                    f'smbus2 를 import 할 수 없습니다 ({e}). pip install smbus2 로 설치하세요. 노드를 종료합니다.')
                raise SystemExit(1)
            try:
                bus = smbus2.SMBus(self.i2c_bus)
            except OSError as e:
                self.get_logger().error(
                    f'I2C 버스 {self.i2c_bus} 를 열 수 없습니다 ({e}). 노드를 종료합니다.')
                raise SystemExit(1)
        self.bus = bus

        # MPU6050 은 전원이 켜지면 sleep 상태이므로 PWR_MGMT_1 에 0x00 을 써서 깨운다
        try:
            self.bus.write_byte_data(self.i2c_addr, REG_PWR_MGMT_1, 0x00)
        except OSError as e:
            self.get_logger().error(
                f'MPU6050(0x{self.i2c_addr:02x}) 을 깨울 수 없습니다 ({e}). 배선과 주소를 확인하세요. 노드를 종료합니다.')
            raise SystemExit(1)

        # ---- 상태 ----
        self.yaw = 0.0
        self.bias = None            # 보정이 끝나기 전에는 None
        self.calib_raw = []
        self.t_calib0 = None        # 첫 번째로 성공한 읽기의 시각 (time.monotonic())
        self.last_t = None          # 이전 적분 시각, 끊김 직후에는 None (그 구간은 적분하지 않는다)
        self.so_loi = 0             # I2C 오류 횟수
        self.last_warn = None       # 마지막 I2C 오류 경고 시각

        self.yaw_pub = self.create_publisher(Float64, self.topic_yaw, 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(
            f'gyro_mpu6050 시작: I2C 버스 {self.i2c_bus}, 주소 0x{self.i2c_addr:02x}, {self.rate_hz} Hz, '
            f'calib_s={self.calib_s} s -> {self.topic_yaw} (보정 중에는 로봇을 움직이지 마세요)')

    def doc_raw(self):
        """GYRO_ZOUT 2 바이트를 big-endian 부호 있는 정수로 읽는다."""
        data = self.bus.read_i2c_block_data(self.i2c_addr, REG_GYRO_ZOUT_H, 2)
        return int.from_bytes(bytes(data), 'big', signed=True)

    def tick(self):
        now = time.monotonic()
        try:
            raw = self.doc_raw()
        except OSError as e:
            # 이 tick 은 건너뛰고, 끊긴 구간은 적분하지 않는다
            self.so_loi += 1
            self.last_t = None
            if self.last_warn is None or now - self.last_warn >= CANH_BAO_GIAN_CACH:
                self.get_logger().warning(f'I2C 읽기 오류 ({e}), 누적 {self.so_loi} 회. 해당 구간은 적분하지 않습니다.')
                self.last_warn = now
            return

        if self.bias is None:
            # ---- 보정: calib_s 초 동안 raw 평균을 bias 로 구하고 적분하지 않는다 ----
            if self.t_calib0 is None:
                self.t_calib0 = now
            if now - self.t_calib0 < self.calib_s:
                self.calib_raw.append(raw)
                return
            n = len(self.calib_raw)
            self.bias = sum(self.calib_raw) / n
            std = math.sqrt(sum((r - self.bias) ** 2 for r in self.calib_raw) / n)
            if std > STD_TOI_DA:
                self.get_logger().warning(
                    f'보정 중 raw 표준편차 {std:.1f} > {STD_TOI_DA}: 로봇이 움직이고 있었을 수 있습니다. '
                    f'bias 가 부정확할 수 있습니다.')
            self.get_logger().info(f'보정 완료: bias={self.bias:.2f} raw (표본 {n} 개, 표준편차 {std:.2f})')
            self.last_t = now
            self.yaw_pub.publish(Float64(data=self.yaw))
            return

        # ---- 적분 ----
        if self.last_t is None:
            self.last_t = now       # 끊김 직후 첫 tick: 기준점만 다시 잡고 적분하지 않는다
        else:
            dt = now - self.last_t
            self.last_t = now
            self.yaw = normalize_angle(
                self.yaw + self.dau_yaw * (raw - self.bias) / self.gyro_scale * dt * math.pi / 180.0)
        self.yaw_pub.publish(Float64(data=self.yaw))


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = GyroMpu6050Node()
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
