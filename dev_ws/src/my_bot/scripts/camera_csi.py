#!/usr/bin/env python3
"""
camera_csi.py — Raspberry Pi CSI 카메라(OV5647) 원시 Bayer 프레임을 읽어
sensor_msgs/Image (bgr8)로 변환 후 /image_raw 토픽에 발행하는 ROS2 노드.

읽기 방식: cv2.VideoCapture(CAP_V4L2) + CAP_PROP_CONVERT_RGB=0 을 먼저 시도하고,
원시 Bayer 프레임을 정상적으로 읽지 못하면 v4l2-ctl --stream-mmap 서브프로세스
파이프로 자동 전환한다. 실제로 어느 방식이 선택되었는지는 노드 시작 로그에
"VideoCapture" 또는 "subprocess pipe" 로 표시된다.
"""

import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# CSI 카메라 고정 해상도 (Bayer 10비트, GB10 포맷)
WIDTH = 640
HEIGHT = 480
FRAME_BYTES = WIDTH * HEIGHT * 2  # GB10 = 픽셀당 16비트(리틀엔디안, 하위 10비트만 유효)

SUBDEV = '/dev/v4l-subdev0'  # analogue_gain 컨트롤이 노출되는 서브디바이스
SKIP_FRAMES = 10  # 카메라 시작 직후 불완전하게 기록된 앞쪽 프레임 개수


class CameraCsiNode(Node):
    def __init__(self):
        super().__init__('camera_csi')

        # ---- 파라미터 선언 ----
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('gain', 96)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('gain_b', 1.43)
        self.declare_parameter('gain_g', 1.0)
        self.declare_parameter('gain_r', 1.47)
        self.declare_parameter('gamma', 1.0 / 2.2)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('vertical_blanking', 5820)  # 0 이면 설정하지 않음

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.gain = self.get_parameter('gain').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.gain_b = self.get_parameter('gain_b').get_parameter_value().double_value
        self.gain_g = self.get_parameter('gain_g').get_parameter_value().double_value
        self.gain_r = self.get_parameter('gain_r').get_parameter_value().double_value
        self.gamma = self.get_parameter('gamma').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.vertical_blanking = self.get_parameter('vertical_blanking').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/image_raw', 10)

        # ---- 센서 초기화: analogue_gain 및 GB10 640x480 포맷 설정 ----
        self._set_analogue_gain()
        self._set_v4l2_format()
        # 프레임을 읽기 시작하기 전(_init_capture 보다 먼저)에 vertical_blanking 을 설정한다
        self._set_vertical_blanking()

        # ---- 프레임 읽기 방식 결정 (VideoCapture 우선, 실패 시 파이프) ----
        self._cap = None
        self._proc = None
        self._use_pipe = False
        self._init_capture()

        self._skip_remaining = SKIP_FRAMES

        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"camera_csi 노드 시작됨 (device={self.device}, fps={self.fps}, "
            f"읽기 방식={'subprocess pipe' if self._use_pipe else 'VideoCapture'})"
        )

    # ------------------------------------------------------------------
    # 초기화 헬퍼
    # ------------------------------------------------------------------
    def _set_analogue_gain(self):
        # v4l2-ctl로 서브디바이스에 analogue_gain 컨트롤 설정
        cmd = ['v4l2-ctl', '-d', SUBDEV, f'--set-ctrl=analogue_gain={self.gain}']
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(
                f"analogue_gain 설정 실패 ({SUBDEV}): {result.stderr.strip()}"
            )

    def _set_v4l2_format(self):
        # 기본 YUYV로는 오류(Invalid argument)가 나므로 GB10 640x480으로 강제 설정
        cmd = [
            'v4l2-ctl', '-d', self.device,
            f'--set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat=GB10',
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(
                f"GB10 포맷 설정 실패 ({self.device}): {result.stderr.strip()}"
            )

    def _set_vertical_blanking(self):
        # 센서 fps = pixel_rate / ((640 + hblank) x (480 + vblank)), 640x480 모드 기준:
        #   vblank=5820 -> 58 333 000 / (1852 x (480 + 5820)) = 58 333 000 / 11 667 600 ~= 4.9996 fps
        #   (5 fps 보다 아주 조금 낮게 일부러 잡았다: 센서가 노드의 5 fps 타이머보다 빠르면
        #    큐에 프레임이 쌓여 영상이 지연되기 때문)
        #   vblank 기본값 24 이면 1852 x 504 -> 약 62.49 fps (영상 지연의 원인)
        # 이 값은 현재 640x480 모드에 묶여 있다. 해상도/모드를 바꾸면 다시 계산할 것.
        # 실패해도 노드는 종료하지 않고 경고만 남긴 채 계속 실행한다.
        if self.vertical_blanking == 0:
            return  # 0 = 설정하지 않음
        fallback = "카메라는 센서 기본 속도(약 62.5 fps)로 동작하므로 영상이 지연될 수 있습니다."
        cmd = ['v4l2-ctl', '-d', SUBDEV, f'--set-ctrl=vertical_blanking={self.vertical_blanking}']
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
        except FileNotFoundError:
            self.get_logger().warn(f"vertical_blanking 설정 실패: v4l2-ctl 을 찾을 수 없습니다. {fallback}")
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"vertical_blanking 설정 실패: v4l2-ctl 이 3초 안에 끝나지 않았습니다 ({SUBDEV}). {fallback}")
            return
        except OSError as e:
            self.get_logger().warn(f"vertical_blanking 설정 실패: v4l2-ctl 실행 오류 ({e}). {fallback}")
            return
        if result.returncode != 0:
            self.get_logger().warn(
                f"vertical_blanking 설정 실패 ({SUBDEV}, returncode={result.returncode}): "
                f"{result.stderr.strip()}. {fallback}")
            return
        self.get_logger().info(f"vertical_blanking={self.vertical_blanking} 설정 완료 ({SUBDEV})")

    def _init_capture(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            ok, frame = cap.read()
            if ok and frame is not None and frame.nbytes == FRAME_BYTES:
                self._cap = cap
                self._use_pipe = False
                return
            cap.release()

        # VideoCapture로 원시 Bayer를 못 읽으면 v4l2-ctl 스트리밍 파이프로 전환
        self.get_logger().warn(
            "VideoCapture로 원시 Bayer 프레임을 읽을 수 없어 subprocess pipe 방식으로 전환합니다."
        )
        self._start_pipe_process()
        self._use_pipe = True

    def _start_pipe_process(self):
        # VideoCapture 시도 과정에서 드라이버 포맷이 YUYV로 되돌아갈 수 있으므로
        # 파이프 서브프로세스를 실행하기 직전에 GB10 포맷을 다시 강제 설정한다
        self._set_v4l2_format()
        # Pi 에서는 이 pipe 방식이 실제로 사용된다. 포맷을 다시 설정하면 vertical_blanking 이
        # 초기화되는지 아직 검증하지 못했으므로 여기서 한 번 더 설정한다
        # (같은 값을 다시 쓰는 것이라 여러 번 호출해도 결과가 같다).
        self._set_vertical_blanking()

        cmd = [
            'v4l2-ctl', '-d', self.device,
            '--stream-mmap', '--stream-to=-', '--stream-count=0',
        ]
        # stderr를 PIPE로 두면 읽지 않을 경우 버퍼가 가득 차 막힐 수 있으므로
        # 파일로 리다이렉트한다
        stderr_log = open('/tmp/camera_csi_v4l2.log', 'ab')
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=stderr_log, bufsize=FRAME_BYTES * 4
        )

    def _read_exact(self, stream, n):
        # 파이프에서 정확히 n바이트가 모일 때까지 반복해서 읽는다
        buf = bytearray()
        while len(buf) < n:
            chunk = stream.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    # ------------------------------------------------------------------
    # 프레임 획득 / 처리 / 발행
    # ------------------------------------------------------------------
    def _on_timer(self):
        raw_bytes = self._grab_raw_frame()
        if raw_bytes is None:
            self.get_logger().warn('프레임을 읽지 못했습니다.')
            return

        if self._skip_remaining > 0:
            # 카메라 시작 직후 불완전한 프레임은 버린다
            self._skip_remaining -= 1
            return

        try:
            bgr8 = self._process_frame(raw_bytes)
        except Exception as e:
            self.get_logger().error(f"프레임 처리 중 오류: {e}")
            return

        msg = self.bridge.cv2_to_imgmsg(bgr8, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def _grab_raw_frame(self):
        if self._use_pipe:
            return self._read_exact(self._proc.stdout, FRAME_BYTES)
        ok, frame = self._cap.read()
        if not ok or frame is None or frame.nbytes != FRAME_BYTES:
            return None
        return frame.tobytes()

    def _process_frame(self, raw_bytes):
        # 리틀엔디안 10비트 Bayer 원시 데이터 -> uint16 배열
        raw16 = np.frombuffer(raw_bytes, dtype='<u2').reshape((HEIGHT, WIDTH))
        # 하위 10비트만 유효하므로 16비트 전체 범위로 정렬
        raw16 = (raw16 << 6).astype(np.uint16)

        # BayerGR2BGR (BayerGB 아님, 검증된 색상 배열)
        bgr16 = cv2.cvtColor(raw16, cv2.COLOR_BayerGR2BGR).astype(np.float32)

        # 채널별 화이트밸런스 게인 적용
        bgr16[:, :, 0] *= self.gain_b
        bgr16[:, :, 1] *= self.gain_g
        bgr16[:, :, 2] *= self.gain_r
        bgr16 = np.clip(bgr16, 0, 65535)

        # 감마 보정 후 8비트로 변환
        norm = bgr16 / 65535.0
        norm = np.clip(norm, 0.0, 1.0) ** self.gamma
        bgr8 = (norm * 255.0).astype(np.uint8)

        # 카메라 장착 방향 보정 (180도 회전)
        bgr8 = cv2.rotate(bgr8, cv2.ROTATE_180)
        return bgr8

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        if self._proc is not None:
            self._proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCsiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
