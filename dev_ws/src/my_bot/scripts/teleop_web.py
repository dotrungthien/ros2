#!/usr/bin/env python3
"""원격 조종(teleoperation) 웹 노드.

- 브라우저에서 버튼/키보드로 조종 명령을 받아 /cmd_vel_teleop 으로 발행한다.
- /image_raw 를 MJPEG 스트림(/video)으로 브라우저에 보여준다.
- 안전장치: 명령이 cmd_timeout 초 이상 끊기면 정지(0) 명령을 발행한다.
"""
import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template, request
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_web')

        # 파라미터
        self.declare_parameter('cmd_topic', '/cmd_vel_teleop')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('max_linear', 0.15)    # m/s (Nav2 권장 범위 0.1~0.2)
        self.declare_parameter('max_angular', 1.0)    # rad/s
        self.declare_parameter('cmd_timeout', 0.3)    # 초, 이 시간 동안 명령 없으면 정지
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('port', 5000)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.port = self.get_parameter('port').value

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None      # 마지막 sensor_msgs/Image (변환은 스트림 쪽에서)
        self.latest_image_time = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = 0.0

        self.pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_callback, 1)
        # 10 Hz 로 발행 (dieu_khien 의 watchdog 0.5초보다 충분히 빠르게)
        self.create_timer(0.1, self.publish_cmd)

    def image_callback(self, msg):
        with self.lock:
            self.latest_image = msg
            self.latest_image_time = time.time()

    def set_command(self, linear_norm, angular_norm):
        # 입력은 -1.0 ~ 1.0 으로 제한한 뒤 최대 속도를 곱한다
        linear_norm = max(-1.0, min(1.0, float(linear_norm)))
        angular_norm = max(-1.0, min(1.0, float(angular_norm)))
        with self.lock:
            self.target_linear = linear_norm * self.max_linear
            self.target_angular = angular_norm * self.max_angular
            self.last_cmd_time = time.time()

    def publish_cmd(self):
        twist = Twist()
        with self.lock:
            fresh = (time.time() - self.last_cmd_time) < self.cmd_timeout
            if fresh:
                twist.linear.x = self.target_linear
                twist.angular.z = self.target_angular
        # 명령이 끊기면 발행을 멈춘다 (twist_mux 가 timeout 후 다음 우선순위로 넘어가도록)
        if not fresh:
            return
        self.pub.publish(twist)

    def get_jpeg(self):
        with self.lock:
            msg = self.latest_image
        if msg is None:
            return None
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        ok, buf = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return buf.tobytes() if ok else None

    def status(self):
        with self.lock:
            return {
                'linear': self.target_linear,
                'angular': self.target_angular,
                'cmd_age': round(time.time() - self.last_cmd_time, 2),
                'image_age': round(time.time() - self.latest_image_time, 2)
                if self.latest_image_time else None,
            }


def create_app(node):
    # symlink-install 이면 realpath 가 src 폴더를 가리키므로 templates 를 찾을 수 있다
    base = os.path.dirname(os.path.realpath(__file__))
    app = Flask(__name__, template_folder=os.path.join(base, '..', 'templates'))

    @app.route('/')
    def index():
        return render_template('index.html')

    @app.route('/cmd', methods=['POST'])
    def cmd():
        data = request.get_json(force=True, silent=True) or {}
        node.set_command(data.get('linear', 0.0), data.get('angular', 0.0))
        return jsonify(ok=True)

    @app.route('/status')
    def status():
        return jsonify(node.status())

    @app.route('/video')
    def video():
        def generate():
            while True:
                jpeg = node.get_jpeg()
                if jpeg is not None:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                           + jpeg + b'\r\n')
                time.sleep(0.2)  # /image_raw 가 5 Hz 이므로 그 이상 인코딩할 필요 없음
        return Response(
            generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    return app


def main():
    rclpy.init()
    node = TeleopNode()
    # ROS 콜백은 별도 스레드, Flask 는 메인 스레드
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app = create_app(node)
    try:
        app.run(host='0.0.0.0', port=node.port, threaded=True, use_reloader=False)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
