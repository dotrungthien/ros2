#!/usr/bin/env python3
"""원격 조종(teleoperation) 웹 노드.

- 브라우저에서 버튼/키보드로 조종 명령을 받아 /cmd_vel_teleop 으로 발행한다.
- /image_raw 를 MJPEG 스트림(/video)으로 브라우저에 보여준다.
- 안전장치: 명령이 cmd_timeout 초 이상 끊기면 발행을 멈춘다 (정지(0) 명령은 발행하지 않음). twist_mux 가 timeout 후 다음 우선순위로 넘어간다.
- 접근 코드(access_code 파라미터)가 설정되면 /login 에서 인증한 브라우저만 사용할 수 있다.
  비어 있으면(기본값) 인증 없이 동작한다 (집에서 테스트용).
"""
import hmac
import math
import os
import secrets
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import (Flask, Response, jsonify, make_response, redirect,
                   render_template, render_template_string, request)
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


# ---------- 접근 코드 / 쿠키 설정 ----------
COOKIE_NAME = 'teleop_token'
COOKIE_MAX_AGE = 8 * 3600     # 초, 쿠키 유효 시간 8시간
MAX_FAILS = 5                 # 연속 실패 허용 횟수
LOCKOUT_SECONDS = 30.0        # 잠금 시간 (time.monotonic 으로 측정)

LOGIN_PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>teleop login</title>
<style>
  body { font-family: sans-serif; background: #111; color: #eee; display: flex;
         justify-content: center; padding-top: 20vh; }
  form { display: flex; flex-direction: column; gap: 12px; width: 260px; }
  input, button { font-size: 1.1rem; padding: 10px; }
  .err { color: #ff6b6b; }
</style></head><body>
<form method="post" action="/login">
  <h3>접근 코드</h3>
  <input type="password" name="code" autocomplete="off" autofocus>
  <button type="submit">로그인</button>
  {% if error %}<div class="err">{{ error }}</div>{% endif %}
</form></body></html>"""


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
        # 접근 코드. 비어 있으면 인증을 요구하지 않는다. 코드에 직접 쓰지 말고
        # 실행 시 -p access_code:=... 또는 launch/param 파일로 전달할 것.
        self.declare_parameter('access_code', '')

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.port = self.get_parameter('port').value
        self.access_code = self.get_parameter('access_code').value
        # 쿠키에 넣는 값은 접근 코드가 아니라 노드 시작 시 만든 무작위 토큰이다.
        # 노드를 껐다 켜면 토큰이 바뀌어 이전 쿠키는 무효가 된다.
        self.auth_token = secrets.token_hex(16)

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
            self.latest_image_time = time.monotonic()

    def set_command(self, linear_norm, angular_norm):
        # 입력은 -1.0 ~ 1.0 으로 제한한 뒤 최대 속도를 곱한다
        linear_norm = max(-1.0, min(1.0, float(linear_norm)))
        angular_norm = max(-1.0, min(1.0, float(angular_norm)))
        with self.lock:
            self.target_linear = linear_norm * self.max_linear
            self.target_angular = angular_norm * self.max_angular
            self.last_cmd_time = time.monotonic()

    def publish_cmd(self):
        twist = Twist()
        with self.lock:
            fresh = (time.monotonic() - self.last_cmd_time) < self.cmd_timeout
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
                'cmd_age': round(time.monotonic() - self.last_cmd_time, 2),
                'image_age': round(time.monotonic() - self.latest_image_time, 2)
                if self.latest_image_time else None,
            }


def create_app(node):
    # symlink-install 이면 realpath 가 src 폴더를 가리키므로 templates 를 찾을 수 있다
    base = os.path.dirname(os.path.realpath(__file__))
    app = Flask(__name__, template_folder=os.path.join(base, '..', 'templates'))

    # 로그인 시도 제한 상태 (Flask 가 멀티스레드이므로 락으로 보호)
    auth_lock = threading.Lock()
    auth_state = {'fails': 0, 'locked_until': 0.0}

    def auth_required():
        return node.access_code != ''

    def is_authed():
        if not auth_required():
            return True
        cookie = request.cookies.get(COOKIE_NAME, '')
        # 비-ASCII 입력에서 TypeError 가 나지 않도록 bytes 로 비교
        return hmac.compare_digest(cookie.encode('utf-8'), node.auth_token.encode('utf-8'))

    def unauthorized():
        return jsonify(ok=False, error='unauthorized'), 401

    @app.route('/login', methods=['GET', 'POST'])
    def login():
        if not auth_required():
            return redirect('/')
        if request.method == 'GET':
            return render_template_string(LOGIN_PAGE, error=None)

        with auth_lock:
            remaining = auth_state['locked_until'] - time.monotonic()
            if remaining > 0:
                # 잠금 중에는 맞는 코드도 거부한다
                msg = f'시도 횟수 초과. {math.ceil(remaining)}초 후에 다시 시도하세요.'
                return render_template_string(LOGIN_PAGE, error=msg), 429
            code = request.form.get('code', '')
            if hmac.compare_digest(code.encode('utf-8'), node.access_code.encode('utf-8')):
                auth_state['fails'] = 0   # 성공하면 카운터 초기화
                ok = True
            else:
                ok = False
                auth_state['fails'] += 1
                if auth_state['fails'] >= MAX_FAILS:
                    auth_state['fails'] = 0
                    auth_state['locked_until'] = time.monotonic() + LOCKOUT_SECONDS
                    msg = (f'{MAX_FAILS}회 연속 실패. '
                           f'{math.ceil(LOCKOUT_SECONDS)}초 동안 시도할 수 없습니다.')
                else:
                    msg = '코드가 올바르지 않습니다.'
        if not ok:
            return render_template_string(LOGIN_PAGE, error=msg), 401

        resp = make_response(redirect('/'))
        resp.set_cookie(COOKIE_NAME, node.auth_token, max_age=COOKIE_MAX_AGE,
                        httponly=True, samesite='Strict')
        return resp

    @app.route('/')
    def index():
        if not is_authed():
            return redirect('/login')
        return render_template('index.html')

    @app.route('/cmd', methods=['POST'])
    def cmd():
        if not is_authed():
            return unauthorized()
        data = request.get_json(force=True, silent=True) or {}
        node.set_command(data.get('linear', 0.0), data.get('angular', 0.0))
        return jsonify(ok=True)

    @app.route('/status')
    def status():
        if not is_authed():
            return unauthorized()
        return jsonify(node.status())

    @app.route('/video')
    def video():
        if not is_authed():
            return unauthorized()

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
    if node.access_code == '':
        node.get_logger().warning('access_code 가 비어 있음: 인증 없이 동작 (테스트 전용)')
    app = create_app(node)
    try:
        app.run(host='0.0.0.0', port=node.port, threaded=True, use_reloader=False)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
