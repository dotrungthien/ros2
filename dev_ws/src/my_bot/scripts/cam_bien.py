#!/usr/bin/env python3
"""
cam_bien.py — SSD MobileNet V2(cv2.dnn)를 이용한 객체 탐지 ROS2 노드.

"두뇌" 아키텍처에서의 역할:
    cam_bien.py (이 노드)  -->  /detections  -->  logic_quyet_dinh.py  -->  /cmd_vel_avoid  -->  dieu_khien.py

입력:
    /image_raw  (sensor_msgs/Image)

출력:
    /detections        (vision_msgs/Detection2DArray)  — 항상 publish
    /detections_image  (sensor_msgs/Image)              — publish_debug_image=True 일 때만 publish

빌드 전 설치 필요:
    sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge python3-opencv
"""

import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge


class CamBienNode(Node):
    def __init__(self):
        super().__init__('cam_bien')

        # ---- 파라미터 (launch 파일이나 CLI로 오버라이드 가능) ----
        self.declare_parameter('model_path', 'models/frozen_inference_graph.pb')
        self.declare_parameter('config_path', 'models/ssd_mobilenet_v2_coco.pbtxt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('input_size', 300)  # SSD MobileNet V2 표준 300x300
        self.declare_parameter('process_every_n', 3)  # N프레임마다 한 번만 추론 실행
        self.declare_parameter('num_threads', 1)  # OpenCV(cv2.dnn)가 사용할 스레드 수

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        self.input_size = self.get_parameter('input_size').get_parameter_value().integer_value
        self.process_every_n = self.get_parameter('process_every_n').get_parameter_value().integer_value
        num_threads = self.get_parameter('num_threads').get_parameter_value().integer_value

        # ---- 모델 로드 전에 OpenCV 스레드 수 설정 ----
        cv2.setNumThreads(num_threads)

        # ---- 모델 로드 ----
        if not os.path.exists(model_path) or not os.path.exists(config_path):
            self.get_logger().error(
                f"Không tìm thấy model tại '{model_path}' hoặc config tại '{config_path}'. "
                f"Node sẽ không xử lý được ảnh cho tới khi có đủ 2 file này."
            )
        self.net = cv2.dnn.readNetFromTensorflow(model_path, config_path)

        # ---- COCO 라벨 — project_37/main37.py (myProjects.zip)에서 정확히 복사 ----
        # 참고: 이 dict는 의도적으로 일부 id(12, 26, 29, 30, 45, 66, 68, 69, 71, 83)를 건너뜀
        # COCO 90개 클래스의 원래 번호 체계이기 때문 — 연속된 list로 바꾸면 안 됨.
        self.class_names = {
            0: 'background',
            1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
            7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
            13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
            18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
            24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
            32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
            37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
            41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
            46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
            51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
            56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
            61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
            67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
            75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
            80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
            86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'
        }

        # ---- ROS 인터페이스 ----
        self.bridge = CvBridge()
        self.frame_count = 0  # process_every_n에 따라 추론을 건너뛰기 위한 프레임 카운터
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub_detections = self.create_publisher(Detection2DArray, '/detections', 10)
        if self.publish_debug:
            self.pub_debug_image = self.create_publisher(Image, '/detections_image', 10)

        self.get_logger().info('cam_bien node đã khởi động, đang chờ ảnh trên /image_raw ...')

    def image_callback(self, msg: Image):
        # N프레임마다 한 번만 추론하고, 나머지 프레임은 리소스 절약을 위해 즉시 건너뜀
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Lỗi chuyển đổi ảnh: {e}")
            return

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, size=(self.input_size, self.input_size), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        detection_array = Detection2DArray()
        detection_array.header = msg.header  # 입력 이미지의 header(stamp, frame_id)를 그대로 복사

        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence < self.conf_threshold:
                continue

            class_id = int(detections[0, 0, i, 1])
            x1 = int(detections[0, 0, i, 3] * w)
            y1 = int(detections[0, 0, i, 4] * h)
            x2 = int(detections[0, 0, i, 5] * w)
            y2 = int(detections[0, 0, i, 6] * h)

            box_w = max(x2 - x1, 0)
            box_h = max(y2 - y1, 0)
            cx = x1 + box_w / 2.0
            cy = y1 + box_h / 2.0

            label_name = self.class_names.get(class_id, str(class_id))

            det = Detection2D()
            det.header = msg.header  # 입력 이미지의 header(stamp, frame_id)를 그대로 복사
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = float(box_w)
            det.bbox.size_y = float(box_h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label_name  # vision_msgs 3.x (Humble): class_id는 string
            hyp.hypothesis.score = confidence
            det.results.append(hyp)

            detection_array.detections.append(det)

            if self.publish_debug:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, f"{label_name} {confidence:.2f}", (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

        self.pub_detections.publish(detection_array)

        if self.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = msg.header  # 원본 이미지 header 유지 (stamp, frame_id 동일)
            self.pub_debug_image.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CamBienNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
