#!/usr/bin/env python3
"""
test_static_image_publisher.py — CHỈ dùng để test cam_bien.py khi KHÔNG có webcam.

Đọc 1 ảnh tĩnh (bất kỳ ảnh nào có chứa vật thể COCO: người, xe, ghế, chó, mèo...)
và publish lặp lại liên tục lên /image_raw, giả lập luồng camera thật.

Cách dùng:
    ros2 run my_bot test_static_image_publisher.py --ros-args -p image_path:=/duong/dan/anh.jpg
"""

import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TestStaticImagePublisher(Node):
    def __init__(self):
        super().__init__('test_static_image_publisher')

        self.declare_parameter('image_path', '')
        self.declare_parameter('fps', 2.0)  # ảnh tĩnh nên không cần fps cao

        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value

        if not image_path or not os.path.exists(image_path):
            self.get_logger().error(
                f"Không tìm thấy ảnh tại '{image_path}'. "
                f"Chạy lại với: --ros-args -p image_path:=/duong/dan/den/anh.jpg"
            )
            self.frame = None
        else:
            self.frame = cv2.imread(image_path)
            if self.frame is None:
                self.get_logger().error(f"File tồn tại nhưng không đọc được ảnh: '{image_path}' (sai định dạng?)")
            else:
                self.get_logger().info(f"Đã load ảnh '{image_path}', kích thước {self.frame.shape[1]}x{self.frame.shape[0]}")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().info("Đang publish ảnh tĩnh lặp lại lên /image_raw ...")

    def timer_callback(self):
        if self.frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link_optical'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestStaticImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
