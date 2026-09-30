#!/usr/bin/env python3
"""집에서 logic_quyet_dinh 를 시험하기 위한 가짜 /scan, /detections 발행 노드.

실행 중에 ros2 param set 으로 장애물 거리/각도와 사람 위치를 바꿀 수 있다.
  ros2 param set /test_fake_scan obstacle_distance 0.25
  ros2 param set /test_fake_scan person_cx 320.0
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class FakeScan(Node):
    def __init__(self):
        super().__init__('test_fake_scan')
        self.declare_parameter('background', 3.0)          # m, 장애물 없는 방향의 거리
        self.declare_parameter('obstacle_distance', 3.0)   # m, background 와 같으면 장애물 없음
        self.declare_parameter('obstacle_angle_deg', 0.0)  # 왼쪽이 양수
        self.declare_parameter('obstacle_width_deg', 20.0)
        self.declare_parameter('person_cx', -1.0)          # 0 이상이면 person 검출 발행 (픽셀)
        self.declare_parameter('person_width_px', 120.0)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.create_timer(0.1, self.publish_scan)
        self.create_timer(1.0, self.publish_det)

    def publish_scan(self):
        bg = self.get_parameter('background').value
        obs = self.get_parameter('obstacle_distance').value
        ang = self.get_parameter('obstacle_angle_deg').value
        width = self.get_parameter('obstacle_width_deg').value

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -math.pi
        msg.angle_increment = math.radians(1.0)
        msg.angle_max = msg.angle_min + 359 * msg.angle_increment
        msg.range_min = 0.12
        msg.range_max = 10.0
        ranges = []
        for i in range(360):
            deg = -180.0 + i
            diff = (deg - ang + 180.0) % 360.0 - 180.0
            ranges.append(float(obs if abs(diff) <= width / 2.0 else bg))
        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def publish_det(self):
        cx = self.get_parameter('person_cx').value
        arr = Detection2DArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'camera_frame'
        if cx >= 0.0:
            det = Detection2D()
            det.header = arr.header
            c = det.bbox.center
            if hasattr(c, 'position'):   # vision_msgs 4.x
                c.position.x = float(cx)
                c.position.y = 240.0
            else:                        # vision_msgs 3.x
                c.x = float(cx)
                c.y = 240.0
            det.bbox.size_x = float(self.get_parameter('person_width_px').value)
            det.bbox.size_y = 300.0
            hyp = ObjectHypothesisWithPose()
            if hasattr(hyp, 'hypothesis'):
                hyp.hypothesis.class_id = 'person'
                hyp.hypothesis.score = 0.9
            else:
                hyp.id = 'person'
                hyp.score = 0.9
            det.results.append(hyp)
            arr.detections.append(det)
        self.det_pub.publish(arr)


def main():
    rclpy.init()
    node = FakeScan()
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
