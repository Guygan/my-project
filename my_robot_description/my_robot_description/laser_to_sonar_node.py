#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
import numpy as np
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy


class LaserToSonarNode(Node):
    def __init__(self):
        super().__init__('laser_to_sonar_node')

        # --- Parameters (เหลือเฉพาะพารามิเตอร์ที่จำเป็น) ---
        # ไม่ต้องประกาศ input_topic แล้ว เพราะ ROS2 remap ได้โดยตรงจาก launch file
        self.declare_parameter('output_topic', '/sonar_front')
        self.declare_parameter('field_of_view_deg', 30.0)

        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.field_of_view_rad = np.deg2rad(
            self.get_parameter('field_of_view_deg').get_parameter_value().double_value
        )

        # QoS สำหรับข้อมูลจากเซ็นเซอร์
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ✅ Subscribe โดยตรงไปยัง /scan (สามารถ remap ได้จาก launch)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # จะถูก remap เป็น /scan_corrected โดย launch file
            self.scan_callback,
            qos_profile=sensor_qos_profile
        )

        # ✅ Publisher สำหรับส่งข้อมูล sonar
        self.publisher_ = self.create_publisher(Range, output_topic, 10)

        self.get_logger().info(
            f"✅ Node started. Listening to '/scan' (remapped if specified), publishing to '{output_topic}'."
        )

    def scan_callback(self, msg: LaserScan):
        """แปลงข้อมูลจาก LaserScan → Range (จำลอง sonar ด้านหน้า)"""
        num_scans = len(msg.ranges)
        if num_scans == 0:
            return

        # --- กำหนดขอบเขตมุมมอง (Field of View) ---
        center_index = num_scans // 2
        fov_half_angle_rad = self.field_of_view_rad / 2.0
        scans_per_radian = num_scans / (msg.angle_max - msg.angle_min)
        fov_scans = int(fov_half_angle_rad * scans_per_radian)
        start_index = max(0, center_index - fov_scans)
        end_index = min(num_scans - 1, center_index + fov_scans)

        # --- ดึงค่าระยะเฉพาะในช่วงมุมที่กำหนด ---
        ranges_in_fov = msg.ranges[start_index:end_index + 1]
        valid_ranges = [r for r in ranges_in_fov if np.isfinite(r) and r >= msg.range_min]

        # --- หาค่าระยะที่ใกล้ที่สุด ---
        min_distance = msg.range_max
        if valid_ranges:
            min_distance = min(valid_ranges)

        # --- สร้างข้อความประเภท Range ---
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()

        # ✅ ใช้ frame_id เดิมจาก LaserScan เพื่อให้ TF สอดคล้องกับ lidar_link
        range_msg.header.frame_id = msg.header.frame_id

        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = self.field_of_view_rad
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = float(min_distance)

        # --- ส่งข้อมูลออก ---
        self.publisher_.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserToSonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 🔧 ตรวจสอบก่อน shutdown เพื่อป้องกัน double shutdown
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
