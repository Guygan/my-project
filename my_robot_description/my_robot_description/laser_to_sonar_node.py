#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32
import math
import time

# --- ค่าคงที่ของโซนาร์ ---
SONAR_HALF_ANGLE_RAD = 0.4637  # 26.57 องศา (ประมาณ tan(0.5))
STOP_DISTANCE = 0.5  # (50 cm)
WARN_DISTANCE = 1.0  # (100 cm)
# --- Topic ที่ Nav2 ส่งคำสั่งมา (แก้ไขแล้ว) ---
NAV_CMD_VEL_TOPIC = '/cmd_vel_smoothed' # ✨ แก้ไขจาก /cmd_vel_nav
# --- Topic ที่จะส่งคำสั่งสุดท้ายไปให้หุ่นยนต์ ---
ROBOT_CMD_VEL_TOPIC = '/cmd_vel'
# ------------------------------------

class LaserToSonarNode(Node):

    def __init__(self):
        super().__init__('laser_to_sonar_node')
        self.get_logger().info(f'Laser to Sonar Node started. Listening to {NAV_CMD_VEL_TOPIC}, Publishing to {ROBOT_CMD_VEL_TOPIC}')

        # --- สถานะของโซนาร์ ---
        self.min_sonar_dist = float('inf')
        self.warn_triggered = False
        self.stop_triggered = False
        self.last_nav_cmd_vel = Twist() # เก็บคำสั่งล่าสุดจาก Nav2

        # --- Topics ---
        # ✨ แก้ไข Topic ที่ Subscribe
        self.cmd_vel_sub = self.create_subscription(
            Twist, NAV_CMD_VEL_TOPIC, self.nav_cmd_vel_callback, 10)

        # Scan subscriber (ใช้ corrected scan)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_corrected', self.scan_callback, 10)

        # Publisher ไปยังหุ่นยนต์
        self.cmd_vel_pub = self.create_publisher(Twist, ROBOT_CMD_VEL_TOPIC, 10)
        # Publisher สำหรับส่งสัญญาณหยุด (ให้ interactive_stuck_detector)
        self.stop_trigger_pub = self.create_publisher(Empty, '/sonar_stop_trigger', 10)
        # Publisher สำหรับส่งระยะทางที่วัดได้ (ให้ interactive_stuck_detector)
        self.min_dist_pub = self.create_publisher(Float32, '/sonar_min_distance', 10)

    def nav_cmd_vel_callback(self, msg):
        """เก็บคำสั่งล่าสุดที่ได้รับจาก Nav2"""
        self.last_nav_cmd_vel = msg

    def scan_callback(self, msg: LaserScan):
        """ประมวลผล LaserScan เพื่อหาระยะที่ใกล้ที่สุดในกรวยโซนาร์"""
        min_dist = float('inf')
        center_index = len(msg.ranges) // 2
        # คำนวณ index ซ้ายและขวาที่ครอบคลุมมุม SONAR_HALF_ANGLE_RAD
        angle_increment = msg.angle_increment
        indices_per_side = int(math.ceil(SONAR_HALF_ANGLE_RAD / angle_increment))

        start_index = max(0, center_index - indices_per_side)
        end_index = min(len(msg.ranges) - 1, center_index + indices_per_side)

        # หาระยะที่ใกล้ที่สุดในกรวย โดยไม่สนใจค่า 0 หรือ inf
        valid_ranges = [r for r in msg.ranges[start_index:end_index+1] if r > 0.0 and not math.isinf(r)]
        if valid_ranges:
            min_dist = min(valid_ranges)

        self.min_sonar_dist = min_dist
        # Publish ระยะทางที่วัดได้
        min_dist_msg = Float32()
        min_dist_msg.data = float(self.min_sonar_dist) # แปลงเป็น float ปกติ
        self.min_dist_pub.publish(min_dist_msg)

        # --- ตรรกะการตัดสินใจ ---
        output_cmd = self.last_nav_cmd_vel # เริ่มต้นด้วยคำสั่งจาก Nav2

        if self.min_sonar_dist <= STOP_DISTANCE:
            # --- โหมดหยุด ---
            if not self.stop_triggered:
                self.get_logger().error(f'SONAR: STOP! Obstacle at {self.min_sonar_dist:.2f}m')
                self.stop_trigger_pub.publish(Empty()) # ส่งสัญญาณหยุดแค่ครั้งแรก
                self.stop_triggered = True
            if self.warn_triggered:
                 self.warn_triggered = False # ออกจากโหมดเตือน
            # ส่งคำสั่งหยุด (Twist ว่างๆ คือหยุด)
            output_cmd = Twist()

        elif self.min_sonar_dist <= WARN_DISTANCE:
            # --- โหมดเตือน (แต่ไม่หยุด) ---
            if not self.warn_triggered:
                self.get_logger().warn(f'SONAR: WARNING! Obstacle at {self.min_sonar_dist:.2f}m')
                self.warn_triggered = True
            if self.stop_triggered:
                 self.get_logger().info('SONAR: Obstacle moved > STOP distance.')
                 self.stop_triggered = False # ออกจากโหมดหยุด
            # output_cmd ยังคงเป็นคำสั่งจาก Nav2 (ไม่แก้ไข)

        else:
            # --- โหมดปกติ (ทางโล่ง) ---
            if self.stop_triggered or self.warn_triggered:
                self.get_logger().info('SONAR: Path is clear.')
            self.stop_triggered = False
            self.warn_triggered = False
            # output_cmd ยังคงเป็นคำสั่งจาก Nav2 (ไม่แก้ไข)

        # --- Publish Command (ไม่ว่าจะเป็นคำสั่งเดิม หรือคำสั่งหยุด) ---
        self.cmd_vel_pub.publish(output_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LaserToSonarNode()
    try:
        # ใช้ MultiThreadedExecutor เผื่อมี Callback อื่นๆ ในอนาคต
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # ส่งคำสั่งหยุดสุดท้ายก่อนปิด Node
        stop_cmd = Twist()
        # ตรวจสอบว่า publisher ยังใช้ได้ไหมก่อน publish
        if node and node.cmd_vel_pub and not node.cmd_vel_pub.is_destroyed:
             node.get_logger().info("Sending final stop command...")
             node.cmd_vel_pub.publish(stop_cmd)
             time.sleep(0.1) # รอเล็กน้อยให้ข้อความถูกส่ง

        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()