#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time


class SmartAutonomousExplorer(Node):
    def __init__(self):
        super().__init__('smart_autonomous_explorer')

        # === PARAMETERS ===
        self.safe_distance = 0.5  # ระยะปลอดภัยจากสิ่งกีดขวาง
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.6  # rad/s
        self.scan_topic = '/scan_corrected'

        # === STATE ===
        self.scan_data = None
        self.moving_forward = True
        self.last_rotation_time = 0.0

        # === ROS2 ===
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("🤖 Smart Autonomous Explorer started.")

    # =============================
    #         CALLBACKS
    # =============================
    def scan_callback(self, msg):
        self.scan_data = np.array(msg.ranges)

    # =============================
    #         MAIN LOOP
    # =============================
    def loop(self):
        if self.scan_data is None:
            return

        # Remove NaN/inf values
        scan = np.array([r if np.isfinite(r) else 10.0 for r in self.scan_data])

        front = np.mean(scan[len(scan)//2 - 10: len(scan)//2 + 10])

        if front < self.safe_distance:
            self.get_logger().info("🚧 Front blocked: evaluating best path...")
            self.avoid_obstacle(scan)
        else:
            self.move_forward()

    # =============================
    #         MOVE FORWARD
    # =============================
    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("🟢 Path is clear. Moving forward.")

    # =============================
    #       OBSTACLE AVOIDANCE
    # =============================
    def avoid_obstacle(self, scan):
        num_sectors = 12  # แบ่ง 360° เป็น 30° ต่อ sector
        sector_angle = 360 / num_sectors
        sector_size = len(scan) // num_sectors

        avg_distances = []
        for i in range(num_sectors):
            start = i * sector_size
            end = start + sector_size
            valid = scan[start:end]
            avg = np.mean(valid)
            avg_distances.append(avg)

        # หาทิศทางที่มีระยะเฉลี่ยมากที่สุด (free space)
        best_sector = int(np.argmax(avg_distances))
        best_distance = avg_distances[best_sector]

        if best_distance < self.safe_distance * 1.2:
            # ทุกทิศทางใกล้เกินไป → หันหนีสิ่งกีดขวางที่ใกล้ที่สุด
            closest_idx = int(np.argmin(scan))
            direction = -1 if closest_idx < len(scan)/2 else 1
            self.get_logger().warn("⚠️ No safe direction found! Turning away from nearest obstacle.")
            self.rotate(direction, 90)
            return

        # ถ้ามีทางว่าง → หมุนไปยังทิศที่ดีที่สุด
        angle_offset = (best_sector - num_sectors/2) * sector_angle
        direction = 1 if angle_offset > 0 else -1
        self.get_logger().info(f"↻ Rotating {'left' if direction > 0 else 'right'} toward open space ({abs(angle_offset):.1f}°)")
        self.rotate(direction, abs(angle_offset))
        time.sleep(0.5)

    # =============================
    #            ROTATE
    # =============================
    def rotate(self, direction, degrees):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_speed * direction
        rotate_time = math.radians(degrees) / self.angular_speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < rotate_time:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    # =============================
    #            STOP
    # =============================
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


# =============================
#            MAIN
# =============================
def main(args=None):
    rclpy.init(args=args)
    node = SmartAutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
