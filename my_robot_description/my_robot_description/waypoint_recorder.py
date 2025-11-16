#!/usr/bin/env python3
# waypoint_recorder.py
# (ฉบับแก้ไข: บันทึกไฟล์ไปยัง config/ ที่ถูกต้อง)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import atexit
import sys # ‼️ 1. เพิ่ม Import
import os # ‼️ 2. เพิ่ม Import
from ament_index_python.packages import get_package_share_directory # ‼️ 3. เพิ่ม Import ที่ขาดไป

# ‼️ 4. [แทนที่] ส่วนนี้ทั้งหมดเพื่อหา Path ที่ถูกต้อง
# หา Path ไปยัง package ของคุณ
package_path = get_package_share_directory('my_robot_description')
# ชี้ไปที่โฟลเดอร์ config
config_path = os.path.join(package_path, 'config')
# นี่คือ Path เต็มของไฟล์ภารกิจ
MISSION_FILE = os.path.join(config_path, 'farm_mission.yaml')


class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')
        self.waypoints = []
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose', 
            self.goal_pose_callback,
            10
        )
        
        self.get_logger().info("✅ Waypoint Recorder พร้อมทำงานแล้ว")
        self.get_logger().warn(f"+++ คลิกปุ่ม '2D Goal Pose' ใน RViz เพื่อบันทึกจุด Waypoint +++")
        self.get_logger().warn(f"+++ พิกัดจะถูกบันทึกลงไฟล์: {MISSION_FILE} +++") # ‼️ 5. อัปเดต Log
        self.get_logger().warn(f"+++ กด [Ctrl+C] ใน Terminal นี้ เมื่อคุณคลิกครบทุกจุด +++")
        
        atexit.register(self.save_to_yaml)

    def goal_pose_callback(self, msg: PoseStamped):
        pose_data = {
            'header': {
                'frame_id': msg.header.frame_id
            },
            'pose': {
                'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}
            }
        }
        self.waypoints.append(pose_data)
        self.get_logger().info(f"✅ บันทึกจุดที่ {len(self.waypoints)} แล้ว! (X: {msg.pose.position.x:.2f}, Y: {msg.pose.position.y:.2f})")

    def save_to_yaml(self):
        if not self.waypoints:
            self.get_logger().info("ไม่มี Waypoints ให้บันทึก.")
            return
            
        self.get_logger().info(f"\n--- กำลังบันทึก {len(self.waypoints)} Waypoints ลงใน {MISSION_FILE} ---")
        try:
            # ‼️ 6. [แก้ไข] ใช้ MISSION_FILE (Path เต็ม)
            with open(MISSION_FILE, 'w') as file:
                yaml.dump(self.waypoints, file, sort_keys=False)
            self.get_logger().info(f"✅ บันทึกไฟล์ '{MISSION_FILE}' สำเร็จ!")
        except Exception as e:
            self.get_logger().error(f"เกิดข้อผิดพลาดในการบันทึกไฟล์ YAML: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
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