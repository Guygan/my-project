import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
import numpy as np

class LaserToSonarNode(Node):
    """
    A node that converts LaserScan data to a simulated Sonar (Range) message.
    It finds the closest object directly in front of the robot within a specified field of view.
    """
    def __init__(self):
        super().__init__('laser_to_sonar_node')

        # --- Parameters ---
        # ประกาศพารามิเตอร์เพื่อให้สามารถปรับค่าจากภายนอกได้
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/sonar_front')
        self.declare_parameter('field_of_view_deg', 30.0) # มุมมองของโซน่าร์ (องศา)
        self.declare_parameter('sonar_frame_id', 'base_footprint') # Frame ของโซน่าร์

        # อ่านค่าพารามิเตอร์
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.field_of_view_rad = np.deg2rad(self.get_parameter('field_of_view_deg').get_parameter_value().double_value)
        self.sonar_frame_id = self.get_parameter('sonar_frame_id').get_parameter_value().string_value

        # --- Publisher and Subscriber ---
        self.publisher_ = self.create_publisher(Range, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10)
        
        self.get_logger().info(f"Node started. Listening to '{input_topic}', publishing to '{output_topic}'.")
        self.get_logger().info(f"Sonar Field of View: {self.get_parameter('field_of_view_deg').value}°")

    def scan_callback(self, msg: LaserScan):
        """
        Callback function to process incoming LaserScan messages.
        """
        # --- Find the center beam ---
        # หา index ของลำแสงที่อยู่ตรงกลาง (0 องศา)
        center_index = len(msg.ranges) // 2
        
        # --- Calculate FOV range ---
        # คำนวณว่าจะต้องดูข้อมูลกี่ลำแสงซ้าย-ขวาจากจุดศูนย์กลาง
        # เพื่อให้ได้มุมมอง (Field of View) ตามที่เราต้องการ
        fov_half_angle_rad = self.field_of_view_rad / 2.0
        scans_per_radian = len(msg.ranges) / (msg.angle_max - msg.angle_min)
        fov_scans = int(fov_half_angle_rad * scans_per_radian)

        start_index = max(0, center_index - fov_scans)
        end_index = min(len(msg.ranges) - 1, center_index + fov_scans)
        
        # --- Find minimum distance ---
        # ดึงข้อมูลระยะทางเฉพาะในมุมที่เราสนใจ
        ranges_in_fov = msg.ranges[start_index:end_index+1]
        
        # กรองค่าที่ไม่ถูกต้องออก (inf, nan)
        valid_ranges = [r for r in ranges_in_fov if np.isfinite(r) and r >= msg.range_min]

        # หาระยะที่ใกล้ที่สุด
        min_distance = msg.range_max
        if valid_ranges:
            min_distance = min(valid_ranges)

        # --- Create and publish Range message ---
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = self.sonar_frame_id
        range_msg.radiation_type = Range.INFRARED # หรือ ULTRASOUND ก็ได้
        range_msg.field_of_view = self.field_of_view_rad
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = float(min_distance)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
