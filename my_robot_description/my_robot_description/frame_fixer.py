import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock

class FrameFixerNode(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        self.publisher_ = self.create_publisher(LaserScan, '/scan_corrected', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # ต้นฉบับจาก Gazebo bridge
            self.listener_callback,
            10)

        # ✅ subscribe เวลา simulation โดยตรง
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)

        self.sim_time = None
        self.get_logger().info('✅ Scan Frame Fixer node started with simulation clock support.')

    def clock_callback(self, msg: Clock):
        self.sim_time = msg.clock  # เก็บ simulation time ล่าสุดไว้

    def listener_callback(self, msg: LaserScan):
        # ✅ แก้ frame_id
        msg.header.frame_id = 'lidar_link'

        # ✅ ใช้เวลาจำลองจริง ถ้ามี
        if self.sim_time is not None:
            msg.header.stamp = self.sim_time
        else:
            # fallback — ใช้เวลาปัจจุบันของ node (เผื่อช่วงแรกยังไม่มี clock)
            msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameFixerNode()
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
