#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ✨ [แก้ไข] Import Service Type ที่ถูกต้อง
from nav2_msgs.srv import ClearEntireCostmap 

CHOICE_TOPIC = '/stuck_recovery_choice'

# ✨ [แก้ไข] แก้ไขชื่อ Service ที่เรียกผิด
CLEAR_COSTMAP_SERVICE = '/global_costmap/clear_entirely_costmap' 

class RecalculatePathNode(Node):

    def __init__(self):
        super().__init__('recalculate_path_node') # (ชื่อ Node 12 ของคุณ)
        
        self.subscription = self.create_subscription(
            String,
            CHOICE_TOPIC,
            self.choice_callback,
            10)
            
        self.clear_costmap_client = self.create_client(
            ClearEntireCostmap, 
            CLEAR_COSTMAP_SERVICE)
        
        self._action_in_progress = False 

        self.get_logger().info('Recalculate Path Node (v_FIXED) started, waiting for choice.')

        # รอ Service
        while not self.clear_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{CLEAR_COSTMAP_SERVICE} not available, waiting...')
        self.get_logger().info(f'Service {CLEAR_COSTMAP_SERVICE} is ready.')

    def choice_callback(self, msg):
        # รอฟังคำสั่ง RECALCULATE
        if msg.data == 'RECALCULATE' and not self._action_in_progress:
            self.get_logger().info('Received RECALCULATE choice. Clearing Global Costmap...')
            self._action_in_progress = True
            
            # สร้าง Request
            req = ClearEntireCostmap.Request()
            
            # เรียก Service
            future = self.clear_costmap_client.call_async(req)
            future.add_done_callback(self.clear_costmap_done_callback)

        elif msg.data == 'RECALCULATE' and self._action_in_progress:
            self.get_logger().warn("RECALCULATE choice received, but already in progress. Ignoring.")

    def clear_costmap_done_callback(self, future):
        """Callback เมื่อเคลียร์ Costmap เสร็จ"""
        try:
            response = future.result()
            self.get_logger().info('Global Costmap cleared successfully.')
            # (ที่นี่ คุณอาจจะอยาก Publish อะไรบางอย่าง
            # เพื่อบอกให้ BT ทำงานต่อ หรือ Re-plan)
        except Exception as e:
            self.get_logger().error(f'Failed to clear Global Costmap: {e}')
        finally:
            self.get_logger().info("Recalculate sequence finished.")
            self._action_in_progress = False # Reset Flag

def main(args=None):
    rclpy.init(args=args)
    node = RecalculatePathNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()