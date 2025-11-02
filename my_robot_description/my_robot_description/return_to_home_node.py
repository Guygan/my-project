#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math
import time
import subprocess # <-- [เพิ่ม] สำหรับ Alert

# --- [ใหม่] Topic ที่จะฟังคำสั่งจาก Detector ---
RECOVERY_CHOICE_TOPIC = '/stuck_recovery_choice'
# -----------------------------------------------

class ReturnToHomeNode(Node):
    def __init__(self):
        super().__init__('return_to_home_node')
        
        # --- [ใหม่] ต้องใช้ ReentrantCallbackGroup เพราะ start_return_to_home() มี time.sleep (Blocking)
        self.callback_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10,
            callback_group=self.callback_group) # <-- ใช้ Group

        # --- [ใหม่] Subscriber ที่รอรับคำสั่ง "START" ---
        self.choice_sub = self.create_subscription(
            String,
            RECOVERY_CHOICE_TOPIC,
            self.choice_callback,
            10,
            callback_group=self.callback_group) # <-- ใช้ Group
        # ------------------------------------------------

        self.latest_odom = None
        self.is_running_recovery = False

        self.get_logger().info("✅ ReturnToHomeNode (ODOM-based) initialized. WAITING FOR 'START' command.")
        
        # --- ❌ [ลบ] ลบ self.start_return_to_home() ออกจาก __init__ ---

    def odom_callback(self, msg):
        self.latest_odom = msg

    def get_current_pose(self):
        if self.latest_odom is None: return None, None, None
        pos = self.latest_odom.pose.pose.position
        quat = self.latest_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return pos.x, pos.y, yaw

    def move_straight(self, speed, duration):
        twist = Twist()
        twist.linear.x = speed
        start = time.time()
        while time.time() - start < duration and rclpy.ok() and self.is_running_recovery:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.stop_motion()

    def rotate(self, angular_speed, duration):
        twist = Twist()
        twist.angular.z = angular_speed
        start = time.time()
        while time.time() - start < duration and rclpy.ok() and self.is_running_recovery:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.stop_motion()

    def stop_motion(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)

    # --- [ใหม่] Callback ที่รอรับคำสั่ง ---
    def choice_callback(self, msg: String):
        if msg.data == 'START':
            if self.is_running_recovery:
                self.get_logger().warn("'START' received, but recovery is already in progress. Ignoring.")
                return
            
            self.get_logger().info("Received 'START' command! Triggering Return-To-Home sequence.")
            self.is_running_recovery = True
            try:
                self.start_return_to_home()
            except Exception as e:
                self.get_logger().error(f"Error during return sequence: {e}")
            finally:
                self.is_running_recovery = False
                self.get_logger().info("Return-To-Home sequence finished. Ready for next command.")
    # ---------------------------------------

    # --- [ใหม่] ฟังก์ชันสำหรับ Alert ตอนจบ ---
    def show_completion_alert(self):
        self.get_logger().info("Showing 'Return to Home Complete' alert.")
        try:
            subprocess.run(
                ['zenity', '--info', '--title=Robot Task', '--text=Return to HOME complete.', '--timeout=5'],
                timeout=5.5
            )
        except Exception as e:
            self.get_logger().warn(f"Could not show zenity alert: {e}")
    # ---------------------------------------

    def start_return_to_home(self):
        self.get_logger().info("⚙️ Starting Return-To-Home sequence (ODOM-based)...")

        timeout = time.time() + 5
        while self.latest_odom is None and time.time() < timeout and rclpy.ok():
            self.get_logger().warn("⏳ Waiting for odom data...")
            time.sleep(0.5)
        if self.latest_odom is None:
            self.get_logger().error("❌ No odom data available. Aborting.")
            self.is_running_recovery = False # <-- [เพิ่ม] ออกจากโหมด Recovery
            return

        # 1️⃣ [แก้ไข] กำหนดตำแหน่งบ้าน (spawn) เป็น 0,0
        home_x, home_y = 0.0, 0.0
        self.get_logger().info(f"🏠 Target home position set to: x={home_x:.2f}, y={home_y:.2f}")

        # 2️⃣ ถอยหลัง 20 cm
        self.get_logger().info("↩️ Moving backward 0.2 m...")
        self.move_straight(-0.1, 2.0) # ถอยหลัง 2 วินาที ที่ความเร็ว 0.1 m/s = 20 cm

        # 3️⃣ หมุนกลับหลังหัน 180°
        self.get_logger().info("🔄 Rotating 180°...")
        self.rotate(0.6, math.pi / 0.6)  # หมุน 180° (pi) ที่ความเร็ว 0.6 rad/s (≈5.2s)
        
        if not self.is_running_recovery: # ตรวจสอบเผื่อมีการยกเลิกระหว่างหมุน
             self.get_logger().warn("Recovery canceled during rotation.")
             return

        # 4️⃣ [แก้ไข] เคลื่อนกลับไปยังบ้าน (0,0) (เพิ่มโค้ดเลี้ยว)
        self.get_logger().info("🚗 Moving toward home position (0,0)...")
        while rclpy.ok() and self.is_running_recovery:
            cur_x, cur_y, cur_yaw = self.get_current_pose()
            if cur_x is None:
                self.get_logger().warn("Odom lost, waiting...")
                time.sleep(0.1)
                continue

            dx = home_x - cur_x
            dy = home_y - cur_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.1: # ถ้าเข้าใกล้ 0,0 ในระยะ 10 cm
                self.get_logger().info("✅ Arrived home (0,0).")
                self.stop_motion()
                
                # --- [✨ นี่คือ Alert ที่คุณขอ ✨] ---
                self.show_completion_alert()
                # -----------------------------------
                break

            # --- [✨ นี่คือโค้ดแก้ Overshoot ✨] ---
            # (P-Controller: หันตัวไปหาเป้าหมาย (0,0) ตลอดเวลา)
            
            twist = Twist()
            
            # คำนวณมุมที่ควรจะเป็น
            angle_to_home = math.atan2(dy, dx)
            angle_diff = angle_to_home - cur_yaw
            
            # Normalize angle (ให้มุมอยู่ในช่วง -pi ถึง +pi)
            if angle_diff > math.pi: angle_diff -= 2 * math.pi
            if angle_diff < -math.pi: angle_diff += 2 * math.pi

            # ถ้ามุมเพี้ยนเยอะ ให้หมุน
            if abs(angle_diff) > 0.2: # (ถ้าเพี้ยนเกิน ~11 องศา)
                twist.angular.z = 0.5 if angle_diff > 0 else -0.5
            
            # ถ้ามุมเกือบตรงแล้ว ให้เดินหน้า
            if abs(angle_diff) < 0.6: # (ถ้าเพี้ยนไม่เกิน ~35 องศา)
                # ลดความเร็วถ้าเข้าใกล้มากแล้ว
                twist.linear.x = 0.15 if distance > 0.5 else 0.08

            # ------------------------------------
            
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop_motion()
        self.get_logger().info("🏁 Return-To-Home sequence completed.")

def main(args=None):
    rclpy.init(args=args)
    node = ReturnToHomeNode()
    
    # --- [ใหม่] ต้องใช้ MultiThreadedExecutor ---
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    # ------------------------------------------

if __name__ == '__main__':
    main()