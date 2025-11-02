#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.timer import Timer
# --- [✨ เพิ่ม] ---
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
# -----------------

import subprocess
import threading
import time
import sys
import traceback

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32, String
from action_msgs.msg import GoalStatusArray, GoalStatus

# --- Constants ---
STUCK_TIME_LIMIT = 15.0
COUNTDOWN_TIME = 10 
CMD_VEL_TOPIC = '/cmd_vel_smoothed' # <-- [✨ แก้ไข] ดักฟัง Topic ที่ถูกต้อง
ODOM_TOPIC = '/odom'
SONAR_STOP_DISTANCE = 0.5
CMD_MOVE_THRESHOLD = 0.1
ODOM_STUCK_THRESHOLD = 0.01
POPUP_TIMEOUT = 60
NAV_STATUS_TOPIC = '/navigate_to_pose/_action/status' 
RECOVERY_CHOICE_TOPIC = '/stuck_recovery_choice'
# -------------------------

class InteractiveStuckDetector(Node):

    def __init__(self):
        super().__init__('interactive_stuck_detector')
        self.get_logger().info('Interactive Stuck Detector Node started (v_FINAL_FIX_v2).')

        self.callback_group = ReentrantCallbackGroup()

        self.stuck_start_time = None
        self.alert_shown = False
        self.last_cmd_vel = None
        self.last_odom_twist = None
        self.user_choice = None
        self.last_sonar_distance = float('inf')
        self.nav_is_active = False 
        self.alert_was_sonar_triggered = False

        self.reset_timer: Timer | None = None
        self.in_wait_mode = False
        self.wait_mode_process: subprocess.Popen | None = None
        self.recovery_in_progress = False 

        # --- [✨ เพิ่ม] Action Client เพื่อยกเลิก Goal ---
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.callback_group)
        self.current_nav_goal_handle: ClientGoalHandle | None = None
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available! Cannot cancel goals.')
        else:
            self.get_logger().info('Nav2 action server found.')
        # ---------------------------------------------

        self.choice_pub = self.create_publisher(
            String, RECOVERY_CHOICE_TOPIC, 10)

        # --- Subscribers ---
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_callback, 10, callback_group=self.callback_group)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10, callback_group=self.callback_group)
        self.sonar_stop_sub = self.create_subscription(Empty, '/sonar_stop_trigger', self.sonar_stop_callback, 10, callback_group=self.callback_group)
        self.sonar_distance_sub = self.create_subscription(Float32, '/sonar_min_distance', self.sonar_distance_callback, 10, callback_group=self.callback_group)
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray, 
            NAV_STATUS_TOPIC, 
            self.nav_status_callback, 
            10, 
            callback_group=self.callback_group)

        # --- Timers (Logic) ---
        self.check_timer = self.create_timer(1.0, self.check_stuck_status, callback_group=self.callback_group)
        self.action_timer = self.create_timer(0.5, self.process_user_choice, callback_group=self.callback_group)

        self.get_logger().info('Node init complete. Waiting for triggers.')

    # --- Callbacks (แก้ไข) ---
    def cmd_vel_callback(self, msg): self.last_cmd_vel = msg
    def odom_callback(self, msg): self.last_odom_twist = msg.twist.twist
    def sonar_distance_callback(self, msg): self.last_sonar_distance = msg.data
    
    def nav_status_callback(self, msg: GoalStatusArray):
        is_active = False
        new_goal_handle_found = False
        
        for status in msg.status_list:
            if (status.status == GoalStatus.STATUS_EXECUTING or 
                status.status == GoalStatus.STATUS_ACCEPTED):
                is_active = True
                
                # --- [✨ เพิ่ม] เก็บ Goal Handle ล่าสุดไว้ ---
                if self.current_nav_goal_handle is None or self.current_nav_goal_handle.goal_id.uuid != status.goal_info.goal_id.uuid:
                    try:
                        goal_handle = self.nav_action_client._goal_handles.get(tuple(status.goal_info.goal_id.uuid))
                        if goal_handle:
                            self.current_nav_goal_handle = goal_handle
                            new_goal_handle_found = True
                    except Exception as e:
                        self.get_logger().warn(f"Could not get goal handle from status: {e}")
                # ------------------------------------------
                break 
        
        if new_goal_handle_found:
             self.get_logger().info(f'Captured new active goal handle: {self.current_nav_goal_handle.goal_id.uuid.hex()}')

        # [✨ แก้ไข] ตรรกะ "UN-MUTE"
        if not is_active and self.nav_is_active:
             self.get_logger().info('Nav2 goal sequence finished.')
             self.current_nav_goal_handle = None 
             if self.recovery_in_progress:
                 self.get_logger().warn('Recovery navigation finished. UN-MUTING detector.')
                 self.recovery_in_progress = False 
                 
        self.nav_is_active = is_active

    # --- [✨ เพิ่ม] ฟังก์ชันยกเลิก Goal ---
    def cancel_current_nav_goal(self):
        if self.current_nav_goal_handle and self.nav_is_active:
            self.get_logger().warn(f"Sending CANCEL request for Nav2 goal: {self.current_nav_goal_handle.goal_id.uuid.hex()}")
            future = self.current_nav_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info("No active Nav2 goal to cancel.")
            
    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            if cancel_response:
                self.get_logger().info(f"Nav2 goal cancellation request accepted.")
            else:
                self.get_logger().error(f"Nav2 goal cancellation request REJECTED.")
        except Exception as e:
            self.get_logger().error(f"Exception during goal cancellation: {e}")
        finally:
            self.current_nav_goal_handle = None
            self.nav_is_active = False
    # -----------------------------------

    # --- Trigger Logic (แก้ไข) ---
    def sonar_stop_callback(self, msg):
        if self.recovery_in_progress:
            self.get_logger().info('Sonar trigger ignored (Recovery in progress).')
            return
            
        self.get_logger().warn('Sonar stop trigger received.')
        is_busy = self.alert_shown or self.user_choice is not None or self.in_wait_mode

        if is_busy:
            self.get_logger().info('Alert busy or in Wait Mode, ignoring sonar trigger.')
            return

        self.get_logger().error('SONAR DETECTED! Triggering alert.')
        if self.stuck_start_time is not None: self.stuck_start_time = None; self.get_logger().info('Reset backup timer.')
        
        self.alert_was_sonar_triggered = True 
        self.trigger_interactive_alert_thread()

    def check_stuck_status(self):
        if self.recovery_in_progress:
            self.get_logger().debug('Stuck check skipped (Recovery in progress).')
            return
            
        if not self.nav_is_active:
            if self.stuck_start_time is not None: 
                self.stuck_start_time = None
                self.get_logger().info('Nav goal inactive. Resetting backup timer.')
            return 

        if self.last_cmd_vel is None or self.last_odom_twist is None: return
        
        is_busy = self.alert_shown or self.user_choice is not None or self.in_wait_mode
        if is_busy: return

        cmd_is_moving = (abs(self.last_cmd_vel.linear.x) > CMD_MOVE_THRESHOLD or abs(self.last_cmd_vel.angular.z) > CMD_MOVE_THRESHOLD)
        odom_is_stopped = (abs(self.last_odom_twist.linear.x) < ODOM_STUCK_THRESHOLD and abs(self.last_odom_twist.angular.z) < ODOM_STUCK_THRESHOLD)

        if cmd_is_moving and odom_is_stopped:
            if self.stuck_start_time is None: self.stuck_start_time = self.get_clock().now(); self.get_logger().warn('Backup Timer Started (Nav is Active).')
            duration = (self.get_clock().now() - self.stuck_start_time).nanoseconds / 1e9
            
            if duration >= STUCK_TIME_LIMIT:
                self.get_logger().error('BACKUP TIMER EXPIRED! Triggering alert.')
                self.last_sonar_distance = -1.0 
                self.alert_was_sonar_triggered = False 
                self.trigger_interactive_alert_thread()
                
        else:
            if self.stuck_start_time is not None: self.stuck_start_time = None; self.get_logger().info('Reset backup timer (Moving freely).')

    # --- User Choice Handling (แก้ไข) ---
    def process_user_choice(self):
        if self.user_choice is None: return
        choice = self.user_choice
        self.user_choice = None # เคลียร์ทันที

        self.stuck_start_time = None
        self.cancel_existing_reset_timer()

        if choice == 'COUNTDOWN_CANCELED':
            self.get_logger().info("Processing COUNTDOWN_CANCELED: Resetting alert state.")
            self.safe_reset_alert_flag()
            return 

        elif choice == 'WAIT':
            self.get_logger().info(f'WAIT selected. Entering indefinite Wait Mode.')
            self.alert_shown = False 
            self.trigger_wait_mode_alert_thread() 

        # --- [✨✨✨ นี่คือส่วนที่แก้ไข Mute Mode + Cancel Goal ✨✨✨] ---
        
        elif choice == 'RECALCULATE': 
            self.get_logger().warn(f"Choice '{choice}' selected. MUTING detector until goal completion.")
            self.recovery_in_progress = True # <-- MUTE
            
            self.cancel_current_nav_goal()
            
            self.get_logger().info(f'Processing choice: {choice}. Publishing to {RECOVERY_CHOICE_TOPIC}')
            msg = String(); msg.data = choice
            self.choice_pub.publish(msg)
            
            if self.alert_shown or self.in_wait_mode:
                self.reset_timer = self.create_timer(1.5, self.safe_reset_alert_flag, callback_group=self.callback_group)
            
            if self.in_wait_mode:
                self.in_wait_mode = False

        elif choice == 'START':
            self.get_logger().warn(f"Choice '{choice}' selected. (NOT Muting detector).")
            
            self.get_logger().warn("Cancelling active Nav2 goal to prevent cmd_vel conflict...")
            self.cancel_current_nav_goal()
            
            self.get_logger().info(f'Processing choice: {choice}. Publishing to {RECOVERY_CHOICE_TOPIC}')
            msg = String(); msg.data = choice
            self.choice_pub.publish(msg)
            
            if self.alert_shown or self.in_wait_mode:
                self.reset_timer = self.create_timer(1.5, self.safe_reset_alert_flag, callback_group=self.callback_group)
            
            if self.in_wait_mode:
                self.in_wait_mode = False
        # --- [✨✨✨ จบส่วนที่แก้ไข ✨✨✨] ---

    # --- (ฟังก์ชัน Helper ของ Zenity - ไม่ต้องแก้ไข) ---

    def trigger_interactive_alert_thread(self):
        if self.alert_shown or self.user_choice is not None or self.in_wait_mode:
            self.get_logger().warn('Alert trigger requested, but already active.')
            return
        self.alert_shown = True
        self.get_logger().info('Starting MAIN alert thread (10s countdown -> 3 choices).')
        threading.Thread(target=self.run_alert_dialog, daemon=True).start()

    def run_alert_dialog(self):
        countdown_cancelled_or_failed = False
        countdown_process = None
        choice_process = None
        alert_result_choice = 'COUNTDOWN_CANCELED' 
        
        try:
            current_distance = self.last_sonar_distance
            is_sonar_trigger_event = self.alert_was_sonar_triggered 
            distance_text = f"{current_distance:.2f}m" if current_distance >= 0.0 else "N/A (Timer)"
            initial_text = f"Obstacle at {distance_text}! Pausing..." if is_sonar_trigger_event else f"Stuck (Backup Timer)! Pausing..."
            
            self.get_logger().info(f'Showing countdown alert ({COUNTDOWN_TIME}s)...')
            countdown_cmd = ['zenity', '--progress', '--width=350', '--title=Robot Paused!', f'--text={initial_text}', '--percentage=0', '--auto-close']
            countdown_process = subprocess.Popen(countdown_cmd, stdin=subprocess.PIPE, text=True, bufsize=1, stderr=subprocess.PIPE)
            
            for i in range(COUNTDOWN_TIME):
                if not rclpy.ok(): countdown_cancelled_or_failed = True; break
                remaining_time = COUNTDOWN_TIME - i; percentage = int(((i + 1) / COUNTDOWN_TIME) * 100)
                update_text = f"# {initial_text}\n# Waiting... {remaining_time}s remaining.\n{percentage}"
                
                try:
                    if is_sonar_trigger_event and self.last_sonar_distance > SONAR_STOP_DISTANCE:
                        self.get_logger().info(f"Obstacle cleared! (Dist: {self.last_sonar_distance:.2f}m). Cancelling countdown.")
                        countdown_cancelled_or_failed = True
                        break 
                    if countdown_process.poll() is None and countdown_process.stdin and not countdown_process.stdin.closed and self.alert_shown:
                        countdown_process.stdin.write(update_text + '\n'); countdown_process.stdin.flush()
                    else:
                        countdown_cancelled_or_failed = True
                        if self.alert_shown: self.get_logger().warn("Zenity countdown closed early by user.")
                        else: self.get_logger().warn("Countdown aborted externally.")
                        break
                except (BrokenPipeError, OSError): countdown_cancelled_or_failed = True; break
                time.sleep(1.0) 
            
            if countdown_process and countdown_process.poll() is None:
                try: countdown_process.stdin.close()
                except Exception: pass
                try: countdown_process.wait(timeout=1.0)
                except subprocess.TimeoutExpired: countdown_process.kill(); countdown_process.wait()

            if countdown_cancelled_or_failed:
                self.get_logger().info('Main countdown cancelled/failed/cleared. Setting choice to COUNTDOWN_CANCELED for reset.')
                alert_result_choice = 'COUNTDOWN_CANCELED' 
            
            else: 
                self.get_logger().info('Countdown finished. Showing 3 recovery options...')
                
                choice_cmd = [
                    'zenity', '--list', '--width=400', '--title=ROBOT STUCK',
                    '--text=Choose recovery action:', '--radiolist',
                    '--column=Choice', '--column=Action',
                    'TRUE', 'Return to Check point',
                    'FALSE', 'Enter Wait Mode (Pause)', 
                    'FALSE', 'Return to Home (0,0)',
                    f'--timeout={POPUP_TIMEOUT}'
                ]
                
                choice_process = subprocess.Popen(choice_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                stdout, stderr = choice_process.communicate(timeout=POPUP_TIMEOUT + 5.0)
                choice = stdout.strip() if stdout else ""
                
                if choice == 'Return to Home (0,0)': alert_result_choice = 'START'
                elif choice == 'Return to Check point': alert_result_choice = 'RECALCULATE' 
                elif choice == 'Enter Wait Mode (Pause)': alert_result_choice = 'WAIT' 
                else:
                    self.get_logger().warn(f'No choice selected in main dialog. Defaulting to WAIT MODE.')
                    alert_result_choice = 'WAIT'

        except subprocess.TimeoutExpired:
             self.get_logger().warn(f'Main choice dialog timed out. Defaulting to WAIT MODE.'); alert_result_choice = 'WAIT'
        except Exception as e:
            self.get_logger().error(f'Failed main dialog: {e}\n{traceback.format_exc()}. Default: COUNTDOWN_CANCELED (Reset).')
            alert_result_choice = 'COUNTDOWN_CANCELED' 
        finally:
            self.user_choice = alert_result_choice
            if countdown_process and countdown_process.poll() is None:
                try: countdown_process.kill(); countdown_process.wait()
                except Exception: pass
            if choice_process and choice_process.poll() is None:
                try: choice_process.kill(); choice_process.wait()
                except Exception: pass

    def trigger_wait_mode_alert_thread(self):
        if self.in_wait_mode or self.alert_shown:
            self.get_logger().warn('Wait Mode trigger requested, but already active.')
            return
        
        if self.alert_was_sonar_triggered and self.last_sonar_distance > SONAR_STOP_DISTANCE:
            self.get_logger().info("Wait Mode entered, but obstacle already clear. Resetting.")
            self.user_choice = 'COUNTDOWN_CANCELED' 
            return

        self.in_wait_mode = True
        self.alert_shown = False 
        self.get_logger().info('Starting WAIT MODE alert thread.')
        threading.Thread(target=self.run_wait_mode_dialog, daemon=True).start()

    def run_wait_mode_dialog(self):
        alert_result_choice = 'COUNTDOWN_CANCELED' 
        self.wait_mode_process = None
        
        try:
            self.get_logger().info('Showing Wait Mode (2 choices)...')
            
            choice_cmd = [
                'zenity', '--list', '--width=400', '--title=ROBOT PAUSED (Wait Mode)',
                '--text=Obstacle present. Robot is paused.\nChoose action:', '--radiolist',
                '--column=Choice', '--column=Action',
                'TRUE', 'Return to Check point (Resume)',
                'FALSE', 'Return to Home (0,0)'
            ]
            
            self.wait_mode_process = subprocess.Popen(choice_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            user_made_choice = False

            while rclpy.ok() and self.in_wait_mode:
                if self.alert_was_sonar_triggered and self.last_sonar_distance > SONAR_STOP_DISTANCE:
                    self.get_logger().info('Obstacle cleared during Wait Mode! Auto-cancelling.')
                    alert_result_choice = 'COUNTDOWN_CANCELED'
                    break 

                poll_result = self.wait_mode_process.poll()
                if poll_result is not None:
                    user_made_choice = True
                    break
                
                time.sleep(0.25)
            
            if self.wait_mode_process and self.wait_mode_process.poll() is None:
                try: self.wait_mode_process.kill(); self.wait_mode_process.wait(timeout=1.0)
                except Exception: pass
            
            if user_made_choice:
                stdout, _ = self.wait_mode_process.communicate()
                choice = stdout.strip()
                
                if choice == 'Return to Check point (Resume)': alert_result_choice = 'RECALCULATE'
                elif choice == 'Return to Home (0,0)': alert_result_choice = 'START'
                else: 
                    self.get_logger().warn('Wait Mode dialog cancelled by user. Resetting.')
                    alert_result_choice = 'COUNTDOWN_CANCELED'
            else:
                self.get_logger().info(f'Wait Mode dialog concluded without user input. Result: {alert_result_choice}')
        
        except Exception as e:
            self.get_logger().error(f'Failed Wait Mode dialog: {e}. Resetting.')
            alert_result_choice = 'COUNTDOWN_CANCELED'
        finally:
            self.user_choice = alert_result_choice
            self.in_wait_mode = False 
            self.wait_mode_process = None

    def cancel_existing_reset_timer(self):
        if self.reset_timer is not None: self.reset_timer.cancel(); self.reset_timer = None

    def safe_reset_alert_flag(self):
        if self.reset_timer is not None: self.reset_timer.cancel(); self.reset_timer = None

        if self.in_wait_mode:
            self.get_logger().info('Safe reset called, canceling active Wait Mode.')
            self.in_wait_mode = False
            if self.wait_mode_process and self.wait_mode_process.poll() is None:
                try: self.wait_mode_process.kill(); self.wait_mode_process.wait()
                except Exception: pass
            self.wait_mode_process = None
        
        if self.alert_shown:
            self.alert_shown = False
            self.last_sonar_distance = float('inf')
            self.alert_was_sonar_triggered = False
            self.get_logger().info('*** Alert flag reset. Ready for next detection. ***')
        elif not self.alert_shown and not self.in_wait_mode: 
            self.get_logger().info('Safe reset called, but all flags already false. Skipping.')

    # --- (จบส่วนฟังก์ชัน Helper) ---

# --- Main (แก้ไข) ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = InteractiveStuckDetector()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt received, shutting down...')
    except Exception as e:
        log_msg = f"Unhandled exception in spin: {e}\n{traceback.format_exc()}"
        if node:
            try: node.get_logger().fatal(log_msg)
            except Exception: pass
        else:
            print(f"FATAL ERROR during node init: {log_msg}", file=sys.stderr)
    finally:
        print("Initiating shutdown...", file=sys.stderr)
        if executor:
            executor.shutdown()
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Shutdown sequence finished.", file=sys.stderr)


if __name__ == '__main__':
    main()