import rclpy
from rclpy.node import Node
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # แก้ตรงนี้ให้ตรงกับพอร์ตของ Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        self.get_logger().info('Motor control node started')

        # วนรับคำสั่งจากผู้ใช้
        while rclpy.ok():
            command = input(
                "\nEnter command:\n"
                "  F = Forward\n"
                "  B = Backward\n"
                "  L = Left\n"
                "  R = Right\n"
                "  S = Stop\n"
                "  Q = Quit\n"
                "Your command: "
            ).strip().upper()

            if command == 'F':
                self.send_command('F')
            elif command == 'B':
                self.send_command('B')
            elif command == 'L':
                self.send_command('L')
            elif command == 'R':
                self.send_command('R')
            elif command == 'S':
                self.send_command('S')
            elif command == 'Q':
                self.get_logger().info('Exiting motor control node.')
                break
            else:
                self.get_logger().warn(f"Unknown command: {command}")

    def send_command(self, cmd):
        try:
            self.ser.write(cmd.encode())
            self.get_logger().info(f'Sent command: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
