import os
import rclpy
from rclpy.node import Node

class SaveAll(Node):
    def __init__(self):
        super().__init__('save_all')
        self.pkg_path = '/home/guygan/ros2_ws/src/my_robot_description'
        self.maps_dir = os.path.join(self.pkg_path, 'maps')
        self.worlds_dir = os.path.join(self.pkg_path, 'worlds')

        os.makedirs(self.maps_dir, exist_ok=True)
        os.makedirs(self.worlds_dir, exist_ok=True)

        self.get_logger().info("💾 Ready to save world and map.")
        self.get_logger().info("🕹️ Press Ctrl+C to exit after saving.")

        self.input_loop()

    def input_loop(self):
        while rclpy.ok():
            try:
                world_name = input("\n🌍 Enter world filename (without .sdf): ").strip()
                map_name = input("🗺️ Enter map filename (without extension): ").strip()
                if not world_name or not map_name:
                    print("⚠️ Please enter valid names.")
                    continue

                world_path = os.path.join(self.worlds_dir, f"{world_name}.sdf")
                map_path = os.path.join(self.maps_dir, map_name)

                print(f"\n💾 Saving world to: {world_path}")
                print(f"💾 Saving map to: {map_path}")

                # === Save world ===
                os.system(f"gz world --save-world {world_path}")

                # === Save map ===
                os.system(f"ros2 run nav2_map_server map_saver_cli -f {map_path}")

                print("\n✅ Save complete!\n")
                again = input("Do you want to save again? (y/n): ").strip().lower()
                if again != 'y':
                    print("👋 Exiting save_all node.")
                    break

            except KeyboardInterrupt:
                print("\n🛑 Save process interrupted.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = SaveAll()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
