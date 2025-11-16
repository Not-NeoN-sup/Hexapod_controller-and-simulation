#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select

class TeleopHexapod(Node):
    def __init__(self):
        super().__init__('teleop_hexapod')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.15      # linear speed m/s
        self.turn = 1.0        # angular speed rad/s

        self.get_logger().info("Teleop active. Use W/A/S/D keys to move, SPACE to stop, Ctrl+C to quit.")
        self.get_logger().info("W/S: forward/back | A/D: rotate | Q/E: strafe (optional)")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        twist = Twist()

        try:
            while rclpy.ok():
                key = self.get_key()

                # --- Handle key input ---
                if key == 'w':
                    twist.linear.x = self.speed
                elif key == 's':
                    twist.linear.x = -self.speed
                elif key == 'a':
                    twist.angular.z = self.turn
                elif key == 'd':
                    twist.angular.z = -self.turn
                elif key == 'q':
                    twist.linear.y = self.speed
                elif key == 'e':
                    twist.linear.y = -self.speed
                elif key in [' ', '\x03', '\x1b']:  # space, Ctrl+C, or ESC
                    break
                else:
                    continue  # ignore other keys

                # --- Publish one movement ---
                self.pub.publish(twist)
                print(f"\r[CMD] linear: ({twist.linear.x:.2f}, {twist.linear.y:.2f}), angular: {twist.angular.z:.2f}", end="", flush=True)

                # --- Immediately reset to 0 ---
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.pub.publish(twist)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            print("\n[EXIT] Teleop terminated cleanly.")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopHexapod()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
