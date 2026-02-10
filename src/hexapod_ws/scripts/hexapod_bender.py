#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class SimpleBender(Node):
    def __init__(self):
        super().__init__('hexapod_bender')
        
        # --- ROBOT DIMENSIONS ---
        self.coxa_len = 0.05
        self.femur_len = 0.12
        self.tibia_len = 0.18
        
        # --- CONFIGURATION ---
        self.HOME_Z = -0.15  # Default standing height
        self.body_pitch = 0.0
        self.body_roll = 0.0
        self.body_height = self.HOME_Z
        
        # --- ROS COMMUNICATION ---
        self.pub = self.create_publisher(Float64MultiArray, '/hexapod_controller/commands', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Run at 30Hz
        self.create_timer(0.033, self.update_pose)
        self.get_logger().info("Simple Bender Active. (Sensors Disabled to stop shaking)")

    def cmd_callback(self, msg):
        # Keyboard 'i' and ',' control Pitch
        self.body_pitch = np.clip(msg.linear.x * 0.5, -0.4, 0.4)
        
        # Keyboard 'j' and 'l' control Roll
        self.body_roll  = np.clip(msg.linear.y * 0.5, -0.4, 0.4)
        
        # Keyboard 'q' and 'z' control Height (if available)
        if msg.linear.z != 0:
            self.body_height = self.HOME_Z + (msg.linear.z * 0.05)

    def calculate_ik(self, x, y, z):
        # The specific IK that works for your URDF
        hip_angle = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2) - self.coxa_len
        
        D = (r**2 + z**2 - self.femur_len**2 - self.tibia_len**2) / (2 * self.femur_len * self.tibia_len)
        D = np.clip(D, -1.0, 1.0)
        
        # Knee Direction Logic
        tibia_angle = -np.arctan2(-np.sqrt(1 - D**2), D)
        
        femur_angle = np.arctan2(z, r) - np.arctan2(
            self.tibia_len * np.sin(tibia_angle), 
            self.femur_len + self.tibia_len * np.cos(tibia_angle)
        )
        
        return hip_angle, femur_angle, tibia_angle

    def update_pose(self):
        # Standard Hexapod Layout (30, 90, 150...)
        leg_angles = np.radians([30, 90, 150, 210, 270, 330])
        R_BODY = 0.12 
        
        joint_commands = []

        for i in range(6):
            # 1. Calculate Hip Position
            lx = R_BODY * np.cos(leg_angles[i])
            ly = R_BODY * np.sin(leg_angles[i])
            
            # 2. Apply Tilt (Pitch/Roll)
            # This moves the body frame relative to the feet
            tilt_z = (lx * np.sin(self.body_pitch)) + (ly * -np.sin(self.body_roll))
            
            # 3. Determine Target for the Foot
            # To tilt the body UP on one side, we must push the foot DOWN on that side.
            # Target Z is relative to the Hip.
            target_z = self.body_height - tilt_z
            
            # Keep feet planted at default distance
            target_x = self.coxa_len + self.femur_len + (self.tibia_len * 0.5)
            target_y = 0.0

            # 4. Solve IK
            q1, q2, q3 = self.calculate_ik(target_x, target_y, target_z)
            joint_commands.extend([q1, q2, q3])

        msg = Float64MultiArray()
        msg.data = joint_commands
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()