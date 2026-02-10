#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import time

class HexapodStableMover(Node):
    def __init__(self):
        super().__init__('hexapod_stable_mover')
        
        self._declare_parameters()
        self._load_parameters()
        self._initialize_leg_geometry()
        
        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        

        self.current_leg_positions = {name: (self.HOME_X, self.HOME_Y, self.HOME_Z) for name in self.LEG_NAMES}
        
        self.get_logger().info("Grounded Smooth Controller Active. Ensuring 6-leg contact...")

    def _declare_parameters(self):
        self.declare_parameter("topic", "/hexapod_controller/commands")
        self.declare_parameter("step_length", 0.06) 
        self.declare_parameter("lift_height", 0.09) 
        self.declare_parameter("coxa_len", 0.05)
        self.declare_parameter("femur_len", 0.12)
        self.declare_parameter("tibia_len", 0.18)

    def _load_parameters(self):
        self.step_len = self.get_parameter("step_length").value
        self.lift_h = self.get_parameter("lift_height").value
        self.coxa = self.get_parameter("coxa_len").value
        self.femur = self.get_parameter("femur_len").value
        self.tibia = self.get_parameter("tibia_len").value
        self.topic = self.get_parameter("topic").value
        
        t1, t2, t3 = 0.0, -0.5, 1.4
        r = self.coxa + self.femur * np.cos(t2) + self.tibia * np.cos(t2 + t3)
        self.HOME_X, self.HOME_Y, self.HOME_Z = r, 0.0, self.femur * np.sin(t2) + self.tibia * np.sin(t2 + t3)
        
        self.total_gait_steps = 40 
        self.stance_wait = 0.08 

    def _initialize_leg_geometry(self):
        self.LEG_NAMES = ["leg1", "leg2", "leg3", "leg4", "leg5", "leg6"]
        self.SWING_A = ["leg1", "leg3", "leg5"]
        self.SWING_B = ["leg2", "leg4", "leg6"]
        self.current_swing_group = self.SWING_A
        
        offset = np.radians(30.0)
        self.LEG_ANGLES = {f"leg{i+1}": np.radians(i*60) + offset for i in range(6)}

    def calculate_ik(self, target_pos):
        x, y, z = target_pos
        hip = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2) - self.coxa
        D = (r**2 + z**2 - self.femur**2 - self.tibia**2) / (2 * self.femur * self.tibia)
        D = np.clip(D, -1.0, 1.0)
        tibia = -np.arctan2(-np.sqrt(1 - D**2), D)
        femur = np.arctan2(z, r) - np.arctan2(self.tibia * np.sin(tibia), self.femur + self.tibia * np.cos(tibia))
        return [hip, femur, tibia]

    def publish_current(self):
        msg = Float64MultiArray()
        joints = []
        for name in self.LEG_NAMES:
            joints.extend(self.calculate_ik(self.current_leg_positions[name]))
        msg.data = joints
        self.pub.publish(msg)

    def execute_gait(self, move_func):
        """Wrapper that executes motion then grounds the robot"""

        for i in range(self.total_gait_steps):
            s = (1.0 - np.cos(np.pi * i / (self.total_gait_steps - 1))) / 2.0
            move_func(s)
            self.publish_current()
            time.sleep(0.01)

        for name in self.LEG_NAMES:
            self.current_leg_positions[name] = (self.HOME_X, self.HOME_Y, self.HOME_Z)
        self.publish_current()
        time.sleep(self.stance_wait) 


        self.current_swing_group = self.SWING_B if self.current_swing_group == self.SWING_A else self.SWING_A


    def walk_logic(self, s, direction):
        dist = self.step_len * direction
        curr_rad = (dist / 2) - (dist * s)
        for name in self.LEG_NAMES:
            angle = self.LEG_ANGLES[name]
            is_swing = name in self.current_swing_group
            radial = -curr_rad if is_swing else curr_rad
            z_off = self.lift_h * (1.0 - 4.0 * (s - 0.5)**2) if is_swing else 0.0
            self.current_leg_positions[name] = (self.HOME_X + radial * np.sin(angle), self.HOME_Y + radial * np.cos(angle), self.HOME_Z + z_off)

    def strafe_logic(self, s, direction):
        side = self.step_len if direction == "left" else -self.step_len
        curr_tang = (side / 2) - (side * s)
        for name in self.LEG_NAMES:
            angle = self.LEG_ANGLES[name]
            is_swing = name in self.current_swing_group
            tang = -curr_tang if is_swing else curr_tang
            z_off = self.lift_h * (1.0 - 4.0 * (s - 0.5)**2) if is_swing else 0.0
            self.current_leg_positions[name] = (self.HOME_X - tang * np.cos(angle), self.HOME_Y + tang * np.sin(angle), self.HOME_Z + z_off)

    def turn_logic(self, s, direction):
        sweep = self.step_len if direction == "right" else -self.step_len
        curr_y = (sweep / 2) - (sweep * s)
        for name in self.LEG_NAMES:
            is_swing = name in self.current_swing_group
            y_pos = -curr_y if is_swing else curr_y
            z_off = self.lift_h * (1.0 - 4.0 * (s - 0.5)**2) if is_swing else 0.0
            self.current_leg_positions[name] = (self.HOME_X, self.HOME_Y + y_pos, self.HOME_Z + z_off)

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) > 0.05:
            self.execute_gait(lambda s: self.walk_logic(s, 1 if msg.linear.x > 0 else -1))
        elif abs(msg.linear.y) > 0.05:
            self.execute_gait(lambda s: self.strafe_logic(s, "left" if msg.linear.y > 0 else "right"))
        elif abs(msg.angular.z) > 0.05:
            self.execute_gait(lambda s: self.turn_logic(s, "left" if msg.angular.z > 0 else "right"))
        else:
            self.publish_current()

def main():
    rclpy.init()
    node = HexapodStableMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()