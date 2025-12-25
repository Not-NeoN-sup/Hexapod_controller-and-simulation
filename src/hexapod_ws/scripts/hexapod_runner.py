#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import rclpy
import math
import time
import numpy as np
from enum import Enum


class GaitState(Enum):
    IDLE = 0
    WALKING = 1


class HexapodMover(Node):
    def __init__(self):
        super().__init__('hexapod_mover')
        
        self._declare_parameters()
        self._load_parameters()
        self._initialize_leg_geometry()
        
        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.walking = False
        self.gait_step = 0
        
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        self.current_leg_positions = {}
        for leg_name in self.LEG_NAMES.keys():
            self.current_leg_positions[leg_name] = (self.HOME_X, self.HOME_Y, self.HOME_Z)
        
        self.get_logger().info("Hexapod controller initialized")

    def _declare_parameters(self):
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("topic", "/hexapod_controller/commands")
        self.declare_parameter("gait_frequency", 1.5)
        self.declare_parameter("step_length", 0.18)
        self.declare_parameter("lift_height", 0.05)
        self.declare_parameter("body_height", 0.12)
        self.declare_parameter("coxa_len", 0.05)
        self.declare_parameter("femur_len", 0.12)
        self.declare_parameter("tibia_len", 0.09)
        self.declare_parameter("y_offset_side", 0.10)
        self.declare_parameter("STAND_ANGLES", [0.0, -0.5, 1.4])
        self.declare_parameter("FORWARD_OFFSET", 30.0)

    def _load_parameters(self):
        self.rate_hz = self.get_parameter("rate_hz").value
        self.dt = 1.0 / self.rate_hz
        self.freq = self.get_parameter("gait_frequency").value
        self.step_len = self.get_parameter("step_length").value
        self.lift_h = self.get_parameter("lift_height").value
        self.body_h = self.get_parameter("body_height").value
        self.coxa = self.get_parameter("coxa_len").value
        self.femur = self.get_parameter("femur_len").value
        self.tibia = self.get_parameter("tibia_len").value
        self.y_side = self.get_parameter("y_offset_side").value
        self.STAND_ANGLES = tuple(self.get_parameter("STAND_ANGLES").value)
        self.FORWARD_OFFSET = np.radians(self.get_parameter("FORWARD_OFFSET").value)
        self.topic = self.get_parameter("topic").value
        
        self.HOME_X, self.HOME_Y, self.HOME_Z = self.calculate_fk(self.STAND_ANGLES)
        self.total_gait_steps = 100
        
        self.get_logger().info(f"Home position: X={self.HOME_X:.3f}, Y={self.HOME_Y:.3f}, Z={self.HOME_Z:.3f}")

    def _initialize_leg_geometry(self):
        self.LEG_NAMES = {
            "leg1": (1, 2, 3),
            "leg2": (4, 5, 6),
            "leg3": (7, 8, 9),
            "leg4": (10, 11, 12),
            "leg5": (13, 14, 15),
            "leg6": (16, 17, 18)
        }

        self.SWING_GROUP_A = ["leg1", "leg3", "leg5"]
        self.SWING_GROUP_B = ["leg2", "leg4", "leg6"]
        self.current_swing_group = self.SWING_GROUP_A

        self.LEG_ANGLES = {
            "leg1": 0 + self.FORWARD_OFFSET,
            "leg2": np.radians(60) + self.FORWARD_OFFSET,
            "leg3": np.radians(120) + self.FORWARD_OFFSET,
            "leg4": np.radians(180) + self.FORWARD_OFFSET,
            "leg5": np.radians(240) + self.FORWARD_OFFSET,
            "leg6": np.radians(300) + self.FORWARD_OFFSET
        }

    def wait_for_subscribers(self, timeout=10.0):
        start = time.time()
        while self.count_subscribers(self.topic) == 0:
            if time.time() - start > timeout:
                self.get_logger().warn(f"Timeout waiting for subscribers on {self.topic}")
                return False
            self.get_logger().info(f"Waiting for subscribers on {self.topic} ...")
            time.sleep(0.5)
        self.get_logger().info(f"Subscriber detected on {self.topic}.")
        return True

    def calculate_ik(self, target_pos):
        x, y, z = target_pos
        
        hip_angle = np.arctan2(y, x)
        r_xy = np.sqrt(x**2 + y**2)
        r = r_xy - self.coxa

        D = (r**2 + z**2 - self.femur**2 - self.tibia**2) / (2 * self.femur * self.tibia)
        D = np.clip(D, -1.0, 1.0)

        tibia_angle = -np.arctan2(-np.sqrt(1 - D**2), D)
        alpha = np.arctan2(z, r)
        gamma = np.arctan2(self.tibia * np.sin(tibia_angle), self.femur + self.tibia * np.cos(tibia_angle))
        femur_angle = alpha - gamma

        return (hip_angle, femur_angle, tibia_angle)

    def calculate_fk(self, angles):
        t1, t2, t3 = angles
        
        r_femur = self.femur * np.cos(t2)
        z_femur = self.femur * np.sin(t2)
        r_tibia = self.tibia * np.cos(t2 + t3)
        z_tibia = self.tibia * np.sin(t2 + t3)
        r_total = self.coxa + r_femur + r_tibia
        
        x = r_total * np.cos(t1)
        y = r_total * np.sin(t1)
        z = z_femur + z_tibia
        return (x, y, z)

    def stand(self):
        msg = Float64MultiArray()
        joints = []
        for _ in range(6):
            joints.extend(self.STAND_ANGLES)
        msg.data = joints
        self.pub.publish(msg)
        self.get_logger().info("Published stand pose.")

    def gait_step_callback(self):
        if not self.walking:
            return

        direction = 1 if self.target_linear_x >= 0 else -1
        dynamic_step = self.step_len * direction
        Z_MAX_LIFT = self.HOME_Z + self.lift_h

        radial_movement = np.linspace(dynamic_step / 2, -dynamic_step / 2, self.total_gait_steps)
        
        for i in range(self.total_gait_steps):
            msg = Float64MultiArray()
            joint = []

            for leg_name in self.LEG_NAMES.keys():
                angle = self.LEG_ANGLES[leg_name]

                radial = (
                    -radial_movement[i]
                    if leg_name in self.current_swing_group
                    else radial_movement[i]
                )
                local_x = radial * np.sin(angle)
                local_y = radial * np.cos(angle)
                X_TARGET = self.HOME_X + local_x
                Y_TARGET = self.HOME_Y + local_y
                
                if leg_name in self.current_swing_group:
                    progress = i / (self.total_gait_steps - 1)
                    if progress < 0.5:
                        Z_TARGET = self.HOME_Z + self.lift_h * (2 * progress)
                    else:
                        Z_TARGET = self.HOME_Z + self.lift_h * (2 * (1 - progress))
                else:
                    Z_TARGET = self.HOME_Z

                self.current_leg_positions[leg_name] = (X_TARGET, Y_TARGET, Z_TARGET)

                hip, femur, tibia = self.calculate_ik((X_TARGET, Y_TARGET, Z_TARGET))
                joint.extend([hip, femur, tibia])

            msg.data = joint
            self.pub.publish(msg)
            time.sleep(self.dt)

        self.current_swing_group = (
            self.SWING_GROUP_B
            if self.current_swing_group == self.SWING_GROUP_A
            else self.SWING_GROUP_A
        )

        self.walking = False
        self.get_logger().info("✅ Full step cycle complete.")

    def strafe(self, direction="right", total_steps=100):
        Z_MAX_LIFT = self.HOME_Z + self.lift_h

        if direction == "right":
            tangential_movement = np.linspace(self.step_len / 2, -self.step_len / 2, total_steps)
        else:
            tangential_movement = np.linspace(-self.step_len / 2, self.step_len / 2, total_steps)

        for i in range(total_steps):
            msg = Float64MultiArray()
            joints = []

            for leg_name in self.LEG_NAMES.keys():
                angle = self.LEG_ANGLES[leg_name]
                tangential = (
                    -tangential_movement[i]
                    if leg_name in self.current_swing_group
                    else tangential_movement[i]
                )

                local_x = -tangential * np.cos(angle)
                local_y = tangential * np.sin(angle)

                X_TARGET = self.HOME_X + local_x
                Y_TARGET = self.HOME_Y + local_y

                if leg_name in self.current_swing_group:
                    swing_progress = -tangential_movement[i]
                    progress = (swing_progress - (-self.step_len/2 if direction=="right" else self.step_len/2)) / self.step_len
                    progress = np.clip(progress, 0.0, 1.0)
                    Z_TARGET = self.HOME_Z + self.lift_h * (1 - 4 * (progress - 0.5)**2)
                else:
                    Z_TARGET = self.HOME_Z

                self.current_leg_positions[leg_name] = (X_TARGET, Y_TARGET, Z_TARGET)

                hip, femur, tibia = self.calculate_ik((X_TARGET, Y_TARGET, Z_TARGET))
                joints.extend([hip, femur, tibia])

            msg.data = joints
            self.pub.publish(msg)
            time.sleep(self.dt)
            
        self.current_swing_group = (
            self.SWING_GROUP_B if self.current_swing_group == self.SWING_GROUP_A else self.SWING_GROUP_A
        )
        self.get_logger().info(f"Strafe {direction} cycle complete.")

    def turn(self, direction="left", total_steps=100):
        Z_MAX_LIFT = self.HOME_Z + self.lift_h

        Y_SWEEP_START = self.HOME_Y + (self.step_len / 2)
        Y_SWEEP_END = self.HOME_Y - (self.step_len / 2)
        
        if direction == "left":
            Y_trajectory_stance = np.linspace(Y_SWEEP_START, Y_SWEEP_END, total_steps)
            Y_trajectory_swing = np.linspace(Y_SWEEP_END, Y_SWEEP_START, total_steps)
        else:
            Y_trajectory_stance = np.linspace(Y_SWEEP_END, Y_SWEEP_START, total_steps)
            Y_trajectory_swing = np.linspace(Y_SWEEP_START, Y_SWEEP_END, total_steps)

        for i in range(total_steps):
            msg = Float64MultiArray()
            joints = []

            for leg_name in self.LEG_NAMES.keys():
                X_TARGET = self.HOME_X
                
                if leg_name in self.current_swing_group:
                    Y_TARGET = Y_trajectory_swing[i]
                    progress = i / (total_steps - 1)
                    Z_TARGET = self.HOME_Z + self.lift_h * (1 - 4 * (progress - 0.5)**2)
                else:
                    Y_TARGET = Y_trajectory_stance[i]
                    Z_TARGET = self.HOME_Z

                self.current_leg_positions[leg_name] = (X_TARGET, Y_TARGET, Z_TARGET)

                hip, femur, tibia = self.calculate_ik((X_TARGET, Y_TARGET, Z_TARGET))
                joints.extend([hip, femur, tibia])

            msg.data = joints
            self.pub.publish(msg)
            time.sleep(self.dt)

        self.current_swing_group = (
            self.SWING_GROUP_B if self.current_swing_group == self.SWING_GROUP_A else self.SWING_GROUP_A
        )
        self.get_logger().info(f"Turn {direction} cycle complete.")

    def hold_position(self):
        msg = Float64MultiArray()
        joints = []
        
        for leg_name in self.LEG_NAMES.keys():
            x, y, z = self.current_leg_positions[leg_name]
            hip, femur, tibia = self.calculate_ik((x, y, z))
            joints.extend([hip, femur, tibia])
        
        msg.data = joints
        self.pub.publish(msg)

    def cmd_vel_callback(self, msg):
        self.target_linear_x = msg.linear.x
        self.target_linear_y = msg.linear.y
        self.target_angular_z = msg.angular.z

        if abs(msg.linear.x) > 0.05:
            if not self.walking:
                direction = "forward" if msg.linear.x > 0 else "backward"
                self.get_logger().info(f"Walk command received! Moving {direction}")
                self.walking = True
                self.gait_step_callback()

        elif abs(msg.angular.z) > 0.05:
            direction = "left" if msg.angular.z > 0 else "right"
            self.get_logger().info(f"Turn command received! Turning {direction}")
            self.turn(direction)
            self.hold_position()

        elif abs(msg.linear.y) > 0.05:
            direction = "left" if msg.linear.y > 0 else "right"
            self.get_logger().info(f"Strafe command received! Strafing {direction}")
            self.strafe(direction)
            self.hold_position()
        
        else:
            self.hold_position()


def main(args=None):
    rclpy.init(args=args)
    node = HexapodMover()

    node.wait_for_subscribers(timeout=10.0)

    node.stand()
    node.get_logger().info("Hexapod in standing position...")
    time.sleep(2.0)

    node.get_logger().info("Ready for teleop control (use i/k/j/l keys).")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Teleop interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()