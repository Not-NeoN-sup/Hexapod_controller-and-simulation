#!/usr/bin/env python3
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import rclpy
import math, time
import numpy as np


class HexapodMover(Node):
    def __init__(self):
        super().__init__('hexapod_mover')
        # Parameters
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("topic", "/hexapod_controller/commands")
        self.declare_parameter("gait_frequency", 1.5)
        self.declare_parameter("step_length", 0.18)
        self.declare_parameter("lift_height", 0.05)
        self.declare_parameter("body_height", 0.12)
        self.declare_parameter("coxa_len", 0.05)
        self.declare_parameter("femur_len", 0.12)
        self.declare_parameter("tibia_len", 0.18)
        self.declare_parameter("y_offset_side", 0.10)
        self.declare_parameter("STAND_ANGLES", (0.0, -0.5, 1.4))
        self.declare_parameter("FORWARD_OFFSET", 30)

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
        self.STAND_ANGLES = self.get_parameter("STAND_ANGLES").value    
        self.HOME_X, self.HOME_Y, self.HOME_Z = self.calculate_fk(self.STAND_ANGLES)
        self.FORWARD_OFFSET = np.radians(self.get_parameter("FORWARD_OFFSET").value)
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

        self.topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        from geometry_msgs.msg import Twist
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel = Twist()

        self.walking = False    
        self.timer = None        
        self.gait_step = 0       
        self.total_gait_steps = 100  
        
        # Smoothing parameters
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.smooth_linear_x = 0.0
        self.smooth_angular_z = 0.0
        self.acceleration = 0.05  

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

    # --- Inverse Kinematics ---
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

    # --- Forward Kinematics ---
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

    def parabola_trajectory(self, y_start, y_end, y_current, z_max, z_home):
        """Smooth parabolic trajectory for leg lifting."""
        if abs(y_end - y_start) < 1e-6:
            return z_home 

        progress = (y_current - y_start) / (y_end - y_start)
        progress = np.clip(progress, 0.0, 1.0)
        
        z_lift = z_max - z_home
        if progress < 0.5:
            t = progress * 2  
            z_current = z_home + z_lift * (2 * t**2)
        else:
            t = (progress - 0.5) * 2 
            z_current = z_max - z_lift * (2 * t**2)
        
        return z_current

    def stand(self):
        msg = Float64MultiArray()
        joints = []
        for _ in range(6):
            joints.extend(self.STAND_ANGLES)
        msg.data = joints
        self.pub.publish(msg)
        self.get_logger().info("Published stand pose.")

    def step(self):
        """Take one walking step when commanded."""
        if not self.walking:
            self.walking = True
            self.gait_step = 0
            self.get_logger().info("Single walking step started.")
            self.gait_step_callback()
    def gait_step_callback(self):
        """Perform one full step gait (A → B tripod) — completes full cycle forward or backward."""
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

                # Tripod movement logic
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

                # Inverse Kinematics
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

        self.stand()  
        self.walking = False
        self.get_logger().info("✅ Full step cycle complete.")

        self.current_swing_group = (
            self.SWING_GROUP_B
            if self.current_swing_group == self.SWING_GROUP_A
            else self.SWING_GROUP_A
        )

        self.walking = False
        self.stand()
        self.get_logger().info("Single step complete.")
        self.current_swing_group = (
            self.SWING_GROUP_B
            if self.current_swing_group == self.SWING_GROUP_A
            else self.SWING_GROUP_A
        )
        self.walking = False
        self.stand()
        self.get_logger().info("Single step gait complete.")

    
    def strafe(self, direction="right", total_steps=100, speed_multiplier=1.0):
        Z_MAX_LIFT = self.HOME_Z - self.lift_h
        STANCE_GROUP = (
            self.SWING_GROUP_B if self.current_swing_group == self.SWING_GROUP_A else self.SWING_GROUP_A
        )

        if direction == "right":
            tangential_movement = np.linspace(self.step_len / 2, -self.step_len / 2, total_steps)
        else:
            tangential_movement = np.linspace(-self.step_len / 2, self.step_len / 2, total_steps)

        for i in range(total_steps):
            msg = Float64MultiArray()
            joints = []

            for leg_name, leg_joints in self.LEG_NAMES.items():
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

                # Lift swing legs using a parabola
                if leg_name in self.current_swing_group:
                    swing_progress = -tangential_movement[i]
                    if direction == "right":
                        Z_TARGET = self.parabola_trajectory(
                            -self.step_len / 2, self.step_len / 2, swing_progress, Z_MAX_LIFT, self.HOME_Z
                        )
                    else:
                        Z_TARGET = self.parabola_trajectory(
                            self.step_len / 2, -self.step_len / 2, swing_progress, Z_MAX_LIFT, self.HOME_Z
                        )
                else:
                    Z_TARGET = self.HOME_Z

                hip, femur, tibia = self.calculate_ik((X_TARGET, Y_TARGET, Z_TARGET))
                joints.extend([hip, femur, tibia])

            msg.data = joints
            self.pub.publish(msg)
            time.sleep(self.dt / speed_multiplier)
        self.current_swing_group = (
            self.SWING_GROUP_B if self.current_swing_group == self.SWING_GROUP_A else self.SWING_GROUP_A
        )
        self.get_logger().info(f"Strafe {direction} cycle complete.")

    def turn(self, direction="left", total_steps=100):
        Z_MAX_LIFT = self.HOME_Z - self.lift_h
        STANCE_GROUP = (
            self.SWING_GROUP_B if self.current_swing_group == self.SWING_GROUP_A else self.SWING_GROUP_A
        )

        Y_SWEEP_START = self.HOME_Y + (self.step_len / 2)
        Y_SWEEP_END = self.HOME_Y - (self.step_len / 2)
        
        if direction == "left":
            Y_trajectory_stance = np.linspace(Y_SWEEP_START, Y_SWEEP_END, total_steps)
            Y_trajectory_swing = np.linspace(Y_SWEEP_END, Y_SWEEP_START, total_steps)
        else:  # right
            Y_trajectory_stance = np.linspace(Y_SWEEP_END, Y_SWEEP_START, total_steps)
            Y_trajectory_swing = np.linspace(Y_SWEEP_START, Y_SWEEP_END, total_steps)

        for i in range(total_steps):
            msg = Float64MultiArray()
            joints = []

            for leg_name, leg_joints in self.LEG_NAMES.items():
                X_TARGET = self.HOME_X
                
                if leg_name in self.current_swing_group:
                    Y_TARGET = Y_trajectory_swing[i]
                    # Lift swing legs
                    if direction == "left":
                        Z_TARGET = self.parabola_trajectory(
                            Y_SWEEP_END, Y_SWEEP_START, Y_TARGET, Z_MAX_LIFT, self.HOME_Z
                        )
                    else:
                        Z_TARGET = self.parabola_trajectory(
                            Y_SWEEP_START, Y_SWEEP_END, Y_TARGET, Z_MAX_LIFT, self.HOME_Z
                        )
                else:
                    Y_TARGET = Y_trajectory_stance[i]
                    Z_TARGET = self.HOME_Z

                hip, femur, tibia = self.calculate_ik((X_TARGET, Y_TARGET, Z_TARGET))
                joints.extend([hip, femur, tibia])

            msg.data = joints
            self.pub.publish(msg)
            time.sleep(self.dt)

        # Swap tripod groups
        self.current_swing_group = (
            self.SWING_GROUP_B if self.current_swing_group == self.SWING_GROUP_A else self.SWING_GROUP_A
        )
        self.get_logger().info(f"Turn {direction} cycle complete.")

    def cmd_vel_callback(self, msg):
        """Handles teleop input for walking, strafing, and turning."""
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z

        # --- Handle forward/backward walking ---
        if abs(msg.linear.x) > 0.05:
            if not self.walking:
                direction = "forward" if msg.linear.x > 0 else "backward"
                self.get_logger().info(f"Walk command received! Moving {direction}")
                self.smooth_linear_x = msg.linear.x * 0.3  # Start at 30% speed
                self.step()

        # --- Handle turning (angular.z) ---
        if abs(msg.angular.z) > 0.05:
            direction = "left" if msg.angular.z > 0 else "right"
            self.get_logger().info(f"Turn command received! Turning {direction}")
            self.turn(direction)

        # --- Handle strafing (linear.y for left/right) ---
        if abs(msg.linear.y) > 0.05:
            direction = "left" if msg.linear.y > 0 else "right"
            self.get_logger().info(f"Strafe command received! Strafing {direction}")
            self.strafe(direction)
            
        self.stand()


def main(args=None):
    rclpy.init(args=args)
    node = HexapodMover()

    # Wait for the Gazebo ROS2 controller to subscribe
    node.wait_for_subscribers(timeout=10.0)

    # Stand first
    node.stand()
    node.get_logger().info("Hexapod in standing position...")
    time.sleep(2.0)

    node.get_logger().info("Ready for teleop control (use i/k/j/l keys).")

    try:
        rclpy.spin(node)  # <-- this lets callbacks like cmd_vel_callback() run
    except KeyboardInterrupt:
        node.get_logger().info("Teleop interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()