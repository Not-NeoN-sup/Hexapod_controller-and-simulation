#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
import sys

class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor')

        self.leg_names = ['leg_1', 'leg_2', 'leg_3', 'leg_4', 'leg_5', 'leg_6']
        self.spring_stiffness = 1000.0 
        self.calibration_offset = 0.0

        self.force_pub = self.create_publisher(Float64MultiArray, '/hexapod/leg_forces', 10)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f'Monitor Active. Stiffness={self.spring_stiffness}. Offset={self.calibration_offset}N')

    def joint_state_callback(self, msg):
        current_forces = []
        display_parts = []
        joint_map = dict(zip(msg.name, msg.position))

        for leg in self.leg_names:
            target = f"{leg}_foot_joint"
            leg_num = leg.split('_')[-1]
            
            if target in joint_map:
                raw_force = abs(joint_map[target]) * self.spring_stiffness
                calibrated_force = raw_force - self.calibration_offset
                if calibrated_force < 1.0: 
                    calibrated_force = 0.0
                if calibrated_force < 2.0:
                    status = 0 
                else:
                    status = 1

                current_forces.append(calibrated_force)
                display_parts.append(f"{leg_num}:{status}")
            else:
                current_forces.append(0.0)
                display_parts.append(f"{leg_num}: ERR ")


        out = Float64MultiArray()
        out.data = current_forces
        self.force_pub.publish(out)
        

        sys.stdout.write(f"\r{'  '.join(display_parts)}")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()