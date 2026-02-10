#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import numpy as np

class ForcePlotter(Node):
    def __init__(self):
        super().__init__('force_plotter')
        
        self.history_len = 200
        self.leg_data = [deque([0.0]*self.history_len, maxlen=self.history_len) for _ in range(6)]
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/hexapod/leg_forces',
            self.listener_callback,
            10
        )
        self.get_logger().info("Plotter connected. Range set to 0N - 5N")

    def listener_callback(self, msg):
        if len(msg.data) >= 6:
            for i in range(6):
                self.leg_data[i].append(msg.data[i])

# --- SETUP ---
rclpy.init()
node = ForcePlotter()
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()

fig, ax = plt.subplots(figsize=(10, 6))
fig.canvas.manager.set_window_title('Hexapod Gait Analyzer')

# Colors: Leg 1 & 2 are bright Red/Blue. Others are faded for clarity.
colors = ['#ff0000', '#0000ff', '#ff880040', '#0088ff40', '#88000040', '#00008840']
labels = ['Leg 1 (0)', 'Leg 2 (1)', 'Leg 3', 'Leg 4', 'Leg 5', 'Leg 6']
lines = []

for i in range(6):
    # Make Leg 1 and 2 thicker (linewidth=2.5) to "check 0 and 1"
    lw = 2.5 if i < 2 else 1.0
    ln, = ax.plot([], [], lw=lw, label=labels[i], color=colors[i])
    lines.append(ln)

# --- CHANGED: Correct Scale for Calibrated Sensors ---
ax.set_ylim(-0.5, 20.0)  # Range: -0.5N to 5.0N (Covers your 3.6N steps)
ax.set_xlim(0, node.history_len)
ax.set_title('Real-time Ground Reaction Forces')
ax.set_ylabel('Force (Newtons)')
ax.set_xlabel('Time (Frames)')
ax.grid(True, linestyle='--', alpha=0.3)
ax.legend(loc='upper right', ncol=6, fontsize='small')

# --- CHANGED: Correct Threshold for "Air" ---
# Anything below 1.0N is considered "Air" (Index 0/1 checking)
ax.axhline(y=1.0, color='green', linestyle='--', label='Contact Threshold (1.0N)')

def update(frame):
    x = np.arange(node.history_len)
    for i, line in enumerate(lines):
        line.set_data(x, list(node.leg_data[i]))
    return lines

ani = FuncAnimation(fig, update, interval=50, blit=True)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()