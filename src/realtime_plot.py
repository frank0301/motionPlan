import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt
from collections import deque

class JointVisualizer(Node):
    def __init__(self):
        super().__init__('joint_visualizer')

        # 缓存最近 300 个数据点
        self.actual_data = deque(maxlen=300)
        self.target_data = deque(maxlen=300)
        self.time = deque(maxlen=300)
        self.counter = 0

        # 订阅 joint_states 和目标关节值
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Float64MultiArray, '/target_joint_positions', self.target_callback, 10)

        # 初始值
        self.current_target = 0.0

        # 启动实时绘图
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def joint_callback(self, msg):
        if len(msg.position) > 0:
            actual_pos = msg.position[1]  # 改这里可以换成 [1] 或 [2]
            self.counter += 1
            self.time.append(self.counter)
            self.actual_data.append(actual_pos)
            self.target_data.append(self.current_target)
            self.update_plot()

    def target_callback(self, msg):
        if len(msg.data) > 0:
            self.current_target = msg.data[0]

    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.time, self.actual_data, label='Actual Position', color='blue')
        self.ax.plot(self.time, self.target_data, label='Target Position', color='red', linestyle='--')
        self.ax.set_ylabel("Joint Position (rad)")
        self.ax.set_xlabel("Time Step")
        self.ax.set_title("Joint Tracking (Joint 0)")
        self.ax.legend()
        self.ax.grid(True)
        plt.pause(0.01)

def main():
    rclpy.init()
    node = JointVisualizer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
