import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt
from collections import deque

class JointVisualizer(Node):
    def __init__(self):
        super().__init__('joint_visualizer')

        # 数据缓存：三个关节
        self.actual_data = [deque(maxlen=300) for _ in range(3)]
        self.target_data = [deque(maxlen=300) for _ in range(3)]
        self.time = deque(maxlen=300)
        self.counter = 0

        # 当前目标值（初始为 0）
        self.current_target = [0.0, 0.0, 0.0]

        # 订阅 joint_states 和目标关节位置
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Float64MultiArray, '/target_joint_positions', self.target_callback, 10)

        # 实时绘图设置
        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        self.fig.tight_layout(pad=3.0)

    def joint_callback(self, msg):
        if len(msg.position) >= 3:
            self.counter += 1
            self.time.append(self.counter)

            # 注意映射关系：joint1 -> position[1], joint2 -> position[0], joint3 -> position[2]
            mapping = [1, 0, 2]
            for i in range(3):
                self.actual_data[i].append(msg.position[mapping[i]])
                self.target_data[i].append(self.current_target[i])

            self.update_plot()

    def target_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_target = list(msg.data[:3])  # 保留前三个关节值

    def update_plot(self):
        joint_names = ['Joint 0 (Base Yaw)', 'Joint 1 (Pitch)', 'Joint 2 (Pitch)']
        for i in range(3):
            ax = self.axs[i]
            ax.clear()
            ax.plot(self.time, self.actual_data[i], label='Actual', color='blue')
            ax.plot(self.time, self.target_data[i], label='Target', color='red', linestyle='--')
            ax.set_ylabel("Position (rad)")
            ax.set_title(joint_names[i])
            ax.legend()
            ax.grid(True)

        self.axs[2].set_xlabel("Time Step")
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
