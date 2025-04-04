import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = 0.02
        self.limit = 2.0
        self.integral = 0.0
        self.prev_error = 0.0

    def pid_calc(self, current):
        error = self.setpoint - current
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt 
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.limit), -self.limit)
        return output


class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')

        self.joint_names = ["base_link_to_link1", "link1_to_link2", "link2_to_link3"]

        # 初始化 PID 控制器，每个关节一个
        self.pids = {
            name: PID(3.0, 0.01, 0.02)
            for name in self.joint_names
        }

        self.joint_positions = {}

        # 订阅目标角度（由 move_to_pose_server 发布）
        self.create_subscription(Float64MultiArray, "/target_joint_positions", self.target_cb, 10)

        # 订阅当前关节状态
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        # 发布速度控制指令
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)

        # 定时控制循环（10Hz）
        self.create_timer(0.1, self.control_loop)

    def target_cb(self, msg: Float64MultiArray):
        for i, joint_name in enumerate(self.joint_names):
            if i < len(msg.data):
                self.pids[joint_name].setpoint = msg.data[i]
                self.get_logger().info(f"Set target for {joint_name}: {msg.data[i]:.2f}")

    def joint_state_cb(self, msg: JointState):
        self.joint_positions = dict(zip(msg.name, msg.position))

    def control_loop(self):
        velocities = []
        for name in self.joint_names:
            pos = self.joint_positions.get(name, None)
            if pos is not None:
                vel = self.pids[name].pid_calc(pos)
                velocities.append(vel)
                self.get_logger().info(f"{name}: pos={pos:.2f}, vel_cmd={vel:.2f}")
            else:
                velocities.append(0.0)

        msg = Float64MultiArray()
        msg.data = velocities
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
