import rclpy
from rclpy.node import Node
from manipulator_interfaces.srv import MoveToPose
from std_msgs.msg import Float64MultiArray

import math
import tf_transformations  # pip install transforms3d OR sudo apt install ros-humble-tf-transformations

class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')

        self.joint_names = [
            "base_link_to_link1",
            "link1_to_link2",
            "link2_to_link3"
        ]
        # 初始化目标角度，避免空值报错
        self.current_target_angles = [0.0, 0.0, 0.0]
        # 发布目标角度 topic
        self.target_pub = self.create_publisher(Float64MultiArray, "/target_joint_positions", 10)

        # 创建服务
        self.srv = self.create_service(MoveToPose, '/move_to_pose', self.move_to_pose_cb)
        self.create_timer(0.01, self.target_pub_loop)
    def move_to_pose_cb(self, request, response):
        pose = request.target_pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation

        try:
            theta1, theta2, theta3 = self.inverse_kinematics_with_orientation(x, y, z, q)

            # 保存当前目标角度供循环发布器使用
            self.current_target_angles = [theta1, theta2, theta3]

            msg = Float64MultiArray()
            msg.data = self.current_target_angles
            self.target_pub.publish(msg)

            response.success = True
            response.message = f"Target angles: {theta1:.2f}, {theta2:.2f}, {theta3:.2f}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"IK failed: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def inverse_kinematics_with_orientation(self, x, y, z, orientation, l1=0.4, l2=0.4, l3=0.3):
        z = z - 0.1  # 末端执行器长度或偏移校正

        # 四元数 → 欧拉角
        try:
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
        except AttributeError:
            q = orientation  # 假设传入的是 list 或 tuple

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)

        # θ1：基座 yaw
        theta1 = math.atan2(y, x)

        # θ2、θ3 计算
        n = math.sqrt(x**2 + y**2)
        l1b = abs(l1 - z)
        m = math.sqrt(n**2 + l1b**2)

        try:
            theta2_a = math.acos((l2**2 + m**2 - l3**2) / (2 * m * l2))
        except ValueError:
            raise ValueError("theta2 acos 输入超出 [-1,1] 范围，位置不可达")

        if not n == 0:
            if l1 > z:
                theta2_b = math.atan(n / l1b)
                theta2 = theta2_a + theta2_b
                theta2 = math.pi - (theta2_a + theta2_b)
            elif l1 < z:
                theta2_b = math.atan(l1b / n)
                theta2 = math.pi / 2 - (theta2_a + theta2_b)
            else:
                theta2_b = 0
        else:
            theta2=0

        try:
            theta3_raw = math.acos((l2**2 + l3**2 - m**2) / (2 * l2 * l3))
        except ValueError:
            raise ValueError("theta3 acos 输入超出 [-1,1] 范围，位置不可达")

        # # ✅ 修改 theta3 表示：以 theta2 连杆为参考，从其方向水平为 pi/2，垂直为 0
        theta3 = math.pi - theta3_raw

        # 角度范围检测
        if not -math.pi <= theta1 <= math.pi:
            raise ValueError(f"theta1 = {theta1:.2f} 超出范围 [-π, π]")
        if not -math.pi / 2 <= theta2 <= math.pi / 2:
            raise ValueError(f"theta2 = {theta2:.2f} 超出范围 [-π/2, π/2]")
        if not -math.pi / 2 <= theta3 <= math.pi / 2:
            raise ValueError(f"theta3 = {theta3:.2f} 超出范围 [-π/2, π/2]")

        return theta1, theta2, theta3



    def target_pub_loop(self):
        msg = Float64MultiArray()
        msg.data = self.current_target_angles
        self.target_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
