import math
import tf_transformations


def inverse_kinematics_with_orientation(x, y, z, orientation, l1=0.4, l2=0.4, l3=0.3):
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
    print("theta3:", theta3_raw)
    # 角度范围检测
    warnings = []
    if not -math.pi <= theta1 <= math.pi:
        warnings.append(f"⚠️ theta1 = {theta1:.2f} 超出范围 [-π, π]")
    if not -math.pi / 2 <= theta2 <= math.pi / 2:
        warnings.append(f"⚠️ theta2 = {theta2:.2f} 超出范围 [-π/2, π/2]")
    if not -math.pi / 2 <= theta3 <= math.pi / 2:
        warnings.append(f"⚠️ theta3 = {theta3:.2f} 超出范围 [-π/2, π/2]")

    if warnings:
        for w in warnings:
            print(w)

    return theta1, theta2, theta3


# 示例测试
if __name__ == "__main__":
    # 示例四元数：[x, y, z, w]，表示无旋转
    orientation = [0, 0, 0, 1]
# x: -0.11, y: 0.56, z: 0.52
    # 示例末端目标点坐标
    # x, y, z = -0.11, 0.56, 0.52
    # x,y,z=0.014,0.034,1.113
    x,y,z=0.1266,0.3102,0.568
    theta1, theta2, theta3 = inverse_kinematics_with_orientation(x, y, z, orientation)
    print(f"theta1: {theta1:.3f}, theta2: {theta2:.3f}, theta3: {theta3:.3f}")
