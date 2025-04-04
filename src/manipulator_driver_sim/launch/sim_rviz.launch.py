from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from xacro import process_file
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('manipulator_driver_sim'),
        'urdf',
        'manipulator_main.urdf.xacro'
    )
    robot_description_config = process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('manipulator_driver_sim'), 'rviz', 'view_robot.rviz')],
            output='screen'
        ),

        # 启动 ros_ign_bridge，仅桥接 /clock
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
            output='screen'
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                # "-string",
                # robot_description,
                '-topic', 'robot_description',
                '-entity', 'manipulator'
            ],
        ),
                # 4. joint_state_broadcaster
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        #     output='screen'
        # ),

        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        #     output='screen'
        # ),
        # 启动 Ignition Gazebo（空世界，模型由 URDF 直接注入）
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
            output='screen'
        ),
    ])
