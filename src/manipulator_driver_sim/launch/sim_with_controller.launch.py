from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    pkg = "manipulator_driver_sim"
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = LaunchConfiguration("world_file")

    # 1. Robot Description（xacro -> robot_description）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(pkg), "urdf", "manipulator_main.urdf.xacro"]),
        " ",
        "simulation_controllers:=",
        PathJoinSubstitution(
        [FindPackageShare(pkg), "config", "ros2_vel_ctrl.yaml"]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=None)
    }

    # 2. Robot State Publisher（发布 robot_description 参数）
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # 3. RViz（可选）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare(pkg), "rviz", "view_robot.rviz"])],
        condition=IfCondition(use_rviz),
        output="log"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(use_rviz),
    )
    # 4. 启动 Ignition Gazebo + 加载 world
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file]
        }.items()
    )

    # 5. 在 Ignition Gazebo 中通过话题 `/robot_description` 创建实体
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_description_content,
            # "-topic", "/robot_description",  # 注意加 /
            "-name", "my_arm"
        ],
        output="screen"
    )

    # 6. 时间同步桥
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen"
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )
    from launch.actions import TimerAction

    delay_velCtrl_after_joint_srobot_controller_spawner = TimerAction(
        period=6.0,
        actions=[initial_joint_controller_spawner_started]
    )

    my_controller = Node(
        package="manipulator_driver_sim",
        executable="my_controller",
        output="screen"
    )
    delay_pidCtrl_after_joint_vel_spawner = TimerAction(
        period=8.0,
        actions=[my_controller]
    )
    move2pose=Node(
        package="manipulator_driver_sim",
        executable="move2pose",
        output="screen"
    )
    return [
            rsp_node,       # Robot State Publisher（pub robot_description param）
            spawn_node,     # in Ignition Gazebo via topic `/robot_description` create entity
            gz_launch,      # start 'Ignition Gazebo' + login world
            gz_sim_bridge,  # time sync bridge
            joint_state_broadcaster_spawner, # create 'joint_state_broadcaster' publisher, is a controller manager, pub vel,pos,effort
            delay_rviz_after_joint_state_broadcaster_spawner, # load Rviz after joint state broadcast
            delay_velCtrl_after_joint_srobot_controller_spawner,
            delay_pidCtrl_after_joint_vel_spawner,
            move2pose,
            # initial_joint_controller_spawner_started, # load true controller
            # jsp_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world_file", default_value="empty.sdf"),
        OpaqueFunction(function=launch_setup)
    ])
