```
git clone https://github.com/frank0301/motionPlan miniPro1
cd src/
colcon build
source install/setup.bash
```

1. start the rviz and gazebo first
   
   `````
   ros2 launch manipulator_driver_sim sim_with_controller.launch.py
   `````
2. test the PID controller:

```
ros2 service call /move_to_pose manipulator_interfaces/srv/MoveToPose "target_pose: {position: {x: -0.11, y: 0.56, z: 0.52}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

```
ros2 service call /move_to_pose manipulator_interfaces/srv/MoveToPose "target_pose: {position: {x: 0.014, y: 0.034, z: 1.113}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

3. to see the target-current:
   
   ```
   python realtime_plot_inAll.py
   ```
4. without PID controller:
   
   ```
   ros2 launch manipulator_driver_sim sim_gazebo.launch.py
   ```
5. use the rqt_plot to see the graph
   
   ```
   ros2 run rqt_plot rqt_plot
   ```
6. publish pose:

```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "
joint_names:
- 'base_link_to_link1'
- 'link1_to_link2'
- 'link2_to_link3'
points:
- positions: [-0.11, 0.56, 0.52]
  time_from_start:
    sec: 2
    nanosec: 0"
```

Folder structure

```
└─src
    │  realtime_plot.py
    │  realtime_plot_inAll.py
    │  realtime_plot_vel.py
    │
    ├─manipulator_driver_sim
    │  │  package.xml
    │  │  setup.cfg
    │  │  setup.py
    │  │
    │  ├─config
    │  │      ros2_controllers.yaml
    │  │      ros2_vel_ctrl.yaml
    │  │
    │  ├─launch
    │  │      sim_gazebo.launch.py
    │  │      sim_rviz.launch.py
    │  │      sim_with_controller.launch.py   # Launches Gazebo simulation, controllers, and nodes
    │  │
    │  ├─manipulator_driver_sim
    │  │      move2pose_server.py   # ROS 2 service node for pose commands and IK calculation
    │  │      my_controller.py      # PID controller for joint velocity control
    │  │      origin.py
    │  │      __init__.py
    │  │
    │  ├─resource
    │  │      manipulator_driver_sim
    │  │
    │  ├─rviz
    │  │      view_robot.rviz
    │  │
    │  ├─srv
    │  │      SetJointTarget.srv
    │  │
    │  ├─test
    │  │      test_copyright.py
    │  │      test_flake8.py
    │  │      test_pep257.py
    │  │
    │  └─urdf
    │          manipulator_gazebo.xacro
    │          manipulator_main.urdf.xacro
    │          manipulator_main_pos.urdf.xacro
    │          manipulator_model2.urdf.xacro
    │          manipulator_transmission.xacro
    │
    └─manipulator_interfaces
        │  CMakeLists.txt
        │  package.xml
        │
        ├─include
        │  └─manipulator_interfaces
        └─srv
                MoveToPose.srv
```

### `move2pose_server.py` – Pose Command & Inverse Kinematics

* Provides a ROS 2 service `/move_to_pose` accepting a target `Pose`
* Computes joint angles via inverse kinematics (IK)
* Publishes joint targets on `/target_joint_positions`

### `my_controller.py` – Custom PID Controller

* Each joint uses a separate PID instance
* Subscribes to `/joint_states` for current positions
* Receives `/target_joint_positions` as setpoints
* Publishes velocity commands to `/velocity_controller/commands`
* PID parameters (tunable in code): `PID(Kp=3.0, Ki=0.01, Kd=0.02)`

### `sim_with_controller.launch.py` – Full ROS 2 Simulation Launch

This launch file handles:

* Loading URDF into `/robot_description`
* Launching Ignition Gazebo with specified world
* Spawning joint state broadcaster and velocity controller
* Starting custom PID controller and IK service node
* Optional RViz visualization

Dependencies

* ROS 2 (Humble)
* Ignition Gazebo (`ros_gz_sim`)
* `rrr_robot_interfaces` (custom service definition)
* `tf_transformations` (for quaternion to Euler conversion)
