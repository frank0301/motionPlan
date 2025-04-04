

```
ros2 service call /move_to_pose manipulator_interfaces/srv/MoveToPose "target_pose: {position: {x: -0.11, y: 0.56, z: 0.52}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

```
ros2 service call /move_to_pose manipulator_interfaces/srv/MoveToPose "target_pose: {position: {x: 0.014, y: 0.034, z: 1.113}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "header:
stamp:
sec: 0
nanosec: 0
frame_id: ''
joint_names:'base_link_to_link1''link1_to_link2''link2_to_link3'
points:positions: [0.0, 0.0, -0.0]
time_from_start:
sec: 2
nanosec: 0"
```
