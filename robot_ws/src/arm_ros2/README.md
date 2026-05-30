# arm_ros2

ROS2 package for arm robot description and visualization.

## Description

This package contains the URDF description, 3D meshes, and launch files for a robotic arm. It has been adapted from ROS1 to ROS2.

## Package Structure

- `urdf/`: Contains the URDF robot description file
- `meshes/`: Contains 3D STL mesh files for visualization and collision
- `launch/`: Contains ROS2 launch files for different use cases
- `config/`: Contains configuration files for RViz2

## Dependencies

- ROS2 Humble (or later)
- joint_state_publisher_gui
- robot_state_publisher
- rviz2
- gazebo_ros_pkgs
- urdf
- xacro

## Usage

### Build the package

```bash
cd ~/RDK_X5/robot_ws
colcon build --packages-select arm_ros2
source install/setup.bash
```

### Launch RViz2 with robot model

```bash
ros2 launch arm_ros2 display.launch.py
```

This will start:
- Robot State Publisher
- Joint State Publisher GUI (for manually controlling joint positions)
- RViz2 with the robot model

### Launch Gazebo with robot model

```bash
ros2 launch arm_ros2 gazebo.launch.py
```

This will start:
- Gazebo simulation with an empty world
- Robot model spawned in Gazebo
- Static transform publisher for base_link to base_footprint

## Robot Description

The robot consists of:
- 1 base link
- 5 arm links (arm_Link1 to arm_Link5)
- 1 gripper link

With 6 revolute joints connecting them.

## License

BSD
