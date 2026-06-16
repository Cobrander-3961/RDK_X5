"""
底盘 + 机械臂 一体化: 导航 + 机械臂控制 + RViz 双模型
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    chassis_pkg = get_package_share_directory('alloy_chassis_mecanum')
    arm_pkg = get_package_share_directory('arm_ros2')
    elite_pkg = get_package_share_directory('elite_test')
    nav2_params = os.path.join(chassis_pkg, 'config', 'nav2_params.yaml')

    # ── 合并 URDF (预生成好的) ──
    combined_urdf_path = os.path.join(chassis_pkg, 'urdf', 'robot_combined.urdf')
    with open(combined_urdf_path) as f:
        combined_urdf = f.read()

    # ── 1. 统一 RSP (底盘+机械臂) ──
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': combined_urdf, 'publish_frequency': 30.0}],
    )

    # ── 1.5 发布 /robot_description topic + joint_state_publisher ──
    robot_desc_pub = Node(
        package='alloy_chassis_mecanum', executable='robot_desc_pub.py',
        name='robot_desc_pub', output='screen',
        parameters=[{'robot_description': combined_urdf}],
    )
    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        parameters=[{'robot_description': combined_urdf, 'use_sim_time': False}],
    )

    # ── 2. 串口里程计 ──
    serial_odom = Node(
        package='alloy_chassis_mecanum', executable='serial_odom_receiver',
        name='serial_odom_receiver', output='screen',
        parameters=[{'serial_port': '/dev/ttyACM0', 'baudrate': 115200}],
    )

    # ── 2.5. EKF 融合 (里程计 + IMU) ──
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[os.path.join(chassis_pkg, 'config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
    )
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ── 3. 激光雷达 + TF ──
    ydlidar = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node', output='screen', emulate_tty=True,
        parameters=[os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'), 'params', 'X2.yaml')],
    )
    laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    # ── 4. SLAM ──
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(chassis_pkg, '..', 'ydlidar_ros2_driver',
                    'config', 'mapper_params_online_async.yaml'),
                    {'map_update_interval': 5.0, 'throttle_scans': 2}],
        remappings=[('/scan', '/scan'), ('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # ── 5. Navigation2 ──
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
        launch_arguments={'params_file': nav2_params, 'autostart': 'True'}.items(),
    )

    # ── 6. 机械臂 MoveGroup (JSP + RSP 提供 TF, 不需 ros2_control) ──
    arm_srdf = open(os.path.join(elite_pkg, 'config', 'arm_combined.srdf')).read()

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        name='move_group', output='screen',
        parameters=[{'robot_description': combined_urdf,
                     'robot_description_semantic': arm_srdf,
                     'robot_description_kinematics': os.path.join(
                         elite_pkg, 'config', 'kinematics.yaml')}],
    )

    # ── 7. 机械臂串口桥接 ──
    arm_bridge = Node(
        package='elite_test', executable='arm_bridge.py',
        name='arm_bridge', output='screen',
    )

    # ── 8. RViz ──
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(chassis_pkg, 'config',
                   'navigation_with_robot.rviz')],
    )

    return LaunchDescription([
        rsp, jsp, serial_odom, ekf_node, imu_tf, ydlidar, laser_tf, slam, nav2,
        move_group,
        TimerAction(period=4.0, actions=[arm_bridge]),
        rviz,
    ])
