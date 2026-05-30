
# 机器人导航配置说明

本目录包含了一个用于机器人导航的launch文件，结合了SLAM Toolbox进行实时建图和Navigation2进行路径规划。

## 文件说明

- `launch/navigation_with_robot.launch.py`: 主launch文件，启动机器人模型、雷达驱动、SLAM Toolbox和Navigation2
- `config/nav2_params.yaml`: Navigation2的配置文件
- `config/navigation_with_robot.rviz`: RViz2配置文件，用于显示导航相关信息
- `scripts/fake_odom_publisher.py`: 模拟里程计节点，发布固定的里程计数据

## 使用方法

1. 编译工作空间：
```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

2. 启动导航系统：
```bash
ros2 launch alloy_chassis_mecanum navigation_with_robot.launch.py
```

3. 在RViz2中：
   - 使用"2D Pose Estimate"工具设置机器人的初始位置
   - 使用"2D Goal Pose"工具设置导航目标点
   - 查看实时建图、代价地图和规划路径

## 注意事项

- 当前配置使用模拟里程计节点，里程计数据固定为0
- SLAM Toolbox会实时构建地图
- Navigation2会使用实时构建的地图进行路径规划
- 如需保存地图，可以使用以下命令：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 自定义配置

您可以根据需要修改以下文件：
- `config/nav2_params.yaml`: 调整导航参数，如机器人半径、膨胀半径等
- `config/navigation_with_robot.rviz`: 调整RViz2显示设置
