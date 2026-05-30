# 串口里程计接收器使用说明

## 概述
串口里程计接收器(serial_odom_receiver)是一个ROS2节点，用于从F407通过串口接收里程计数据，并将其转换为标准的ROS2消息格式发布到`/odom`话题。

## 功能
- 从F407接收自定义格式的串口数据
- 将数据转换为标准的`nav_msgs/Odometry`消息
- 发布odom到base_link的TF变换
- 支持两种数据类型：里程计数据和TF数据

## 数据格式
F407通过串口发送的数据格式如下：

### 里程计数据 (类型: 0x01)
- 帧头: 0xAA
- 类型: 0x01
- 长度: 48字节
- 数据: position_x(double), position_y(double), orientation_z(double), orientation_w(double), linear_velocity_x(double), angular_velocity_z(double)
- 帧尾: 0x55

### TF数据 (类型: 0x02)
- 帧头: 0xAA
- 类型: 0x02
- 长度: 24字节
- 数据: x(double), y(double), theta(double)
- 帧尾: 0x55

## 使用方法

### 1. 测试串口里程计接收器
```bash
source install/setup.bash
ros2 launch alloy_chassis_mecanum test_serial_odom.launch.py
```

### 2. 在导航系统中使用串口里程计接收器
```bash
source install/setup.bash
ros2 launch alloy_chassis_mecanum navigation_with_robot.launch.py
```

### 3. 自定义串口参数
如果需要使用不同的串口设备或波特率，可以修改启动文件中的参数：
```python
parameters=[{
    'serial_port': '/dev/ttyUSB1',  # 修改为你的串口设备
    'baudrate': 9600  # 修改为你的波特率
}]
```

## 注意事项
1. 确保F407通过串口正确连接到计算机
2. 确保串口设备有读写权限，可以使用以下命令添加权限：
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```
3. 如果串口设备不是`/dev/ttyUSB0`，需要修改启动文件中的`serial_port`参数
4. 确保F407发送的数据格式与上述定义一致

## 故障排除
1. 如果无法打开串口，检查串口设备名称和权限
2. 如果没有接收到数据，检查F407是否正确发送数据
3. 如果数据格式不正确，检查F407代码中的数据结构是否与ROS2节点中的解析代码一致

## 下一步工作
1. 实现基于编码器的真实里程计计算
2. 添加IMU数据融合，提高里程计精度
3. 添加里程计校准功能
