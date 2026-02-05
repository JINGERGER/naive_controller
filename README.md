# ROS2 导航控制器测试环境

这是一个将目标点导航分解为"旋转+直走"两个动作的简单控制器，使用ODOM增量进行闭环控制。

## 文件说明

- `naive_controller.py` - 主控制器节点
- `fake_odom.py` - 模拟里程计节点（用于测试）
- `test_goal_publisher.py` - 测试目标点发布器
- `visualizer.py` - 2D可视化工具

## 测试步骤

### 1. 启动模拟里程计

```bash
python3 fake_odom.py
```

### 2. 启动控制器

在新终端中：

```bash
python3 naive_controller.py
```

### 3. 启动可视化工具

在新终端中：

```bash
python3 visualizer.py
```

会弹出一个matplotlib窗口，实时显示：
- 蓝色圆点：机器人当前位置
- 蓝色箭头：机器人朝向
- 蓝色线：运动轨迹
- 红色星号：目标点（odom坐标系）
- 绿色星号：目标点（base_link坐标系，会跟随机器人移动）

### 4. 发送目标点

有两种方式：

#### 方式1：交互式手动输入（推荐）

在新终端中运行：

```bash
python3 interactive_goal.py
```

然后根据提示手动输入目标点坐标：

```
请输入目标点 (x y): 2.0 0.0    # 前方2米
请输入目标点 (x y): 0.0 1.0    # 左侧1米
请输入目标点 (x y): -1.0 0.0   # 后方1米
请输入目标点 (x y): 1.0 -1.0   # 右前方
请输入目标点 (x y): q          # 退出
```

#### 方式2：命令行参数

```bash
# 前方2米
python3 test_goal_publisher.py 2.0 0.0

# 左侧1米
python3 test_goal_publisher.py 0.0 1.0

# 后方1米（会先转180度，再前进）
python3 test_goal_publisher.py -1.0 0.0

# 右前方（45度，距离√2米）
python3 test_goal_publisher.py 1.0 -1.0
```

## 观察点

1. **旋转阶段**
   - 机器人会先旋转到目标方向
   - 控制器会打印旋转进度
   - 可视化中可以看到箭头旋转

2. **移动阶段**
   - 旋转完成后开始直线前进
   - 控制器会打印移动进度
   - 可视化中可以看到轨迹应该是直线

3. **到达目标**
   - 控制器打印 "goal reached!"
   - 机器人停止
   - 可以发送下一个目标点

## 可调参数

在 `naive_controller.py` 中可以调整：

```python
self.declare_parameter('angle_threshold', 0.1)          # 角度阈值（弧度）
self.declare_parameter('distance_threshold', 0.05)      # 距离阈值（米）
self.declare_parameter('max_linear_velocity', 0.3)      # 最大线速度（m/s）
self.declare_parameter('max_angular_velocity', 1.0)     # 最大角速度（rad/s）
self.declare_parameter('kp_linear', 0.5)                # 线速度增益
self.declare_parameter('kp_angular', 2.0)               # 角速度增益
```

通过ROS2参数也可以设置：

```bash
python3 naive_controller.py --ros-args -p max_linear_velocity:=0.5
```

## 测试用例建议

1. **简单前进**: `python3 test_goal_publisher.py 1.0 0.0`
2. **旋转90度后前进**: `python3 test_goal_publisher.py 0.0 1.0`
3. **旋转180度后前进**: `python3 test_goal_publisher.py -1.0 0.0`
4. **斜向移动**: `python3 test_goal_publisher.py 1.0 1.0`
5. **短距离精度测试**: `python3 test_goal_publisher.py 0.1 0.1`

## 工作原理

### 目标点处理
- 输入：base_link坐标系下的目标点 (target_x, target_y)
- 计算：
  - 目标旋转角度：`target_yaw = atan2(target_y, target_x)`
  - 目标前进距离：`target_distance = sqrt(target_x² + target_y²)`

### 旋转控制
- 记录起始yaw：`start_yaw`
- 使用ODOM监控：`rotated_angle = current_yaw - start_yaw`
- 比例控制：`angular_vel = Kp * (target_yaw - rotated_angle)`

### 移动控制
- 记录起始位置：`(start_x, start_y)`
- 使用ODOM监控：`moved_distance = sqrt((x-start_x)² + (y-start_y)²)`
- 比例控制：`linear_vel = Kp * (target_distance - moved_distance)`
- 微调朝向保持直线

## 故障排除

### 可视化窗口无响应
- 确保安装了matplotlib：`pip3 install matplotlib`
- 可能需要安装tkinter：`sudo apt install python3-tk`

### 机器人不动
- 检查是否所有节点都在运行
- 检查话题连接：`ros2 topic list`
- 检查消息流：`ros2 topic echo /cmd_vel`

### 机器人运动不准确
- 调整控制参数（kp_linear, kp_angular）
- 调整阈值（angle_threshold, distance_threshold）
- 降低最大速度
# naive_controller
