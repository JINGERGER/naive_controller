# DWA避障导航控制器使用指南

## 简介

`dwa_controller.py` 是一个带有**局部路径规划和避障功能**的导航控制器，使用 **DWA (Dynamic Window Approach)** 算法。

### 核心特性

1. **DWA局部路径规划** - 实时计算最优轨迹避开障碍物
2. **激光雷达集成** - 订阅 `/scan` 话题获取障碍物信息
3. **动态避障** - 在移动过程中实时规划安全路径
4. **可配置参数** - 支持调整规划器参数和机器人参数
5. **向后兼容** - 可以禁用DWA使用简单控制

## 文件说明

- `dwa_controller.py` - 带DWA避障的导航控制器
- `fake_scan.py` - 模拟激光雷达节点（用于测试）
- `naive_controller.py` - 原始简单控制器（无避障）

## DWA算法原理

```
1. 采样速度空间 (v, w)
   └─ 在动态窗口内采样多组候选速度

2. 预测轨迹
   └─ 对每组速度预测未来2秒的运动轨迹

3. 评估代价
   ├─ 目标代价：轨迹终点到目标的距离
   ├─ 障碍物代价：轨迹与障碍物的最小距离
   └─ 速度代价：鼓励更快的速度

4. 选择最优
   └─ 选择总代价最小且无碰撞的速度命令
```

## 使用方法

### 测试方案1：带模拟激光雷达

**终端1：启动fake_odom**
```bash
python3 fake_odom.py
```

**终端2：启动fake_scan（模拟激光雷达）**
```bash
python3 fake_scan.py
```
> 默认会在 (2.0, 1.0), (3.0, -0.5), (1.5, 0.5) 放置虚拟障碍物

**终端3：启动DWA控制器**
```bash
python3 dwa_controller.py
```

**终端4：启动可视化**
```bash
python3 visualizer.py
```

**终端5：发送目标点**
```bash
python3 interactive_goal.py
```
然后输入：
```
请选择坐标系 (1/2, 默认1): 1
请输入目标点 (x y): 4.0 0.0
```

机器人会：
1. 旋转朝向目标
2. 使用DWA规划路径绕过障碍物
3. 到达目标点

### 测试方案2：真实机器人

如果您有真实的激光雷达和里程计：

**启动控制器**
```bash
python3 dwa_controller.py
```

**发送目标点**
```bash
python3 interactive_goal.py
```

控制器会自动订阅：
- `/odom` - 里程计
- `/scan` - 激光雷达
- `/goal_pose` - 目标点

并发布：
- `/cmd_vel` - 速度命令

## 参数配置

### 控制器参数

```python
# 在dwa_controller.py中或通过ROS2参数设置

# 基本参数
angle_threshold: 0.1          # 旋转精度（弧度）
distance_threshold: 0.05      # 到达阈值（米）
max_linear_velocity: 0.3      # 最大线速度（m/s）
max_angular_velocity: 1.0     # 最大角速度（rad/s）

# DWA参数
max_linear_acc: 0.2           # 最大线加速度（m/s²）
max_angular_acc: 1.0          # 最大角加速度（rad/s²）
robot_radius: 0.3             # 机器人半径（米）
safe_distance: 0.5            # 安全距离（米）
use_dwa: True                 # 是否启用DWA避障
```

### DWA算法参数

在 `DWAPlanner` 类中：

```python
dt: 0.1                       # 预测时间步长（秒）
predict_time: 2.0             # 预测时长（秒）
linear_samples: 10            # 线速度采样数
angular_samples: 20           # 角速度采样数

# 代价函数权重
goal_cost_weight: 1.0         # 目标代价权重
obstacle_cost_weight: 2.0     # 障碍物代价权重
velocity_cost_weight: 0.2     # 速度代价权重
```

### 通过ROS2参数设置

```bash
python3 dwa_controller.py --ros-args \
  -p max_linear_velocity:=0.5 \
  -p safe_distance:=0.8 \
  -p use_dwa:=true
```

## 测试场景

### 场景1：简单避障

```bash
# 障碍物在前方
# fake_scan默认在(2.0, 1.0)有障碍物

# 发送目标点到障碍物后方
请输入目标点 (x y): 3.0 1.0
```

机器人会绕过障碍物到达目标。

### 场景2：窄通道

修改 `fake_scan.py` 添加通道：

```python
self.obstacles = [
    (2.0, 0.8),   # 左侧墙
    (2.0, -0.8),  # 右侧墙
    (2.5, 0.8),
    (2.5, -0.8),
]
```

### 场景3：动态添加障碍物

在fake_scan运行时：

```bash
ros2 topic pub /add_obstacle geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 2.5, y: 0.5, z: 0.0}}
}"
```

## 对比测试

### 禁用DWA（简单控制）

```bash
python3 dwa_controller.py --ros-args -p use_dwa:=false
```

机器人会直线前进，可能撞到障碍物。

### 启用DWA（避障控制）

```bash
python3 dwa_controller.py --ros-args -p use_dwa:=true
```

机器人会智能绕过障碍物。

## 调试信息

控制器会输出：

```
[INFO] DWA: v=0.25, w=0.15, dist=2.34, obs=156
```

- `v`: 规划的线速度
- `w`: 规划的角速度
- `dist`: 到目标距离
- `obs`: 检测到的障碍物点数

## 故障排除

### 机器人不动

1. 检查是否收到激光雷达数据：`ros2 topic echo /scan`
2. 检查DWA是否启用：日志中查看 "DWA Avoidance: ENABLED"
3. 检查是否所有候选轨迹都会碰撞（增大safe_distance）

### 绕路太远

调整代价函数权重：
- 增大 `goal_cost_weight` - 更直接地朝向目标
- 减小 `obstacle_cost_weight` - 允许离障碍物更近

### 速度太慢

- 增大 `max_linear_velocity`
- 减小 `velocity_cost_weight`
- 增大 `max_linear_acc`

### 震荡/不稳定

- 减小 `angular_samples` 和 `linear_samples`
- 增大 `dt` 或 `predict_time`
- 调整 `safe_distance`

## 性能优化

### 减少计算量

```python
# 减少采样数
linear_samples: 5
angular_samples: 10

# 减少预测时长
predict_time: 1.5
```

### 提高安全性

```python
# 增大安全距离
safe_distance: 0.8
robot_radius: 0.4

# 增大障碍物代价权重
obstacle_cost_weight: 3.0
```

## 与原始控制器对比

| 特性 | naive_controller.py | dwa_controller.py |
|------|-------------------|------------------|
| 避障 | ❌ 无 | ✅ DWA动态避障 |
| 激光雷达 | ❌ 不需要 | ✅ 必需 |
| 轨迹规划 | ❌ 直线 | ✅ 曲线绕行 |
| 计算复杂度 | 低 | 中等 |
| 适用场景 | 空旷环境 | 复杂环境 |

## 扩展功能

### 添加costmap

可以集成2D costmap来更精确地表示障碍物：

```python
# 将obstacles列表改为2D网格
# 每个cell表示占用概率
```

### 添加全局路径

可以结合A*或Dijkstra全局规划：

```python
# DWA作为局部规划器
# 跟随全局路径的局部片段
```

### 多机器人避障

添加动态障碍物预测：

```python
# 预测其他机器人的轨迹
# 避免未来碰撞
```

## 参考资料

- DWA原论文: Fox, D., et al. "The dynamic window approach to collision avoidance" (1997)
- ROS Navigation Stack: http://wiki.ros.org/navigation
- Local Planner对比: http://wiki.ros.org/base_local_planner
