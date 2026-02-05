# ROS2 导航控制器测试环境

本项目包含两个导航控制器：
1. **Naive Controller** - 简单的"旋转+直走"控制器，无避障
2. **DWA Controller** - 带避障功能的动态窗口法(DWA)控制器

## 文件说明

| 文件 | 说明 |
|------|------|
| `naive_controller.py` | 简单控制器（旋转+直走） |
| `dwa_controller.py` | DWA避障控制器 |
| `fake_odom.py` | 模拟里程计节点 |
| `fake_scan.py` | 模拟激光扫描节点 |
| `visualizer.py` | 2D可视化工具 |
| `interactive_goal.py` | 交互式目标点发布器 |
| `interactive_obstacle.py` | 交互式障碍物发布器 |
| `test_goal_publisher.py` | 命令行目标点发布器 |

---

## 方案一：简单控制器（无避障）

### 启动步骤

```bash
# 终端1：模拟里程计
python3 fake_odom.py

# 终端2：简单控制器
python3 naive_controller.py

# 终端3：可视化
python3 visualizer.py

# 终端4：发送目标点
python3 interactive_goal.py
```

### 工作原理

1. 接收目标点（base_link或odom坐标系）
2. 转换到odom坐标系作为固定目标
3. 旋转至目标方向
4. 直线前进至目标点

---

## 方案二：DWA避障控制器

### 启动步骤

```bash
# 终端1：模拟里程计
python3 fake_odom.py

# 终端2：模拟激光扫描
python3 fake_scan.py

# 终端3：DWA控制器
python3 dwa_controller.py

# 终端4：可视化
python3 visualizer.py

# 终端5：发送目标点
python3 interactive_goal.py

# 终端6（可选）：添加障碍物
python3 interactive_obstacle.py
```

### DWA参数

通过ROS2参数设置：

```bash
python3 dwa_controller.py --ros-args \
  -p max_linear_vel:=0.3 \
  -p max_angular_vel:=2.5 \
  -p predict_time:=2.0 \
  -p goal_cost_weight:=1.0 \
  -p obstacle_cost_weight:=1.0 \
  -p robot_radius:=0.3 \
  -p safe_distance:=0.15
```

### OctoMap 支持

DWA控制器支持从OctoMap获取障碍物信息：

```bash
python3 dwa_controller.py --ros-args \
  -p use_octomap:=true \
  -p octomap_topic:=/octomap_point_cloud_centers \
  -p octomap_height_min:=0.0 \
  -p octomap_height_max:=0.5 \
  -p octomap_resolution:=0.1 \
  -p octomap_range:=5.0
```

**支持的OctoMap话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/octomap_point_cloud_centers` | PointCloud2 | 占用体素中心点（推荐） |
| `/occupied_cells_vis_array` | MarkerArray | 可视化Markers（备选） |

**OctoMap工作流程：**
```
传感器点云 → [octomap_server] → OctoMap话题 → [DWA控制器]
```

需要安装octomap_server：
```bash
sudo apt install ros-humble-octomap-server

ros2 run octomap_server octomap_server_node --ros-args \
  -p frame_id:=odom \
  -r cloud_in:=/your_pointcloud_topic
```

---

## 可视化说明

可视化窗口显示：

| 元素 | 说明 |
|------|------|
| 彩色圆点 | 机器人位置（颜色随状态变化） |
| 箭头 | 机器人朝向 |
| 蓝色线 | 运动轨迹 |
| 红色星号 | 目标点 |
| 红色圆圈 | 障碍物 |
| 绿色线 | DWA规划路径 |

**机器人颜色状态：**
- 灰色：IDLE（空闲）
- 橙色：ROTATING（旋转中）
- 蓝色：MOVING（移动中）
- 绿色：REACHED（已到达）

---

## 交互式目标点

```bash
python3 interactive_goal.py
```

支持两种坐标系输入：
- **相对坐标（base_link）**：默认，相对于机器人当前位置
- **绝对坐标（odom）**：输入 `abs` 切换

```
=== 交互式目标点发布器 ===
当前坐标系: base_link (相对坐标)
输入 'abs' 切换到绝对坐标, 'rel' 切换到相对坐标
输入 'q' 退出

请输入目标点 (x y): 2.0 0.0     # 前方2米
请输入目标点 (x y): abs          # 切换到绝对坐标
请输入目标点 (x y): 5.0 3.0     # 去往odom坐标(5,3)
```

---

## 交互式障碍物

```bash
python3 interactive_obstacle.py
```

支持多种障碍物添加方式：

```
命令:
  x y        - 添加单个障碍物
  wall x1 y1 x2 y2 [n]  - 添加墙壁
  circle x y r [n]      - 添加圆形障碍
  preset N   - 预设场景 (1-3)
  clear      - 清除所有障碍物
  q          - 退出

> 2.0 0.0              # 在(2,0)添加障碍物
> wall 1 -1 1 1 10     # 从(1,-1)到(1,1)的墙壁
> circle 3 3 1 20      # 圆心(3,3)半径1的圆形障碍
> preset 1             # 加载预设场景1
> clear                # 清除所有
```

---

## 话题列表

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/odom` | Odometry | 输入 | 机器人里程计 |
| `/scan` | LaserScan | 输入 | 激光扫描（DWA） |
| `/goal_pose` | PoseStamped | 输入 | 目标点 |
| `/cmd_vel` | Twist | 输出 | 速度指令 |
| `/controller_state` | String | 输出 | 控制器状态 |
| `/add_obstacle` | PoseStamped | 输入 | 添加障碍物 |
| `/clear_obstacles` | String | 输入 | 清除障碍物 |
| `/dwa_trajectory` | Path | 输出 | DWA规划路径 |

---

## 可调参数

### Naive Controller

```python
angle_threshold = 0.1          # 角度阈值（弧度）
distance_threshold = 0.05      # 距离阈值（米）
max_linear_velocity = 0.3      # 最大线速度（m/s）
max_angular_velocity = 1.0     # 最大角速度（rad/s）
kp_linear = 0.5                # 线速度增益
kp_angular = 2.0               # 角速度增益
```

### DWA Controller

```python
max_linear_vel = 0.3           # 最大线速度
min_linear_vel = 0.0           # 最小线速度
max_angular_vel = 2.5          # 最大角速度
max_linear_acc = 0.2           # 最大线加速度
max_angular_acc = 5.0          # 最大角加速度
predict_time = 2.0             # 轨迹预测时间
linear_samples = 10            # 线速度采样数
angular_samples = 50           # 角速度采样数
goal_cost_weight = 1.0         # 目标代价权重
obstacle_cost_weight = 1.0     # 障碍代价权重
velocity_cost_weight = 0.1     # 速度代价权重
robot_radius = 0.3             # 机器人半径
safe_distance = 0.15           # 安全距离
obstacle_radius = 0.15         # 障碍物半径
```

---

## 故障排除

### 可视化窗口无响应
```bash
pip3 install matplotlib
sudo apt install python3-tk
```

### 机器人不动
```bash
# 检查话题
ros2 topic list
ros2 topic echo /cmd_vel

# 检查里程计
ros2 topic echo /odom
```

### DWA机器人卡住
- 增大 `max_angular_vel` 允许更大转弯
- 减小 `obstacle_cost_weight` 减少避障保守程度
- 增大 `predict_time` 让规划更远

### DWA机器人撞墙
- 增大 `safe_distance` 增加安全距离
- 增大 `obstacle_cost_weight` 增加避障权重
- 减小 `max_linear_vel` 降低速度

---

## 依赖

```bash
# ROS2 (Humble)
sudo apt install ros-humble-desktop

# Python依赖
pip3 install matplotlib numpy

# OctoMap（可选）
sudo apt install ros-humble-octomap-server
```
