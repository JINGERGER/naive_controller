#!/usr/bin/env python3
"""
带避障功能的导航控制器
使用DWA (Dynamic Window Approach) 进行局部路径规划
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from enum import Enum
import math
import numpy as np
import struct


class ControllerState(Enum):
    """控制器状态机"""
    IDLE = 0       # 等待目标点
    ROTATING = 1   # 旋转阶段
    MOVING = 2     # 移动阶段（带避障）
    REACHED = 3    # 到达目标


class DWAPlanner:
    """DWA局部路径规划器"""
    
    def __init__(self, config):
        # 机器人参数
        self.max_linear_vel = config.get('max_linear_vel', 0.3)
        self.min_linear_vel = config.get('min_linear_vel', 0.0)
        self.max_angular_vel = config.get('max_angular_vel', 1.0)
        self.max_linear_acc = config.get('max_linear_acc', 0.2)
        self.max_angular_acc = config.get('max_angular_acc', 1.0)
        
        # DWA参数
        self.dt = config.get('dt', 0.1)  # 预测时间步长
        self.predict_time = config.get('predict_time', 2.0)  # 预测时长
        self.linear_samples = config.get('linear_samples', 10)
        self.angular_samples = config.get('angular_samples', 20)
        
        # 代价函数权重
        self.goal_cost_weight = config.get('goal_cost_weight', 1.0)
        self.obstacle_cost_weight = config.get(
            'obstacle_cost_weight', 2.0)
        self.velocity_cost_weight = config.get(
            'velocity_cost_weight', 0.2)
        
        # 安全距离
        self.robot_radius = config.get('robot_radius', 0.3)
        self.safe_distance = config.get('safe_distance', 0.5)
    
    def calc_dynamic_window(self, current_v, current_w):
        """
        计算动态窗口（考虑加速度约束）
        """
        # 机器人能达到的速度范围
        v_min = self.min_linear_vel
        v_max = self.max_linear_vel
        w_min = -self.max_angular_vel
        w_max = self.max_angular_vel
        
        # 考虑加速度约束
        v_min_acc = current_v - self.max_linear_acc * self.dt
        v_max_acc = current_v + self.max_linear_acc * self.dt
        w_min_acc = current_w - self.max_angular_acc * self.dt
        w_max_acc = current_w + self.max_angular_acc * self.dt
        
        # 动态窗口
        dw = [
            max(v_min, v_min_acc),
            min(v_max, v_max_acc),
            max(w_min, w_min_acc),
            min(w_max, w_max_acc)
        ]
        
        return dw
    
    def predict_trajectory(self, x, y, yaw, v, w):
        """
        预测给定速度下的轨迹
        """
        trajectory = [(x, y, yaw)]
        time = 0
        
        while time <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            trajectory.append((x, y, yaw))
            time += self.dt
        
        return trajectory
    
    def calc_obstacle_cost(self, trajectory, obstacles):
        """
        计算障碍物代价
        obstacles: list of (x, y) - 障碍物中心点
        障碍物也有半径（假设与机器人相同）
        """
        if not obstacles:
            return 0.0

        min_dist = float('inf')
        obstacle_radius = 0.15  # 障碍物半径
        collision_threshold = 0.05  # 5cm碰撞缓冲

        # 检查整条轨迹的每个点（优化：提前退出）
        for tx, ty, _ in trajectory:
            for ox, oy in obstacles:
                # 快速距离检查（避免sqrt）
                dx = tx - ox
                dy = ty - oy
                dist_sq = dx*dx + dy*dy
                
                # 如果距离平方大于安全距离平方的4倍，跳过精确计算
                safe_dist_sq = (self.robot_radius + obstacle_radius + self.safe_distance) ** 2
                if dist_sq > safe_dist_sq * 4:
                    continue
                
                # 精确计算边缘距离
                center_dist = math.sqrt(dist_sq)
                edge_dist = center_dist - self.robot_radius - obstacle_radius
                
                # 碰撞检测：提前退出
                if edge_dist < collision_threshold:
                    return float('inf')
                
                if edge_dist < min_dist:
                    min_dist = edge_dist

        # 安全距离内增加代价
        if min_dist < self.safe_distance:
            return 3.0 / max(min_dist, 0.01)

        # 距离越近代价越高（如果没有近距离障碍物，返回低代价）
        if min_dist == float('inf'):
            return 0.0
        return 0.5 / min_dist
    
    def calc_goal_cost(self, trajectory, goal_x, goal_y):
        """
        计算到目标点的代价
        """
        # 使用轨迹终点到目标的距离
        tx, ty, tyaw = trajectory[-1]
        
        # 距离代价
        dist = math.sqrt((tx - goal_x)**2 + (ty - goal_y)**2)
        
        # 朝向代价
        angle_to_goal = math.atan2(goal_y - ty, goal_x - tx)
        angle_diff = abs(self.normalize_angle(angle_to_goal - tyaw))
        
        return dist + angle_diff * 0.5
    
    def calc_velocity_cost(self, v):
        """
        速度代价（鼓励更快的速度）
        """
        return self.max_linear_vel - v
    
    def normalize_angle(self, angle):
        """归一化角度到[-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def plan(self, x, y, yaw, v, w, goal_x, goal_y, obstacles):
        """
        DWA规划主函数
        返回最优的 (v, w)
        """
        # 计算动态窗口
        dw = self.calc_dynamic_window(v, w)

        # 采样速度空间
        best_v = 0.0
        best_w = 0.0
        min_cost = float('inf')
        best_trajectory = None
        valid_count = 0

        v_samples = np.linspace(dw[0], dw[1], self.linear_samples)
        w_samples = np.linspace(dw[2], dw[3], self.angular_samples)

        for sample_v in v_samples:
            for sample_w in w_samples:
                # 预测轨迹
                trajectory = self.predict_trajectory(
                    x, y, yaw, sample_v, sample_w)

                # 计算代价
                obstacle_cost = self.calc_obstacle_cost(
                    trajectory, obstacles)

                # 如果碰撞，跳过
                if obstacle_cost == float('inf'):
                    continue

                valid_count += 1

                goal_cost = self.calc_goal_cost(
                    trajectory, goal_x, goal_y)
                velocity_cost = self.calc_velocity_cost(sample_v)

                # 总代价
                total_cost = (
                    self.goal_cost_weight * goal_cost +
                    self.obstacle_cost_weight * obstacle_cost +
                    self.velocity_cost_weight * velocity_cost
                )

                # 选择最优
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_v = sample_v
                    best_w = sample_w
                    best_trajectory = trajectory

        # 如果没有找到有效轨迹，智能选择绕行方向
        if valid_count == 0:
            # 分析障碍物位置，选择空旷的方向
            left_clear = 0.0  # 左侧空旷程度
            right_clear = 0.0  # 右侧空旷程度
            
            for ox, oy in obstacles:
                # 转换到机器人坐标系
                dx_obs = ox - x
                dy_obs = oy - y
                # 转换到机器人局部坐标
                local_x = dx_obs * math.cos(-yaw) - dy_obs * math.sin(-yaw)
                local_y = dx_obs * math.sin(-yaw) + dy_obs * math.cos(-yaw)
                
                # 只考虑前方的障碍物
                if local_x > 0 and local_x < 3.0:
                    dist = math.sqrt(local_x**2 + local_y**2)
                    weight = 1.0 / max(dist, 0.1)
                    if local_y > 0:  # 障碍物在左边
                        left_clear -= weight
                    else:  # 障碍物在右边
                        right_clear -= weight
            
            # 选择更空旷的方向
            if left_clear > right_clear:
                best_w = self.max_angular_vel * 0.8  # 左转
            else:
                best_w = -self.max_angular_vel * 0.8  # 右转
            
            # 纯旋转，不前进（避免撞墙）
            best_v = 0.0
            
            # 生成一个旋转轨迹用于显示
            best_trajectory = self.predict_trajectory(x, y, yaw, best_v, best_w)

        return best_v, best_w, best_trajectory, valid_count


class DWANavController(Node):
    """带DWA避障的导航控制器"""

    def __init__(self):
        super().__init__('dwa_nav_controller')

        # 声明参数
        self.declare_parameter('angle_threshold', 0.1)
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('max_linear_velocity', 0.5)  # Bunker Mini 最大1m/s，设置0.5m/s
        self.declare_parameter('max_angular_velocity', 1.0)
        # 最小速度阈值（克服电机死区 - 实车测试值）
        self.declare_parameter('min_linear_velocity', 0.1)  # 低于此值的线速度会被置0或提升
        self.declare_parameter('min_angular_velocity', 0.3)  # 低于此值的角速度会被置0或提升
        # 纯运动模式（避免小速度下打滑）
        self.declare_parameter('pure_motion_mode', True)  # 是否启用纯旋转/纯直行模式
        # 纯旋转速度（提升旋转效率）
        self.declare_parameter('pure_rotation_speed', 0.6)  # 纯旋转时的角速度
        # 防打滑线速度阈值（低于此值时禁止组合运动）
        self.declare_parameter('slip_prevention_linear_threshold', 0.2)  # 线速度低于此值时，强制纯运动
        self.declare_parameter('pure_motion_angle_threshold', 0.15)  # 纯运动模式角度阈值(弧度), 约8.6°
        self.declare_parameter('debug_performance', False)  # 是否输出性能DEBUG日志
        self.declare_parameter('max_linear_acc', 0.2)
        self.declare_parameter('max_angular_acc', 1.0)
        self.declare_parameter('control_frequency', 10.0)
        # Bunker Mini 尺寸: 690 x 570 x 335 mm
        # robot_radius = sqrt((0.345)² + (0.285)²) ≈ 0.45m (外接圆半径)
        self.declare_parameter('robot_radius', 0.45)
        self.declare_parameter('safe_distance', 0.2)
        self.declare_parameter('use_dwa', True)  # 是否启用DWA避障
        self.declare_parameter('scan_fov', 3.14159)  # scan视野角度(弧度), 默认±90°=180°总视野
        self.declare_parameter('use_octomap', False)  # 是否使用OctoMap障碍物
        self.declare_parameter('octomap_topic', '/octomap_point_cloud_centers')  # OctoMap点云话题
        self.declare_parameter('octomap_height_min', -0.1)  # 提取障碍物的最小高度
        self.declare_parameter('octomap_height_max', 1.0)  # 提取障碍物的最大高度
        self.declare_parameter('octomap_resolution', 0.1)  # 下采样分辨率
        self.declare_parameter('octomap_range', 5.0)  # 只处理此范围内的障碍物
        # 激光雷达相对于车辆中心的偏移量
        self.declare_parameter('lidar_offset_x', 0.30)  # 前方30cm
        self.declare_parameter('lidar_offset_y', 0.0)   # 左右居中

        # 获取参数
        self.angle_threshold = self.get_parameter(
            'angle_threshold').value
        self.distance_threshold = self.get_parameter(
            'distance_threshold').value
        max_linear_vel = self.get_parameter(
            'max_linear_velocity').value
        max_angular_vel = self.get_parameter(
            'max_angular_velocity').value
        control_freq = self.get_parameter('control_frequency').value
        self.use_dwa = self.get_parameter('use_dwa').value
        self.use_octomap = self.get_parameter('use_octomap').value
        octomap_topic = self.get_parameter('octomap_topic').value
        self.octomap_height_min = self.get_parameter('octomap_height_min').value
        self.octomap_height_max = self.get_parameter('octomap_height_max').value
        self.octomap_resolution = self.get_parameter('octomap_resolution').value
        self.octomap_range = self.get_parameter('octomap_range').value
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.lidar_offset_y = self.get_parameter('lidar_offset_y').value

        # DWA配置（优化绕行能力）
        dwa_config = {
            'max_linear_vel': max_linear_vel,
            'min_linear_vel': 0.0,
            'max_angular_vel': max(max_angular_vel, 2.5),  # 至少2.5 rad/s
            'max_linear_acc': self.get_parameter('max_linear_acc').value,
            'max_angular_acc': 5.0,  # 更大的角加速度，快速转向
            'dt': 0.1,
            'predict_time': 3.0,  # 增加预测时间，看得更远
            'linear_samples': 10,
            'angular_samples': 50,  # 更多角速度采样
            'goal_cost_weight': 2.5,  # 增加目标权重，更快朝向目标
            'obstacle_cost_weight': 0.8,  # 降低障碍物权重，更积极绕行
            'velocity_cost_weight': 0.2,  # 稍微鼓励速度
            'robot_radius': self.get_parameter('robot_radius').value,
            'safe_distance': self.get_parameter('safe_distance').value,
        }
        
        self.dwa_planner = DWAPlanner(dwa_config)

        # 订阅话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 也直接订阅障碍物话题（更可靠）
        self.obstacle_sub = self.create_subscription(
            PoseStamped,
            '/add_obstacle',
            self.obstacle_callback,
            10
        )

        self.clear_sub = self.create_subscription(
            String,
            '/clear_obstacles',
            self.clear_obstacles_callback,
            10
        )

        # OctoMap障碍物订阅
        if self.use_octomap:
            # 订阅 octomap_server 发布的点云（占用格子中心点）
            # 常见话题: /octomap_point_cloud_centers
            self.octomap_pc_sub = self.create_subscription(
                PointCloud2,
                octomap_topic,
                self.octomap_pointcloud_callback,
                10
            )
            # 也订阅可视化Marker（备选输入）
            # 常见话题: /occupied_cells_vis_array
            self.octomap_marker_sub = self.create_subscription(
                MarkerArray,
                '/occupied_cells_vis_array',
                self.octomap_marker_callback,
                10
            )
            self.get_logger().info(
                f'OctoMap mode enabled, subscribing to: {octomap_topic}'
            )
            self.get_logger().info(
                f'OctoMap height filter: [{self.octomap_height_min}, '
                f'{self.octomap_height_max}]m, range: {self.octomap_range}m'
            )

        # 发布话题
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(
            String, '/controller_state', 10)
        self.trajectory_pub = self.create_publisher(
            Path, '/dwa_trajectory', 10)

        # 创建控制定时器
        self.timer = self.create_timer(
            1.0 / control_freq, self.control_loop)

        # 状态变量
        self.state = ControllerState.IDLE

        # ODOM数据
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.odom_received = False

        # 障碍物数据
        self.obstacles = []  # list of (x, y) in odom frame
        self.direct_obstacles = []  # 直接从/add_obstacle获取的障碍物
        self.octomap_obstacles = []  # 从OctoMap获取的障碍物
        self.scan_received = False

        # 性能统计
        self.debug_performance = False
        self.perf_scan_count = 0
        self.perf_scan_time_total = 0.0
        self.perf_dwa_count = 0
        self.perf_dwa_time_total = 0.0
        self.perf_loop_count = 0
        self.perf_last_report = None
        
        # 目标点（odom坐标系）
        self.goal_x = None
        self.goal_y = None

        # 到达目标后的停留时间
        self.reach_time = None
        self.wait_duration = 3.0

        # 卡住检测（放宽条件，避免误触发）
        self.stuck_count = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.stuck_threshold = 30  # 连续30个周期没动才认为卡住（3秒）
        self.stuck_move_threshold = 0.01  # 移动阈值降低到1cm
        self.recovery_direction = 1  # 1=左转, -1=右转
        
        # 绕行方向锁定（防止左右震荡）
        self.bypass_direction = 0  # 0=未锁定, 1=左绕, -1=右绕
        self.bypass_lock_count = 0  # 锁定计数器
        self.bypass_lock_threshold = 50  # 增加锁定时间，因为智能选择后应坚持
        self.last_angular_z = 0.0  # 上一次的角速度，用于平滑
        self.last_turn_dir = 0  # 上一次的转向方向（防止左右摆动）
        self.last_distance = float('inf')  # 上一次到目标的距离
        self.min_obstacle_dist = float('inf')  # 最近障碍物距离
        self.initial_goal_angle = None  # 初始目标角度（用于检测是否绑过障碍物）

        # 读取性能调试开关
        self.debug_performance = self.get_parameter('debug_performance').value
        
        self.get_logger().info('DWA NavController initialized')
        self.get_logger().info(
            f'DWA Avoidance: {"ENABLED" if self.use_dwa else "DISABLED"}')
        scan_fov_deg = math.degrees(self.get_parameter('scan_fov').value)
        self.get_logger().info(
            f'Scan FOV: ±{scan_fov_deg/2:.0f}° ({scan_fov_deg:.0f}° total)')
        self.get_logger().info(
            f'LiDAR Offset: x={self.lidar_offset_x:.2f}m, y={self.lidar_offset_y:.2f}m')
        self.get_logger().info(
            f'OctoMap Mode: {"ENABLED" if self.use_octomap else "DISABLED"}')
        if self.debug_performance:
            self.get_logger().info('Performance Debug: ENABLED (will report every 5s)')

    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def downsample_obstacles(self, obstacles, grid_size=0.15):
        """
        使用网格法对障碍物进行下采样
        每个grid_size×grid_size的格子只保留一个障碍物
        """
        if not obstacles:
            return obstacles
        
        grid = {}
        for ox, oy in obstacles:
            # 计算网格索引
            gx = int(ox / grid_size)
            gy = int(oy / grid_size)
            key = (gx, gy)
            
            # 每个格子只保留一个点（第一个遇到的）
            if key not in grid:
                grid[key] = (ox, oy)
        
        return list(grid.values())

    def quaternion_to_yaw(self, x, y, z, w):
        """从四元数转换为yaw角"""
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        return yaw

    def clamp(self, value, min_value, max_value):
        """限制数值范围"""
        return max(min_value, min(value, max_value))
    
    def apply_min_velocity(self, cmd_vel):
        """
        应用最小速度阈值，克服电机死区
        如果速度太小但不为0，提升到最小值
        """
        min_linear = self.get_parameter('min_linear_velocity').value
        min_angular = self.get_parameter('min_angular_velocity').value
        
        # 处理线速度
        if abs(cmd_vel.linear.x) > 0.001 and abs(cmd_vel.linear.x) < min_linear:
            # 线速度太小，提升到最小值
            cmd_vel.linear.x = min_linear if cmd_vel.linear.x > 0 else -min_linear
        
        # 处理角速度
        if abs(cmd_vel.angular.z) > 0.001 and abs(cmd_vel.angular.z) < min_angular:
            # 角速度太小，提升到最小值
            cmd_vel.angular.z = min_angular if cmd_vel.angular.z > 0 else -min_angular
        
        return cmd_vel

    def odom_callback(self, msg):
        """ODOM回调函数，更新当前位姿和速度"""
        # 提取位置
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # 提取姿态（四元数转yaw）
        orientation = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        # 提取速度
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

        self.odom_received = True
        
        # [DEBUG] 周期性打印 odom 状态（每2秒一次）
        self.get_logger().debug(
            f'[ODOM] pos=({self.current_x:.3f}, {self.current_y:.3f}), '
            f'yaw={math.degrees(self.current_yaw):.1f}°, '
            f'vel=({self.current_v:.2f}, {self.current_w:.2f})',
            throttle_duration_sec=2.0)

    def obstacle_callback(self, msg):
        """直接接收障碍物"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.direct_obstacles.append((x, y))
        self.get_logger().info(
            f'Obstacle added: ({x:.2f}, {y:.2f}), '
            f'total: {len(self.direct_obstacles)}')
        # 合并所有障碍物源
        self.merge_obstacles()
        self.scan_received = True

    def clear_obstacles_callback(self, msg):
        """清除所有障碍物（仅清除直接添加的，八叉树障碍物由传感器更新）"""
        count = len(self.direct_obstacles)
        self.direct_obstacles = []
        self.merge_obstacles()
        self.get_logger().info(f'Cleared {count} direct obstacles')

    def scan_callback(self, msg):
        """激光雷达回调函数，转换障碍物到odom坐标系"""
        if self.debug_performance:
            import time
            self._scan_start = time.time()
            
        if not self.odom_received:
            return

        # 如果有直接障碍物，使用直接障碍物（更可靠）
        if self.direct_obstacles:
            self.obstacles = self.direct_obstacles.copy()
            self.scan_received = True
            return

        obstacles = []
        angle = msg.angle_min
        
        # === 只考虑前方扇形区域 ===
        # scan_fov: 前方 ±90° = 180° 总视野（可通过参数调整）
        scan_fov = self.get_parameter('scan_fov').value
        half_fov = scan_fov / 2

        for i, r in enumerate(msg.ranges):
            # 跳过无效数据
            if r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue
            
            # === 角度过滤：只考虑前方区域 ===
            # angle 在 base_link 坐标系中，0 = 正前方
            normalized_angle = angle
            while normalized_angle > math.pi:
                normalized_angle -= 2 * math.pi
            while normalized_angle < -math.pi:
                normalized_angle += 2 * math.pi
            
            # 只保留前方扇形范围内的点
            if abs(normalized_angle) > half_fov:
                angle += msg.angle_increment
                continue

            # 限制检测距离
            if r > 3.0:  # 只考虑3米内的障碍物
                angle += msg.angle_increment
                continue

            # 转换到base_link坐标系（激光雷达原始数据）
            x_lidar = r * math.cos(angle)
            y_lidar = r * math.sin(angle)
            
            # 补偿激光雷达偏移量（从激光雷达坐标系转到车辆中心坐标系）
            x_base = x_lidar + self.lidar_offset_x
            y_base = y_lidar + self.lidar_offset_y

            # 转换到odom坐标系
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            x_odom = self.current_x + x_base * cos_yaw - y_base * sin_yaw
            y_odom = self.current_y + x_base * sin_yaw + y_base * cos_yaw

            obstacles.append((x_odom, y_odom))
            angle += msg.angle_increment

        # === 障碍物下采样（提升DWA性能）===
        # 使用网格法将障碍物数量从~200减少到~50
        if len(obstacles) > 50:
            obstacles = self.downsample_obstacles(obstacles, grid_size=0.15)
        
        self.obstacles = obstacles
        self.scan_received = True
        
        # 性能统计
        if self.debug_performance:
            import time
            scan_end = time.time()
            if hasattr(self, '_scan_start'):
                scan_time = (scan_end - self._scan_start) * 1000
                self.perf_scan_count += 1
                self.perf_scan_time_total += scan_time

    def octomap_pointcloud_callback(self, msg):
        """
        从PointCloud2消息中提取障碍物点
        通常订阅 /octomap_point_cloud_centers
        """
        if not self.odom_received:
            return

        obstacles = []
        
        # 解析PointCloud2消息 - 获取字段偏移
        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None:
            self.get_logger().warn('PointCloud2 missing x/y fields')
            return

        # 读取点
        point_step = msg.point_step
        data = msg.data
        
        for i in range(0, len(data), point_step):
            try:
                x = struct.unpack_from('f', data, i + x_offset)[0]
                y = struct.unpack_from('f', data, i + y_offset)[0]
                if z_offset is not None:
                    z = struct.unpack_from('f', data, i + z_offset)[0]
                else:
                    z = 0.0
                
                # 过滤高度范围（投影到2D平面）
                if z < self.octomap_height_min or z > self.octomap_height_max:
                    continue
                
                # 只考虑一定范围内的障碍物
                dist = math.sqrt(
                    (x - self.current_x)**2 + (y - self.current_y)**2)
                if dist > self.octomap_range:
                    continue
                
                obstacles.append((x, y))
            except (struct.error, IndexError):
                continue
        
        # 下采样（网格化）以减少点数
        if obstacles and self.octomap_resolution > 0:
            obstacles = self.downsample_octomap(obstacles)
        
        self.octomap_obstacles = obstacles
        
        # 合并所有障碍物源
        self.merge_obstacles()
        self.scan_received = True
        
        if len(obstacles) > 0:
            self.get_logger().info(
                f'OctoMap: {len(obstacles)} obstacles in range',
                throttle_duration_sec=2.0)

    def octomap_marker_callback(self, msg):
        """
        从MarkerArray消息中提取障碍物点
        通常订阅 /occupied_cells_vis_array
        """
        if not self.odom_received:
            return

        obstacles = []
        
        for marker in msg.markers:
            # 处理CUBE_LIST类型的marker（octomap_server常用格式）
            if marker.type == Marker.CUBE_LIST:
                for point in marker.points:
                    # 转换到全局坐标
                    x = marker.pose.position.x + point.x
                    y = marker.pose.position.y + point.y
                    z = marker.pose.position.z + point.z
                    
                    # 过滤高度范围
                    if z < self.octomap_height_min or z > self.octomap_height_max:
                        continue
                    
                    # 只考虑一定范围内的障碍物
                    dist = math.sqrt(
                        (x - self.current_x)**2 + (y - self.current_y)**2)
                    if dist > self.octomap_range:
                        continue
                    
                    obstacles.append((x, y))
            
            # 处理单个CUBE
            elif marker.type == Marker.CUBE:
                x = marker.pose.position.x
                y = marker.pose.position.y
                z = marker.pose.position.z
                
                if z < self.octomap_height_min or z > self.octomap_height_max:
                    continue
                
                dist = math.sqrt(
                    (x - self.current_x)**2 + (y - self.current_y)**2)
                if dist > self.octomap_range:
                    continue
                
                obstacles.append((x, y))
        
        # 下采样
        if obstacles and self.octomap_resolution > 0:
            obstacles = self.downsample_octomap(obstacles)
        
        self.octomap_obstacles = obstacles
        self.merge_obstacles()
        self.scan_received = True

    def downsample_octomap(self, obstacles):
        """
        使用网格下采样减少障碍物点数
        将3D体素投影到2D后合并相同网格的点
        """
        if not obstacles:
            return []
        
        res = self.octomap_resolution
        grid = {}
        
        for x, y in obstacles:
            # 网格化坐标
            gx = int(x / res)
            gy = int(y / res)
            key = (gx, gy)
            
            if key not in grid:
                # 使用网格中心作为代表点
                grid[key] = ((gx + 0.5) * res, (gy + 0.5) * res)
        
        return list(grid.values())

    def merge_obstacles(self):
        """
        合并所有障碍物源
        """
        all_obstacles = []
        
        # 直接添加的障碍物（最高优先级）
        all_obstacles.extend(self.direct_obstacles)
        
        # OctoMap障碍物
        if self.use_octomap:
            all_obstacles.extend(self.octomap_obstacles)
        
        self.obstacles = all_obstacles

    def goal_callback(self, msg):
        """目标点回调函数，支持odom和base_link坐标系"""
        if not self.odom_received:
            self.get_logger().warn(
                'ODOM not received yet, ignoring goal')
            return

        frame_id = msg.header.frame_id
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        # [DEBUG] 打印收到目标时的机器人状态
        self.get_logger().info(
            f'[DEBUG] 收到目标时机器人状态:\n'
            f'  位置: odom({self.current_x:.3f}, {self.current_y:.3f})\n'
            f'  朝向: yaw={math.degrees(self.current_yaw):.1f}°')

        # 根据坐标系处理目标点
        if frame_id == 'odom':
            self.goal_x = target_x
            self.goal_y = target_y
            self.get_logger().info(
                f'New goal received: odom({self.goal_x:.3f}, '
                f'{self.goal_y:.3f})')
        elif frame_id == 'base_link':
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            self.goal_x = (self.current_x + target_x * cos_yaw -
                           target_y * sin_yaw)
            self.goal_y = (self.current_y + target_x * sin_yaw +
                           target_y * cos_yaw)
            
            # [DEBUG] 详细打印坐标转换过程
            self.get_logger().info(
                f'[DEBUG] 坐标转换:\n'
                f'  输入: base_link({target_x:.3f}, {target_y:.3f})\n'
                f'  cos(yaw)={cos_yaw:.3f}, sin(yaw)={sin_yaw:.3f}\n'
                f'  输出: odom({self.goal_x:.3f}, {self.goal_y:.3f})')
            
            # [DEBUG] 计算预期行为
            expected_angle = math.atan2(
                self.goal_y - self.current_y,
                self.goal_x - self.current_x)
            angle_diff = self.normalize_angle(expected_angle - self.current_yaw)
            self.get_logger().info(
                f'[DEBUG] 预期行为:\n'
                f'  目标方向: {math.degrees(expected_angle):.1f}°\n'
                f'  当前朝向: {math.degrees(self.current_yaw):.1f}°\n'
                f'  角度差: {math.degrees(angle_diff):.1f}° '
                f'(阈值: {math.degrees(self.angle_threshold):.1f}°)\n'
                f'  {"需要先旋转" if abs(angle_diff) > self.angle_threshold else "可以直接前进"}')
        else:
            self.get_logger().warn(
                f'Unknown frame_id: {frame_id}, ignoring goal')
            return

        # 切换到旋转状态
        self.state = ControllerState.ROTATING
        self.stuck_count = 0
        self.last_x = self.current_x
        self.last_y = self.current_y
        
        # 重置方向锁定
        self.bypass_direction = 0
        self.bypass_lock_count = 0
        self.last_angular_z = 0.0
        self.last_turn_dir = 0
        self.last_distance = float('inf')
        self.initial_goal_angle = math.atan2(
            self.goal_y - self.current_y,
            self.goal_x - self.current_x)
        
        self.get_logger().info('Starting navigation...')

    def control_loop(self):
        """控制循环，根据当前状态执行相应的控制"""
        if self.debug_performance:
            import time
            loop_start = time.time()
            self.perf_loop_count += 1
            
        if not self.odom_received:
            return

        cmd_vel = Twist()

        if self.state == ControllerState.IDLE:
            # 空闲状态，等待目标点
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        elif self.state == ControllerState.ROTATING:
            # 检查目标点是否存在
            if self.goal_x is None or self.goal_y is None:
                self.get_logger().warn('No goal set, returning to IDLE')
                self.state = ControllerState.IDLE
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            else:
                # 旋转阶段：基于ODOM计算到目标点的朝向
                dx = self.goal_x - self.current_x
                dy = self.goal_y - self.current_y
                target_yaw = math.atan2(dy, dx)
                angle_error = self.normalize_angle(
                    target_yaw - self.current_yaw)

                # [DEBUG] 旋转阶段日志
                self.get_logger().info(
                    f'[ROTATING] 目标yaw={math.degrees(target_yaw):.1f}°, '
                    f'当前yaw={math.degrees(self.current_yaw):.1f}°, '
                    f'角度差={math.degrees(angle_error):.1f}° '
                    f'(阈值={math.degrees(self.angle_threshold):.1f}°)',
                    throttle_duration_sec=0.5)

                if abs(angle_error) > self.angle_threshold:
                    # 继续旋转
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = self.clamp(
                        2.0 * angle_error,
                        -self.dwa_planner.max_angular_vel,
                        self.dwa_planner.max_angular_vel
                    )
                    # [DEBUG] 旋转指令
                    self.get_logger().info(
                        f'[ROTATING] cmd_vel: linear=0.0, angular={cmd_vel.angular.z:.3f}',
                        throttle_duration_sec=0.5)
                else:
                    # 旋转完成，切换到移动状态
                    self.state = ControllerState.MOVING
                    self.get_logger().info(
                        f'[DEBUG] 旋转完成! 角度差={math.degrees(angle_error):.1f}° < 阈值')
                    self.get_logger().info(
                        'Rotation complete, starting movement with DWA')
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0

        elif self.state == ControllerState.MOVING:
            # 检查目标点是否存在
            if self.goal_x is None or self.goal_y is None:
                self.get_logger().warn('No goal set, returning to IDLE')
                self.state = ControllerState.IDLE
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            else:
                # 移动阶段：使用DWA或简单控制
                dx = self.goal_x - self.current_x
                dy = self.goal_y - self.current_y
                distance = math.sqrt(dx**2 + dy**2)

                if distance > self.distance_threshold:
                    # 卡住检测
                    moved = math.sqrt(
                        (self.current_x - self.last_x)**2 +
                        (self.current_y - self.last_y)**2)
                    if moved < self.stuck_move_threshold:  # 几乎没动
                        self.stuck_count += 1
                    else:
                        self.stuck_count = 0
                        self.last_x = self.current_x
                        self.last_y = self.current_y

                    # 紧急停止检测：如果障碍物太近，停止前进但允许旋转脱困
                    is_emergency = self.check_emergency_stop()
                    self._decision_path = 'normal'  # 默认路径
                    if is_emergency:
                        self._decision_path = 'EMERGENCY'
                        self.emergency_count = getattr(self, 'emergency_count', 0) + 1
                        self.get_logger().info(
                            f'[DEBUG] Emergency检测=True, 最近障碍物边缘距离={self.min_obstacle_dist:.3f}m (阈值0.05m)',
                            throttle_duration_sec=0.5)
                        
                        # 如果旋转超过30个周期（约3秒）还没脱困，尝试后退
                        if self.emergency_count > 30:
                            # 检查后方是否有障碍
                            back_clear = self.check_back_clear()
                            if back_clear:
                                cmd_vel.linear.x = -0.1  # 后退
                                cmd_vel.angular.z = 0.0
                                self.get_logger().warn(
                                    f'Emergency! Backing up to escape (count={self.emergency_count})',
                                    throttle_duration_sec=0.5)
                            else:
                                # 后方也有障碍，继续旋转
                                cmd_vel.linear.x = 0.0
                                if self.last_turn_dir == 0:
                                    self.last_turn_dir = 1
                                cmd_vel.angular.z = self.last_turn_dir * 0.8
                                self.get_logger().warn(
                                    f'Emergency! Trapped, rotating (count={self.emergency_count})',
                                    throttle_duration_sec=0.5)
                        else:
                            # 先尝试旋转脱困
                            cmd_vel.linear.x = 0.0
                            if self.last_turn_dir == 0:
                                escape_direction = self.get_escape_direction()
                                self.last_turn_dir = escape_direction if escape_direction != 0 else 1
                            
                            cmd_vel.angular.z = self.last_turn_dir * 0.8
                            self.get_logger().warn(
                                f'Emergency! Rotating to escape '
                                f'({"LEFT" if self.last_turn_dir > 0 else "RIGHT"})',
                                throttle_duration_sec=0.5)
                    else:
                        # 脱离Emergency状态，重置计数
                        self.emergency_count = 0
                    
                    # 如果是紧急状态，跳过后续所有逻辑（防止被覆盖）
                    if is_emergency:
                        # 已经在上面处理过了，直接跳过
                        self.get_logger().debug('[DEBUG] 紧急状态，跳过DWA/Simple模式')
                    # 卡住恢复：只旋转，不前进（避免穿墙）
                    elif self.stuck_count > self.stuck_threshold:
                        # 如果已有方向锁定，使用锁定方向；否则选择新方向
                        if self.bypass_direction != 0:
                            direction = self.bypass_direction
                            # 延长锁定时间
                            self.bypass_lock_count = max(
                                self.bypass_lock_count, 
                                self.bypass_lock_threshold)
                        else:
                            direction = self.choose_recovery_direction()
                            # 锁定这个方向
                            self.bypass_direction = direction
                            self.bypass_lock_count = self.bypass_lock_threshold * 2
                            self.get_logger().info(
                                f'[Recovery] 锁定恢复方向: {"左" if direction > 0 else "右"}')
                        
                        # 纯旋转脱困（避免打滑）
                        self._decision_path = 'STUCK_RECOVERY'
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = direction * 0.6  # 温和转向
                        self.last_angular_z = cmd_vel.angular.z  # 更新平滑器
                        
                        self.get_logger().warn(
                            f'Stuck! Recovery: {"LEFT" if direction > 0 else "RIGHT"}, '
                            f'纯旋转脱困, lock={self.bypass_lock_count}',
                            throttle_duration_sec=1.0)
                        # 重置卡住计数（但不换方向，保持锁定）
                        if self.stuck_count > self.stuck_threshold + 50:
                            self.stuck_count = 0
                    elif self.use_dwa and self.scan_received:
                        # === 检查是否前方完全无障碍物 ===
                        # 如果前方2m内无障碍物，直接用简单比例控制
                        front_clear_distance = self.get_front_clear_distance()
                        # 只检查前方是否畅通，不管侧面有多少障碍物
                        if front_clear_distance > 2.0:
                            # 前方畅通，使用纯旋转+纯直行模式（避免打滑）
                            target_yaw = math.atan2(dy, dx)
                            heading_error = self.normalize_angle(
                                target_yaw - self.current_yaw)
                            
                            # 纯运动模式：先旋转对准，再直行
                            angle_threshold = self.get_parameter('pure_motion_angle_threshold').value
                            if abs(heading_error) > angle_threshold:
                                # 角度偏差大，纯旋转（不前进）
                                self._decision_path = 'STRAIGHT_ROTATE'
                                cmd_vel.linear.x = 0.0
                                cmd_vel.angular.z = self.clamp(
                                    2.0 * heading_error, -0.8, 0.8)
                                self.get_logger().info(
                                    f'[直行模式-旋转] 角度偏差={math.degrees(heading_error):.1f}°, '
                                    f'w={cmd_vel.angular.z:.2f}',
                                    throttle_duration_sec=0.5)
                            else:
                                # 角度偏差小，纯直行（不转向）
                                self._decision_path = 'STRAIGHT_FORWARD'
                                cmd_vel.linear.x = min(0.5, 0.5 * distance)
                                cmd_vel.angular.z = 0.0
                                self.get_logger().info(
                                    f'[直行模式-直行] 前方畅通({front_clear_distance:.1f}m), '
                                    f'v={cmd_vel.linear.x:.2f}, dist={distance:.2f}',
                                    throttle_duration_sec=0.5)
                            
                            # 重置绕行状态
                            if self.bypass_direction != 0:
                                self.bypass_direction = 0
                                self.bypass_lock_count = 0
                        else:
                            # 使用DWA规划
                            if self.debug_performance:
                                import time
                                dwa_start = time.time()
                                
                            result = self.dwa_planner.plan(
                                self.current_x,
                                self.current_y,
                                self.current_yaw,
                                self.current_v,
                                self.current_w,
                                self.goal_x,
                                self.goal_y,
                                self.obstacles
                            )
                            best_v, best_w, trajectory, valid_count = result
                            
                            if self.debug_performance:
                                dwa_time = (time.time() - dwa_start) * 1000
                                self.perf_dwa_count += 1
                                self.perf_dwa_time_total += dwa_time

                            # === 方向锁定机制：防止左右震荡 ===
                            # 检测是否需要锁定方向
                            if abs(best_w) > 0.3:  # 有明显转向
                                if self.bypass_direction == 0:
                                    # 首次选择方向，使用智能分析选择最优绕行方向
                                    self.bypass_direction = self.choose_bypass_direction()
                                    self.bypass_lock_count = self.bypass_lock_threshold
                                    self.get_logger().info(
                                        f'[方向锁定] 智能选择: {"左绕" if self.bypass_direction > 0 else "右绕"}')
                                elif self.bypass_lock_count > 0:
                                    # 方向已锁定，强制使用锁定方向
                                    if (best_w > 0 and self.bypass_direction < 0) or \
                                       (best_w < 0 and self.bypass_direction > 0):
                                        # DWA想换方向，但我们强制保持
                                        best_w = self.bypass_direction * abs(best_w)
                                        self.get_logger().info(
                                            f'[方向锁定] 强制保持{"左绕" if self.bypass_direction > 0 else "右绕"}, '
                                            f'剩余{self.bypass_lock_count}周期',
                                            throttle_duration_sec=1.0)
                            
                            # 计算最近障碍物距离
                            self.min_obstacle_dist = float('inf')
                            for ox, oy in self.obstacles:
                                obs_dist = math.sqrt(
                                    (ox - self.current_x)**2 + 
                                    (oy - self.current_y)**2)
                                self.min_obstacle_dist = min(
                                    self.min_obstacle_dist, obs_dist)
                            
                            # 检测是否已绑过障碍物
                            # 条件：距离在减少 且 前方清晰 且 远离障碍物
                            bypass_complete = False
                            if self.bypass_direction != 0:
                                # 检查是否已绑过：距离减少 + 障碍物不在前方
                                if distance < self.last_distance - 0.05 and \
                                   self.min_obstacle_dist > 0.8 and \
                                   self.check_front_clear():
                                    # 快速减少锁定计数
                                    self.bypass_lock_count = max(0, self.bypass_lock_count - 3)
                                    if self.bypass_lock_count < 10:
                                        bypass_complete = True
                            
                            # 锁定计数递减
                            if self.bypass_lock_count > 0:
                                self.bypass_lock_count -= 1
                                
                                # 如果靠近障碍物，延长锁定
                                if self.min_obstacle_dist < 0.8:
                                    self.bypass_lock_count = max(
                                        self.bypass_lock_count, 15)
                                
                                if self.bypass_lock_count == 0 or bypass_complete:
                                    # 解除锁定
                                    if self.min_obstacle_dist > 0.8 or bypass_complete:
                                        self.bypass_direction = 0
                                        self.get_logger().info(
                                            f'[方向锁定] 解除锁定 (dist={distance:.2f}, obs={self.min_obstacle_dist:.1f}m)')
                                    else:
                                        self.bypass_lock_count = 15
                                        self.get_logger().info(
                                            f'[方向锁定] 障碍物较近({self.min_obstacle_dist:.1f}m)，保持锁定')
                            
                            # 更新距离记录
                            self.last_distance = distance
                            
                            # === 角速度平滑：防止剧烈震荡 ===
                            max_angular_change = 0.5  # 每周期最大角速度变化
                            angular_diff = best_w - self.last_angular_z
                            if abs(angular_diff) > max_angular_change:
                                best_w = self.last_angular_z + \
                                    max_angular_change * (1 if angular_diff > 0 else -1)
                            self.last_angular_z = best_w

                            # === 绕行时强制最小前进速度 ===
                            if self.bypass_direction != 0 and self.check_front_clear():
                                # 在绕行模式且前方清晰时，强制最小前进速度
                                min_bypass_vel = 0.1
                                if best_v < min_bypass_vel:
                                    best_v = min_bypass_vel
                                    self.get_logger().info(
                                        f'[绕行] 强制最小前进速度 {min_bypass_vel}',
                                        throttle_duration_sec=1.0)
                            
                            # === 纯运动模式：仅在无障碍时启用 ===
                            pure_motion = self.get_parameter('pure_motion_mode').value
                            motion_mode = "DWA"
                            
                            # 获取最小速度阈值（用于后续判断）
                            min_v_for_motion = self.get_parameter('min_linear_velocity').value
                            
                            # 检查前方是否有障碍物（决定是否使用纯运动模式）
                            # 注意：如果正在绕行（bypass_direction != 0），不要切换到直行模式
                            front_clear_for_pure = self.min_obstacle_dist > 1.0 and self.bypass_direction == 0
                            
                            if pure_motion and front_clear_for_pure:
                                # 无障碍：使用纯运动模式
                                target_yaw = math.atan2(dy, dx)
                                heading_error_raw = self.normalize_angle(target_yaw - self.current_yaw)
                                heading_error = abs(heading_error_raw)
                                angle_threshold = self.get_parameter('pure_motion_angle_threshold').value
                                
                                if heading_error > angle_threshold:
                                    # 航向偏差大 → 纯旋转
                                    cmd_vel.linear.x = 0.0
                                    # 如果有避障锁定，跟随避障方向；否则用航向偏差
                                    if self.bypass_direction != 0:
                                        self.last_turn_dir = self.bypass_direction
                                    elif self.last_turn_dir == 0:
                                        self.last_turn_dir = 1 if heading_error_raw > 0 else -1
                                    # 使用配置的纯旋转速度（提升效率）
                                    pure_rot_speed = self.get_parameter('pure_rotation_speed').value
                                    cmd_vel.angular.z = self.last_turn_dir * pure_rot_speed
                                    motion_mode = f"纯旋转(偏差{math.degrees(heading_error):.0f}°,{'左' if self.last_turn_dir > 0 else '右'})"
                                else:
                                    # 航向偏差小 → 纯直行，重置方向锁
                                    self.last_turn_dir = 0
                                    cmd_vel.linear.x = best_v if best_v > min_v_for_motion else 0.2
                                    cmd_vel.angular.z = 0.0
                                    motion_mode = "纯直行"
                            else:
                                # 有障碍：使用DWA避障
                                # === 优化后的防打滑逻辑 ===
                                # 使用实车测试的最小速度值作为阈值
                                min_v_threshold = self.get_parameter('min_linear_velocity').value
                                min_w_threshold = self.get_parameter('min_angular_velocity').value
                                pure_rot_speed = self.get_parameter('pure_rotation_speed').value
                                
                                # 检查前方是否畅通
                                front_clear = self.check_front_clear()
                                
                                # === 新的防打滑策略：只在极端情况下强制纯运动 ===
                                # 1. 极低速+转向 → 纯旋转（真正的打滑风险）
                                if best_v < min_v_threshold and abs(best_w) > min_w_threshold:
                                    cmd_vel.linear.x = 0.0
                                    if self.last_turn_dir == 0:
                                        if self.bypass_direction != 0:
                                            self.last_turn_dir = self.bypass_direction
                                        else:
                                            self.last_turn_dir = 1 if best_w >= 0 else -1
                                    cmd_vel.angular.z = self.last_turn_dir * max(pure_rot_speed, abs(best_w))
                                    motion_mode = "DWA-防滑纯旋转"
                                
                                # 2. 极低速+低角速度 → 根据前方决定
                                elif best_v < min_v_threshold and abs(best_w) < min_w_threshold:
                                    if front_clear:
                                        self.last_turn_dir = 0
                                        cmd_vel.linear.x = 0.15
                                        cmd_vel.angular.z = 0.0
                                        motion_mode = "DWA-前方畅通直行"
                                    else:
                                        cmd_vel.linear.x = 0.0
                                        if self.last_turn_dir == 0:
                                            if self.bypass_direction != 0:
                                                self.last_turn_dir = self.bypass_direction
                                            else:
                                                self.last_turn_dir = 1 if best_w >= 0 else -1
                                        cmd_vel.angular.z = self.last_turn_dir * pure_rot_speed
                                        motion_mode = "DWA-避障旋转"
                                
                                # 3. 中低速（min_v ~ 0.2） + 转向 → 允许组合运动（不强制纯旋转）
                                elif best_v >= min_v_threshold and best_v < 0.2 and abs(best_w) > 0.1:
                                    # ✅ 关键改动：允许 DWA 的组合运动（0.15m/s + 0.5rad/s 是安全的）
                                    cmd_vel.linear.x = best_v
                                    cmd_vel.angular.z = best_w
                                    motion_mode = "DWA-组合(中低速)"
                                
                                # 4. 纯直行场景
                                elif best_v >= min_v_threshold and abs(best_w) < min_w_threshold:
                                    cmd_vel.linear.x = best_v
                                    cmd_vel.angular.z = 0.0
                                    motion_mode = "DWA-纯直行"
                                
                                # 5. 正常速度组合运动
                                else:
                                    cmd_vel.linear.x = best_v
                                    cmd_vel.angular.z = best_w
                                    motion_mode = "DWA-组合"

                            # 记录决策路径
                            self._decision_path = motion_mode
                            
                            # 发布规划轨迹
                            if trajectory is not None:
                                self.publish_trajectory(trajectory)

                            if distance > 0.2:
                                bypass_str = ""
                                if self.bypass_direction != 0:
                                    bypass_str = f', 锁定:{"左" if self.bypass_direction > 0 else "右"}'
                                self.get_logger().info(
                                    f'DWA: v={best_v:.2f}, w={best_w:.2f} [{motion_mode}], '
                                    f'dist={distance:.2f}, '
                                    f'obs={len(self.obstacles)}, '
                                    f'valid={valid_count}{bypass_str}',
                                    throttle_duration_sec=1.0
                                )
                    else:
                        # 简单控制（无避障）- 纯运动模式
                        target_yaw = math.atan2(dy, dx)
                        heading_error = self.normalize_angle(
                            target_yaw - self.current_yaw)
                        
                        angle_threshold = self.get_parameter('pure_motion_angle_threshold').value
                        if abs(heading_error) > angle_threshold:
                            # 角度偏差大，纯旋转
                            cmd_vel.linear.x = 0.0
                            self._decision_path = 'SIMPLE_ROTATE'
                            cmd_vel.angular.z = self.clamp(
                                2.0 * heading_error,
                                -0.8, 0.8
                            )
                            self.get_logger().info(
                                f'[Simple-旋转] 角度偏差={math.degrees(heading_error):.1f}°, '
                                f'w={cmd_vel.angular.z:.2f}',
                                throttle_duration_sec=0.5)
                        else:
                            # 角度偏差小，纯直行
                            cmd_vel.linear.x = self.clamp(
                                0.5 * distance,
                                0.0,
                                self.dwa_planner.max_linear_vel
                            )
                            self._decision_path = 'SIMPLE_FORWARD'
                            cmd_vel.angular.z = 0.0
                            self.get_logger().info(
                                f'[Simple-直行] dist={distance:.2f}m, '
                                f'v={cmd_vel.linear.x:.2f}',
                                throttle_duration_sec=0.5)
                else:
                    # 移动完成
                    self.state = ControllerState.REACHED
                    self.get_logger().info(
                        'Movement complete, goal reached!')
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0

        elif self.state == ControllerState.REACHED:
            # 到达目标，停止并等待3秒
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

            if self.reach_time is None:
                self.reach_time = self.get_clock().now()
                self.get_logger().info(
                    'Goal reached! Waiting for 3 seconds...')

            elapsed = (self.get_clock().now() -
                       self.reach_time).nanoseconds / 1e9
            if elapsed >= self.wait_duration:
                self.state = ControllerState.IDLE
                self.reach_time = None
                self.get_logger().info('Ready for next goal')

        # [DEBUG] 最终发布的速度命令（包含决策路径）
        if self.state != ControllerState.IDLE and self.state != ControllerState.REACHED:
            # 决策路径标识
            decision_path = getattr(self, '_decision_path', 'unknown')
            self.get_logger().info(
                f'[CMD_VEL] state={self.state.name}, '
                f'linear.x={cmd_vel.linear.x:.3f}, '
                f'angular.z={cmd_vel.angular.z:.3f}, '
                f'path={decision_path}',
                throttle_duration_sec=0.3)

        # 应用最小速度阈值（克服电机死区）
        cmd_vel = self.apply_min_velocity(cmd_vel)
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)

        # 发布状态
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        
        # 性能统计报告（每5秒输出一次）
        if self.debug_performance:
            import time
            now = time.time()
            if self.perf_last_report is None:
                self.perf_last_report = now
            elif now - self.perf_last_report >= 5.0:
                avg_scan = (self.perf_scan_time_total / self.perf_scan_count) if self.perf_scan_count > 0 else 0
                avg_dwa = (self.perf_dwa_time_total / self.perf_dwa_count) if self.perf_dwa_count > 0 else 0
                loop_hz = self.perf_loop_count / 5.0
                self.get_logger().info(
                    f'[PERF] 5s统计: Loop={loop_hz:.1f}Hz, '
                    f'Scan={avg_scan:.1f}ms(n={self.perf_scan_count}), '
                    f'DWA={avg_dwa:.1f}ms(n={self.perf_dwa_count}), '
                    f'Obstacles={len(self.obstacles)}')
                # 重置统计
                self.perf_scan_count = 0
                self.perf_scan_time_total = 0.0
                self.perf_dwa_count = 0
                self.perf_dwa_time_total = 0.0
                self.perf_loop_count = 0
                self.perf_last_report = now

    def check_emergency_stop(self):
        """检查是否需要紧急停止（障碍物太近）"""
        obstacle_radius = 0.15
        # 边缘距离小于5cm就紧急停止
        emergency_edge_dist = 0.05
        
        min_edge_dist = float('inf')
        emergency_obstacle = None
        emergency_dist = None
        
        for ox, oy in self.obstacles:
            center_dist = math.sqrt(
                (ox - self.current_x)**2 + (oy - self.current_y)**2)
            edge_dist = center_dist - self.dwa_planner.robot_radius - obstacle_radius
            if edge_dist < min_edge_dist:
                min_edge_dist = edge_dist
            if edge_dist < emergency_edge_dist and emergency_obstacle is None:
                # 记录第一个触发emergency的障碍物
                emergency_obstacle = (ox, oy)
                emergency_dist = edge_dist
        
        # 记录最近障碍物信息（用于调试）
        self.min_obstacle_dist = min_edge_dist
        
        if emergency_obstacle is not None:
            self.get_logger().debug(
                f'[Emergency] 触发! 障碍物({emergency_obstacle[0]:.2f},{emergency_obstacle[1]:.2f}) '
                f'边缘距离={emergency_dist:.3f}m < {emergency_edge_dist}m')
            return True
        return False
    
    def get_escape_direction(self):
        """计算脱困旋转方向：分析障碍物分布，选择空旷的方向"""
        obstacle_radius = 0.15
        emergency_dist = self.dwa_planner.robot_radius + obstacle_radius + 0.1
        
        left_danger = 0.0  # 左前方危险程度
        right_danger = 0.0  # 右前方危险程度
        
        cos_yaw = math.cos(-self.current_yaw)
        sin_yaw = math.sin(-self.current_yaw)
        
        for ox, oy in self.obstacles:
            dx = ox - self.current_x
            dy = oy - self.current_y
            dist = math.sqrt(dx**2 + dy**2)
            
            # 只考虑近距离障碍物
            if dist > emergency_dist * 2:
                continue
            
            # 转换到机器人坐标系
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw
            
            # 只考虑前方和侧前方的障碍物
            if local_x > -0.2:
                weight = 1.0 / max(dist, 0.1)
                if local_y > 0:  # 左前方
                    left_danger += weight
                else:  # 右前方
                    right_danger += weight
        
        # 选择危险较小的方向
        if left_danger < right_danger:
            return 1  # 左转
        elif right_danger < left_danger:
            return -1  # 右转
        elif self.bypass_direction != 0:
            return self.bypass_direction  # 保持当前绕行方向
        else:
            return 1  # 默认左转

    def check_front_clear(self):
        """检查前方是否有障碍物（用于恢复模式）"""
        obstacle_radius = 0.15
        check_distance = 0.5  # 检查前方0.5m
        path_width = self.dwa_planner.robot_radius + obstacle_radius + 0.1
        
        for ox, oy in self.obstacles:
            # 转换到机器人坐标系
            dx = ox - self.current_x
            dy = oy - self.current_y
            local_x = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            local_y = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            
            # 检查前方矩形区域
            if local_x > -obstacle_radius and local_x < check_distance:
                # 在前方，检查是否在路径上
                if abs(local_y) < path_width:
                    return False  # 前方有障碍
        
        return True  # 前方清晰
    
    def check_back_clear(self):
        """检查后方是否有障碍物（用于后退脱困）"""
        obstacle_radius = 0.15
        check_distance = 0.5  # 检查后方0.5m
        path_width = self.dwa_planner.robot_radius + obstacle_radius + 0.1
        
        for ox, oy in self.obstacles:
            # 转换到机器人坐标系
            dx = ox - self.current_x
            dy = oy - self.current_y
            local_x = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            local_y = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            
            # 检查后方矩形区域（local_x < 0）
            if local_x < obstacle_radius and local_x > -check_distance:
                # 在后方，检查是否在路径上
                if abs(local_y) < path_width:
                    return False  # 后方有障碍
        
        return True  # 后方清晰
    
    def get_front_clear_distance(self):
        """获取前方最近障碍物的距离"""
        obstacle_radius = 0.15
        path_width = 1.0  # 检测前方1米宽的通道
        min_distance = float('inf')
        
        cos_yaw = math.cos(-self.current_yaw)
        sin_yaw = math.sin(-self.current_yaw)
        
        for ox, oy in self.obstacles:
            # 转换到机器人坐标系
            dx = ox - self.current_x
            dy = oy - self.current_y
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw
            
            # 只考虑前方的障碍物（在路径宽度内）
            if local_x > 0 and abs(local_y) < path_width:
                # 计算到障碍物边缘的距离
                edge_dist = local_x - obstacle_radius
                min_distance = min(min_distance, edge_dist)
        
        return min_distance

    def choose_bypass_direction(self):
        """智能选择绕行方向 - 分析障碍物分布，选择更短的路径"""
        if len(self.obstacles) == 0:
            return 1  # 默认左转
        
        # 计算目标相对于当前位置的方向
        goal_dx = self.goal_x - self.current_x
        goal_dy = self.goal_y - self.current_y
        
        # 转换到机器人坐标系，分析障碍物分布
        left_obstacles = []  # 左边的障碍物 (local_y > 0)
        right_obstacles = []  # 右边的障碍物 (local_y < 0)
        
        cos_yaw = math.cos(-self.current_yaw)
        sin_yaw = math.sin(-self.current_yaw)
        
        for ox, oy in self.obstacles:
            dx = ox - self.current_x
            dy = oy - self.current_y
            dist = math.sqrt(dx**2 + dy**2)
            
            # 只考虑前方 3m 内的障碍物
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw
            
            if local_x > 0 and dist < 3.0:
                if local_y > 0:
                    left_obstacles.append((local_x, local_y, dist))
                else:
                    right_obstacles.append((local_x, local_y, dist))
        
        # 计算每侧的"绕行代价"
        # 代价 = 需要绕行的距离（障碍物的边界位置）
        left_boundary = 0.0  # 左边障碍物的最远y值
        right_boundary = 0.0  # 右边障碍物的最远y值（取绝对值）
        
        for lx, ly, _ in left_obstacles:
            if lx < 2.0:  # 只考虑近处的
                left_boundary = max(left_boundary, ly)
        
        for lx, ly, _ in right_obstacles:
            if lx < 2.0:
                right_boundary = max(right_boundary, abs(ly))
        
        # 加入目标偏好：如果目标偏向某一侧，给那一侧加分
        goal_local_y = goal_dx * sin_yaw + goal_dy * cos_yaw
        goal_bias = 0.3  # 目标偏好权重
        
        # 最终代价 = 绕行距离 - 目标偏好
        left_cost = left_boundary
        right_cost = right_boundary
        
        if goal_local_y > 0:  # 目标偏左
            left_cost -= goal_bias
        else:  # 目标偏右
            right_cost -= goal_bias
        
        self.get_logger().info(
            f'[绕行分析] 左边界={left_boundary:.2f}m, 右边界={right_boundary:.2f}m, '
            f'目标偏{"左" if goal_local_y > 0 else "右"} -> '
            f'选择{"左绕" if left_cost < right_cost else "右绕"}')
        
        # 选择代价更低的方向
        if left_cost < right_cost:
            return 1  # 左绕（逆时针）
        else:
            return -1  # 右绕（顺时针）
    
    def choose_recovery_direction(self):
        """选择恢复转向的方向 - 使用智能绕行分析"""
        return self.choose_bypass_direction()

    def publish_trajectory(self, trajectory):
        """发布DWA规划的轨迹"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for x, y, yaw in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # 四元数
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        self.trajectory_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = DWANavController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        cmd_vel = Twist()
        controller.cmd_vel_pub.publish(cmd_vel)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
