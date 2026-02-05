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

        # 检查整条轨迹的每个点
        for tx, ty, _ in trajectory:
            for ox, oy in obstacles:
                # 机器人边缘到障碍物边缘的距离
                center_dist = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                edge_dist = center_dist - self.robot_radius - obstacle_radius
                if edge_dist < min_dist:
                    min_dist = edge_dist

        # 碰撞检测：边缘距离小于0就是碰撞
        if min_dist < 0.05:  # 5cm缓冲
            return float('inf')

        # 安全距离内增加代价
        if min_dist < self.safe_distance:
            return 3.0 / max(min_dist, 0.01)

        # 距离越近代价越高
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
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('max_linear_acc', 0.2)
        self.declare_parameter('max_angular_acc', 1.0)
        self.declare_parameter('control_frequency', 10.0)
        # Bunker Mini 尺寸: 690 x 570 x 335 mm
        # robot_radius = sqrt((0.345)² + (0.285)²) ≈ 0.45m (外接圆半径)
        self.declare_parameter('robot_radius', 0.45)
        self.declare_parameter('safe_distance', 0.2)
        self.declare_parameter('use_dwa', True)  # 是否启用DWA避障
        self.declare_parameter('use_octomap', False)  # 是否使用OctoMap障碍物
        self.declare_parameter('octomap_topic', '/octomap_point_cloud_centers')  # OctoMap点云话题
        self.declare_parameter('octomap_height_min', -0.1)  # 提取障碍物的最小高度
        self.declare_parameter('octomap_height_max', 1.0)  # 提取障碍物的最大高度
        self.declare_parameter('octomap_resolution', 0.1)  # 下采样分辨率
        self.declare_parameter('octomap_range', 5.0)  # 只处理此范围内的障碍物

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

        # DWA配置（优化绕行能力）
        dwa_config = {
            'max_linear_vel': max_linear_vel,
            'min_linear_vel': 0.0,
            'max_angular_vel': max(max_angular_vel, 2.5),  # 至少2.5 rad/s
            'max_linear_acc': self.get_parameter('max_linear_acc').value,
            'max_angular_acc': 5.0,  # 更大的角加速度，快速转向
            'dt': 0.1,
            'predict_time': 2.0,  # 缩短预测时间，更灵活
            'linear_samples': 10,
            'angular_samples': 50,  # 更多角速度采样
            'goal_cost_weight': 1.0,  # 目标权重
            'obstacle_cost_weight': 1.0,  # 降低障碍物权重
            'velocity_cost_weight': 0.1,  # 速度权重
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

        # 目标点（odom坐标系）
        self.goal_x = None
        self.goal_y = None

        # 到达目标后的停留时间
        self.reach_time = None
        self.wait_duration = 3.0

        # 卡住检测
        self.stuck_count = 0
        self.last_x = 0.0
        self.last_y = 0.0
        self.stuck_threshold = 10  # 连续10个周期没动就认为卡住了
        self.recovery_direction = 1  # 1=左转, -1=右转

        self.get_logger().info('DWA NavController initialized')
        self.get_logger().info(
            f'DWA Avoidance: {"ENABLED" if self.use_dwa else "DISABLED"}')
        self.get_logger().info(
            f'OctoMap Mode: {"ENABLED" if self.use_octomap else "DISABLED"}')

    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, x, y, z, w):
        """从四元数转换为yaw角"""
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        return yaw

    def clamp(self, value, min_value, max_value):
        """限制数值范围"""
        return max(min_value, min(value, max_value))

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
        if not self.odom_received:
            return

        # 如果有直接障碍物，使用直接障碍物（更可靠）
        if self.direct_obstacles:
            self.obstacles = self.direct_obstacles.copy()
            self.scan_received = True
            return

        obstacles = []
        angle = msg.angle_min

        for i, r in enumerate(msg.ranges):
            # 跳过无效数据
            if r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            # 限制检测范围（只考虑前方和侧方）
            if r > 3.0:  # 只考虑3米内的障碍物
                angle += msg.angle_increment
                continue

            # 转换到base_link坐标系
            x_base = r * math.cos(angle)
            y_base = r * math.sin(angle)

            # 转换到odom坐标系
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            x_odom = self.current_x + x_base * cos_yaw - y_base * sin_yaw
            y_odom = self.current_y + x_base * sin_yaw + y_base * cos_yaw

            obstacles.append((x_odom, y_odom))
            angle += msg.angle_increment

        self.obstacles = obstacles
        self.scan_received = True

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
            self.get_logger().info(
                f'New goal received: base_link({target_x:.3f}, '
                f'{target_y:.3f}) -> odom({self.goal_x:.3f}, '
                f'{self.goal_y:.3f})')
        else:
            self.get_logger().warn(
                f'Unknown frame_id: {frame_id}, ignoring goal')
            return

        # 切换到旋转状态
        self.state = ControllerState.ROTATING
        self.stuck_count = 0
        self.last_x = self.current_x
        self.last_y = self.current_y
        self.get_logger().info('Starting navigation...')

    def control_loop(self):
        """控制循环，根据当前状态执行相应的控制"""
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

                if abs(angle_error) > self.angle_threshold:
                    # 继续旋转
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = self.clamp(
                        2.0 * angle_error,
                        -self.dwa_planner.max_angular_vel,
                        self.dwa_planner.max_angular_vel
                    )
                else:
                    # 旋转完成，切换到移动状态
                    self.state = ControllerState.MOVING
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
                    if moved < 0.02:  # 几乎没动
                        self.stuck_count += 1
                    else:
                        self.stuck_count = 0
                        self.last_x = self.current_x
                        self.last_y = self.current_y

                    # 紧急停止检测：如果障碍物太近，停止
                    if self.check_emergency_stop():
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                        self.get_logger().warn(
                            'Emergency stop! Obstacle too close!',
                            throttle_duration_sec=1.0)
                    # 卡住恢复：只旋转，不前进（避免穿墙）
                    elif self.stuck_count > self.stuck_threshold:
                        # 选择绕行方向（基于障碍物位置）
                        direction = self.choose_recovery_direction()
                        
                        # 检查前方是否有障碍物
                        front_clear = self.check_front_clear()
                        
                        if front_clear:
                            cmd_vel.linear.x = 0.05  # 前方无障碍，可以缓慢前进
                        else:
                            cmd_vel.linear.x = 0.0  # 前方有障碍，只旋转
                        
                        cmd_vel.angular.z = direction * 1.5  # 强力转向
                        self.get_logger().warn(
                            f'Stuck! Recovery: {"LEFT" if direction > 0 else "RIGHT"}, '
                            f'front={"clear" if front_clear else "blocked"}',
                            throttle_duration_sec=1.0)
                        # 重置卡住计数
                        if self.stuck_count > self.stuck_threshold + 30:
                            self.stuck_count = 0
                            self.recovery_direction *= -1  # 换方向
                    elif self.use_dwa and self.scan_received:
                        # 使用DWA规划
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

                        cmd_vel.linear.x = best_v
                        cmd_vel.angular.z = best_w

                        # 发布规划轨迹
                        if trajectory is not None:
                            self.publish_trajectory(trajectory)

                        if distance > 0.2:
                            self.get_logger().info(
                                f'DWA: v={best_v:.2f}, w={best_w:.2f}, '
                                f'dist={distance:.2f}, '
                                f'obs={len(self.obstacles)}, '
                                f'valid={valid_count}',
                                throttle_duration_sec=1.0
                            )
                    else:
                        # 简单控制（无避障）
                        cmd_vel.linear.x = self.clamp(
                            0.5 * distance,
                            0.0,
                            self.dwa_planner.max_linear_vel
                        )

                        target_yaw = math.atan2(dy, dx)
                        heading_error = self.normalize_angle(
                            target_yaw - self.current_yaw)
                        cmd_vel.angular.z = self.clamp(
                            2.0 * heading_error,
                            -self.dwa_planner.max_angular_vel * 0.5,
                            self.dwa_planner.max_angular_vel * 0.5
                        )
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

        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)

        # 发布状态
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

    def check_emergency_stop(self):
        """检查是否需要紧急停止（障碍物太近）"""
        obstacle_radius = 0.15
        # 边缘距离小于5cm就紧急停止
        emergency_edge_dist = 0.05
        
        for ox, oy in self.obstacles:
            center_dist = math.sqrt(
                (ox - self.current_x)**2 + (oy - self.current_y)**2)
            edge_dist = center_dist - self.dwa_planner.robot_radius - obstacle_radius
            if edge_dist < emergency_edge_dist:
                return True
        return False

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

    def choose_recovery_direction(self):
        """选择恢复转向的方向（选择障碍物较少的一侧）"""
        left_weight = 0.0
        right_weight = 0.0
        
        for ox, oy in self.obstacles:
            # 转换到机器人坐标系
            dx = ox - self.current_x
            dy = oy - self.current_y
            local_x = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            local_y = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            
            # 只考虑前方障碍物
            if local_x > 0 and local_x < 2.0:
                dist = math.sqrt(local_x**2 + local_y**2)
                weight = 1.0 / max(dist, 0.1)
                if local_y > 0:  # 左边有障碍物
                    left_weight += weight
                else:  # 右边有障碍物
                    right_weight += weight
        
        # 选择障碍物较少的方向
        if left_weight < right_weight:
            return 1  # 左转
        elif right_weight < left_weight:
            return -1  # 右转
        else:
            return self.recovery_direction  # 保持之前的方向

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
