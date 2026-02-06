#!/usr/bin/env python3
"""
简单的2D可视化工具
实时显示机器人位置、轨迹、目标点和障碍物

支持自适应分辨率显示
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import matplotlib
matplotlib.use('TkAgg')  # 使用TkAgg后端支持窗口调整
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection, LineCollection
import math
import numpy as np


def get_screen_dpi():
    """获取屏幕DPI，用于自适应缩放"""
    try:
        import tkinter as tk
        root = tk.Tk()
        dpi = root.winfo_fpixels('1i')
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        root.destroy()
        return dpi, screen_width, screen_height
    except Exception:
        return 96, 1920, 1080  # 默认值


class Visualizer(Node):
    """可视化节点"""

    def __init__(self):
        super().__init__('visualizer')

        # 声明参数
        self.declare_parameter('map_size', 20.0)  # 地图尺寸（米），默认20x20
        self.declare_parameter('fullscreen', False)  # 是否全屏
        self.declare_parameter('window_scale', 0.7)  # 窗口占屏幕比例 (0.5-1.0)
        
        self.map_size = self.get_parameter('map_size').value
        self.fullscreen = self.get_parameter('fullscreen').value
        self.window_scale = self.get_parameter('window_scale').value
        
        # 获取屏幕信息
        self.dpi, self.screen_width, self.screen_height = get_screen_dpi()
        
        # 计算自适应的尺寸参数
        self.base_size = min(self.screen_width, self.screen_height) / self.dpi
        self.scale_factor = self.base_size / 10.0  # 相对于10英寸基准的缩放

        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # 轨迹历史
        self.trajectory_x = []
        self.trajectory_y = []

        # 目标点（odom坐标系）
        self.goal_x = None
        self.goal_y = None
        self.goal_x_base = None  # base_link系下的目标
        self.goal_y_base = None

        # 控制器状态
        self.controller_state = 'IDLE'

        # 障碍物列表
        self.obstacles = []
        self.obstacle_radius = 0.15

        # DWA规划轨迹
        self.dwa_trajectory_x = []
        self.dwa_trajectory_y = []

        # LaserScan 数据
        self.scan_points_x = []
        self.scan_points_y = []
        # 前方120度 (±60°) 的scan点（用于实际判断）
        self.front_scan_x = []
        self.front_scan_y = []
        self.scan_fov = math.radians(180)  # 前方180度 (±90°)
        self.show_scan = True  # 是否显示 scan 数据

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

        self.state_sub = self.create_subscription(
            String,
            '/controller_state',
            self.state_callback,
            10
        )

        # 订阅障碍物话题
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

        # 订阅DWA轨迹
        self.trajectory_sub = self.create_subscription(
            Path,
            '/dwa_trajectory',
            self.trajectory_callback,
            10
        )

        # 订阅LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 计算自适应的图形尺寸
        if self.fullscreen:
            fig_width = self.screen_width / self.dpi
            fig_height = self.screen_height / self.dpi
        else:
            # 限制最大尺寸为8英寸，避免元素过大
            fig_size = min(8.0, min(self.screen_width, self.screen_height) * self.window_scale / self.dpi)
            fig_width = fig_size
            fig_height = fig_size
        
        # 自适应的绘图参数 - 使用更保守的缩放
        # 基准：8英寸图形
        base_fig_size = 8.0
        actual_scale = min(1.5, fig_width / base_fig_size)  # 限制最大缩放1.5倍
        
        self.marker_scale = actual_scale
        self.font_scale = max(8, min(14, int(10 * actual_scale)))  # 字体8-14
        self.line_scale = max(1.0, min(2.5, 1.5 * actual_scale))  # 线宽1-2.5
        
        self.get_logger().info(
            f'Screen: {self.screen_width}x{self.screen_height}, DPI: {self.dpi:.0f}, '
            f'Scale: {self.scale_factor:.2f}')
        
        # 设置matplotlib为交互模式
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(fig_width, fig_height), dpi=self.dpi)
        
        # 尝试最大化窗口
        try:
            mng = plt.get_current_fig_manager()
            if self.fullscreen:
                mng.full_screen_toggle()
            else:
                # 尝试不同的后端方法来调整窗口
                if hasattr(mng, 'window'):
                    if hasattr(mng.window, 'state'):
                        mng.window.state('zoomed')  # TkAgg
                elif hasattr(mng, 'resize'):
                    mng.resize(int(fig_width * self.dpi), int(fig_height * self.dpi))
        except Exception as e:
            self.get_logger().debug(f'Window resize not supported: {e}')
        
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)', fontsize=self.font_scale)
        self.ax.set_ylabel('Y (m)', fontsize=self.font_scale)
        self.ax.set_title('Robot Navigation Visualization', fontsize=self.font_scale + 2)
        self.ax.tick_params(axis='both', labelsize=self.font_scale - 2)
        
        # 固定坐标系，中心在(0,0)
        half_size = self.map_size / 2
        self.ax.set_xlim(-half_size, half_size)
        self.ax.set_ylim(-half_size, half_size)

        # 绘图元素 - 固定合理大小，不随窗口过度缩放
        self.robot_plot, = self.ax.plot([], [], 'bo', 
                                         markersize=8,  # 固定大小
                                         label='Robot')
        self.trajectory_plot, = self.ax.plot([], [], 'b-', alpha=0.5,
                                              linewidth=1.5,
                                              label='Trajectory')
        self.goal_plot, = self.ax.plot([], [], 'r*', 
                                        markersize=15,  # 固定大小
                                        label='Goal')
        self.dwa_trajectory_plot, = self.ax.plot([], [], 'g-', alpha=0.8,
                                                  linewidth=1.5,
                                                  label='DWA Plan')
        self.robot_arrow = None
        
        # 箭头尺寸（相对于地图尺寸，但有上限）
        map_scale = min(1.5, self.map_size / 10.0)
        self.arrow_length = 0.3 * map_scale
        self.arrow_head_width = 0.12 * map_scale
        self.arrow_head_length = 0.08 * map_scale

        # LaserScan 绘图元素（黑色点 - 所有scan）
        self.scan_plot, = self.ax.plot([], [], '.', color='black',
                                        markersize=1,  # 小点
                                        alpha=0.3,
                                        label='LaserScan')
        # 前方120°的scan点（紫色 - 实际用于判断）
        self.front_scan_plot, = self.ax.plot([], [], '.', color='purple',
                                              markersize=2,  # 稍大
                                              alpha=0.7,
                                              label='Front FOV')
        # LaserScan 射线（可选，默认关闭）
        self.scan_lines = None
        self.show_scan_rays = False  # 是否显示射线

        # 障碍物绘图元素
        self.obstacle_patches = []

        # 状态文本 - 固定合理大小
        self.state_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=9,
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

        # 图例放在右上角，紧凑显示
        self.ax.legend(loc='upper right', fontsize=8, 
                       framealpha=0.8, handlelength=1.0,
                       labelspacing=0.3, borderpad=0.3)
        
        # 紧凑布局
        self.fig.tight_layout(pad=0.5)

        # 定时器更新显示
        self.timer = self.create_timer(0.1, self.update_plot)

        # 添加窗口resize事件处理
        self.fig.canvas.mpl_connect('resize_event', self.on_resize)
        
        self.get_logger().info(
            f'Visualizer initialized: map={self.map_size}x{self.map_size}m, '
            f'fig={fig_width:.1f}x{fig_height:.1f}in')
    
    def on_resize(self, event):
        """窗口大小改变时重新调整布局"""
        try:
            self.fig.tight_layout()
        except Exception:
            pass

    def quaternion_to_yaw(self, x, y, z, w):
        """四元数转yaw角"""
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        return yaw

    def odom_callback(self, msg):
        """更新机器人位姿"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        self.robot_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # 记录轨迹（降采样，每10cm记录一次）
        if (len(self.trajectory_x) == 0 or
            math.sqrt((self.robot_x - self.trajectory_x[-1])**2 +
                     (self.robot_y - self.trajectory_y[-1])**2) > 0.1):
            self.trajectory_x.append(self.robot_x)
            self.trajectory_y.append(self.robot_y)

    def state_callback(self, msg):
        """接收控制器状态"""
        self.controller_state = msg.data

    def obstacle_callback(self, msg):
        """接收障碍物"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.obstacles.append((x, y))
        self.get_logger().info(
            f'Obstacle added: ({x:.2f}, {y:.2f}), '
            f'total: {len(self.obstacles)}')

    def clear_obstacles_callback(self, msg):
        """清除所有障碍物"""
        count = len(self.obstacles)
        self.obstacles = []
        self.get_logger().info(f'Cleared {count} obstacles')

    def trajectory_callback(self, msg):
        """接收DWA规划的轨迹"""
        self.dwa_trajectory_x = []
        self.dwa_trajectory_y = []
        for pose in msg.poses:
            self.dwa_trajectory_x.append(pose.pose.position.x)
            self.dwa_trajectory_y.append(pose.pose.position.y)

    def scan_callback(self, msg):
        """接收LaserScan数据，转换到odom坐标系"""
        if not self.show_scan:
            return

        self.scan_points_x = []
        self.scan_points_y = []
        self.front_scan_x = []
        self.front_scan_y = []

        angle = msg.angle_min
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        half_fov = self.scan_fov / 2  # ±90°

        for r in msg.ranges:
            # 跳过无效数据
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            # base_link 坐标系下的点
            local_x = r * math.cos(angle)
            local_y = r * math.sin(angle)

            # 转换到 odom 坐标系
            world_x = self.robot_x + local_x * cos_yaw - local_y * sin_yaw
            world_y = self.robot_y + local_x * sin_yaw + local_y * cos_yaw

            self.scan_points_x.append(world_x)
            self.scan_points_y.append(world_y)

            # 检查是否在前方±60°范围内
            normalized_angle = angle
            while normalized_angle > math.pi:
                normalized_angle -= 2 * math.pi
            while normalized_angle < -math.pi:
                normalized_angle += 2 * math.pi
            
            if abs(normalized_angle) <= half_fov:
                self.front_scan_x.append(world_x)
                self.front_scan_y.append(world_y)

            angle += msg.angle_increment

    def goal_callback(self, msg):
        """接收目标点（base_link坐标系），转换到odom坐标系显示"""
        # 保存base_link系下的目标
        self.goal_x_base = msg.pose.position.x
        self.goal_y_base = msg.pose.position.y

        # 转换到odom坐标系用于显示（固定在odom系中）
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        self.goal_x = (self.robot_x + self.goal_x_base * cos_yaw -
                      self.goal_y_base * sin_yaw)
        self.goal_y = (self.robot_y + self.goal_x_base * sin_yaw +
                      self.goal_y_base * cos_yaw)

        self.get_logger().info(
            f'Goal received: base_link({self.goal_x_base:.2f}, '
            f'{self.goal_y_base:.2f}) -> '
            f'odom({self.goal_x:.2f}, {self.goal_y:.2f})')

        # 清除旧轨迹
        self.trajectory_x = [self.robot_x]
        self.trajectory_y = [self.robot_y]

    def update_plot(self):
        """更新可视化"""
        # 根据状态选择机器人颜色
        if self.controller_state == 'IDLE':
            robot_color = 'gray'
            arrow_color = 'gray'
        elif self.controller_state == 'ROTATING':
            robot_color = 'orange'
            arrow_color = 'orange'
        elif self.controller_state == 'MOVING':
            robot_color = 'blue'
            arrow_color = 'blue'
        elif self.controller_state == 'REACHED':
            robot_color = 'green'
            arrow_color = 'green'
        else:
            robot_color = 'gray'
            arrow_color = 'gray'

        # 更新机器人位置和颜色
        self.robot_plot.set_data([self.robot_x], [self.robot_y])
        self.robot_plot.set_color(robot_color)

        # 更新轨迹
        if len(self.trajectory_x) > 0:
            self.trajectory_plot.set_data(self.trajectory_x,
                                          self.trajectory_y)

        # 更新目标点（odom系，固定位置）
        if self.goal_x is not None:
            self.goal_plot.set_data([self.goal_x], [self.goal_y])

        # 更新DWA规划轨迹
        if len(self.dwa_trajectory_x) > 0:
            self.dwa_trajectory_plot.set_data(
                self.dwa_trajectory_x, self.dwa_trajectory_y)
        else:
            self.dwa_trajectory_plot.set_data([], [])

        # 更新LaserScan点云（黑色 - 所有scan）
        if self.show_scan and len(self.scan_points_x) > 0:
            self.scan_plot.set_data(self.scan_points_x, self.scan_points_y)
        else:
            self.scan_plot.set_data([], [])
        
        # 更新前方120°的scan点（紫色）
        if self.show_scan and len(self.front_scan_x) > 0:
            self.front_scan_plot.set_data(self.front_scan_x, self.front_scan_y)
        else:
            self.front_scan_plot.set_data([], [])

        # 移除旧的scan射线
        if self.scan_lines is not None:
            self.scan_lines.remove()
            self.scan_lines = None

        # 绘制scan射线（可选）
        if self.show_scan_rays and len(self.scan_points_x) > 0:
            lines = []
            for px, py in zip(self.scan_points_x, self.scan_points_y):
                lines.append([(self.robot_x, self.robot_y), (px, py)])
            self.scan_lines = LineCollection(lines, colors='lightgray',
                                              alpha=0.3, linewidths=0.5)
            self.ax.add_collection(self.scan_lines)

        # 移除旧障碍物图形
        for patch in self.obstacle_patches:
            patch.remove()
        self.obstacle_patches = []

        # 绘制障碍物
        for ox, oy in self.obstacles:
            circle = patches.Circle(
                (ox, oy), self.obstacle_radius,
                facecolor='red', edgecolor='darkred',
                alpha=0.6, linewidth=2
            )
            self.ax.add_patch(circle)
            self.obstacle_patches.append(circle)

        # 移除旧箭头
        if self.robot_arrow is not None:
            self.robot_arrow.remove()

        # 绘制机器人朝向箭头（自适应大小）
        dx = self.arrow_length * math.cos(self.robot_yaw)
        dy = self.arrow_length * math.sin(self.robot_yaw)
        self.robot_arrow = self.ax.arrow(
            self.robot_x, self.robot_y, dx, dy,
            head_width=self.arrow_head_width, 
            head_length=self.arrow_head_length,
            fc=arrow_color, ec=arrow_color, alpha=0.7
        )

        # 更新状态文本
        state_color_map = {
            'IDLE': 'gray',
            'ROTATING': 'orange',
            'MOVING': 'blue',
            'REACHED': 'green'
        }
        state_text = (f'State: {self.controller_state}\n'
                     f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f})\n'
                     f'Yaw: {math.degrees(self.robot_yaw):.1f}°\n'
                     f'Obstacles: {len(self.obstacles)}\n'
                     f'Scan: {len(self.scan_points_x)} (Front: {len(self.front_scan_x)})')
        self.state_text.set_text(state_text)
        self.state_text.set_bbox(dict(
            boxstyle='round',
            facecolor=state_color_map.get(self.controller_state, 'wheat'),
            alpha=0.5))

        # 坐标系固定为10x10，完全不动

        # 刷新显示
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    visualizer = Visualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
