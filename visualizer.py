#!/usr/bin/env python3
"""
简单的2D可视化工具
实时显示机器人位置、轨迹、目标点和障碍物
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection, LineCollection
import math
import numpy as np


class Visualizer(Node):
    """可视化节点"""

    def __init__(self):
        super().__init__('visualizer')

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

        # 设置matplotlib为交互模式
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Robot Navigation Visualization')
        
        # 固定坐标系为10x10，中心在(0,0)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        # 绘图元素
        self.robot_plot, = self.ax.plot([], [], 'bo', markersize=10,
                                         label='Robot')
        self.trajectory_plot, = self.ax.plot([], [], 'b-', alpha=0.5,
                                              linewidth=2,
                                              label='Trajectory')
        self.goal_plot, = self.ax.plot([], [], 'r*', markersize=20,
                                        label='Goal')
        self.dwa_trajectory_plot, = self.ax.plot([], [], 'g-', alpha=0.8,
                                                  linewidth=2,
                                                  label='DWA Plan')
        self.robot_arrow = None

        # LaserScan 绘图元素（淡灰色点）
        self.scan_plot, = self.ax.plot([], [], '.', color='lightgray',
                                        markersize=2, alpha=0.6,
                                        label='LaserScan')
        # LaserScan 射线（可选，默认关闭）
        self.scan_lines = None
        self.show_scan_rays = False  # 是否显示射线

        # 障碍物绘图元素
        self.obstacle_patches = []

        # 状态文本
        self.state_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=12,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        self.ax.legend(loc='upper right')

        # 定时器更新显示
        self.timer = self.create_timer(0.1, self.update_plot)

        self.get_logger().info('Visualizer initialized')

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

        angle = msg.angle_min
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

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

        # 更新LaserScan点云
        if self.show_scan and len(self.scan_points_x) > 0:
            self.scan_plot.set_data(self.scan_points_x, self.scan_points_y)
        else:
            self.scan_plot.set_data([], [])

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

        # 绘制机器人朝向箭头
        arrow_length = 0.3
        dx = arrow_length * math.cos(self.robot_yaw)
        dy = arrow_length * math.sin(self.robot_yaw)
        self.robot_arrow = self.ax.arrow(
            self.robot_x, self.robot_y, dx, dy,
            head_width=0.15, head_length=0.1,
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
                     f'Scan points: {len(self.scan_points_x)}')
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
