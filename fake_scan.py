#!/usr/bin/env python3
"""
Fake LaserScan Node for ROS2
用于测试避障功能，模拟激光雷达数据
可以在指定位置放置虚拟障碍物
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math


class FakeLaserScan(Node):
    """模拟激光雷达节点"""

    def __init__(self):
        super().__init__('fake_laser_scan')

        # 声明参数
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('obstacle_radius', 0.15)

        # 获取参数
        pub_freq = self.get_parameter('publish_frequency').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter(
            'angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.obstacle_radius = self.get_parameter(
            'obstacle_radius').value

        # 计算扫描点数
        self.num_readings = int(
            (self.angle_max - self.angle_min) / self.angle_increment)

        # 虚拟障碍物（odom坐标系）
        # 初始为空，通过话题动态添加
        self.obstacles = []

        # 机器人当前位姿
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # 订阅机器人位姿（从/odom获取）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 订阅障碍物添加话题
        self.obstacle_sub = self.create_subscription(
            PoseStamped,
            '/add_obstacle',
            self.add_obstacle_callback,
            10
        )

        # 订阅清除障碍物话题
        self.clear_sub = self.create_subscription(
            String,
            '/clear_obstacles',
            self.clear_obstacles_callback,
            10
        )

        # 发布激光雷达数据
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # 创建定时器
        self.timer = self.create_timer(1.0 / pub_freq, self.publish_scan)

        self.get_logger().info('Fake LaserScan initialized')
        self.get_logger().info('No initial obstacles. Use:')
        self.get_logger().info('  python3 interactive_obstacle.py')
        self.get_logger().info('to add obstacles interactively.')

    def odom_callback(self, msg):
        """更新机器人位姿"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # 四元数转yaw
        orientation = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y +
                         orientation.z * orientation.z)
        )

    def add_obstacle_callback(self, msg):
        """添加障碍物"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.obstacles.append((x, y))
        self.get_logger().info(
            f'Added obstacle at ({x:.2f}, {y:.2f}), '
            f'total: {len(self.obstacles)}')

    def clear_obstacles_callback(self, msg):
        """清除所有障碍物"""
        count = len(self.obstacles)
        self.obstacles = []
        self.get_logger().info(f'Cleared {count} obstacles')

    def calculate_range(self, angle):
        """
        计算给定角度下的激光测距值
        angle: 相对于base_link的角度
        使用射线-圆形碰撞检测
        """
        # 将角度转换到odom坐标系
        global_angle = self.robot_yaw + angle

        # 计算射线方向
        ray_dx = math.cos(global_angle)
        ray_dy = math.sin(global_angle)

        min_range = self.range_max

        # 检查每个障碍物
        for ox, oy in self.obstacles:
            # 障碍物相对机器人的位置
            dx = ox - self.robot_x
            dy = oy - self.robot_y
            dist_to_center = math.sqrt(dx**2 + dy**2)

            # 射线-圆形碰撞检测
            # 计算射线到圆心的最近点
            # t = (d . ray) 其中d是从机器人到圆心的向量
            t = dx * ray_dx + dy * ray_dy

            if t < 0:
                # 障碍物在机器人后方
                continue

            # 射线上最近点到圆心的距离
            closest_x = t * ray_dx
            closest_y = t * ray_dy
            dist_to_ray = math.sqrt(
                (dx - closest_x)**2 + (dy - closest_y)**2)

            # 如果射线穿过障碍物（考虑障碍物半径）
            if dist_to_ray < self.obstacle_radius:
                # 计算交点距离
                half_chord = math.sqrt(
                    self.obstacle_radius**2 - dist_to_ray**2)
                hit_dist = t - half_chord

                if hit_dist > self.range_min and hit_dist < min_range:
                    min_range = hit_dist

        return min_range

    def normalize_angle(self, angle):
        """归一化角度"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_scan(self):
        """发布激光雷达数据"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # 生成距离数据
        ranges = []
        angle = self.angle_min

        for i in range(self.num_readings):
            r = self.calculate_range(angle)
            ranges.append(r)
            angle += self.angle_increment

        scan.ranges = ranges

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    fake_scan = FakeLaserScan()

    try:
        rclpy.spin(fake_scan)
    except KeyboardInterrupt:
        pass
    finally:
        fake_scan.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
