#!/usr/bin/env python3
"""
Naive Controller Node for ROS2
将目标点导航分解为两个动作：旋转和直走
使用ODOM增量进行闭环控制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from enum import Enum
import math


class ControllerState(Enum):
    """控制器状态机"""
    IDLE = 0       # 等待目标点
    ROTATING = 1   # 旋转阶段
    MOVING = 2     # 移动阶段
    REACHED = 3    # 到达目标


class NavController(Node):
    """导航控制器节点"""

    def __init__(self):
        super().__init__('nav_controller')

        # 声明参数
        self.declare_parameter('angle_threshold', 0.1)
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('max_linear_velocity', 0.5)  # Bunker Mini 最大1m/s，设置0.5m/s
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('control_frequency', 10.0)

        # 获取参数
        self.angle_threshold = self.get_parameter(
            'angle_threshold').value
        self.distance_threshold = self.get_parameter(
            'distance_threshold').value
        self.max_linear_vel = self.get_parameter(
            'max_linear_velocity').value
        self.max_angular_vel = self.get_parameter(
            'max_angular_velocity').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        control_freq = self.get_parameter('control_frequency').value

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

        # 发布话题
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(
            String, '/controller_state', 10)

        # 创建控制定时器
        self.timer = self.create_timer(
            1.0 / control_freq, self.control_loop)

        # 状态变量
        self.state = ControllerState.IDLE

        # ODOM数据
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # 目标点（odom坐标系）
        self.goal_x = None
        self.goal_y = None

        # 起始状态（用于计算增量）
        self.start_yaw = 0.0        # 旋转起始yaw
        self.start_x = 0.0          # 移动起始x
        self.start_y = 0.0          # 移动起始y

        # 到达目标后的停留时间
        self.reach_time = None
        self.wait_duration = 3.0    # 停留3秒

        self.get_logger().info('NavController initialized')
        self.get_logger().info(
            f'Parameters: angle_threshold={self.angle_threshold:.3f} '
            f'rad, distance_threshold={self.distance_threshold:.3f} m')

    def normalize_angle(self, angle):
        """
        将角度归一化到[-π, π]范围
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, x, y, z, w):
        """
        从四元数转换为yaw角
        """
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        return yaw

    def clamp(self, value, min_value, max_value):
        """
        限制数值范围
        """
        return max(min_value, min(value, max_value))

    def odom_callback(self, msg):
        """
        ODOM回调函数，更新当前位姿
        """
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

        self.odom_received = True

    def goal_callback(self, msg):
        """
        目标点回调函数，支持odom和base_link坐标系
        """
        if not self.odom_received:
            self.get_logger().warn(
                'ODOM not received yet, ignoring goal')
            return

        frame_id = msg.header.frame_id
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        # 根据坐标系处理目标点
        if frame_id == 'odom':
            # 直接使用odom坐标系的目标点
            self.goal_x = target_x
            self.goal_y = target_y
            self.get_logger().info(
                f'New goal received: odom({self.goal_x:.3f}, '
                f'{self.goal_y:.3f})')
        elif frame_id == 'base_link':
            # 转换base_link到odom坐标系
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

        # 记录旋转起始状态
        self.start_yaw = self.current_yaw

        # 切换到旋转状态
        self.state = ControllerState.ROTATING

        self.get_logger().info(
            f'Starting rotation from yaw={self.start_yaw:.3f} rad')

    def control_loop(self):
        """
        控制循环，根据当前状态执行相应的控制
        """
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
                        self.kp_angular * angle_error,
                        -self.max_angular_vel,
                        self.max_angular_vel
                    )

                    if abs(angle_error) > 0.5:  # 只在误差较大时打印
                        self.get_logger().info(
                            f'Rotating: target_yaw={target_yaw:.3f}, '
                            f'current_yaw={self.current_yaw:.3f}, '
                            f'error={angle_error:.3f}',
                            throttle_duration_sec=1.0
                        )
                else:
                    # 旋转完成，切换到移动状态
                    self.state = ControllerState.MOVING
                    self.get_logger().info(
                        'Rotation complete, starting movement')
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
                # 移动阶段：基于ODOM计算到目标点的距离
                dx = self.goal_x - self.current_x
                dy = self.goal_y - self.current_y
                distance = math.sqrt(dx**2 + dy**2)

                if distance > self.distance_threshold:
                    # 继续前进
                    cmd_vel.linear.x = self.clamp(
                        self.kp_linear * distance,
                        0.0,  # 不后退
                        self.max_linear_vel
                    )

                    # 微调角度指向目标
                    target_yaw = math.atan2(dy, dx)
                    heading_error = self.normalize_angle(
                        target_yaw - self.current_yaw)
                    cmd_vel.angular.z = self.clamp(
                        self.kp_angular * heading_error,
                        -self.max_angular_vel * 0.5,
                        self.max_angular_vel * 0.5
                    )

                    if distance > 0.1:  # 只在误差较大时打印
                        self.get_logger().info(
                            f'Moving: distance={distance:.3f}, '
                            f'position=({self.current_x:.3f}, '
                            f'{self.current_y:.3f})',
                            throttle_duration_sec=1.0
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

            # 记录到达时间
            if self.reach_time is None:
                self.reach_time = self.get_clock().now()
                self.get_logger().info(
                    'Goal reached! Waiting for 3 seconds...')

            # 检查是否已经等待足够时间
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


def main(args=None):
    rclpy.init(args=args)

    controller = NavController()

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
