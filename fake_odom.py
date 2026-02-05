#!/usr/bin/env python3
"""
Fake Odometry Node for ROS2
用于测试导航控制器，模拟机器人运动并发布ODOM
订阅cmd_vel，积分计算位姿，发布odom话题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


class FakeOdom(Node):
    """模拟里程计节点"""

    def __init__(self):
        super().__init__('fake_odom')

        # 声明参数
        self.declare_parameter('publish_frequency', 20.0)  # 发布频率（Hz）
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # 获取参数
        pub_freq = self.get_parameter('publish_frequency').value
        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value

        # 速度指令
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # 时间管理
        self.last_time = self.get_clock().now()

        # 订阅cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 发布odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 创建定时器发布odom
        self.timer = self.create_timer(1.0 / pub_freq, self.publish_odom)

        self.get_logger().info('FakeOdom initialized')
        self.get_logger().info(
            f'Initial pose: x={self.x:.3f}, y={self.y:.3f}, '
            f'yaw={self.yaw:.3f}')

    def cmd_vel_callback(self, msg):
        """
        接收速度指令
        """
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def yaw_to_quaternion(self, yaw):
        """
        将yaw角转换为四元数
        """
        # 对于2D平面运动，roll=0, pitch=0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = qz
        q.w = qw
        return q

    def publish_odom(self):
        """
        积分计算位姿并发布odom
        """
        # 计算时间差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 积分计算位姿（差速驱动运动学模型）
        if abs(self.angular_vel) < 1e-6:
            # 直线运动
            self.x += self.linear_vel * math.cos(self.yaw) * dt
            self.y += self.linear_vel * math.sin(self.yaw) * dt
        else:
            # 圆弧运动
            radius = self.linear_vel / self.angular_vel
            dtheta = self.angular_vel * dt

            self.x += radius * (math.sin(self.yaw + dtheta) -
                                math.sin(self.yaw))
            self.y += radius * (-math.cos(self.yaw + dtheta) +
                                math.cos(self.yaw))
            self.yaw += dtheta

        # 归一化yaw角到[-π, π]
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        # 创建并发布odom消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 设置位姿
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # 设置速度（直接使用cmd_vel的值）
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom)

        # 定期打印位姿（每秒一次）
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            self.get_logger().info(
                f'Pose: x={self.x:.3f}, y={self.y:.3f}, '
                f'yaw={self.yaw:.3f} | '
                f'Vel: lin={self.linear_vel:.3f}, '
                f'ang={self.angular_vel:.3f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)

    fake_odom = FakeOdom()

    try:
        rclpy.spin(fake_odom)
    except KeyboardInterrupt:
        pass
    finally:
        fake_odom.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
