#!/usr/bin/env python3
"""
测试目标点发布器
用于向控制器发送测试目标点
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys


class GoalPublisher(Node):
    """目标点发布器节点"""

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

    def publish_goal(self, x, y):
        """发布目标点"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published goal: x={x:.3f}, y={y:.3f}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: test_goal_publisher.py <x> <y>")
        print("Example: test_goal_publisher.py 2.0 0.0")
        print("  - (2.0, 0.0): 前方2米")
        print("  - (0.0, 1.0): 左侧1米")
        print("  - (-1.0, 0.0): 后方1米")
        print("  - (1.0, 1.0): 前方1米+左侧1米")
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    except ValueError:
        print("Error: x and y must be numbers")
        sys.exit(1)

    node = GoalPublisher()
    node.publish_goal(x, y)

    # 等待一小段时间确保消息发送
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
