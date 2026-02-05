#!/usr/bin/env python3
"""
交互式障碍物添加器
用于向fake_scan添加虚拟障碍物
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json


class InteractiveObstaclePublisher(Node):
    """交互式障碍物发布器节点"""

    def __init__(self):
        super().__init__('interactive_obstacle_publisher')
        
        # 发布障碍物
        self.obstacle_pub = self.create_publisher(
            PoseStamped, '/add_obstacle', 10)
        
        # 发布清除命令
        self.clear_pub = self.create_publisher(
            String, '/clear_obstacles', 10)
        
        # 本地障碍物列表（用于显示）
        self.obstacles = []
        
        self.get_logger().info(
            'Interactive Obstacle Publisher initialized')

    def publish_obstacle(self, x, y):
        """发布障碍物"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.obstacle_pub.publish(msg)
        self.obstacles.append((x, y))
        self.get_logger().info(
            f'Published obstacle at ({x:.3f}, {y:.3f})')

    def clear_obstacles(self):
        """清除所有障碍物"""
        msg = String()
        msg.data = 'clear'
        self.clear_pub.publish(msg)
        self.obstacles = []
        self.get_logger().info('Cleared all obstacles')

    def list_obstacles(self):
        """列出所有障碍物"""
        return self.obstacles


def print_help():
    """打印帮助信息"""
    print("\n命令说明:")
    print("  x y          - 添加障碍物，如: 2.0 1.0")
    print("  list         - 列出已添加的障碍物")
    print("  clear        - 清除所有障碍物")
    print("  preset       - 添加预设障碍物（测试用）")
    print("  wall x y1 y2 - 添加垂直墙，如: wall 2.0 -1.0 1.0")
    print("  hwall x1 x2 y - 添加水平墙，如: hwall 1.0 3.0 1.5")
    print("  circle x y r - 添加圆形障碍物，如: circle 2.0 0.0 0.5")
    print("  help         - 显示此帮助")
    print("  q/quit       - 退出")
    print()


def main(args=None):
    rclpy.init(args=args)

    node = InteractiveObstaclePublisher()

    print("\n" + "="*60)
    print("交互式障碍物添加器")
    print("="*60)
    print("在odom坐标系中添加虚拟障碍物")
    print("这些障碍物会被fake_scan节点检测并生成激光雷达数据")
    print_help()

    try:
        while rclpy.ok():
            try:
                user_input = input("障碍物> ").strip().lower()

                # 检查退出命令
                if user_input in ['q', 'quit', 'exit']:
                    print("退出程序...")
                    break

                # 跳过空输入
                if not user_input:
                    continue

                # 帮助
                if user_input == 'help':
                    print_help()
                    continue

                # 列出障碍物
                if user_input == 'list':
                    obstacles = node.list_obstacles()
                    if obstacles:
                        print(f"已添加 {len(obstacles)} 个障碍物:")
                        for i, (x, y) in enumerate(obstacles):
                            print(f"  [{i+1}] ({x:.2f}, {y:.2f})")
                    else:
                        print("没有障碍物")
                    continue

                # 清除障碍物
                if user_input == 'clear':
                    node.clear_obstacles()
                    rclpy.spin_once(node, timeout_sec=0.1)
                    print("已清除所有障碍物")
                    continue

                # 预设障碍物
                if user_input == 'preset':
                    presets = [
                        (2.0, 0.5),
                        (2.0, -0.5),
                        (3.0, 0.0),
                        (1.5, 1.0),
                    ]
                    print("添加预设障碍物...")
                    for x, y in presets:
                        node.publish_obstacle(x, y)
                        rclpy.spin_once(node, timeout_sec=0.05)
                    print(f"已添加 {len(presets)} 个预设障碍物")
                    continue

                # 添加垂直墙: wall x y1 y2
                if user_input.startswith('wall '):
                    parts = user_input.split()
                    if len(parts) != 4:
                        print("错误: 格式为 wall x y1 y2")
                        continue
                    try:
                        x = float(parts[1])
                        y1 = float(parts[2])
                        y2 = float(parts[3])
                        
                        # 生成墙上的点
                        num_points = int(abs(y2 - y1) / 0.1) + 1
                        num_points = max(num_points, 2)
                        
                        import numpy as np
                        ys = np.linspace(y1, y2, num_points)
                        
                        print(f"添加垂直墙 x={x}, y=[{y1}, {y2}], "
                              f"{num_points}个点...")
                        for y in ys:
                            node.publish_obstacle(x, float(y))
                            rclpy.spin_once(node, timeout_sec=0.02)
                        print(f"已添加垂直墙")
                        continue
                    except ValueError:
                        print("错误: 请输入有效的数字")
                        continue

                # 添加水平墙: hwall x1 x2 y
                if user_input.startswith('hwall '):
                    parts = user_input.split()
                    if len(parts) != 4:
                        print("错误: 格式为 hwall x1 x2 y")
                        continue
                    try:
                        x1 = float(parts[1])
                        x2 = float(parts[2])
                        y = float(parts[3])
                        
                        # 生成墙上的点
                        num_points = int(abs(x2 - x1) / 0.1) + 1
                        num_points = max(num_points, 2)
                        
                        import numpy as np
                        xs = np.linspace(x1, x2, num_points)
                        
                        print(f"添加水平墙 x=[{x1}, {x2}], y={y}, "
                              f"{num_points}个点...")
                        for x in xs:
                            node.publish_obstacle(float(x), y)
                            rclpy.spin_once(node, timeout_sec=0.02)
                        print(f"已添加水平墙")
                        continue
                    except ValueError:
                        print("错误: 请输入有效的数字")
                        continue

                # 添加圆形障碍物: circle x y r
                if user_input.startswith('circle '):
                    parts = user_input.split()
                    if len(parts) != 4:
                        print("错误: 格式为 circle x y r")
                        continue
                    try:
                        cx = float(parts[1])
                        cy = float(parts[2])
                        r = float(parts[3])
                        
                        # 生成圆周上的点
                        import numpy as np
                        num_points = max(int(2 * 3.14159 * r / 0.1), 8)
                        angles = np.linspace(0, 2 * 3.14159, num_points)
                        
                        print(f"添加圆形障碍物 中心({cx}, {cy}), "
                              f"半径{r}, {num_points}个点...")
                        for angle in angles:
                            x = cx + r * np.cos(angle)
                            y = cy + r * np.sin(angle)
                            node.publish_obstacle(float(x), float(y))
                            rclpy.spin_once(node, timeout_sec=0.02)
                        print(f"已添加圆形障碍物")
                        continue
                    except ValueError:
                        print("错误: 请输入有效的数字")
                        continue

                # 解析单个障碍物坐标
                parts = user_input.split()
                if len(parts) != 2:
                    print("错误: 请输入两个数字 (x y) 或输入 help 查看帮助")
                    continue

                x = float(parts[0])
                y = float(parts[1])

                # 发布障碍物
                node.publish_obstacle(x, y)
                rclpy.spin_once(node, timeout_sec=0.1)

                print(f"障碍物已添加: ({x:.3f}, {y:.3f})")
                print("-" * 60)

            except ValueError:
                print("错误: 请输入有效的数字")
            except KeyboardInterrupt:
                print("\n收到中断信号，退出...")
                break
            except Exception as e:
                print(f"错误: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
