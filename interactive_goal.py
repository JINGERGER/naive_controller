#!/usr/bin/env python3
"""
交互式目标点发布器
在终端中手动输入目标点坐标
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class InteractiveGoalPublisher(Node):
    """交互式目标点发布器节点"""

    def __init__(self):
        super().__init__('interactive_goal_publisher')
        self.publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.get_logger().info('Interactive Goal Publisher initialized')

    def publish_goal(self, x, y, frame_id='base_link'):
        """发布目标点"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published goal: x={x:.3f}, y={y:.3f} ({frame_id})')


def main(args=None):
    rclpy.init(args=args)

    node = InteractiveGoalPublisher()

    print("\n" + "="*60)
    print("交互式目标点发布器")
    print("="*60)
    print("选择坐标系：")
    print("  1. odom   - 绝对坐标系（推荐，固定不动）")
    print("  2. base_link - 相对坐标系（相对机器人当前位置）")
    print("="*60)
    
    frame_id = 'odom'
    while True:
        frame_choice = input("请选择坐标系 (1/2, 默认1): ").strip()
        if frame_choice == '' or frame_choice == '1':
            frame_id = 'odom'
            print("使用 odom 坐标系（绝对坐标）")
            break
        elif frame_choice == '2':
            frame_id = 'base_link'
            print("使用 base_link 坐标系（相对坐标）")
            break
        else:
            print("无效输入，请输入 1 或 2")
    
    print("\n" + "="*60)
    print(f"当前坐标系: {frame_id}")
    if frame_id == 'odom':
        print("输入目标点的绝对坐标")
        print("例如: 2.0 0.0  (odom系中的(2,0)位置)")
        print("     0.0 2.0  (odom系中的(0,2)位置)")
        print("     -1.0 1.0 (odom系中的(-1,1)位置)")
    else:
        print("输入目标点相对机器人当前位置的坐标")
        print("例如: 2.0 0.0  (前方2米)")
        print("     0.0 1.0  (左侧1米)")
        print("     -1.0 0.0 (后方1米)")
    print("输入 'q' 或 'quit' 退出")
    print("="*60 + "\n")

    try:
        while rclpy.ok():
            try:
                user_input = input("请输入目标点 (x y): ").strip()

                # 检查退出命令
                if user_input.lower() in ['q', 'quit', 'exit']:
                    print("退出程序...")
                    break

                # 跳过空输入
                if not user_input:
                    continue

                # 解析输入（支持空格、逗号、分号分隔）
                # 替换常见分隔符为空格
                cleaned_input = user_input.replace(',', ' ').replace(';', ' ')
                parts = cleaned_input.split()
                
                # 过滤空字符串
                parts = [p.strip() for p in parts if p.strip()]
                
                if len(parts) != 2:
                    print(f"错误: 请输入两个数字 (x y)，收到: {parts}")
                    continue

                try:
                    x = float(parts[0])
                    y = float(parts[1])
                except ValueError:
                    print(f"错误: 无法解析数字 '{parts[0]}' 或 '{parts[1]}'")
                    continue
                
                print(f"解析结果: x={x}, y={y}")

                # 发布目标点
                node.publish_goal(x, y, frame_id)

                # 等待一小段时间确保消息发送
                rclpy.spin_once(node, timeout_sec=0.1)

                print(f"目标点已发送: ({x:.3f}, {y:.3f}) [{frame_id}]")
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
