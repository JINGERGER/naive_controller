#!/bin/bash
# 检查ROS2话题和节点

echo "=== 检查所有ROS2节点 ==="
ros2 node list 2>/dev/null || echo "未检测到运行中的ROS2节点"

echo ""
echo "=== 检查/goal_pose话题 ==="
ros2 topic info /goal_pose 2>/dev/null || echo "话题/goal_pose不存在"

echo ""
echo "=== 检查/goal_pose的发布者 ==="
ros2 topic info /goal_pose -v 2>/dev/null || echo "无发布者信息"

echo ""
echo "=== 监听/goal_pose话题（5秒） ==="
timeout 5 ros2 topic echo /goal_pose 2>/dev/null || echo "5秒内未收到消息"
