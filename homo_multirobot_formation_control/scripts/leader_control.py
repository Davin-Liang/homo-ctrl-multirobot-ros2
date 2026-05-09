#!/usr/bin/env python3
"""
领航者轨迹控制器。
生成正弦参考轨迹，通过 /cmd_vel 驱动机器人运动。

适配项目命名空间约定:
- 订阅: /<robot_ns>/odometry/filtered（EKF 融合里程计）
- 发布: cmd_vel（相对话题，在命名空间下解析）
- 节点运行在领航者命名空间下（如 __ns:=/robot1）

使用:
  ros2 run homo_multirobot_formation_control leader_control.py --ros-args -r __ns:=/robot1
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
import time


class LeaderController(Node):
    def __init__(self):
        super().__init__('leader_controller')

        # ---- 参数 ----
        # mass: 控制器模型质量（调参用，非物理质量）
        self.declare_parameter('mass', 2.2)
        self.declare_parameter('h', 0.1)
        self.declare_parameter('robot_ns', '/robot1')

        self.m = self.get_parameter('mass').value
        self.h = self.get_parameter('h').value
        self.robot_ns = self.get_parameter('robot_ns').value

        # 系统矩阵（双重积分器，注意 B 使用 8/m 而非 1/m）
        self.A = np.array([[0, 0, 1, 0],
                           [0, 0, 0, 1],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0]])
        self.B = np.array([[0, 0],
                           [0, 0],
                           [8.0 / self.m, 0],
                           [0, 8.0 / self.m]])

        # 状态向量: [px, py, vx, vy]
        self.x = np.zeros(4)

        # 订阅 EKF 融合里程计
        odom_topic = f'{self.robot_ns}/odometry/filtered'
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        # 发布 cmd_vel（相对话题，命名空间下自动补全）
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # 定时器控制循环
        self.timer = self.create_timer(self.h, self.control_loop)

        self.start_time = time.time()
        self.get_logger().info(
            f'领航者控制器已启动 (mass={self.m}, h={self.h}, odom={odom_topic})')

    def odom_callback(self, msg: Odometry):
        """从 EKF 里程计更新领航者状态"""
        self.x = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        ])

    def control_loop(self):
        """计算正弦轨迹控制输入并发布目标速度"""
        t = time.time() - self.start_time

        # u = -[I I]·x + [sin(t), cos(t)]^T
        # 第一项为阻尼（使速度不爆炸），第二项为正弦激励
        M = np.hstack([np.eye(2), np.eye(2)])
        u = -M @ self.x + np.array([math.sin(t), math.sin(t)])

        # 前向欧拉: goal_x = x + h·(A·x + B·u)
        goal_x = self.x + self.h * (self.A @ self.x + self.B @ u)

        cmd_msg = Twist()
        cmd_msg.linear.x = float(goal_x[2])
        cmd_msg.linear.y = float(goal_x[3])
        self.pub_cmd.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
