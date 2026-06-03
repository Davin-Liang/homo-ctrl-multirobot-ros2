#!/usr/bin/env python3
"""
领航者 8 字运动（纯开环速度指令，无位置反馈）。

轨迹参数方程 (map 帧):
  x(t) = A_x * sin(ω*t)
  y(t) = A_y * sin(2*ω*t)
  ω = 2π / T

cmd_vel 发布 map 帧瞬时速度旋转到车体帧后的速度，angular.z = 0（航向不变）。

使用:
  ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1
  ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1 \
    -p amplitude_x:=2.0 -p amplitude_y:=1.0 -p period:=10.0 -p heading:=0.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class LeaderEight(Node):
    def __init__(self):
        super().__init__('leader_eight')

        self.declare_parameter('amplitude_x', 2.0)
        self.declare_parameter('amplitude_y', 1.0)
        self.declare_parameter('period', 10.0)
        self.declare_parameter('heading', 0.0)
        self.declare_parameter('rate', 20.0)

        Ax   = self.get_parameter('amplitude_x').value
        Ay   = self.get_parameter('amplitude_y').value
        T    = self.get_parameter('period').value
        yaw  = math.radians(self.get_parameter('heading').value)
        hz   = self.get_parameter('rate').value

        self.Ax = Ax
        self.Ay = Ay
        self.omega = 2.0 * math.pi / T

        self.ch = math.cos(yaw)
        self.sh = math.sin(yaw)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)
        self.t0 = time.time()

        self.get_logger().info(
            f'8 字运动: Ax={Ax:.1f}m Ay={Ay:.1f}m '
            f'T={T:.1f}s ω={self.omega:.3f}rad/s '
            f'航向={math.degrees(yaw):.0f}°')

    def timer_cb(self):
        t = time.time() - self.t0
        w = self.omega
        Ax = self.Ax
        Ay = self.Ay

        # map-frame position (for reference, not used in control)
        # x_map = Ax * sin(w*t)
        # y_map = Ay * sin(2*w*t)

        # map-frame velocity (derivative of position)
        vx_map = Ax * w * math.cos(w * t)
        vy_map = Ay * 2.0 * w * math.cos(2.0 * w * t)

        # rotate to body frame
        vx_b =  self.ch * vx_map + self.sh * vy_map
        vy_b = -self.sh * vx_map + self.ch * vy_map

        cmd = Twist()
        cmd.linear.x = vx_b
        cmd.linear.y = vy_b
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderEight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
