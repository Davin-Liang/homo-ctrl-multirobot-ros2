#!/usr/bin/env python3
"""
领航者绕圈（纯开环速度指令，无位置反馈）。

cmd_vel 直接发布正弦速度，车体 vx/vy 合成为 map 系圆周运动，
angular.z=0（航向不变）。

圆心由速度模式自然产生，无需指定。

使用:
  ros2 run homo_multirobot_formation_control leader_circle.py --ros-args -r __ns:=/robot1
  ros2 run homo_multirobot_formation_control leader_circle.py --ros-args -r __ns:=/robot1 \
    -p radius:=1.0 -p speed:=0.3 -p heading:=0.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class LeaderCircle(Node):
    def __init__(self):
        super().__init__('leader_circle')

        self.declare_parameter('radius', 1.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('heading', 0.0)
        self.declare_parameter('direction', 'ccw')
        self.declare_parameter('rate', 20.0)

        R   = self.get_parameter('radius').value
        v   = self.get_parameter('speed').value
        yaw = math.radians(self.get_parameter('heading').value)
        ccw = self.get_parameter('direction').value == 'ccw'
        hz  = self.get_parameter('rate').value

        omega = v / R * (1.0 if ccw else -1.0)

        self.ch = math.cos(yaw)
        self.sh = math.sin(yaw)
        self.v  = v
        self.omega = omega
        self.period = 2 * math.pi / abs(omega)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / hz, self.timer_cb)
        self.t0 = time.time()

        self.get_logger().info(
            f'绕圈: R={R:.1f}m v={v:.2f}m/s ω={omega:.3f}rad/s '
            f'周期={self.period:.1f}s 航向={math.degrees(yaw):.0f}° '
            f'{"逆时针" if ccw else "顺时针"}')

    def timer_cb(self):
        t = time.time() - self.t0
        a = self.omega * t

        # map-frame tangent velocity at angle a (circle of radius R, speed v)
        vx_map = -self.v * math.sin(a)
        vy_map =  self.v * math.cos(a)

        # rotate to body frame
        vx_b =  self.ch * vx_map + self.sh * vy_map
        vy_b = -self.sh * vx_map + self.ch * vy_map

        cmd = Twist()
        cmd.linear.x = vx_b
        cmd.linear.y = vy_b
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
