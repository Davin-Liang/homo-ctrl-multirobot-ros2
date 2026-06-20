#!/usr/bin/env python3
"""
虚拟 Leader 绕圈运动 — 不依赖仿真/实车，直接发布 odometry/filtered + 静态 TF。

发布内容:
  <ns>/odometry/filtered  (nav_msgs/Odometry) — 绕圈完整状态
  TF: map -> <prefix>_odom               — 恒等变换
  TF: <prefix>_odom -> <prefix>_base_footprint — 恒等变换

运动模型（CCW）:
  ω = speed / radius
  θ(t) = ω·t
  position:  (cx + radius·cos(θ),  cy + radius·sin(θ))
  yaw:       θ + π/2   （始终切向）
  body vx:   speed,  body vy: 0,  body wz: ω

使用:
  ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
    --ros-args -r __ns:=/virtual_leader

  ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
    --ros-args -r __ns:=/virtual_leader \
    -p center_x:=0.0 -p center_y:=0.0 -p radius:=2.0 -p speed:=0.5
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class VirtualLeaderCircle(Node):
    def __init__(self):
        super().__init__('virtual_leader_circle')

        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('direction', 'ccw')
        self.declare_parameter('rate', 50.0)

        cx = self.get_parameter('center_x').value
        cy = self.get_parameter('center_y').value
        R = self.get_parameter('radius').value
        v = self.get_parameter('speed').value
        ccw = self.get_parameter('direction').value == 'ccw'
        hz = self.get_parameter('rate').value

        self.cx = cx
        self.cy = cy
        self.R = R
        self.v = v
        self.omega = v / R * (1.0 if ccw else -1.0)
        self.period = 2.0 * math.pi / abs(self.omega)
        self.dt = 1.0 / hz

        # 从 namespace 推导 TF 前缀: /virtual_leader -> virtual_leader
        ns = self.get_namespace()
        self.prefix = ns.lstrip('/') + '_' if ns and ns != '/' else ''

        self.odom_frame = self.prefix + 'odom'
        self.base_frame = self.prefix + 'base_footprint'

        self.odom_pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.t0 = time.time()

        self._broadcast_static_tfs()

        self.get_logger().info(
            f'虚拟 Leader 绕圈: 圆心=({cx:.1f},{cy:.1f}) R={R:.1f}m '
            f'v={v:.2f}m/s ω={self.omega:.3f}rad/s 周期={self.period:.1f}s '
            f'{"逆时针" if ccw else "顺时针"} '
            f'odom_frame={self.odom_frame} base_frame={self.base_frame} '
            f'频率={hz:.0f}Hz'
        )

    def _broadcast_static_tfs(self):
        now = self.get_clock().now().to_msg()

        # map -> <prefix>_odom (identity)
        tf1 = TransformStamped()
        tf1.header.stamp = now
        tf1.header.frame_id = 'map'
        tf1.child_frame_id = self.odom_frame
        tf1.transform.rotation.w = 1.0

        # <prefix>_odom -> <prefix>_base_footprint (identity)
        tf2 = TransformStamped()
        tf2.header.stamp = now
        tf2.header.frame_id = self.odom_frame
        tf2.child_frame_id = self.base_frame
        tf2.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([tf1, tf2])

    def timer_cb(self):
        t = time.time() - self.t0
        theta = self.omega * t

        px = self.cx + self.R * math.cos(theta)
        py = self.cy + self.R * math.sin(theta)
        yaw = theta + (math.pi / 2.0 if self.omega >= 0 else -math.pi / 2.0)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualLeaderCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
