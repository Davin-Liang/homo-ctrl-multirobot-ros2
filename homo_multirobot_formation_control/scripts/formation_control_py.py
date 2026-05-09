#!/usr/bin/env python3
"""Python formation control with original ANN-based Lpc_Controller."""

import sys
import os
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Use original Python controller from lft_control (with ANN + cvxpy)
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..',
    'lft_control', 'scripts'))
from homo_ctrl_using_ann1 import Lpc_Controller


class FormationControllerPy(Node):
    def __init__(self):
        super().__init__('formation_control_py')

        self.leader_ns = self.declare_parameter('leader_ns', '/robot1').value
        self.follower_ns = self.declare_parameter('follower_ns', '/robot2').value
        mass = self.declare_parameter('mass', 8.0).value
        radius = self.declare_parameter('radius', 2.0).value
        m_p = self.declare_parameter('m_p', 4).value
        tol = self.declare_parameter('tol', 0.1).value
        rate = self.declare_parameter('control_rate', 20.0).value
        self.Kp_yaw = self.declare_parameter('Kp_yaw', 4.0).value
        self.K_ff = self.declare_parameter('K_ff', 1.0).value

        # Original controller — uses ANN approximator (cvxpy, scipy)
        self.ctrl = Lpc_Controller(m_p=m_p, radius=radius, tol=tol, m=mass)

        self.lock = threading.Lock()
        self.leader_x = np.zeros(4)
        self.follower_x = np.zeros(4)
        self.leader_yaw = 0.0
        self.follower_yaw = 0.0
        self.leader_az = 0.0
        self.leader_amcl_ok = False
        self.follower_amcl_ok = False
        self.leader_odom_ok = False
        self.controller_init = False

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        amcl_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            PoseWithCovarianceStamped, f'{self.leader_ns}/amcl_pose',
            self._leader_amcl_cb, amcl_qos)
        self.create_subscription(
            Odometry, f'{self.leader_ns}/odometry/filtered',
            self._leader_odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, f'{self.follower_ns}/amcl_pose',
            self._follower_amcl_cb, amcl_qos)
        self.create_subscription(
            Odometry, f'{self.follower_ns}/odometry/filtered',
            self._follower_odom_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(f'Python ANN formation node: {self.leader_ns} -> {self.follower_ns}')

    def _yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _leader_amcl_cb(self, msg):
        with self.lock:
            self.leader_x[0] = msg.pose.pose.position.x
            self.leader_x[1] = msg.pose.pose.position.y
            self.leader_yaw = self._yaw(msg.pose.pose.orientation)
            self.leader_amcl_ok = True

    def _leader_odom_cb(self, msg):
        with self.lock:
            vx_body = msg.twist.twist.linear.x
            vy_body = msg.twist.twist.linear.y
            yaw = self.leader_yaw
            # Rotate body-frame velocity to map frame
            self.leader_x[2] = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
            self.leader_x[3] = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
            self.leader_az = msg.twist.twist.angular.z
            self.leader_odom_ok = True

    def _follower_amcl_cb(self, msg):
        with self.lock:
            self.follower_x[0] = msg.pose.pose.position.x
            self.follower_x[1] = msg.pose.pose.position.y
            self.follower_yaw = self._yaw(msg.pose.pose.orientation)
            self.follower_amcl_ok = True

    def _follower_odom_cb(self, msg):
        with self.lock:
            vx_body = msg.twist.twist.linear.x
            vy_body = msg.twist.twist.linear.y
            yaw = self.follower_yaw
            self.follower_x[2] = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
            self.follower_x[3] = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

    def _ready(self):
        return (self.leader_amcl_ok and self.leader_odom_ok and self.follower_amcl_ok)

    def _timer_cb(self):
        if not self._ready():
            return

        with self.lock:
            x1 = self.leader_x.copy()
            x2 = self.follower_x.copy()
            ly, fy = self.leader_yaw, self.follower_yaw
            la = self.leader_az

        if not self.controller_init:
            self.ctrl.controller_initial_(x1, x2)
            self.controller_init = True
            self.get_logger().info('Controller initialized (ANN).')

        out = self.ctrl.lpc_calculate(x1, x2)

        cmd = Twist()
        cmd.linear.x = float(out[0])
        cmd.linear.y = float(out[1])

        re = ly - fy
        ne = math.atan2(math.sin(re), math.cos(re))
        cmd.angular.z = ne * self.Kp_yaw + la * self.K_ff

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = FormationControllerPy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
