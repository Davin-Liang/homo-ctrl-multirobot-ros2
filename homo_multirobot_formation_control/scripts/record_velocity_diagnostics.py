#!/usr/bin/env python3
"""Record cmd/odom velocity diagnostics and save a comparison plot + CSV."""

import csv
import math
import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VelocityDiagnosticsRecorder(Node):
    def __init__(self):
        super().__init__('velocity_diagnostics_recorder')

        self.declare_parameter('leader_ns', '/robot1')
        self.declare_parameter('follower_ns', '/robot2')
        self.declare_parameter('duration', 30.0)
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('tag', '')
        self.declare_parameter('out_dir', '')
        self.declare_parameter('sample_rate', 50.0)
        self.declare_parameter('raw_cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('cmd_topic', 'cmd_vel')

        self.leader_ns = self.get_parameter('leader_ns').value.rstrip('/')
        self.follower_ns = self.get_parameter('follower_ns').value.rstrip('/')
        self.duration = float(self.get_parameter('duration').value)
        self.mode = self.get_parameter('mode').value
        self.tag = self.get_parameter('tag').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.raw_cmd_topic = self.get_parameter('raw_cmd_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.out_dir = self.get_parameter('out_dir').value
        if not self.out_dir:
            pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            self.out_dir = os.path.join(pkg_dir, 'robot_traj')
        self.out_dir = os.path.join(self.out_dir, self.mode)
        os.makedirs(self.out_dir, exist_ok=True)

        self.t0 = None
        self.done = False
        self.raw_cmd = None
        self.cmd = None
        self.leader_odom = None
        self.follower_odom = None
        self.rows = []

        self.create_subscription(Twist, self._topic(self.follower_ns, self.raw_cmd_topic), self.raw_cmd_cb, 10)
        self.create_subscription(Twist, self._topic(self.follower_ns, self.cmd_topic), self.cmd_cb, 10)
        self.create_subscription(Odometry, self._topic(self.leader_ns, 'odometry/filtered'), self.leader_odom_cb, 10)
        self.create_subscription(Odometry, self._topic(self.follower_ns, 'odometry/filtered'), self.follower_odom_cb, 10)

        period = 1.0 / max(self.sample_rate, 1.0)
        self.create_timer(period, self.sample)
        self.get_logger().info(
            f'recording velocity diagnostics for {self.duration:.1f}s: '
            f'raw={self._topic(self.follower_ns, self.raw_cmd_topic)} '
            f'cmd={self._topic(self.follower_ns, self.cmd_topic)}')

    @staticmethod
    def _topic(ns, name):
        if name.startswith('/'):
            return name
        return ns + '/' + name

    @staticmethod
    def _twist_values(msg):
        if msg is None:
            return (math.nan, math.nan, math.nan)
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        return (vx, vy, math.hypot(vx, vy))

    @staticmethod
    def _odom_values(msg):
        if msg is None:
            return (math.nan, math.nan, math.nan)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        return (vx, vy, math.hypot(vx, vy))

    def raw_cmd_cb(self, msg):
        self.raw_cmd = msg

    def cmd_cb(self, msg):
        self.cmd = msg

    def leader_odom_cb(self, msg):
        self.leader_odom = msg

    def follower_odom_cb(self, msg):
        self.follower_odom = msg

    def sample(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        raw_vx, raw_vy, raw_v = self._twist_values(self.raw_cmd)
        cmd_vx, cmd_vy, cmd_v = self._twist_values(self.cmd)
        leader_vx, leader_vy, leader_v = self._odom_values(self.leader_odom)
        follower_vx, follower_vy, follower_v = self._odom_values(self.follower_odom)
        self.rows.append({
            'time_s': t,
            'raw_cmd_vx_ms': raw_vx,
            'raw_cmd_vy_ms': raw_vy,
            'raw_cmd_v_ms': raw_v,
            'cmd_vx_ms': cmd_vx,
            'cmd_vy_ms': cmd_vy,
            'cmd_v_ms': cmd_v,
            'leader_odom_vx_ms': leader_vx,
            'leader_odom_vy_ms': leader_vy,
            'leader_odom_v_ms': leader_v,
            'follower_odom_vx_ms': follower_vx,
            'follower_odom_vy_ms': follower_vy,
            'follower_odom_v_ms': follower_v,
        })
        if t >= self.duration and not self.done:
            self.done = True
            self.save()
            rclpy.shutdown()

    def save(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        tag = self.tag.strip() or f'vel_diag_{self.mode}'
        base = os.path.join(self.out_dir, f'{tag}_{stamp}')
        csv_path = base + '.csv'
        png_path = base + '.png'

        with open(csv_path, 'w', newline='') as fp:
            writer = csv.DictWriter(fp, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        t = [r['time_s'] for r in self.rows]
        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'Velocity diagnostics: {tag} ({self.duration:.1f}s)')

        axes[0].plot(t, [r['raw_cmd_v_ms'] for r in self.rows], label='cmd_vel_raw |V|')
        axes[0].plot(t, [r['cmd_v_ms'] for r in self.rows], label='cmd_vel |V|')
        axes[0].plot(t, [r['follower_odom_v_ms'] for r in self.rows], label='follower odom |V|')
        axes[0].plot(t, [r['leader_odom_v_ms'] for r in self.rows], label='leader odom |V|')
        axes[0].set_ylabel('|V| (m/s)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='best')

        axes[1].plot(t, [r['raw_cmd_vx_ms'] for r in self.rows], label='raw vx')
        axes[1].plot(t, [r['cmd_vx_ms'] for r in self.rows], label='cmd vx')
        axes[1].plot(t, [r['follower_odom_vx_ms'] for r in self.rows], label='odom vx')
        axes[1].set_ylabel('Vx (m/s)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='best')

        axes[2].plot(t, [r['raw_cmd_vy_ms'] for r in self.rows], label='raw vy')
        axes[2].plot(t, [r['cmd_vy_ms'] for r in self.rows], label='cmd vy')
        axes[2].plot(t, [r['follower_odom_vy_ms'] for r in self.rows], label='odom vy')
        axes[2].set_ylabel('Vy (m/s)')
        axes[2].set_xlabel('Time (s)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='best')

        fig.tight_layout(rect=[0, 0.02, 1, 0.96])
        fig.savefig(png_path, dpi=150)
        plt.close(fig)

        self.get_logger().info(f'saved CSV: {csv_path}')
        self.get_logger().info(f'saved plot: {png_path}')


def main():
    rclpy.init()
    node = VelocityDiagnosticsRecorder()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
