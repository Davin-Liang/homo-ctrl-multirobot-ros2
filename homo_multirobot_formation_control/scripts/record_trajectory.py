#!/usr/bin/env python3
"""
记录两台车在 map 坐标系下的轨迹并画图保存。

通过 TF (map -> <ns>_odom) 将 EKF 里程计位置转换到 map 系。
话题有数据时开始计时，到时自动停止。文件名含时间戳防止覆盖。

使用:
  # 默认: leader=/robot1, follower=/robot2
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args -p duration:=30.0 -p out_dir:=/tmp/robot_traj

  # 虚拟 leader + 画编队距离参考线
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args \
    -p leader_ns:=/virtual_leader -p follower_ns:=/robot2 \
    -p radius:=1.0 \
    -p duration:=30.0 -p out_dir:=/tmp/robot_traj
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import time
import math
from datetime import datetime


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('leader_ns', '/robot1')
        self.declare_parameter('follower_ns', '/robot2')
        self.declare_parameter('duration', 30.0)
        self.declare_parameter('out_dir', '/tmp/robot_traj')
        self.declare_parameter('radius', 0.0)

        self.leader_ns = self.get_parameter('leader_ns').value
        self.follower_ns = self.get_parameter('follower_ns').value
        self.duration = self.get_parameter('duration').value
        self.out_dir = self.get_parameter('out_dir').value
        self.ideal_radius = self.get_parameter('radius').value
        os.makedirs(self.out_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.t1_x = []; self.t1_y = []; self.t1_t = []
        self.t2_x = []; self.t2_y = []; self.t2_t = []

        self.t0 = None
        self.done = False

        self.sub1 = self.create_subscription(
            Odometry, self.leader_ns + '/odometry/filtered', self.cb_leader, 10)
        self.sub2 = self.create_subscription(
            Odometry, self.follower_ns + '/odometry/filtered', self.cb_follower, 10)
        self.timer = self.create_timer(0.1, self.check_done)

        leader_short = self.leader_ns.lstrip('/')
        follower_short = self.follower_ns.lstrip('/')
        self.leader_label = f'Leader ({leader_short})'
        self.follower_label = f'Follower ({follower_short})'
        self.get_logger().info(
            f'等待数据... leader={self.leader_ns} follower={self.follower_ns} '
            f'时长={self.duration:.0f}s 输出={self.out_dir}'
            + (f' 理想半径={self.ideal_radius:.1f}m' if self.ideal_radius > 0 else ''))

    def _odom_to_map(self, ns, msg):
        odom_frame = ns.lstrip('/') + '_odom'
        try:
            t = self.tf_buffer.lookup_transform(
                'map', odom_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        tf_x = t.transform.translation.x
        tf_y = t.transform.translation.y
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        tf_yaw = math.atan2(siny, cosy)
        ekf_x = msg.pose.pose.position.x
        ekf_y = msg.pose.pose.position.y
        return (tf_x + ekf_x * math.cos(tf_yaw) - ekf_y * math.sin(tf_yaw),
                tf_y + ekf_x * math.sin(tf_yaw) + ekf_y * math.cos(tf_yaw))

    def _record(self, msg, ns, xl, yl, tl):
        if self.done:
            return
        pos = self._odom_to_map(ns, msg)
        if pos is None:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('收到第一条数据, 开始计时')
        tl.append(now - self.t0)
        xl.append(pos[0])
        yl.append(pos[1])

    def cb_leader(self, msg):
        self._record(msg, self.leader_ns, self.t1_x, self.t1_y, self.t1_t)
    def cb_follower(self, msg):
        self._record(msg, self.follower_ns, self.t2_x, self.t2_y, self.t2_t)

    def check_done(self):
        if self.done or self.t0 is None:
            return
        if time.time() - self.t0 >= self.duration:
            self.done = True
            self._plot_and_save()

    def _plot_and_save(self):
        elapsed = time.time() - self.t0 if self.t0 else 0
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # ---- 子图 1: 轨迹 ----
        ax = axes[0][0]
        for xl, yl, name, c in [
            (self.t1_x, self.t1_y, self.leader_label, 'tab:blue'),
            (self.t2_x, self.t2_y, self.follower_label, 'tab:orange'),
        ]:
            if not xl:
                continue
            ax.plot(xl, yl, linewidth=0.8, label=name, color=c)
            ax.scatter(xl[0], yl[0], c=c, marker='o', s=60, zorder=5)
            ax.scatter(xl[-1], yl[-1], c=c, marker='s', s=60, zorder=5)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'Trajectory in map frame ({elapsed:.1f}s)')
        ax.legend(fontsize=7); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        # ---- 子图 2: X over time ----
        ax = axes[0][1]
        for xl, yl, tl, name, c in [
            (self.t1_x, self.t1_y, self.t1_t, self.leader_label, 'tab:blue'),
            (self.t2_x, self.t2_y, self.t2_t, self.follower_label, 'tab:orange'),
        ]:
            if not tl:
                continue
            ax.plot(tl, xl, linewidth=0.8, label=name, color=c)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('X (m)')
        ax.set_title('X over time')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # ---- 子图 3: Y over time ----
        ax = axes[1][0]
        for xl, yl, tl, name, c in [
            (self.t1_x, self.t1_y, self.t1_t, self.leader_label, 'tab:blue'),
            (self.t2_x, self.t2_y, self.t2_t, self.follower_label, 'tab:orange'),
        ]:
            if not tl:
                continue
            ax.plot(tl, yl, linewidth=0.8, label=name, color=c)
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Y (m)')
        ax.set_title('Y over time')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        # ---- 子图 4: 编队距离 ----
        ax = axes[1][1]
        n = min(len(self.t1_x), len(self.t2_x))
        if n > 0:
            dist_t = self.t2_t[:n]
            dist = [math.hypot(self.t1_x[i] - self.t2_x[i], self.t1_y[i] - self.t2_y[i])
                    for i in range(n)]
            ax.plot(dist_t, dist, linewidth=1.0, color='tab:red',
                    label=f'Actual distance (mean={sum(dist)/n:.2f}m)')
        if self.ideal_radius > 0:
            ax.axhline(y=self.ideal_radius, color='gray', linestyle='--', linewidth=1.2,
                       label=f'Ideal radius = {self.ideal_radius:.1f}m')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Distance (m)')
        ax.set_title('Formation distance')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.out_dir, f'trajectory_{ts}.png')
        plt.tight_layout(); plt.savefig(path, dpi=150); plt.close()

        n1, n2 = len(self.t1_x), len(self.t2_x)
        self.get_logger().info(
            f'已保存 {path}  ({self.leader_label}={n1}, {self.follower_label}={n2})')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
