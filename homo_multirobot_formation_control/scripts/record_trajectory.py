#!/usr/bin/env python3
"""
记录两台车在 map 坐标系下的轨迹并画图保存。

通过 TF (map → <ns>_odom) 将 EKF 里程计位置转换到 map 系。
话题有数据时开始计时，到时自动停止。文件名含时间戳防止覆盖。

使用:
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args -p duration:=30.0 -p out_dir:=/tmp/robot_traj
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

        self.declare_parameter('duration', 30.0)
        self.declare_parameter('out_dir', '/tmp/robot_traj')

        self.duration = self.get_parameter('duration').value
        self.out_dir = self.get_parameter('out_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.t1_x = []; self.t1_y = []; self.t1_t = []
        self.t2_x = []; self.t2_y = []; self.t2_t = []

        self.t0 = None
        self.done = False

        self.sub1 = self.create_subscription(
            Odometry, '/robot1/odometry/filtered', self.cb_robot1, 10)
        self.sub2 = self.create_subscription(
            Odometry, '/robot2/odometry/filtered', self.cb_robot2, 10)
        self.timer = self.create_timer(0.1, self.check_done)

        self.get_logger().info(
            f'等待数据... 时长={self.duration:.0f}s, 输出={self.out_dir}')

    def _odom_to_map(self, ns, msg):
        odom_frame = ns.lstrip('/') + '_odom'
        try:
            t = self.tf_buffer.lookup_transform(
                'map', odom_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
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

    def cb_robot1(self, msg):
        self._record(msg, '/robot1', self.t1_x, self.t1_y, self.t1_t)
    def cb_robot2(self, msg):
        self._record(msg, '/robot2', self.t2_x, self.t2_y, self.t2_t)

    def check_done(self):
        if self.done or self.t0 is None:
            return
        if time.time() - self.t0 >= self.duration:
            self.done = True
            self._plot_and_save()

    def _plot_and_save(self):
        elapsed = time.time() - self.t0 if self.t0 else 0
        fig, axes = plt.subplots(1, 3, figsize=(18, 5))

        # trajectory
        ax = axes[0]
        for xl, yl, tl, name, c in [
            (self.t1_x, self.t1_y, self.t1_t, 'Leader (robot1)', 'tab:blue'),
            (self.t2_x, self.t2_y, self.t2_t, 'Follower (robot2)', 'tab:orange'),
        ]:
            if not xl:
                continue
            ax.plot(xl, yl, linewidth=0.8, label=name, color=c)
            ax.scatter(xl[0], yl[0], c=c, marker='o', s=60, zorder=5)
            ax.scatter(xl[-1], yl[-1], c=c, marker='s', s=60, zorder=5)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'Trajectory in map frame ({elapsed:.1f}s)')
        ax.legend(fontsize=7); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        # X / Y over time
        for i, axis_name in enumerate(['X', 'Y']):
            ax = axes[i + 1]
            for xl, yl, tl, name, c in [
                (self.t1_x, self.t1_y, self.t1_t, 'Leader', 'tab:blue'),
                (self.t2_x, self.t2_y, self.t2_t, 'Follower', 'tab:orange'),
            ]:
                if not tl:
                    continue
                vals = xl if axis_name == 'X' else yl
                ax.plot(tl, vals, linewidth=0.8, label=name, color=c)
            ax.set_xlabel('Time (s)'); ax.set_ylabel(f'{axis_name} (m)')
            ax.set_title(f'{axis_name} over time')
            ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.out_dir, f'trajectory_{ts}.png')
        plt.tight_layout(); plt.savefig(path, dpi=150); plt.close()

        n1, n2 = len(self.t1_x), len(self.t2_x)
        self.get_logger().info(f'已保存 {path}  (robot1={n1}, robot2={n2})')
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
