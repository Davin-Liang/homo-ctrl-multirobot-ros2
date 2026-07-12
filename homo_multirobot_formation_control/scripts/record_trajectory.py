#!/usr/bin/env python3
"""
记录两台车在 map 坐标系下的轨迹、画图并保存 CSV 数据。

使用:
  # 仿真 + 自定义标签
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args -p mode:=sim -p tag:=hpc_0.3m -p duration:=30.0

  # 实物 + 编队半径参考线
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args -p mode:=real -p tag:=4d_mass8_r2 \
    -p leader_ns:=/virtual_leader -p follower_ns:=/robot2 \
    -p radius:=2.0 -p duration:=30.0

输出:
  {out_dir}/{mode}/trajectory_{mode}_{tag}_{timestamp}.png   ← 四子图
  {out_dir}/{mode}/trajectory_{mode}_{tag}_{timestamp}.csv   ← MATLAB 可用
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
import tf2_ros
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import csv
import time
import math
from datetime import datetime

# 需要自动读取的控制器参数
CTRL_PARAM_NAMES = ['mass', 'radius', 'omega_d', 'control_rate',
                    'm_p', 'Kp_yaw', 'K_ff', 'tol']


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('leader_ns', '/robot1')
        self.declare_parameter('follower_ns', '/robot2')
        self.declare_parameter('duration', 30.0)
        self.declare_parameter('out_dir', '')
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('tag', '')

        self.leader_ns = self.get_parameter('leader_ns').value
        self.follower_ns = self.get_parameter('follower_ns').value
        self.duration = self.get_parameter('duration').value
        self.out_dir = self.get_parameter('out_dir').value
        if not self.out_dir:
            pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            self.out_dir = os.path.join(pkg_dir, 'robot_traj')
        self.ideal_radius = self.get_parameter('radius').value
        self.mode = self.get_parameter('mode').value
        self.tag = self.get_parameter('tag').value

        # 查询控制器参数（自动生成 tag 和图上标题）
        self.ctrl_params = self._query_controller_params()
        if not self.tag:
            self.tag = self._build_auto_tag()
        self.ctrl_title = self._build_params_title()

        # 输出到 {out_dir}/{mode}/ 子目录
        out_subdir = os.path.join(self.out_dir, self.mode)
        os.makedirs(out_subdir, exist_ok=True)

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
            f'记录中... leader={self.leader_ns} follower={self.follower_ns} '
            f'时长={self.duration:.0f}s 模式={self.mode} 标签={self.tag or "无"}'
            + (f' 理想半径={self.ideal_radius:.1f}m' if self.ideal_radius > 0 else ''))

    def _query_controller_params(self):
        """从 follower 命名空间下的 formation_control_node 读取参数。"""
        node_path = self.follower_ns.rstrip('/') + '/formation_control_node'
        svc_name = node_path + '/get_parameters'
        client = self.create_client(GetParameters, svc_name)

        # 等控制器就绪（最多等 3 秒）
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'控制器参数服务未就绪 ({svc_name})，使用默认标签')
            return {}

        req = GetParameters.Request()
        req.names = list(CTRL_PARAM_NAMES)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done() and future.result() is not None:
            result = future.result()
            params = {}
            for name, pv in zip(CTRL_PARAM_NAMES, result.values):
                if pv.type == 3:       # PARAMETER_DOUBLE
                    val = pv.double_value
                elif pv.type == 2:     # PARAMETER_INTEGER
                    val = pv.integer_value
                else:
                    continue
                params[name] = val
            if params:
                self.get_logger().info(f'已读取控制器参数: {params}')
            return params
        else:
            self.get_logger().warn('查询控制器参数失败，使用默认标签')
            return {}

    def _build_auto_tag(self):
        """根据控制器参数自动生成文件名标签。"""
        p = self.ctrl_params
        if not p:
            return 'default'
        parts = []
        parts.append(f"m{self._v(p, 'mass')}")
        parts.append(f"r{self._v(p, 'radius')}")
        parts.append(f"od{self._v(p, 'omega_d')}")
        parts.append(f"f{self._v(p, 'control_rate')}")
        return '_'.join(parts)

    @staticmethod
    def _v(params, key):
        """格式化单个参数值（去掉无意义的小数位）。"""
        v = params.get(key, 0)
        if abs(v - round(v)) < 0.01:
            return str(int(round(v)))
        return f'{v:.1f}'

    def _build_params_title(self):
        """生成控制器参数摘要字符串，画在图上。"""
        p = self.ctrl_params
        if not p:
            return ''
        names = ['mass', 'radius', 'omega_d', 'm_p', 'control_rate', 'Kp_yaw', 'K_ff', 'tol']
        parts = []
        for n in names:
            if n in p:
                parts.append(f'{n}={self._v(p, n)}')
        return '  |  '.join(parts)

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
            self._save_and_plot()

    def _build_filename(self, ext):
        """生成带 mode/tag/时间戳 的文件名"""
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        parts = [self.mode]
        if self.tag:
            parts.append(self.tag)
        parts.append(ts)
        return '_'.join(parts) + '.' + ext

    def _save_csv(self, out_subdir, basename):
        """保存对齐后的 CSV 数据（MATLAB 可直接 readtable）"""
        # 用 follower 的时间为基准，找最接近的 leader 数据点
        csv_path = os.path.join(out_subdir, basename.replace('.png', '.csv'))

        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'leader_x_m', 'leader_y_m',
                        'follower_x_m', 'follower_y_m', 'distance_m'])
            n2 = len(self.t2_t)
            n1 = len(self.t1_t)
            for i2 in range(n2):
                t = self.t2_t[i2]
                fx = self.t2_x[i2]
                fy = self.t2_y[i2]
                # 找最接近的 leader 点
                i1 = min(range(n1), key=lambda j: abs(self.t1_t[j] - t))
                lx = self.t1_x[i1]
                ly = self.t1_y[i1]
                dist = math.hypot(lx - fx, ly - fy)
                w.writerow([f'{t:.4f}', f'{lx:.4f}', f'{ly:.4f}',
                            f'{fx:.4f}', f'{fy:.4f}', f'{dist:.4f}'])
        self.get_logger().info(f'CSV 已保存: {csv_path}')

    def _plot_and_save(self, out_subdir, basename):
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
        ax.set_title(f'[{self.mode}] {self.tag} ({elapsed:.1f}s)')
        ax.legend(fontsize=7); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        # 控制器参数显示在图最上方
        if self.ctrl_title:
            fig.suptitle(self.ctrl_title, fontsize=9, family='monospace',
                         y=0.98, bbox=dict(boxstyle='round,pad=0.3',
                                          facecolor='lightyellow', alpha=0.9))

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

        png_path = os.path.join(out_subdir, basename)
        plt.tight_layout(); plt.savefig(png_path, dpi=150); plt.close()

        n1, n2 = len(self.t1_x), len(self.t2_x)
        self.get_logger().info(
            f'PNG 已保存: {png_path}  ({self.leader_label}={n1}, {self.follower_label}={n2})')

    def _save_and_plot(self):
        out_subdir = os.path.join(self.out_dir, self.mode)
        os.makedirs(out_subdir, exist_ok=True)
        basename = self._build_filename('png')
        self._save_csv(out_subdir, basename)
        self._plot_and_save(out_subdir, basename)
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
