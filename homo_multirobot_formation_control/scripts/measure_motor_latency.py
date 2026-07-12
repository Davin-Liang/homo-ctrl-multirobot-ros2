#!/usr/bin/env python3
"""
实物电机响应延迟测试（兼顾安全，不走丢）

每轮: 静止检测 → 发阶跃(前进) → 检测响应 → 立刻倒车复位 → 静止 → 下一轮

用法:
  python3 measure_motor_latency.py --ns /robot2 --trials 5
  python3 measure_motor_latency.py --ns /robot2 --trials 5 --raw-odom-topic /odom
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import argparse
import time
import numpy as np

VELOCITY_THRESHOLD = 0.02  # m/s
HOLD_DURATION = 0.3        # 必须连续静止这么久才算真停
FORWARD_DURATION = 1.5     # 前进最长时间（超时也停车）
REVERSE_DURATION = 2.0     # 倒车复位时间
REVERSE_VEL = 0.15         # 倒车速度（慢一点）


class MotorLatencyMeter(Node):
    def __init__(self, ns, step_vel, trials, raw_odom_topic):
        super().__init__('motor_latency_meter')
        self.ns = ns.rstrip('/')
        self.step_vel = step_vel
        self.trials = trials
        self.raw_odom_topic = raw_odom_topic

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.has_raw = False
        if raw_odom_topic:
            topic = raw_odom_topic if raw_odom_topic.startswith('/') else f'{self.ns}/{raw_odom_topic}'
            self.raw_sub = self.create_subscription(Odometry, topic, self.raw_odom_cb, qos)
            self.has_raw = True

        self.ekf_sub = self.create_subscription(
            Odometry, f'{self.ns}/odometry/filtered', self.ekf_odom_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, f'{self.ns}/cmd_vel', 10)

        self.raw_odom_v = None
        self.raw_odom_time = None
        self.ekf_odom_v = None
        self.ekf_odom_time = None

        self.results_raw = []
        self.results_ekf = []

    def raw_odom_cb(self, msg):
        self.raw_odom_v = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.linear.y)
        self.raw_odom_time = time.time()

    def ekf_odom_cb(self, msg):
        self.ekf_odom_v = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.linear.y)
        self.ekf_odom_time = time.time()

    def wait_stationary(self, timeout=10.0):
        self.get_logger().info('  Waiting for robot to stop...')
        stationary_since = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            is_stop = self.ekf_odom_v is not None and self.ekf_odom_v < VELOCITY_THRESHOLD
            if is_stop:
                if stationary_since is None:
                    stationary_since = time.time()
                elif time.time() - stationary_since > HOLD_DURATION:
                    self.get_logger().info('  Robot stationary.')
                    return True
            else:
                stationary_since = None
        self.get_logger().error('  Timeout waiting for robot to stop!')
        return False

    def reverse_to_start(self):
        """倒车复位：按 REVERSE_DURATION 秒慢速后退"""
        self.get_logger().info('  Reversing to reset position...')
        twist = Twist()
        twist.linear.x = -REVERSE_VEL
        t0 = time.time()
        while time.time() - t0 < REVERSE_DURATION:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)

    def measure_one_trial(self, trial_idx):
        # 1. 等静止
        if not self.wait_stationary():
            return None, None

        # 2. 发阶跃
        twist = Twist()
        twist.linear.x = float(self.step_vel)
        cmd_time = time.time()
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'  Trial {trial_idx}: cmd_vel={self.step_vel} m/s sent')

        # 3. 检测响应
        raw_delay = None
        ekf_delay = None
        t0 = time.time()
        while time.time() - t0 < FORWARD_DURATION:
            rclpy.spin_once(self, timeout_sec=0.005)
            if raw_delay is None and self.has_raw and self.raw_odom_v is not None \
                    and self.raw_odom_v > VELOCITY_THRESHOLD:
                raw_delay = self.raw_odom_time - cmd_time
                self.get_logger().info(
                    f'    raw  odom responded in {raw_delay*1000:.1f} ms')
            if ekf_delay is None and self.ekf_odom_v is not None \
                    and self.ekf_odom_v > VELOCITY_THRESHOLD:
                ekf_delay = self.ekf_odom_time - cmd_time
                self.get_logger().info(
                    f'    EKF  odom responded in {ekf_delay*1000:.1f} ms')
            if ((not self.has_raw or raw_delay is not None) and ekf_delay is not None):
                break

        # 4. 立即停车
        self.cmd_pub.publish(Twist())

        # 5. 倒车复位
        self.reverse_to_start()

        if self.has_raw and raw_delay is None:
            self.get_logger().warn(f'  Trial {trial_idx}: raw odom did not respond!')
        if ekf_delay is None:
            self.get_logger().warn(f'  Trial {trial_idx}: EKF odom did not respond!')

        return raw_delay, ekf_delay

    def run(self):
        self.get_logger().info(
            f'ns={self.ns}, step_vel={self.step_vel} m/s, trials={self.trials}, '
            f'raw_odom={"N/A" if not self.has_raw else self.raw_odom_topic}')

        self.get_logger().info('Warming up subscriptions...')
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.02)

        for i in range(self.trials):
            raw_d, ekf_d = self.measure_one_trial(i + 1)
            if raw_d is not None:
                self.results_raw.append(raw_d)
            if ekf_d is not None:
                self.results_ekf.append(ekf_d)

        self.print_report()

    def print_report(self):
        def stats(name, data):
            if not data:
                print(f'\n  {name}: NO DATA')
                return
            arr = np.array(data) * 1000
            print(f'\n  {name}:')
            print(f'    avg = {np.mean(arr):.1f} ms')
            print(f'    min = {np.min(arr):.1f} ms')
            print(f'    max = {np.max(arr):.1f} ms')
            print(f'    P50  = {np.median(arr):.1f} ms')
            print(f'    std  = {np.std(arr):.1f} ms')
            print(f'    trials = {len(arr)}')

        print('\n' + '=' * 55)
        print('  Motor Response Latency Report')
        print('=' * 55)
        stats('EKF FILTERED (/odometry/filtered)', self.results_ekf)
        if self.has_raw:
            stats(f'RAW ODOM   ({self.raw_odom_topic})', self.results_raw)
            if self.results_raw and self.results_ekf:
                overhead = (np.mean(self.results_ekf) - np.mean(self.results_raw)) * 1000
                print(f'\n  EKF filtering overhead: {overhead:.1f} ms')
        print('=' * 55)


def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', default='/robot2')
    parser.add_argument('--velocity', type=float, default=0.3)
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--raw-odom-topic', default='',
                        help='Raw odom topic (e.g. /odom). Omit to measure EKF only.')
    args = parser.parse_args()

    meter = MotorLatencyMeter(args.ns, args.velocity, args.trials, args.raw_odom_topic)
    meter.run()
    meter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
