#!/usr/bin/env python3
"""
电机响应延迟测试：cmd_vel 发出 → 速度到达目标值的延迟。

用法:
  python3 measure_motor_latency.py --ns /robot2 --trials 5
  python3 measure_motor_latency.py --ns /robot2 --trials 5 --raw-odom-topic /odom
  python3 measure_motor_latency.py --ns /robot2 --velocity 0.3 --target-fraction 0.9
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import argparse
import time
import numpy as np

STATIONARY_THRESHOLD = 0.05   # m/s, 判断静止
HOLD_DURATION = 0.3          # 必须连续静止这么久才算真停
FORWARD_DURATION = 5.0       # 前进最长时间（超时也停车）


class MotorLatencyMeter(Node):
    def __init__(self, ns, step_vel, trials, raw_odom_topic,
                 cmd_vel_topic='cmd_vel', target_fraction=0.9):
        super().__init__('motor_latency_meter')
        self.ns = ns.rstrip('/')
        self.step_vel = step_vel
        self.trials = trials
        self.raw_odom_topic = raw_odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.target_speed = step_vel * target_fraction
        self.target_fraction = target_fraction

        qos = 10

        self.has_raw = False
        if raw_odom_topic:
            topic = raw_odom_topic if raw_odom_topic.startswith('/') else f'{self.ns}/{raw_odom_topic}'
            self.raw_sub = self.create_subscription(Odometry, topic, self.raw_odom_cb, qos)
            self.has_raw = True

        self.ekf_sub = self.create_subscription(
            Odometry, f'{self.ns}/odometry/filtered', self.ekf_odom_cb, qos)

        cmd_topic = cmd_vel_topic if cmd_vel_topic.startswith('/') else f'{self.ns}/{cmd_vel_topic}'
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

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
            is_stop = self.ekf_odom_v is not None and self.ekf_odom_v < STATIONARY_THRESHOLD
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

    def measure_one_trial(self, trial_idx):
        if not self.wait_stationary():
            return None, None

        twist = Twist()
        twist.linear.x = float(self.step_vel)
        cmd_time = time.time()
        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f'  Trial {trial_idx}: cmd_vel={self.step_vel} m/s, '
            f'target={self.target_speed:.2f} m/s ({self.target_fraction*100:.0f}%)')

        raw_delay = None
        ekf_delay = None
        while time.time() - cmd_time < FORWARD_DURATION:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.005)
            if raw_delay is None and self.has_raw and self.raw_odom_v is not None \
                    and self.raw_odom_v >= self.target_speed:
                raw_delay = self.raw_odom_time - cmd_time
                self.get_logger().info(
                    f'    raw  odom reached {self.raw_odom_v:.2f} m/s '
                    f'in {raw_delay*1000:.0f} ms')
            if ekf_delay is None and self.ekf_odom_v is not None \
                    and self.ekf_odom_v >= self.target_speed:
                ekf_delay = self.ekf_odom_time - cmd_time
                self.get_logger().info(
                    f'    EKF  odom reached {self.ekf_odom_v:.2f} m/s '
                    f'in {ekf_delay*1000:.0f} ms')
            if ((not self.has_raw or raw_delay is not None) and ekf_delay is not None):
                break

        self.cmd_pub.publish(Twist())

        if self.has_raw and raw_delay is None:
            self.get_logger().warn(f'  Trial {trial_idx}: raw odom did not respond!')
        if ekf_delay is None:
            self.get_logger().warn(f'  Trial {trial_idx}: EKF odom did not respond!')

        return raw_delay, ekf_delay

    def run(self):
        self.get_logger().info(
            f'ns={self.ns}, step_vel={self.step_vel} m/s, '
            f'target={self.target_speed:.2f} m/s ({self.target_fraction*100:.0f}%), '
            f'trials={self.trials}, raw_odom={"N/A" if not self.has_raw else self.raw_odom_topic}')

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
        print(f'  Motor Response Latency Report')
        print(f'  Target: {self.target_fraction*100:.0f}% of {self.step_vel} m/s '
              f'(={self.target_speed:.2f} m/s)')
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
    parser.add_argument('--target-fraction', type=float, default=0.9,
                        help='Target speed fraction (0-1). 0.9 = time to reach 90%% of cmd_vel')
    parser.add_argument('--raw-odom-topic', default='',
                        help='Raw odom topic (e.g. /odom). Omit to measure EKF only.')
    parser.add_argument('--cmd-vel-topic', default='cmd_vel',
                        help='Cmd_vel topic (relative to ns, or absolute). Default: cmd_vel')
    args = parser.parse_args()

    meter = MotorLatencyMeter(args.ns, args.velocity, args.trials,
                              args.raw_odom_topic, args.cmd_vel_topic,
                              args.target_fraction)
    meter.run()
    meter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
